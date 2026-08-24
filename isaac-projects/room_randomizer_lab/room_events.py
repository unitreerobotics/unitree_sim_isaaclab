# room_events.py
# Event term functions for room randomization.
# Uses OBB collision + continuous wall-zone sampling.

from __future__ import annotations

import math
import random
from typing import List, Optional

import torch
import omni.usd
from pxr import Usd, UsdGeom

from isaaclab.envs import ManagerBasedEnv

from .constants import (
    CHAIR_BBOX,
    CHAIR_ORBIT_OFFSET,
    DESK_BBOX,
    DESK_LOCAL_X_MAX,
    DESK_LOCAL_Y_MAX,
    DESK_OBJECT_MARGIN,
    DESK_OBJECT_Z,
    DESPAWN_Z,
    FALLBACK_NO_SPAWN_BOUNDARIES,
    FLOOR_Z,
    OBB_PLACEMENT_MARGIN,
    ROBOT_BBOX,
    ROBOT_FACING_LAYOUTS,
    ROBOT_FACING_MAX_YAW_OFFSET_RAD,
    ROBOT_FACING_YAW_JITTER_RAD,
    ROBOT_ORBIT_OFFSET,
    ROBOT_Z,
    RIGHT_WALL_CONTACT_CLEARANCE,
    SPAWN_BOUNDARY_TOLERANCE,
    SPAWN_REGION_SEED,
    STATIC_ROOM_OBSTACLES,
    TABLE_GROUP_X_MAX,
    TABLE_GROUP_X_MIN,
    TABLE_GROUP_Y_MAX,
    TABLE_GROUP_Y_MIN,
    TABLE_GROUP_MAX_TRIES,
    TABLE_PROP_META,
    TABLE_SAMPLE_X_MAX,
    TABLE_SAMPLE_X_MIN,
    TABLE_SAMPLE_Y_MAX,
    TABLE_SAMPLE_Y_MIN,
    WALL_PROP_META,
    WALL_ZONES,
    BBox,
    RobotFacingLayout,
    WallZone,
)
from .placement_utils import (
    OBB,
    build_root_state,
    make_obb,
    obb_corners,
    obb_inside_room,
    obb_overlap,
    obb_overlap_any,
    offset_from_yaw,
    offset_from_yaw_batched,
)


_DUPLICATE_VISUAL_PROP_NAMES_TO_HIDE = {
    "SM_Desk_04a",
    "SM_Chair_04a",
    "SM_MedicalCabinet_01a",
    "SM_ShelfSet_01a",
    "SM_SupplyCabinet_01c",
    "SM_SupplyCart_02a",
    "SM_SupplyCart_03a",
    "SM_TrashCan",
    "SM_Plant01",
    "SM_Plant02",
    "SM_CoffeeToGo",
    "SM_Lamp02",
    "SM_BoxPortableC",
}


_visual_props_hidden = False
_EMPTY_USD_BOUND_ABS_LIMIT = 1.0e20
_MIN_STATIC_WALL_HALF_EXTENT = 0.005

def _hide_duplicate_visual_props(env_ids: torch.Tensor) -> None:
    """Hides the original duplicate meshes inside RoomShell."""
    global _visual_props_hidden
    if _visual_props_hidden:
        return
    
    stage = omni.usd.get_context().get_stage()
    for env_idx in range(len(env_ids)):
        env_id = _env_id_int(env_ids, env_idx)
        room_shell = stage.GetPrimAtPath(f"/World/envs/env_{env_id}/RoomShell")
        if not room_shell.IsValid():
            continue
        hidden_count = 0
        for prim in Usd.PrimRange(room_shell):
            if prim.GetName() not in _DUPLICATE_VISUAL_PROP_NAMES_TO_HIDE:
                continue
            imageable = UsdGeom.Imageable(prim)
            if imageable:
                imageable.MakeInvisible()
                hidden_count += 1
        print(
            f"[PLACEMENT_DEBUG] env={env_id} hidden_room_shell_duplicates={hidden_count}",
            flush=True,
        )
    
    _visual_props_hidden = True


# ======================================================================
# Combined event term
# ======================================================================

def randomize_room_layout(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    wall_prop_names: list[str],
    table_prop_names: list[str],
    min_table_objects: int = 2,
):
    """Randomize the full room layout for the given environments.

    Uses OBB collision detection and continuous zone sampling.
    """
    M = len(env_ids)
    device = env.device

    # Hide original static meshes embedded in RoomShell to avoid duplicates
    _hide_duplicate_visual_props(env_ids)

    # Per-environment list of placed OBBs.
    all_placed: List[List[OBB]] = [[] for _ in range(M)]
    all_placed_names: List[List[str]] = [[] for _ in range(M)]

    # Per-environment placement results for later phases.
    desk_positions = torch.zeros(M, 3, device=device)
    desk_yaws = torch.zeros(M, device=device)

    static_wall_obbs, spawn_boundaries = _get_room_geometry(env, env_ids)

    # --- Phase 1: Wall props ------------------------------------------
    wall_debug_obbs = _place_wall_props(
        env,
        env_ids,
        wall_prop_names,
        all_placed,
        all_placed_names,
        static_wall_obbs,
        spawn_boundaries,
    )

    # --- Static RoomShell walls ---------------------------------------
    static_wall_debug_obbs = _add_static_room_obstacles(static_wall_obbs, env_ids, all_placed, all_placed_names)

    # --- Phase 2: Table group (desk + chair + robot) ------------------
    table_debug_obbs = _place_table_group(
        env, env_ids, all_placed, all_placed_names, desk_positions, desk_yaws, spawn_boundaries
    )

    # --- Phase 3: Tabletop objects ------------------------------------
    tabletop_debug_obbs = _place_desk_objects(
        env, env_ids, table_prop_names, desk_positions, desk_yaws, spawn_boundaries, min_table_objects
    )

    # Keep the accepted OBBs available for an optional viewport overlay. Store
    # them by global environment id because reset events may target a subset of
    # environments.
    debug_obbs = getattr(env, "_room_randomizer_debug_obbs", {})
    for env_idx in range(M):
        env_id = _env_id_int(env_ids, env_idx)
        records = []
        records.extend(
            {
                "name": name,
                "category": "wall",
                "box": box,
                "z": FLOOR_Z + 0.04,
            }
            for name, box in wall_debug_obbs[env_idx]
            if box is not None
        )
        records.extend(
            {
                "name": name,
                "category": "static_wall",
                "box": box,
                "z": FLOOR_Z + 0.04,
            }
            for name, box in static_wall_debug_obbs[env_idx]
        )
        records.extend(
            {
                "name": name,
                "category": "robot" if name == "ridgeback" else "table_group",
                "box": box,
                "z": FLOOR_Z + 0.04,
            }
            for name, box in table_debug_obbs[env_idx]
        )
        records.extend(
            {
                "name": name,
                "category": "tabletop",
                "box": box,
                "z": DESK_OBJECT_Z + 0.04,
            }
            for name, box in tabletop_debug_obbs[env_idx]
        )
        debug_obbs[env_id] = records
    env._room_randomizer_debug_obbs = debug_obbs


# ======================================================================
# Phase 1: Wall prop placement — continuous zone sampling
# ======================================================================

def _wall_offset_for_zone(meta, zone: WallZone) -> float:
    offsets = getattr(meta, "wall_offsets", None)
    if offsets is not None and zone.wall in offsets:
        return offsets[zone.wall]
    return meta.wall_offset


def _sample_wall_position(zone: WallZone, meta, rng: random.Random) -> tuple[float, float, float]:
    """Sample a random (cx, cy, yaw) along a wall zone strip.

    Returns:
        (cx, cy, yaw_rad) for the prop centre.
    """
    pos_along_wall = rng.uniform(zone.sample_min, zone.sample_max)

    # Apply per-prop wall offset (push away from wall surface).
    offset = _wall_offset_for_zone(meta, zone)

    if zone.wall == "back":
        cx = pos_along_wall
        cy = zone.fixed_coord + offset  # push into room (+Y)
    else:  # "right"
        cx = zone.fixed_coord - offset  # push into room (-X)
        cy = pos_along_wall

    yaw = zone.base_yaw + meta.yaw_offset

    return cx, cy, yaw


def _wall_prop_footprint_obb(root_x: float, root_y: float, yaw: float, meta) -> OBB:
    """Return the asset footprint OBB, accounting for authored root offsets."""
    footprint_x, footprint_y = offset_from_yaw(
        root_x, root_y, yaw, meta.bbox_center[0], meta.bbox_center[1]
    )
    return make_obb(footprint_x, footprint_y, meta.bbox, yaw)


def _right_wall_support_surfaces(
    candidate: OBB,
    static_wall_obbs: list[tuple[str, OBB]],
) -> list[tuple[str, float]]:
    """Return right-wall segments under the candidate and their room-facing X surfaces."""
    candidate_corners = obb_corners(*candidate)
    candidate_y_min = min(y for _, y in candidate_corners)
    candidate_y_max = max(y for _, y in candidate_corners)
    support_surfaces: list[tuple[str, float]] = []

    for wall_name, wall_box in static_wall_obbs:
        if _spawn_boundary_axis(wall_box) != 0 or SPAWN_REGION_SEED[0] >= wall_box[0]:
            continue
        wall_corners = obb_corners(*wall_box)
        wall_y_min = min(y for _, y in wall_corners)
        wall_y_max = max(y for _, y in wall_corners)
        if candidate_y_max < wall_y_min or candidate_y_min > wall_y_max:
            continue
        support_surfaces.append((wall_name, min(x for x, _ in wall_corners)))

    return support_surfaces


def _snap_right_wall_root_to_surface(
    root_x: float,
    root_y: float,
    yaw: float,
    meta,
    static_wall_obbs: list[tuple[str, OBB]],
) -> tuple[float, tuple[str, ...], float] | None:
    """Return a root X snapped to the wall, supporting segments, and final gap."""
    candidate = _wall_prop_footprint_obb(root_x, root_y, yaw, meta)
    candidate_corners = obb_corners(*candidate)
    support_surfaces = _right_wall_support_surfaces(candidate, static_wall_obbs)
    if not support_surfaces:
        return None

    # The low-X face is encountered first when moving from the room interior
    # toward the right wall. If a prop spans a seam, the lowest face is the
    # limiting surface that keeps it out of every supporting wall segment.
    wall_face_x = min(face_x for _, face_x in support_surfaces)
    target_face_x = wall_face_x - RIGHT_WALL_CONTACT_CLEARANCE
    candidate_right_x = max(x for x, _ in candidate_corners)
    snapped_root_x = root_x + target_face_x - candidate_right_x
    snapped_candidate = _wall_prop_footprint_obb(snapped_root_x, root_y, yaw, meta)
    snapped_right_x = max(x for x, _ in obb_corners(*snapped_candidate))
    gap = wall_face_x - snapped_right_x
    support_names = tuple(name for name, _ in support_surfaces)
    return snapped_root_x, support_names, gap


def _env_id_int(env_ids: torch.Tensor, env_idx: int) -> int:
    """Return a printable env id from a tensor slice."""
    return int(env_ids[env_idx].item())


def _format_corners(box: OBB) -> str:
    """Compact corner formatting for placement diagnostics."""
    return "[" + ", ".join(f"({x:+.3f},{y:+.3f})" for x, y in obb_corners(*box)) + "]"


def _print_obb_debug(
    name: str,
    env_id: int,
    box: OBB,
    prefix: str = "[PLACEMENT_DEBUG]",
    *,
    check_inside_room: bool = True,
):
    inside = obb_inside_room(box)
    print(
        f"{prefix} env={env_id} object={name} "
        f"pos=({box[0]:+.3f},{box[1]:+.3f}) yaw={box[4]:+.3f} "
        f"corners={_format_corners(box)} inside_room={inside}",
        flush=True,
    )
    if check_inside_room and not inside:
        print(
            f"[PLACEMENT_ERROR] env={env_id} object={name} outside_room "
            f"pos=({box[0]:+.3f},{box[1]:+.3f}) yaw={box[4]:+.3f} "
            f"corners={_format_corners(box)}",
            flush=True,
        )


def _is_valid_usd_range(range_box) -> bool:
    """Return whether a USD bbox range has finite, non-empty XY extents."""
    min_pt = range_box.GetMin()
    max_pt = range_box.GetMax()
    values = (min_pt[0], min_pt[1], max_pt[0], max_pt[1])
    if any(abs(float(value)) > _EMPTY_USD_BOUND_ABS_LIMIT for value in values):
        return False
    return float(max_pt[0]) > float(min_pt[0]) and float(max_pt[1]) > float(min_pt[1])


def _find_room_shell_walls_prim(stage: Usd.Stage, room_shell_path: str) -> Usd.Prim:
    """Find the authored Walls prim inside a referenced RoomShell."""
    expected_path = f"{room_shell_path}/Environment/room_static/Walls"
    walls_prim = stage.GetPrimAtPath(expected_path)
    if walls_prim.IsValid():
        return walls_prim

    room_shell_prim = stage.GetPrimAtPath(room_shell_path)
    if not room_shell_prim.IsValid():
        return Usd.Prim()

    for prim in Usd.PrimRange(room_shell_prim):
        if prim.GetName() == "Walls":
            return prim
    return Usd.Prim()


def _wall_obbs_from_stage(stage: Usd.Stage, env_id: int, origin_xy: tuple[float, float]) -> list[tuple[str, OBB]]:
    """Build static wall OBBs from the spawned RoomShell USD geometry."""
    room_shell_path = f"/World/envs/env_{env_id}/RoomShell"
    walls_prim = _find_room_shell_walls_prim(stage, room_shell_path)
    if not walls_prim.IsValid():
        return []

    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        useExtentsHint=True,
    )
    obstacles: list[tuple[str, OBB]] = []
    origin_x, origin_y = origin_xy

    for wall_prim in walls_prim.GetChildren():
        if not wall_prim.IsActive():
            continue

        aligned_range = bbox_cache.ComputeWorldBound(wall_prim).ComputeAlignedBox()
        if not _is_valid_usd_range(aligned_range):
            continue

        min_pt = aligned_range.GetMin()
        max_pt = aligned_range.GetMax()
        center_x = (float(min_pt[0]) + float(max_pt[0])) * 0.5 - origin_x
        center_y = (float(min_pt[1]) + float(max_pt[1])) * 0.5 - origin_y
        half_x = max((float(max_pt[0]) - float(min_pt[0])) * 0.5, _MIN_STATIC_WALL_HALF_EXTENT)
        half_y = max((float(max_pt[1]) - float(min_pt[1])) * 0.5, _MIN_STATIC_WALL_HALF_EXTENT)
        obstacles.append(
            (wall_prim.GetName(), make_obb(center_x, center_y, BBox(half_x, half_y), 0.0))
        )

    return obstacles


def _fallback_spawn_boundary_obbs() -> list[tuple[str, OBB]]:
    """Return fallback no-spawn half-plane boundaries."""
    return [
        (boundary.name, make_obb(boundary.center[0], boundary.center[1], boundary.bbox, boundary.yaw))
        for boundary in FALLBACK_NO_SPAWN_BOUNDARIES
    ]


def _fallback_static_room_obstacle_obbs() -> list[tuple[str, OBB]]:
    """Return conservative fallback wall OBBs when USD bounds are unavailable."""
    return [
        (obstacle.name, make_obb(obstacle.center[0], obstacle.center[1], obstacle.bbox, obstacle.yaw))
        for obstacle in STATIC_ROOM_OBSTACLES
    ]


def _get_room_geometry(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
) -> tuple[List[List[tuple[str, OBB]]], List[List[tuple[str, OBB]]]]:
    """Return static wall OBBs and no-spawn boundaries for each env."""
    static_records: List[List[tuple[str, OBB]]] = [[] for _ in range(len(env_ids))]
    boundary_records: List[List[tuple[str, OBB]]] = [[] for _ in range(len(env_ids))]
    stage = omni.usd.get_context().get_stage()
    env_origin_xy = env.scene.env_origins[env_ids, :2].detach().cpu().tolist()
    fallback_obbs = _fallback_static_room_obstacle_obbs()
    fallback_boundaries = _fallback_spawn_boundary_obbs()

    for env_idx in range(len(env_ids)):
        env_id = _env_id_int(env_ids, env_idx)
        obstacle_obbs = []
        if stage is not None:
            obstacle_obbs = _wall_obbs_from_stage(stage, env_id, tuple(env_origin_xy[env_idx]))
        if obstacle_obbs:
            boundary_obbs = obstacle_obbs
            source = "stage"
        else:
            obstacle_obbs = fallback_obbs
            boundary_obbs = fallback_boundaries
            source = "fallback"

        static_records[env_idx] = obstacle_obbs
        boundary_records[env_idx] = boundary_obbs
        print(
            f"[ROOM_GEOMETRY_DEBUG] env={env_id} static_wall_source={source} "
            f"count={len(obstacle_obbs)} spawn_boundary_source={source} "
            f"boundary_count={len(boundary_obbs)}",
            flush=True,
        )

    return static_records, boundary_records


def _spawn_boundary_axis(boundary_box: OBB) -> int:
    """Return the boundary normal axis: 0 for X-normal, 1 for Y-normal."""
    return 0 if boundary_box[2] <= boundary_box[3] else 1


def _outside_spawn_region_issue(name: str, box: OBB, spawn_boundaries: list[tuple[str, OBB]]) -> str | None:
    """Return an issue string when a box crosses past any no-spawn boundary."""
    for boundary_name, boundary_box in spawn_boundaries:
        axis = _spawn_boundary_axis(boundary_box)
        center = boundary_box[axis]
        seed = SPAWN_REGION_SEED[axis]
        valid_sign = 1.0 if seed >= center else -1.0
        for corner in obb_corners(*box):
            coord = corner[axis]
            if valid_sign * (coord - center) < -SPAWN_BOUNDARY_TOLERANCE:
                axis_name = "x" if axis == 0 else "y"
                return (
                    f"{name} outside_spawn_region boundary={boundary_name} "
                    f"axis={axis_name} limit={center:+.3f} corners={_format_corners(box)}"
                )
    return None


def _wall_zone_supports_boundary(zone: WallZone, boundary_box: OBB) -> bool:
    """Return whether a wall-zone prop is intentionally mounted on this boundary."""
    axis = _spawn_boundary_axis(boundary_box)
    center = boundary_box[axis]
    seed = SPAWN_REGION_SEED[axis]
    if zone.wall == "back":
        return axis == 1 and seed > center
    if zone.wall == "right":
        return axis == 0 and seed < center
    return False


def _blocking_static_wall_overlap(
    candidate: OBB,
    zone: WallZone,
    static_wall_obbs: list[tuple[str, OBB]],
) -> str | None:
    """Return the first non-support wall that overlaps a wall prop candidate."""
    for wall_name, wall_box in static_wall_obbs:
        if _wall_zone_supports_boundary(zone, wall_box):
            continue
        if obb_overlap(candidate, wall_box, margin=OBB_PLACEMENT_MARGIN):
            return wall_name
    return None


def _add_static_room_obstacles(
    static_wall_obbs: List[List[tuple[str, OBB]]],
    env_ids: torch.Tensor,
    all_placed: List[List[OBB]],
    all_placed_names: List[List[str]],
) -> List[List[tuple[str, OBB]]]:
    """Add static RoomShell wall OBBs to the placement set."""
    debug_records: List[List[tuple[str, OBB]]] = [[] for _ in range(len(env_ids))]

    for env_idx in range(len(env_ids)):
        env_id = _env_id_int(env_ids, env_idx)
        for name, box in static_wall_obbs[env_idx]:
            all_placed[env_idx].append(box)
            all_placed_names[env_idx].append(name)
            debug_records[env_idx].append((name, box))
            _print_obb_debug(name, env_id, box, check_inside_room=False)

    return debug_records


def _obb_inside_table_group_bounds(box: OBB) -> bool:
    """Check the stricter robot/table usable area, including virtual open-side limits."""
    for wx, wy in obb_corners(*box):
        if wx < TABLE_GROUP_X_MIN or wx > TABLE_GROUP_X_MAX:
            return False
        if wy < TABLE_GROUP_Y_MIN or wy > TABLE_GROUP_Y_MAX:
            return False
    return True


def _validate_table_group(
    table_obbs: list[tuple[str, OBB]],
    placed: List[OBB],
    placed_names: list[str],
    spawn_boundaries: list[tuple[str, OBB]],
) -> tuple[bool, list[str]]:
    """Validate table-group room bounds and overlaps against placed objects."""
    issues: list[str] = []

    for name, box in table_obbs:
        if not obb_inside_room(box):
            issues.append(f"{name} outside_room corners={_format_corners(box)}")
        if not _obb_inside_table_group_bounds(box):
            issues.append(
                f"{name} outside_table_group_bounds "
                f"bounds=({TABLE_GROUP_X_MIN:+.3f},{TABLE_GROUP_X_MAX:+.3f},"
                f"{TABLE_GROUP_Y_MIN:+.3f},{TABLE_GROUP_Y_MAX:+.3f}) "
                f"corners={_format_corners(box)}"
            )
        spawn_issue = _outside_spawn_region_issue(name, box, spawn_boundaries)
        if spawn_issue is not None:
            issues.append(spawn_issue)

    for name, box in table_obbs:
        for placed_name, placed_box in zip(placed_names, placed):
            if obb_overlap(box, placed_box, margin=OBB_PLACEMENT_MARGIN):
                issues.append(
                    f"{name} overlaps {placed_name} "
                    f"{name}_corners={_format_corners(box)} "
                    f"{placed_name}_corners={_format_corners(placed_box)}"
                )

    for i, (a_name, a_box) in enumerate(table_obbs):
        for b_name, b_box in table_obbs[i + 1:]:
            if obb_overlap(a_box, b_box, margin=OBB_PLACEMENT_MARGIN):
                issues.append(
                    f"{a_name} overlaps {b_name} "
                    f"{a_name}_corners={_format_corners(a_box)} "
                    f"{b_name}_corners={_format_corners(b_box)}"
                )

    return len(issues) == 0, issues


def _make_table_group(dx: float, dy: float, dyaw: float) -> tuple[list[tuple[str, OBB]], tuple[float, float], tuple[float, float]]:
    """Build desk/chair/robot OBBs and satellite XY positions."""
    cx, cy = offset_from_yaw(dx, dy, dyaw, CHAIR_ORBIT_OFFSET[0], CHAIR_ORBIT_OFFSET[1])
    rx, ry = offset_from_yaw(dx, dy, dyaw, ROBOT_ORBIT_OFFSET[0], ROBOT_ORBIT_OFFSET[1])

    chair_yaw = dyaw + math.pi
    robot_yaw = dyaw - math.pi / 2

    table_obbs = [
        ("desk", make_obb(dx, dy, DESK_BBOX, dyaw)),
        ("chair", make_obb(cx, cy, CHAIR_BBOX, chair_yaw)),
        ("ridgeback", make_obb(rx, ry, ROBOT_BBOX, robot_yaw)),
    ]
    return table_obbs, (cx, cy), (rx, ry)


def _make_table_group_from_robot(
    rx: float,
    ry: float,
    robot_yaw: float,
) -> tuple[list[tuple[str, OBB]], tuple[float, float], float]:
    """Build desk/chair/robot OBBs using the robot as the placement anchor."""
    desk_yaw = robot_yaw + math.pi / 2
    dx, dy = offset_from_yaw(
        rx,
        ry,
        desk_yaw,
        -ROBOT_ORBIT_OFFSET[0],
        -ROBOT_ORBIT_OFFSET[1],
    )
    table_obbs, _, _ = _make_table_group(dx, dy, desk_yaw)
    return table_obbs, (dx, dy), desk_yaw


def _normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _sample_wall_facing_robot_pose(rng: random.Random) -> tuple[float, float, float, RobotFacingLayout]:
    """Sample robot placement and aim it at an allowed wall target."""
    layout = rng.choice(ROBOT_FACING_LAYOUTS)
    rx = rng.uniform(TABLE_SAMPLE_X_MIN, TABLE_SAMPLE_X_MAX)
    ry = rng.uniform(TABLE_SAMPLE_Y_MIN, TABLE_SAMPLE_Y_MAX)
    target = rng.uniform(layout.sample_min, layout.sample_max)
    if layout.target_axis == "y":
        target_x, target_y = target, layout.fixed_coord
    else:
        target_x, target_y = layout.fixed_coord, target
    raw_robot_yaw = math.atan2(target_y - ry, target_x - rx)
    yaw_delta = _normalize_angle(raw_robot_yaw - layout.yaw_center)
    yaw_delta = max(-ROBOT_FACING_MAX_YAW_OFFSET_RAD, min(ROBOT_FACING_MAX_YAW_OFFSET_RAD, yaw_delta))
    robot_yaw = layout.yaw_center + yaw_delta
    jitter = 0.0
    if ROBOT_FACING_YAW_JITTER_RAD > 0.0:
        jitter = rng.uniform(-ROBOT_FACING_YAW_JITTER_RAD, ROBOT_FACING_YAW_JITTER_RAD)
    return rx, ry, robot_yaw + jitter, layout


def _debug_table_group(env_id: int, table_obbs: list[tuple[str, OBB]], placed: List[OBB], placed_names: list[str]):
    """Print table group OBBs and overlap diagnostics."""
    for name, box in table_obbs:
        _print_obb_debug(name, env_id, box)

    for i, (a_name, a_box) in enumerate(table_obbs):
        for b_name, b_box in table_obbs[i + 1:]:
            overlaps = obb_overlap(a_box, b_box, margin=OBB_PLACEMENT_MARGIN)
            print(
                f"[PLACEMENT_DEBUG] env={env_id} overlap_check "
                f"a={a_name} b={b_name} overlaps={overlaps}",
                flush=True,
            )
            if overlaps:
                print(
                    f"[PLACEMENT_ERROR] env={env_id} a={a_name} b={b_name} overlap "
                    f"{a_name}_pos=({a_box[0]:+.3f},{a_box[1]:+.3f}) {a_name}_yaw={a_box[4]:+.3f} "
                    f"{a_name}_corners={_format_corners(a_box)} "
                    f"{b_name}_pos=({b_box[0]:+.3f},{b_box[1]:+.3f}) {b_name}_yaw={b_box[4]:+.3f} "
                    f"{b_name}_corners={_format_corners(b_box)}",
                    flush=True,
                )

    for name, box in table_obbs:
        for placed_name, placed_box in zip(placed_names, placed):
            overlaps = obb_overlap(box, placed_box, margin=OBB_PLACEMENT_MARGIN)
            print(
                f"[PLACEMENT_DEBUG] env={env_id} overlap_check "
                f"a={name} b={placed_name} overlaps={overlaps}",
                flush=True,
            )
            if overlaps:
                print(
                    f"[PLACEMENT_ERROR] env={env_id} a={name} b={placed_name} overlap "
                    f"{name}_pos=({box[0]:+.3f},{box[1]:+.3f}) {name}_yaw={box[4]:+.3f} "
                    f"{name}_corners={_format_corners(box)} "
                    f"{placed_name}_pos=({placed_box[0]:+.3f},{placed_box[1]:+.3f}) "
                    f"{placed_name}_yaw={placed_box[4]:+.3f} "
                    f"{placed_name}_corners={_format_corners(placed_box)}",
                    flush=True,
                )


def _place_wall_props(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    wall_prop_names: list[str],
    all_placed: List[List[OBB]],
    all_placed_names: List[List[str]],
    static_wall_obbs: List[List[tuple[str, OBB]]],
    spawn_boundaries: List[List[tuple[str, OBB]]],
) -> List[List[tuple[str, Optional[OBB]]]]:
    """Place wall props using continuous zone sampling + OBB collision."""
    M = len(env_ids)
    device = env.device
    rng = random.Random()

    # Preserve the configured order within each group while giving all tall
    # furniture priority over the smaller wall props.
    sorted_names = sorted(
        wall_prop_names,
        key=lambda n: not WALL_PROP_META[n].tall,
    )
    debug_records: List[List[tuple[str, Optional[OBB]]]] = [[] for _ in range(M)]

    for name in sorted_names:
        meta = WALL_PROP_META[name]
        asset = env.scene[name]

        pos_local = torch.zeros(M, 3, device=device)
        yaw_rad = torch.zeros(M, device=device)

        for env_idx in range(M):
            # Try random positions across allowed walls.
            success = False
            last_reject_reason = "no_valid_candidate"

            # Shuffle which walls to try.
            allowed_zones = [z for z in WALL_ZONES if z.wall in meta.allowed_walls]
            rng.shuffle(allowed_zones)

            for _ in range(100):
                # Pick a random wall zone.
                zone = rng.choice(allowed_zones)
                cx, cy, yaw = _sample_wall_position(zone, meta, rng)
                wall_contact_debug = ""
                if zone.wall == "right":
                    wall_contact = _snap_right_wall_root_to_surface(
                        cx, cy, yaw, meta, static_wall_obbs[env_idx]
                    )
                    if wall_contact is None:
                        last_reject_reason = "right_wall_support_not_found"
                        continue
                    cx, support_names, wall_gap = wall_contact
                    wall_contact_debug = (
                        f" right_wall_support={','.join(support_names)}"
                        f" right_wall_gap={wall_gap:.6f}"
                    )
                candidate = _wall_prop_footprint_obb(cx, cy, yaw, meta)

                # Check room bounds.
                if not obb_inside_room(candidate):
                    last_reject_reason = (
                        f"outside_room corners={_format_corners(candidate)}{wall_contact_debug}"
                    )
                    continue

                spawn_issue = _outside_spawn_region_issue(name, candidate, spawn_boundaries[env_idx])
                if spawn_issue is not None:
                    last_reject_reason = f"{spawn_issue}{wall_contact_debug}"
                    continue

                wall_overlap = _blocking_static_wall_overlap(candidate, zone, static_wall_obbs[env_idx])
                if wall_overlap is not None:
                    last_reject_reason = (
                        f"{name} overlaps_static_wall wall={wall_overlap}{wall_contact_debug}"
                    )
                    continue

                # Check overlap with already-placed props.
                if obb_overlap_any(candidate, all_placed[env_idx], margin=OBB_PLACEMENT_MARGIN):
                    last_reject_reason = f"{name} overlaps_existing_wall_prop{wall_contact_debug}"
                    continue

                # Valid placement!
                pos_local[env_idx] = torch.tensor([cx, cy, FLOOR_Z], device=device)
                yaw_rad[env_idx] = yaw
                all_placed[env_idx].append(candidate)
                all_placed_names[env_idx].append(name)
                debug_records[env_idx].append((name, candidate))
                if wall_contact_debug:
                    print(
                        f"[PLACEMENT_DEBUG] env={_env_id_int(env_ids, env_idx)} "
                        f"object={name}{wall_contact_debug}",
                        flush=True,
                    )
                success = True
                break

            if not success:
                pos_local[env_idx, 2] = DESPAWN_Z
                debug_records[env_idx].append((name, None))
                print(
                    f"[PLACEMENT_ERROR] env={_env_id_int(env_ids, env_idx)} "
                    f"object={name} wall_prop_placement_failed despawning=true "
                    f"last_reject_reason={last_reject_reason}",
                    flush=True,
                )

        # Write to simulation.
        root_state = build_root_state(
            pos_local, yaw_rad,
            env.scene.env_origins, env_ids,
            asset.data.default_root_state,
        )
        asset.write_root_state_to_sim(root_state, env_ids=env_ids)

    for env_idx in range(M):
        env_id = _env_id_int(env_ids, env_idx)
        for name, box in debug_records[env_idx]:
            if box is None:
                print(f"[PLACEMENT_DEBUG] env={env_id} object={name} despawned=true", flush=True)
                continue
            _print_obb_debug(name, env_id, box)
        for i, a_box in enumerate(all_placed[env_idx]):
            for j, b_box in enumerate(all_placed[env_idx][i + 1:], start=i + 1):
                if obb_overlap(a_box, b_box, margin=OBB_PLACEMENT_MARGIN):
                    a_name = all_placed_names[env_idx][i]
                    b_name = all_placed_names[env_idx][j]
                    print(
                        f"[PLACEMENT_ERROR] env={env_id} a={a_name} b={b_name} overlap "
                        f"{a_name}_pos=({a_box[0]:+.3f},{a_box[1]:+.3f}) {a_name}_yaw={a_box[4]:+.3f} "
                        f"{a_name}_corners={_format_corners(a_box)} "
                        f"{b_name}_pos=({b_box[0]:+.3f},{b_box[1]:+.3f}) {b_name}_yaw={b_box[4]:+.3f} "
                        f"{b_name}_corners={_format_corners(b_box)}",
                        flush=True,
                    )

    return debug_records


# ======================================================================
# Phase 2: Table group — continuous interior sampling
# ======================================================================

def _place_table_group(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    all_placed: List[List[OBB]],
    all_placed_names: List[List[str]],
    desk_positions: torch.Tensor,
    desk_yaws: torch.Tensor,
    spawn_boundaries: List[List[tuple[str, OBB]]],
) -> list[list[tuple[str, OBB]]]:
    """Place desk + chair + robot with continuous sampling and OBB collision."""
    M = len(env_ids)
    device = env.device
    rng = random.Random()
    env_origins = env.scene.env_origins
    group_placed_mask = torch.zeros(M, dtype=torch.bool, device=device)
    final_table_obbs: list[list[tuple[str, OBB]]] = [[] for _ in range(M)]

    for env_idx in range(M):
        success = False
        selected_obbs: list[tuple[str, OBB]] = []
        selected_layout_name = "none"

        for _ in range(TABLE_GROUP_MAX_TRIES):
            rx, ry, robot_yaw, layout = _sample_wall_facing_robot_pose(rng)
            table_obbs, (dx, dy), dyaw = _make_table_group_from_robot(rx, ry, robot_yaw)
            valid, _ = _validate_table_group(
                table_obbs, all_placed[env_idx], all_placed_names[env_idx], spawn_boundaries[env_idx]
            )
            if not valid:
                continue

            # Valid!
            desk_positions[env_idx] = torch.tensor([dx, dy, FLOOR_Z], device=device)
            desk_yaws[env_idx] = dyaw
            selected_obbs = table_obbs
            selected_layout_name = layout.name
            success = True
            break

        if not success:
            # Fallback: keep the same constrained wall-facing policy.
            fallback_issues: list[str] = []
            for _ in range(TABLE_GROUP_MAX_TRIES):
                rx, ry, robot_yaw, layout = _sample_wall_facing_robot_pose(rng)
                table_obbs, (dx, dy), dyaw = _make_table_group_from_robot(rx, ry, robot_yaw)
                valid, fallback_issues = _validate_table_group(
                    table_obbs, all_placed[env_idx], all_placed_names[env_idx], spawn_boundaries[env_idx]
                )
                if valid:
                    desk_positions[env_idx] = torch.tensor([dx, dy, FLOOR_Z], device=device)
                    desk_yaws[env_idx] = dyaw
                    selected_obbs = table_obbs
                    selected_layout_name = layout.name
                    success = True
                    print(
                        f"[PLACEMENT_DEBUG] env={_env_id_int(env_ids, env_idx)} "
                        f"table_group fallback_validated=true",
                        flush=True,
                    )
                    break

            if not success:
                env_id = _env_id_int(env_ids, env_idx)
                desk_positions[env_idx] = torch.tensor([0.0, 0.0, DESPAWN_Z], device=device)
                desk_yaws[env_idx] = 0.0
                print(
                    f"[PLACEMENT_ERROR] env={env_id} table_group placement_failed despawning=true "
                    f"last_issues={fallback_issues}",
                    flush=True,
                )

        if success:
            final_table_obbs[env_idx] = selected_obbs
            all_placed[env_idx].extend([box for _, box in selected_obbs])
            all_placed_names[env_idx].extend([name for name, _ in selected_obbs])
            group_placed_mask[env_idx] = True
            print(
                f"[PLACEMENT_DEBUG] env={_env_id_int(env_ids, env_idx)} "
                f"table_group_layout={selected_layout_name}",
                flush=True,
            )

    # --- Write desk, chair, ridgeback to sim --------------------------
    for env_idx in range(M):
        env_id = _env_id_int(env_ids, env_idx)
        if final_table_obbs[env_idx]:
            pre_group_count = len(all_placed[env_idx]) - len(final_table_obbs[env_idx])
            _debug_table_group(
                env_id,
                final_table_obbs[env_idx],
                all_placed[env_idx][:pre_group_count],
                all_placed_names[env_idx][:pre_group_count],
            )

    # Build batched positions for satellites.
    chair_pos = offset_from_yaw_batched(
        desk_positions, desk_yaws,
        CHAIR_ORBIT_OFFSET[0], CHAIR_ORBIT_OFFSET[1], FLOOR_Z,
    )
    chair_yaw = desk_yaws + math.pi

    robot_pos = offset_from_yaw_batched(
        desk_positions, desk_yaws,
        ROBOT_ORBIT_OFFSET[0], ROBOT_ORBIT_OFFSET[1], ROBOT_Z,
    )
    robot_yaw = desk_yaws - math.pi / 2

    invalid_mask = ~group_placed_mask
    if torch.any(invalid_mask):
        desk_positions[invalid_mask, 0:2] = 0.0
        desk_positions[invalid_mask, 2] = DESPAWN_Z
        chair_pos[invalid_mask, 0:2] = 0.0
        chair_pos[invalid_mask, 2] = DESPAWN_Z
        robot_pos[invalid_mask, 0:2] = 0.0
        robot_pos[invalid_mask, 2] = DESPAWN_Z
        chair_yaw[invalid_mask] = 0.0
        robot_yaw[invalid_mask] = 0.0

    desk_asset = env.scene["desk"]
    desk_state = build_root_state(desk_positions, desk_yaws, env_origins, env_ids, desk_asset.data.default_root_state)
    desk_asset.write_root_state_to_sim(desk_state, env_ids=env_ids)

    chair_asset = env.scene["chair"]
    chair_state = build_root_state(chair_pos, chair_yaw, env_origins, env_ids, chair_asset.data.default_root_state)
    chair_asset.write_root_state_to_sim(chair_state, env_ids=env_ids)

    try:
        robot_asset = env.scene["ridgeback"]
    except KeyError:
        robot_asset = None
    if robot_asset is not None:
        robot_state = build_root_state(robot_pos, robot_yaw, env_origins, env_ids, robot_asset.data.default_root_state)
        robot_asset.write_root_state_to_sim(robot_state, env_ids=env_ids)

    return final_table_obbs


# ======================================================================
# Phase 3: Tabletop objects — OBB collision on desk surface
# ======================================================================

def _place_desk_objects(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    table_prop_names: list[str],
    desk_pos: torch.Tensor,
    desk_yaw_rad: torch.Tensor,
    spawn_boundaries: List[List[tuple[str, OBB]]],
    min_table_objects: int = 2,
) -> list[list[tuple[str, OBB]]]:
    """Place tabletop objects on the desk surface with rejection sampling.

    Randomly despawns 0–1 objects to achieve variable count (min_table_objects
    to len(table_prop_names)).
    """
    M = len(env_ids)
    debug_obbs: list[list[tuple[str, OBB]]] = [[] for _ in range(M)]
    if not table_prop_names:
        return debug_obbs

    device = env.device
    env_origins = env.scene.env_origins
    rng = random.Random()

    num_total = len(table_prop_names)

    for env_idx in range(M):
        if desk_pos[env_idx, 2].item() <= DESPAWN_Z * 0.5:
            for name in table_prop_names:
                asset = env.scene[name]
                pos = torch.tensor([0.0, 0.0, DESPAWN_Z], device=device).unsqueeze(0)
                yaw = torch.tensor([0.0], device=device)
                eid = env_ids[env_idx:env_idx+1]
                root_state = build_root_state(pos, yaw, env_origins, eid, asset.data.default_root_state)
                asset.write_root_state_to_sim(root_state, env_ids=eid)
            print(
                f"[PLACEMENT_ERROR] env={_env_id_int(env_ids, env_idx)} "
                f"tabletop_objects skipped table_group_not_placed=true",
                flush=True,
            )
            continue

        # How many objects in this env (2 or 3).
        count = rng.randint(min_table_objects, num_total)
        desk_placed: List[OBB] = []

        for i, name in enumerate(table_prop_names):
            asset = env.scene[name]
            meta = TABLE_PROP_META[name]
            visible = i < count

            if visible:
                # Rejection-sample local (x, y) on desk surface.
                placed = False
                last_reject_reason = "no_valid_candidate"
                for _ in range(100):
                    lx = rng.uniform(-DESK_LOCAL_X_MAX, DESK_LOCAL_X_MAX)
                    ly = rng.uniform(-DESK_LOCAL_Y_MAX, DESK_LOCAL_Y_MAX)
                    obj_yaw = rng.uniform(0, 2 * math.pi)

                    candidate = make_obb(lx, ly, meta.bbox, obj_yaw)
                    if obb_overlap_any(candidate, desk_placed, margin=DESK_OBJECT_MARGIN):
                        last_reject_reason = f"{name} overlaps_tabletop_object"
                        continue

                    # Transform to world.
                    wx, wy = offset_from_yaw(
                        desk_pos[env_idx, 0].item(),
                        desk_pos[env_idx, 1].item(),
                        desk_yaw_rad[env_idx].item(),
                        lx, ly,
                    )
                    world_yaw = desk_yaw_rad[env_idx].item() + obj_yaw
                    world_box = make_obb(wx, wy, meta.bbox, world_yaw)
                    spawn_issue = _outside_spawn_region_issue(name, world_box, spawn_boundaries[env_idx])
                    if spawn_issue is not None:
                        last_reject_reason = spawn_issue
                        continue

                    desk_placed.append(candidate)

                    pos = torch.tensor([wx, wy, DESK_OBJECT_Z], device=device).unsqueeze(0)
                    yaw = torch.tensor([world_yaw], device=device)
                    eid = env_ids[env_idx:env_idx+1]

                    root_state = build_root_state(pos, yaw, env_origins, eid, asset.data.default_root_state)
                    asset.write_root_state_to_sim(root_state, env_ids=eid)
                    debug_obbs[env_idx].append(
                        (name, world_box)
                    )
                    placed = True
                    break

                if not placed:
                    # Couldn't fit safely; despawn instead of stacking objects.
                    pos = torch.tensor([0.0, 0.0, DESPAWN_Z], device=device).unsqueeze(0)
                    yaw = torch.tensor([0.0], device=device)
                    eid = env_ids[env_idx:env_idx+1]
                    root_state = build_root_state(pos, yaw, env_origins, eid, asset.data.default_root_state)
                    asset.write_root_state_to_sim(root_state, env_ids=eid)
                    print(
                        f"[PLACEMENT_ERROR] env={_env_id_int(env_ids, env_idx)} "
                        f"object={name} tabletop_placement_failed despawning=true "
                        f"last_reject_reason={last_reject_reason}",
                        flush=True,
                    )
            else:
                # Despawn.
                pos = torch.tensor([0.0, 0.0, DESPAWN_Z], device=device).unsqueeze(0)
                yaw = torch.tensor([0.0], device=device)
                eid = env_ids[env_idx:env_idx+1]
                root_state = build_root_state(pos, yaw, env_origins, eid, asset.data.default_root_state)
                asset.write_root_state_to_sim(root_state, env_ids=eid)

    return debug_obbs
