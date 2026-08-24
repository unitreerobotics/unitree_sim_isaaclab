# room_events.py
# Event term functions for room randomization in pick & place tasks.

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import List, Optional

import torch
import omni.usd
from pxr import Usd, UsdGeom

from isaaclab.envs import ManagerBasedEnv

from .constants import (
    DESK_BBOX,
    DESK_LAMP_LOCAL_X_RANGE,
    DESK_LAMP_LOCAL_Y_RANGE,
    DESK_LAMP_LOCAL_YAW,
    DESK_LOCAL_X_MIN,
    DESK_LOCAL_X_MAX,
    DESK_LOCAL_Y_MIN,
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
    ROBOT_TABLE_MARGIN,
    RIDGEBACK_BBOX,
    RIDGEBACK_CORRIDOR_HALF_WIDTH,
    RIDGEBACK_DELIVERY_ROBOT_LOCAL,
    RIDGEBACK_GROUP_MARGIN,
    RIDGEBACK_PLANAR_YAW,
    RIDGEBACK_ROOT_YAW_OFFSET,
    RIDGEBACK_STAGING_ROBOT_LOCAL,
    RIDGEBACK_WAITING_ROBOT_LOCAL,
    RIGHT_WALL_CONTACT_CLEARANCE,
    SPAWN_BOUNDARY_TOLERANCE,
    SPAWN_REGION_SEED,
    STATIC_ROOM_OBSTACLES,
    TABLE_GROUP_X_MAX,
    TABLE_GROUP_X_MIN,
    TABLE_GROUP_Y_MAX,
    TABLE_GROUP_Y_MIN,
    TABLE_GROUP_MAX_TRIES,
    TABLE_RESERVED_AREAS,
    TABLETOP_CUBE_LOCAL_X_MAX,
    TABLETOP_CUBE_LOCAL_X_MIN,
    TABLETOP_CUBE_PROP_NAMES,
    TABLE_SAMPLE_X_MAX,
    TABLE_SAMPLE_X_MIN,
    TABLE_SAMPLE_Y_MAX,
    TABLE_SAMPLE_Y_MIN,
    TABLE_PROP_META,
    TablePropMeta,
    WALL_PROP_META,
    WALL_ZONES,
    BBox,
    WallZone,
    RobotFacingLayout,
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
_DYNAMIC_TABLETOP_OBJECT_NAMES = {
    name for name, meta in TABLE_PROP_META.items() if meta.dynamic
}
_TABLE_RESERVED_AREA_NAMES = {area.name for area in TABLE_RESERVED_AREAS}
_EMPTY_USD_BOUND_ABS_LIMIT = 1.0e20
_MIN_STATIC_WALL_HALF_EXTENT = 0.005


@dataclass(frozen=True)
class Pose2D:
    """World-space planar pose, with the authored root height retained."""

    position: tuple[float, float, float]
    yaw: float


@dataclass(frozen=True)
class TabletopPlacement:
    """A collision-checked pose expressed in the packing-table frame."""

    name: str
    local_pose: tuple[float, float, float, float]
    bbox: BBox


@dataclass(frozen=True)
class TabletopSpawnRegion:
    """Table-local center bounds used when sampling tabletop objects."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def __post_init__(self) -> None:
        if not (DESK_LOCAL_X_MIN <= self.x_min < self.x_max <= DESK_LOCAL_X_MAX):
            raise ValueError(
                "tabletop spawn x bounds must stay inside the packing table: "
                f"[{self.x_min}, {self.x_max}]"
            )
        if not (DESK_LOCAL_Y_MIN <= self.y_min < self.y_max <= DESK_LOCAL_Y_MAX):
            raise ValueError(
                "tabletop spawn y bounds must stay inside the packing table: "
                f"[{self.y_min}, {self.y_max}]"
            )


@dataclass(frozen=True)
class StaticClusterMember:
    """Kinematic scene asset anchored to the randomized G1 frame."""

    asset_name: str
    robot_local_xy: tuple[float, float]
    yaw_offset: float
    bbox: BBox
    allow_protected_overlap: bool = False


@dataclass
class RoomLayoutState:
    """Single source of truth for one randomized environment layout."""

    env_id: int
    robot_pose: Pose2D
    packing_table_pose: Pose2D
    target_object_local_pose: tuple[float, float, float, float]
    ridgeback_waiting_pose: Pose2D
    selected_wall_facing_layout: str
    tabletop_placements: dict[str, TabletopPlacement] = field(default_factory=dict)
    ridgeback_joint_targets: dict[str, tuple[float, float, float]] = field(default_factory=dict)
    static_cluster_poses: dict[str, Pose2D] = field(default_factory=dict)


def _room_rng(env: ManagerBasedEnv) -> random.Random:
    """Return a persistent RNG seeded from the environment configuration."""
    if not hasattr(env, "_room_randomizer_rng"):
        seed = int(getattr(getattr(env, "cfg", None), "seed", 0) or 0)
        env._room_randomizer_rng = random.Random(seed)
    return env._room_randomizer_rng


def _write_root_pose_to_sim(asset, root_state: torch.Tensor, env_ids: torch.Tensor) -> None:
    """Write only root pose for kinematic props to avoid PhysX velocity errors."""
    asset.write_root_pose_to_sim(root_state[:, :7], env_ids=env_ids)


def _write_tabletop_root_state(asset, name: str, root_state: torch.Tensor, env_ids: torch.Tensor) -> None:
    """Write tabletop object state, preserving kinematic handling for static props."""
    if name in _DYNAMIC_TABLETOP_OBJECT_NAMES:
        asset.write_root_state_to_sim(root_state, env_ids=env_ids)
    else:
        _write_root_pose_to_sim(asset, root_state, env_ids)


def _despawn_props(env: ManagerBasedEnv, env_ids: torch.Tensor, prop_names: list[str]) -> None:
    if not prop_names:
        return

    device = env.device
    env_origins = env.scene.env_origins
    pos = torch.zeros(len(env_ids), 3, device=device)
    pos[:, 2] = DESPAWN_Z
    yaw = torch.zeros(len(env_ids), device=device)

    for name in prop_names:
        if name == "object" or name not in env.scene.keys():
            continue
        asset = env.scene[name]
        root_state = build_root_state(pos, yaw, env_origins, env_ids, asset.data.default_root_state)
        if name in _DYNAMIC_TABLETOP_OBJECT_NAMES:
            asset.write_root_state_to_sim(root_state, env_ids=env_ids)
        else:
            _write_root_pose_to_sim(asset, root_state, env_ids)
        print(
            f"[PLACEMENT_DEBUG] object={name} disabled_table_or_wall_prop despawned=true",
            flush=True,
        )


def _hide_duplicate_visual_props(env_ids: torch.Tensor) -> None:
    """Hides the original duplicate meshes inside RoomShell."""
    global _visual_props_hidden
    if _visual_props_hidden:
        return

    stage = omni.usd.get_context().get_stage()
    for env_idx in range(len(env_ids)):
        env_id = int(env_ids[env_idx].item())
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

def randomize_pickplace_room_layout(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    wall_prop_names: list[str],
    table_prop_names: list[str],
    min_table_objects: int = 0,
    randomize_table_position: bool = True,
    table_prop_meta_overrides: dict[str, TablePropMeta] | None = None,
    tabletop_spawn_region: TabletopSpawnRegion | None = None,
    tabletop_object_margin: float = DESK_OBJECT_MARGIN,
    static_cluster_members: tuple[StaticClusterMember, ...] = (),
):
    """Randomize a room layout for G1 pick-and-place environments.

    ``randomize_table_position=False`` keeps the authored table anchor while
    still scrambling wall and tabletop props. This gives teleoperation a
    predictable initial workspace; a full reset opts back into the original
    table-group randomization. Uses OBB collision detection and continuous
    zone sampling. ``table_prop_meta_overrides`` lets a task supply footprints
    for locally scaled assets without changing the shared room defaults.
    ``static_cluster_members`` are collision-checked and moved rigidly with G1;
    ``tabletop_spawn_region`` and ``tabletop_object_margin`` can define a
    denser task-local reach area without changing shared placement defaults.
    """
    M = len(env_ids)
    device = env.device
    rng = _room_rng(env)
    table_prop_meta = dict(TABLE_PROP_META)
    if table_prop_meta_overrides:
        unknown_names = set(table_prop_meta_overrides) - set(TABLE_PROP_META)
        if unknown_names:
            raise KeyError(
                "tabletop metadata overrides contain unknown scene names: "
                f"{sorted(unknown_names)}"
            )
        table_prop_meta.update(table_prop_meta_overrides)
    missing_cluster_assets = [
        member.asset_name
        for member in static_cluster_members
        if member.asset_name not in env.scene.keys()
    ]
    if missing_cluster_assets:
        raise KeyError(
            "static cluster members are missing from the scene: "
            f"{sorted(missing_cluster_assets)}"
        )

    # Hide duplicate meshes inside RoomShell
    _hide_duplicate_visual_props(env_ids)
    active_wall_props = set(wall_prop_names)
    active_table_props = set(table_prop_names)
    _despawn_props(
        env,
        env_ids,
        [name for name in WALL_PROP_META.keys() if name not in active_wall_props]
        + [name for name in TABLE_PROP_META.keys() if name != "object" and name not in active_table_props],
    )

    # Per-environment list of placed OBBs
    all_placed: List[List[OBB]] = [[] for _ in range(M)]
    all_placed_names: List[List[str]] = [[] for _ in range(M)]

    # Per-environment placement results for table group
    desk_positions = torch.zeros(M, 3, device=device)
    desk_yaws = torch.zeros(M, device=device)

    static_wall_obbs, spawn_boundaries = _get_room_geometry(env, env_ids)

    if randomize_table_position:
        # Preserve the established full-randomization order: furniture first,
        # then sample a table group around the accepted furniture layout.
        wall_debug_obbs = _place_wall_props(
            env,
            env_ids,
            wall_prop_names,
            all_placed,
            all_placed_names,
            static_wall_obbs,
            spawn_boundaries,
            rng,
        )
        static_wall_debug_obbs = _add_static_room_obstacles(
            static_wall_obbs, env_ids, all_placed, all_placed_names
        )
        table_debug_obbs, selected_layout_names = _place_table_group(
            env,
            env_ids,
            all_placed,
            all_placed_names,
            desk_positions,
            desk_yaws,
            spawn_boundaries,
            rng,
            randomize_position=True,
            static_cluster_members=static_cluster_members,
        )
    else:
        # Reserve the fixed table/robot/Ridgeback group before placing wall
        # furniture so randomized props cannot overlap the teleop workspace.
        static_counts = [len(records) for records in static_wall_obbs]
        static_wall_debug_obbs = _add_static_room_obstacles(
            static_wall_obbs, env_ids, all_placed, all_placed_names
        )
        table_debug_obbs, selected_layout_names = _place_table_group(
            env,
            env_ids,
            all_placed,
            all_placed_names,
            desk_positions,
            desk_yaws,
            spawn_boundaries,
            rng,
            randomize_position=False,
            static_cluster_members=static_cluster_members,
        )
        # Wall props may intentionally touch their supporting room wall. Keep
        # the fixed table group in the collision set, but let the dedicated
        # wall-support checks handle static RoomShell geometry.
        for env_idx, static_count in enumerate(static_counts):
            del all_placed[env_idx][:static_count]
            del all_placed_names[env_idx][:static_count]
        wall_debug_obbs = _place_wall_props(
            env,
            env_ids,
            wall_prop_names,
            all_placed,
            all_placed_names,
            static_wall_obbs,
            spawn_boundaries,
            rng,
        )

    # --- Phase 3: Tabletop objects (target object + distractors) ---
    tabletop_debug_obbs, tabletop_placements = _place_desk_objects(
        env,
        env_ids,
        ["object"] + list(table_prop_names),
        desk_positions,
        desk_yaws,
        spawn_boundaries,
        min_table_objects,
        rng,
        table_prop_meta,
        tabletop_spawn_region,
        tabletop_object_margin,
    )

    _store_layout_state(
        env,
        env_ids,
        desk_positions,
        desk_yaws,
        selected_layout_names,
        tabletop_placements,
        static_cluster_members,
    )

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
                "category": "robot" if name == "robot" else "table_group",
                "box": box,
                "z": FLOOR_Z + 0.04,
            }
            for name, box in table_debug_obbs[env_idx]
        )
        records.extend(
            {
                "name": name,
                "category": "table_reserved" if name in _TABLE_RESERVED_AREA_NAMES else "tabletop",
                "box": box,
                "z": DESK_OBJECT_Z + 0.04,
            }
            for name, box in tabletop_debug_obbs[env_idx]
        )
        debug_obbs[env_id] = records
    env._room_randomizer_debug_obbs = debug_obbs


def randomize_wall_props_layout(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    wall_prop_names: list[str],
):
    """Randomize wall props without moving the task's robot, table, or tabletop objects."""
    num_envs = len(env_ids)
    rng = _room_rng(env)

    _hide_duplicate_visual_props(env_ids)
    active_wall_props = set(wall_prop_names)
    _despawn_props(
        env,
        env_ids,
        [name for name in WALL_PROP_META if name not in active_wall_props],
    )

    all_placed: List[List[OBB]] = [[] for _ in range(num_envs)]
    all_placed_names: List[List[str]] = [[] for _ in range(num_envs)]
    static_wall_obbs, spawn_boundaries = _get_room_geometry(env, env_ids)

    wall_debug_obbs = _place_wall_props(
        env,
        env_ids,
        wall_prop_names,
        all_placed,
        all_placed_names,
        static_wall_obbs,
        spawn_boundaries,
        rng,
    )
    static_wall_debug_obbs = _add_static_room_obstacles(
        static_wall_obbs,
        env_ids,
        all_placed,
        all_placed_names,
    )

    debug_obbs = getattr(env, "_room_randomizer_debug_obbs", {})
    for env_idx in range(num_envs):
        env_id = _env_id_int(env_ids, env_idx)
        records = [
            {
                "name": name,
                "category": "wall",
                "box": box,
                "z": FLOOR_Z + 0.04,
            }
            for name, box in wall_debug_obbs[env_idx]
            if box is not None
        ]
        records.extend(
            {
                "name": name,
                "category": "static_wall",
                "box": box,
                "z": FLOOR_Z + 0.04,
            }
            for name, box in static_wall_debug_obbs[env_idx]
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
    """Sample a random (cx, cy, yaw) along a wall zone strip."""
    pos_along_wall = rng.uniform(zone.sample_min, zone.sample_max)
    offset = _wall_offset_for_zone(meta, zone)

    if zone.wall == "back":
        cx = pos_along_wall
        cy = zone.fixed_coord + offset  # push into room
    else:  # "right"
        cx = zone.fixed_coord - offset  # push into room
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


def _place_wall_props(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    wall_prop_names: list[str],
    all_placed: List[List[OBB]],
    all_placed_names: List[List[str]],
    static_wall_obbs: List[List[tuple[str, OBB]]],
    spawn_boundaries: List[List[tuple[str, OBB]]],
    rng: random.Random,
) -> List[List[tuple[str, Optional[OBB]]]]:
    """Place wall props using continuous zone sampling + OBB collision."""
    M = len(env_ids)
    device = env.device

    sorted_names = sorted(
        wall_prop_names,
        key=lambda n: not WALL_PROP_META[n].tall,
    )
    debug_records: List[List[tuple[str, Optional[OBB]]]] = [[] for _ in range(M)]

    for name in sorted_names:
        if name not in env.scene.keys():
            continue
        meta = WALL_PROP_META[name]
        asset = env.scene[name]

        pos_local = torch.zeros(M, 3, device=device)
        yaw_rad = torch.zeros(M, device=device)

        for env_idx in range(M):
            success = False
            last_reject_reason = "no_valid_candidate"
            allowed_zones = [z for z in WALL_ZONES if z.wall in meta.allowed_walls]
            rng.shuffle(allowed_zones)

            for _ in range(100):
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

                if obb_overlap_any(candidate, all_placed[env_idx], margin=OBB_PLACEMENT_MARGIN):
                    last_reject_reason = f"{name} overlaps_existing_wall_prop{wall_contact_debug}"
                    continue

                # Valid placement
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

        root_state = build_root_state(
            pos_local, yaw_rad,
            env.scene.env_origins, env_ids,
            asset.data.default_root_state,
        )
        _write_root_pose_to_sim(asset, root_state, env_ids)

    for env_idx in range(M):
        env_id = _env_id_int(env_ids, env_idx)
        for name, box in debug_records[env_idx]:
            if box is None:
                print(f"[PLACEMENT_DEBUG] env={env_id} object={name} despawned=true", flush=True)
                continue
            _print_obb_debug(name, env_id, box)
        for i, a_box in enumerate(all_placed[env_idx]):
            for j, b_box in enumerate(all_placed[env_idx][i + 1:], start=i + 1):
                a_name = all_placed_names[env_idx][i]
                b_name = all_placed_names[env_idx][j]
                if (
                    a_name.startswith(_RIDGEBACK_GEOMETRY_PREFIX)
                    and b_name.startswith(_RIDGEBACK_GEOMETRY_PREFIX)
                ) or {a_name, b_name} == {"packing_table", "robot"}:
                    # These are intentional overlaps inside the reserved table
                    # group, not wall-prop placement failures.
                    continue
                if obb_overlap(a_box, b_box, margin=OBB_PLACEMENT_MARGIN):
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

_RIDGEBACK_GEOMETRY_PREFIX = "ridgeback_"


def _ridgeback_world_xy(
    rx: float, ry: float, robot_yaw: float, robot_local_xy: tuple[float, float]
) -> tuple[float, float]:
    return offset_from_yaw(rx, ry, robot_yaw, robot_local_xy[0], robot_local_xy[1])


def _swept_ridgeback_obb(start: tuple[float, float], end: tuple[float, float]) -> OBB:
    """Conservative rectangular footprint for one straight motion segment."""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    distance = math.hypot(dx, dy)
    return make_obb(
        (start[0] + end[0]) * 0.5,
        (start[1] + end[1]) * 0.5,
        BBox(distance * 0.5 + RIDGEBACK_BBOX.half_w, RIDGEBACK_CORRIDOR_HALF_WIDTH),
        math.atan2(dy, dx),
    )


def _make_ridgeback_group(rx: float, ry: float, robot_yaw: float) -> list[tuple[str, OBB]]:
    waiting = _ridgeback_world_xy(rx, ry, robot_yaw, RIDGEBACK_WAITING_ROBOT_LOCAL)
    records: list[tuple[str, OBB]] = [
        ("ridgeback_waiting", make_obb(*waiting, RIDGEBACK_BBOX, robot_yaw)),
    ]
    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        staging_local = (
            RIDGEBACK_STAGING_ROBOT_LOCAL[0],
            side_sign * RIDGEBACK_STAGING_ROBOT_LOCAL[1],
        )
        delivery_local = (
            RIDGEBACK_DELIVERY_ROBOT_LOCAL[0],
            side_sign * RIDGEBACK_DELIVERY_ROBOT_LOCAL[1],
        )
        staging = _ridgeback_world_xy(rx, ry, robot_yaw, staging_local)
        delivery = _ridgeback_world_xy(rx, ry, robot_yaw, delivery_local)
        records.extend(
            [
                (f"ridgeback_staging_{side_name}", make_obb(*staging, RIDGEBACK_BBOX, robot_yaw)),
                (f"ridgeback_delivery_{side_name}", make_obb(*delivery, RIDGEBACK_BBOX, robot_yaw)),
                (f"ridgeback_corridor_waiting_{side_name}", _swept_ridgeback_obb(waiting, staging)),
                (f"ridgeback_corridor_delivery_{side_name}", _swept_ridgeback_obb(staging, delivery)),
            ]
        )
    return records


def _is_moving_ridgeback_geometry(name: str) -> bool:
    """Return whether an OBB is one pose/corridor of the mobile assistant."""
    return name == "ridgeback_waiting" or name.startswith(
        ("ridgeback_staging_", "ridgeback_delivery_", "ridgeback_corridor_")
    )


def _make_static_cluster_group(
    rx: float,
    ry: float,
    robot_yaw: float,
    members: tuple[StaticClusterMember, ...],
) -> list[tuple[str, OBB]]:
    records: list[tuple[str, OBB]] = []
    for member in members:
        wx, wy = offset_from_yaw(
            rx,
            ry,
            robot_yaw,
            member.robot_local_xy[0],
            member.robot_local_xy[1],
        )
        records.append(
            (
                member.asset_name,
                make_obb(wx, wy, member.bbox, robot_yaw + member.yaw_offset),
            )
        )
    return records

def _make_table_group_from_robot(
    rx: float,
    ry: float,
    robot_yaw: float,
    include_ridgeback: bool = True,
    static_cluster_members: tuple[StaticClusterMember, ...] = (),
) -> tuple[list[tuple[str, OBB]], tuple[float, float], float]:
    """Build table/robot OBBs using the robot as the placement anchor."""
    table_yaw = robot_yaw - math.pi / 2
    dx, dy = offset_from_yaw(
        rx,
        ry,
        table_yaw,
        -ROBOT_ORBIT_OFFSET[0],
        -ROBOT_ORBIT_OFFSET[1],
    )
    table_obbs = [
        ("packing_table", make_obb(dx, dy, DESK_BBOX, table_yaw)),
        ("robot", make_obb(rx, ry, ROBOT_BBOX, robot_yaw)),
    ]
    if include_ridgeback:
        table_obbs.extend(_make_ridgeback_group(rx, ry, robot_yaw))
    table_obbs.extend(
        _make_static_cluster_group(rx, ry, robot_yaw, static_cluster_members)
    )
    return table_obbs, (dx, dy), table_yaw


def _make_table_group_from_table(
    dx: float,
    dy: float,
    table_yaw: float,
) -> list[tuple[str, OBB]]:
    """Build the complete group from an authored fixed table transform."""
    rx, ry = offset_from_yaw(
        dx,
        dy,
        table_yaw,
        ROBOT_ORBIT_OFFSET[0],
        ROBOT_ORBIT_OFFSET[1],
    )
    robot_yaw = table_yaw + math.pi / 2
    table_obbs = [
        ("packing_table", make_obb(dx, dy, DESK_BBOX, table_yaw)),
        ("robot", make_obb(rx, ry, ROBOT_BBOX, robot_yaw)),
    ]
    table_obbs.extend(_make_ridgeback_group(rx, ry, robot_yaw))
    return table_obbs


def _quat_wxyz_yaw(quat: torch.Tensor) -> float:
    """Return Z-axis yaw from one Isaac Lab ``(w, x, y, z)`` quaternion."""
    w, x, y, z = (float(value) for value in quat)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _sample_wall_facing_robot_pose(rng: random.Random) -> tuple[float, float, float, RobotFacingLayout]:
    """Sample robot placement and aim it at a random point on a wall."""
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


def _obb_inside_table_group_bounds(box: OBB) -> bool:
    """Check the stricter robot/table usable area, including virtual open-side limits."""
    for wx, wy in obb_corners(*box):
        if wx < TABLE_GROUP_X_MIN or wx > TABLE_GROUP_X_MAX:
            return False
        if wy < TABLE_GROUP_Y_MIN or wy > TABLE_GROUP_Y_MAX:
            return False
    return True


def _debug_table_group(env_id: int, table_obbs: list[tuple[str, OBB]], placed: List[OBB], placed_names: list[str]):
    """Print table group OBBs and overlap diagnostics."""
    for name, box in table_obbs:
        _print_obb_debug(name, env_id, box)

    for i, (a_name, a_box) in enumerate(table_obbs):
        for b_name, b_box in table_obbs[i + 1:]:
            if _is_moving_ridgeback_geometry(a_name) and _is_moving_ridgeback_geometry(
                b_name
            ):
                continue
            if {a_name, b_name} == {"packing_table", "robot"}:
                print(
                    f"[PLACEMENT_DEBUG] env={env_id} overlap_check "
                    f"a={a_name} b={b_name} overlaps=skipped_regular_pickplace_transform",
                    flush=True,
                )
                continue
            if a_name.startswith(_RIDGEBACK_GEOMETRY_PREFIX) or b_name.startswith(
                _RIDGEBACK_GEOMETRY_PREFIX
            ):
                margin = RIDGEBACK_GROUP_MARGIN
            else:
                margin = ROBOT_TABLE_MARGIN if {a_name, b_name} == {"packing_table", "robot"} else OBB_PLACEMENT_MARGIN
            overlaps = obb_overlap(a_box, b_box, margin=margin)
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


def _validate_table_group(
    table_obbs: list[tuple[str, OBB]],
    placed: List[OBB],
    placed_names: list[str],
    spawn_boundaries: list[tuple[str, OBB]],
    allowed_protected_overlap_names: frozenset[str] = frozenset(),
) -> tuple[bool, list[str]]:
    """Validate table-group room bounds and overlaps."""
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
            if (
                {a_name, b_name} & allowed_protected_overlap_names
                and {a_name, b_name} & {"packing_table", "robot"}
            ):
                continue
            if _is_moving_ridgeback_geometry(a_name) and _is_moving_ridgeback_geometry(
                b_name
            ):
                continue
            if {a_name, b_name} == {"packing_table", "robot"}:
                continue
            if a_name.startswith(_RIDGEBACK_GEOMETRY_PREFIX) or b_name.startswith(
                _RIDGEBACK_GEOMETRY_PREFIX
            ):
                margin = RIDGEBACK_GROUP_MARGIN
            else:
                margin = ROBOT_TABLE_MARGIN if {a_name, b_name} == {"packing_table", "robot"} else OBB_PLACEMENT_MARGIN
            if obb_overlap(a_box, b_box, margin=margin):
                issues.append(
                    f"{a_name} overlaps {b_name} "
                    f"{a_name}_corners={_format_corners(a_box)} "
                    f"{b_name}_corners={_format_corners(b_box)}"
                )

    return len(issues) == 0, issues


def _place_table_group(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    all_placed: List[List[OBB]],
    all_placed_names: List[List[str]],
    desk_positions: torch.Tensor,
    desk_yaws: torch.Tensor,
    spawn_boundaries: List[List[tuple[str, OBB]]],
    rng: random.Random,
    randomize_position: bool = True,
    static_cluster_members: tuple[StaticClusterMember, ...] = (),
) -> tuple[list[list[tuple[str, OBB]]], list[str]]:
    M = len(env_ids)
    device = env.device
    env_origins = env.scene.env_origins
    group_placed_mask = torch.zeros(M, dtype=torch.bool, device=device)
    final_table_obbs: list[list[tuple[str, OBB]]] = [[] for _ in range(M)]
    selected_layout_names = ["none" for _ in range(M)]
    desk_asset = env.scene["packing_table"]
    robot_asset = env.scene["robot"]
    include_ridgeback = "ridgeback" in env.scene.keys()
    allowed_protected_overlap_names = frozenset(
        member.asset_name
        for member in static_cluster_members
        if member.allow_protected_overlap
    )

    for env_idx in range(M):
        success = False
        selected_obbs: list[tuple[str, OBB]] = []
        selected_layout_name = "none"

        if not randomize_position:
            env_id = _env_id_int(env_ids, env_idx)
            default_state = desk_asset.data.default_root_state[env_id]
            origin = env_origins[env_id]
            dx = float(default_state[0] - origin[0])
            dy = float(default_state[1] - origin[1])
            dyaw = _quat_wxyz_yaw(default_state[3:7])
            robot_default_state = robot_asset.data.default_root_state[env_id]
            rx = float(robot_default_state[0] - origin[0])
            ry = float(robot_default_state[1] - origin[1])
            robot_yaw = _quat_wxyz_yaw(robot_default_state[3:7])
            table_obbs = [
                ("packing_table", make_obb(dx, dy, DESK_BBOX, dyaw)),
                ("robot", make_obb(rx, ry, ROBOT_BBOX, robot_yaw)),
            ]
            if include_ridgeback:
                table_obbs.extend(_make_ridgeback_group(rx, ry, robot_yaw))
            table_obbs.extend(
                _make_static_cluster_group(
                    rx, ry, robot_yaw, static_cluster_members
                )
            )
            # Fixed teleoperation layouts intentionally bypass the conservative
            # room-boundary rejection below. Static logistics platforms retain
            # the protected G1/table overlap check unless their task explicitly
            # opts out (the close hospital Ridgeback arc does so).
            static_boxes = dict(table_obbs)
            for member in static_cluster_members:
                if member.allow_protected_overlap:
                    continue
                for protected_name in ("packing_table", "robot"):
                    if obb_overlap(
                        static_boxes[member.asset_name],
                        static_boxes[protected_name],
                        margin=RIDGEBACK_GROUP_MARGIN,
                    ):
                        raise RuntimeError(
                            "static cluster member overlaps fixed teleoperation "
                            f"{protected_name}: {member.asset_name}"
                        )
            # The fixed pose is an explicitly calibrated task authoring choice.
            # Do not reject it using the deliberately conservative randomized-
            # placement OBBs: the packing-table proxy slightly crosses a
            # segmented wall boundary even though the real mesh is valid.
            # Adding the group to ``all_placed`` below still reserves the
            # workspace before wall furniture is randomized.
            desk_positions[env_idx] = torch.tensor([dx, dy, FLOOR_Z], device=device)
            desk_yaws[env_idx] = dyaw
            selected_obbs = table_obbs
            selected_layout_name = "fixed_teleop"
            success = True
        else:
            for _ in range(TABLE_GROUP_MAX_TRIES):
                rx, ry, robot_yaw, layout = _sample_wall_facing_robot_pose(rng)

                table_obbs, (dx, dy), dyaw = _make_table_group_from_robot(
                    rx,
                    ry,
                    robot_yaw,
                    include_ridgeback=include_ridgeback,
                    static_cluster_members=static_cluster_members,
                )
                valid, _ = _validate_table_group(
                    table_obbs,
                    all_placed[env_idx],
                    all_placed_names[env_idx],
                    spawn_boundaries[env_idx],
                    allowed_protected_overlap_names,
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

        if not success and randomize_position:
            # Fallback
            fallback_issues: list[str] = []
            for _ in range(TABLE_GROUP_MAX_TRIES):
                rx, ry, robot_yaw, layout = _sample_wall_facing_robot_pose(rng)
                table_obbs, (dx, dy), dyaw = _make_table_group_from_robot(
                    rx,
                    ry,
                    robot_yaw,
                    include_ridgeback=include_ridgeback,
                    static_cluster_members=static_cluster_members,
                )
                valid, fallback_issues = _validate_table_group(
                    table_obbs,
                    all_placed[env_idx],
                    all_placed_names[env_idx],
                    spawn_boundaries[env_idx],
                    allowed_protected_overlap_names,
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
            selected_layout_names[env_idx] = selected_layout_name
            final_table_obbs[env_idx] = selected_obbs
            all_placed[env_idx].extend([box for _, box in selected_obbs])
            all_placed_names[env_idx].extend([name for name, _ in selected_obbs])
            group_placed_mask[env_idx] = True
            print(
                f"[PLACEMENT_DEBUG] env={_env_id_int(env_ids, env_idx)} "
                f"table_group_layout={selected_layout_name}",
                flush=True,
            )

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

    # Get default Z coordinates dynamically from env assets
    desk_default_z = (desk_asset.data.default_root_state[0, 2] - env_origins[0, 2]).item()
    desk_positions[:, 2] = desk_default_z

    robot_default_z = (robot_asset.data.default_root_state[0, 2] - env_origins[0, 2]).item()

    # Build batched positions for robot.  A fixed-table reset must preserve
    # each task's authored robot/table calibration; not every teleoperation
    # task uses the generic randomized-table orbit transform.
    if randomize_position:
        robot_pos = offset_from_yaw_batched(
            desk_positions, desk_yaws,
            ROBOT_ORBIT_OFFSET[0], ROBOT_ORBIT_OFFSET[1], robot_default_z,
        )
        robot_yaw = desk_yaws + math.pi / 2
    else:
        robot_defaults = robot_asset.data.default_root_state[env_ids]
        robot_pos = robot_defaults[:, 0:3] - env_origins[env_ids]
        robot_yaw = torch.tensor(
            [_quat_wxyz_yaw(state[3:7]) for state in robot_defaults],
            device=device,
        )

    # Handle failures by despawning
    invalid_mask = ~group_placed_mask
    if torch.any(invalid_mask):
        desk_positions[invalid_mask, 0:2] = 0.0
        desk_positions[invalid_mask, 2] = DESPAWN_Z
        robot_pos[invalid_mask, 0:2] = 0.0
        robot_pos[invalid_mask, 2] = DESPAWN_Z
        robot_yaw[invalid_mask] = 0.0

    desk_state = build_root_state(desk_positions, desk_yaws, env_origins, env_ids, desk_asset.data.default_root_state)
    _write_root_pose_to_sim(desk_asset, desk_state, env_ids)
    actual_desk_pos = desk_asset.data.root_pos_w[env_ids] - env_origins[env_ids]
    for env_idx in range(M):
        if not final_table_obbs[env_idx]:
            continue
        pos = actual_desk_pos[env_idx]
        print(
            f"[PLACEMENT_DEBUG] env={_env_id_int(env_ids, env_idx)} "
            f"packing_table_actual_root_pos=({pos[0].item():+.3f},{pos[1].item():+.3f},{pos[2].item():+.3f})",
            flush=True,
        )

    robot_state = build_root_state(robot_pos, robot_yaw, env_origins, env_ids, robot_asset.data.default_root_state)
    robot_asset.write_root_state_to_sim(robot_state, env_ids=env_ids)
    robot_asset.write_joint_state_to_sim(
        robot_asset.data.default_joint_pos[env_ids],
        robot_asset.data.default_joint_vel[env_ids],
        env_ids=env_ids,
    )
    robot_asset.set_joint_position_target(robot_asset.data.default_joint_pos[env_ids], env_ids=env_ids)
    robot_asset.set_joint_velocity_target(robot_asset.data.default_joint_vel[env_ids], env_ids=env_ids)

    # The articulation root is the randomized waiting pose.  Its -90-degree
    # yaw offset keeps the existing planar joint convention aligned with the
    # G1-local right/forward axes for every room orientation.
    if "ridgeback" in env.scene.keys():
        ridgeback = env.scene["ridgeback"]
        waiting_pos = offset_from_yaw_batched(
            robot_pos,
            robot_yaw,
            RIDGEBACK_WAITING_ROBOT_LOCAL[0],
            RIDGEBACK_WAITING_ROBOT_LOCAL[1],
            (ridgeback.data.default_root_state[0, 2] - env_origins[0, 2]).item(),
        )
        ridgeback_root_yaw = robot_yaw + RIDGEBACK_ROOT_YAW_OFFSET
        ridgeback_state = build_root_state(
            waiting_pos,
            ridgeback_root_yaw,
            env_origins,
            env_ids,
            ridgeback.data.default_root_state,
        )
        ridgeback.write_root_state_to_sim(ridgeback_state, env_ids=env_ids)

    for member in static_cluster_members:
        asset = env.scene[member.asset_name]
        member_pos = offset_from_yaw_batched(
            robot_pos,
            robot_yaw,
            member.robot_local_xy[0],
            member.robot_local_xy[1],
            (asset.data.default_root_state[0, 2] - env_origins[0, 2]).item(),
        )
        member_yaw = robot_yaw + member.yaw_offset
        member_state = build_root_state(
            member_pos,
            member_yaw,
            env_origins,
            env_ids,
            asset.data.default_root_state,
        )
        _write_root_pose_to_sim(asset, member_state, env_ids)

    return final_table_obbs, selected_layout_names


# ======================================================================
# Phase 3: Tabletop objects — OBB collision on desk surface
# ======================================================================


def _obb_inside_tabletop(
    candidate: OBB,
    spawn_region: TabletopSpawnRegion | None = None,
) -> bool:
    x_min = spawn_region.x_min if spawn_region is not None else DESK_LOCAL_X_MIN
    x_max = spawn_region.x_max if spawn_region is not None else DESK_LOCAL_X_MAX
    y_min = spawn_region.y_min if spawn_region is not None else DESK_LOCAL_Y_MIN
    y_max = spawn_region.y_max if spawn_region is not None else DESK_LOCAL_Y_MAX
    return all(
        x_min <= x <= x_max and y_min <= y <= y_max
        for x, y in obb_corners(*candidate)
    )


def _asset_room_z(env: ManagerBasedEnv, name: str, env_id: int) -> float:
    asset = env.scene[name]
    return float(asset.data.default_root_state[env_id, 2] - env.scene.env_origins[env_id, 2])


def _ridgeback_joint_targets() -> dict[str, tuple[float, float, float]]:
    targets: dict[str, tuple[float, float, float]] = {"waiting": (0.0, 0.0, 0.0)}
    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        targets[f"staging_{side_name}"] = (
            -side_sign * RIDGEBACK_STAGING_ROBOT_LOCAL[1],
            RIDGEBACK_STAGING_ROBOT_LOCAL[0] - RIDGEBACK_WAITING_ROBOT_LOCAL[0],
            RIDGEBACK_PLANAR_YAW,
        )
        targets[f"delivery_{side_name}"] = (
            -side_sign * RIDGEBACK_DELIVERY_ROBOT_LOCAL[1],
            RIDGEBACK_DELIVERY_ROBOT_LOCAL[0] - RIDGEBACK_WAITING_ROBOT_LOCAL[0],
            RIDGEBACK_PLANAR_YAW,
        )
    return targets


def _store_layout_state(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    desk_positions: torch.Tensor,
    desk_yaws: torch.Tensor,
    selected_layout_names: list[str],
    tabletop_placements: list[dict[str, TabletopPlacement]],
    static_cluster_members: tuple[StaticClusterMember, ...] = (),
) -> None:
    states = getattr(env, "_room_layout_state", {})
    origins = env.scene.env_origins
    robot = env.scene["robot"]
    robot_room_z = float(robot.data.default_root_state[0, 2] - origins[0, 2])
    ridgeback_room_z = FLOOR_Z
    if "ridgeback" in env.scene.keys():
        ridgeback = env.scene["ridgeback"]
        ridgeback_room_z = float(ridgeback.data.default_root_state[0, 2] - origins[0, 2])

    for env_idx in range(len(env_ids)):
        env_id = _env_id_int(env_ids, env_idx)
        origin = origins[env_id]
        table_x = float(desk_positions[env_idx, 0] + origin[0])
        table_y = float(desk_positions[env_idx, 1] + origin[1])
        table_z = float(desk_positions[env_idx, 2] + origin[2])
        table_yaw = float(desk_yaws[env_idx])
        # Read back the actual robot pose instead of reconstructing it from
        # the generic randomized-table offset.  Fixed teleoperation tasks can
        # have their own authored robot/table calibration.
        robot_x = float(robot.data.root_pos_w[env_id, 0])
        robot_y = float(robot.data.root_pos_w[env_id, 1])
        robot_yaw = _quat_wxyz_yaw(robot.data.root_quat_w[env_id])
        waiting_x, waiting_y = _ridgeback_world_xy(
            robot_x, robot_y, robot_yaw, RIDGEBACK_WAITING_ROBOT_LOCAL
        )
        target = tabletop_placements[env_idx].get("object")
        target_local_pose = (
            target.local_pose if target is not None else (math.nan, math.nan, math.nan, math.nan)
        )
        static_cluster_poses = {}
        for member in static_cluster_members:
            asset = env.scene[member.asset_name]
            static_cluster_poses[member.asset_name] = Pose2D(
                position=tuple(
                    float(value) for value in asset.data.root_pos_w[env_id, :3]
                ),
                yaw=_quat_wxyz_yaw(asset.data.root_quat_w[env_id]),
            )
        states[env_id] = RoomLayoutState(
            env_id=env_id,
            robot_pose=Pose2D(
                position=(robot_x, robot_y, robot_room_z + float(origin[2])),
                yaw=robot_yaw,
            ),
            packing_table_pose=Pose2D(position=(table_x, table_y, table_z), yaw=table_yaw),
            target_object_local_pose=target_local_pose,
            ridgeback_waiting_pose=Pose2D(
                position=(waiting_x, waiting_y, ridgeback_room_z + float(origin[2])),
                yaw=robot_yaw,
            ),
            selected_wall_facing_layout=selected_layout_names[env_idx],
            tabletop_placements=dict(tabletop_placements[env_idx]),
            ridgeback_joint_targets=_ridgeback_joint_targets(),
            static_cluster_poses=static_cluster_poses,
        )
    env._room_layout_state = states


def reset_target_on_current_table(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    asset_name: str = "object",
    table_prop_meta_overrides: dict[str, TablePropMeta] | None = None,
    tabletop_spawn_region: TabletopSpawnRegion | None = None,
    tabletop_object_margin: float = DESK_OBJECT_MARGIN,
) -> None:
    """Respawn a dynamic target collision-free on its current randomized table."""
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    states = getattr(env, "_room_layout_state", None)
    if not states:
        raise RuntimeError("room layout state is unavailable; run the full randomized reset first")
    table_prop_meta = dict(TABLE_PROP_META)
    if table_prop_meta_overrides:
        table_prop_meta.update(table_prop_meta_overrides)
    if asset_name not in table_prop_meta:
        raise KeyError(f"no tabletop metadata registered for {asset_name!r}")

    rng = _room_rng(env)
    meta = table_prop_meta[asset_name]
    if not meta.dynamic:
        raise ValueError(f"tabletop respawn requires a dynamic object, got {asset_name!r}")
    asset = env.scene[asset_name]
    env_origins = env.scene.env_origins

    for env_id_tensor in env_ids:
        env_id = int(env_id_tensor.item())
        layout = states.get(env_id)
        if layout is None:
            raise RuntimeError(f"room layout state is unavailable for env {env_id}")
        occupied = [
            make_obb(
                placement.local_pose[0],
                placement.local_pose[1],
                placement.bbox,
                placement.local_pose[3],
            )
            for name, placement in layout.tabletop_placements.items()
            if name != asset_name
        ]
        occupied.extend(
            make_obb(area.center[0], area.center[1], area.bbox, area.yaw)
            for area in TABLE_RESERVED_AREAS
        )

        selected: tuple[float, float, float] | None = None
        for _ in range(300):
            x_min = (
                tabletop_spawn_region.x_min
                if tabletop_spawn_region is not None
                else DESK_LOCAL_X_MIN
            )
            x_max = (
                tabletop_spawn_region.x_max
                if tabletop_spawn_region is not None
                else DESK_LOCAL_X_MAX
            )
            y_min = (
                tabletop_spawn_region.y_min
                if tabletop_spawn_region is not None
                else DESK_LOCAL_Y_MIN
            )
            y_max = (
                tabletop_spawn_region.y_max
                if tabletop_spawn_region is not None
                else DESK_LOCAL_Y_MAX
            )
            lx = rng.uniform(x_min, x_max)
            ly = rng.uniform(y_min, y_max)
            local_yaw = rng.uniform(0.0, 2.0 * math.pi)
            candidate = make_obb(lx, ly, meta.bbox, local_yaw)
            if not _obb_inside_tabletop(candidate, tabletop_spawn_region):
                continue
            if obb_overlap_any(candidate, occupied, margin=tabletop_object_margin):
                continue
            selected = (lx, ly, local_yaw)
            break
        if selected is None:
            raise RuntimeError(f"no collision-free tabletop respawn pose for {asset_name!r} in env {env_id}")

        previous = layout.tabletop_placements.get(asset_name)
        local_z = (
            previous.local_pose[2]
            if previous is not None
            else _asset_room_z(env, asset_name, env_id)
            - (layout.packing_table_pose.position[2] - float(env_origins[env_id, 2]))
        )
        lx, ly, local_yaw = selected
        world_x, world_y = offset_from_yaw(
            layout.packing_table_pose.position[0],
            layout.packing_table_pose.position[1],
            layout.packing_table_pose.yaw,
            lx,
            ly,
        )
        world_z = layout.packing_table_pose.position[2] + local_z
        room_pos = torch.tensor(
            [[
                world_x - float(env_origins[env_id, 0]),
                world_y - float(env_origins[env_id, 1]),
                world_z - float(env_origins[env_id, 2]),
            ]],
            device=env.device,
        )
        world_yaw = torch.tensor(
            [layout.packing_table_pose.yaw + local_yaw], device=env.device
        )
        eid = env_ids.new_tensor([env_id])
        root_state = build_root_state(
            room_pos,
            world_yaw,
            env_origins,
            eid,
            asset.data.default_root_state,
            base_orientation_wxyz=meta.base_orientation_wxyz,
        )
        root_state[:, 7:13] = 0.0
        asset.write_root_state_to_sim(root_state, env_ids=eid)

        placement = TabletopPlacement(
            name=asset_name,
            local_pose=(lx, ly, local_z, local_yaw),
            bbox=meta.bbox,
        )
        layout.tabletop_placements[asset_name] = placement
        if asset_name == "object":
            layout.target_object_local_pose = placement.local_pose

def _place_desk_objects(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    table_prop_names: list[str],
    desk_pos: torch.Tensor,
    desk_yaw_rad: torch.Tensor,
    spawn_boundaries: List[List[tuple[str, OBB]]],
    min_table_objects: int = 0,
    rng: random.Random | None = None,
    table_prop_meta: dict[str, TablePropMeta] | None = None,
    tabletop_spawn_region: TabletopSpawnRegion | None = None,
    tabletop_object_margin: float = DESK_OBJECT_MARGIN,
) -> tuple[list[list[tuple[str, OBB]]], list[dict[str, TabletopPlacement]]]:
    M = len(env_ids)
    debug_obbs: list[list[tuple[str, OBB]]] = [[] for _ in range(M)]
    placement_records: list[dict[str, TabletopPlacement]] = [{} for _ in range(M)]
    if not table_prop_names:
        return debug_obbs, placement_records

    device = env.device
    env_origins = env.scene.env_origins
    rng = rng or _room_rng(env)
    if table_prop_meta is None:
        table_prop_meta = TABLE_PROP_META

    for env_idx in range(M):
        if desk_pos[env_idx, 2].item() <= DESPAWN_Z * 0.5:
            # Table is despawned, so despawn the object and distractors too
            for name in table_prop_names:
                if name in env.scene.keys():
                    asset = env.scene[name]
                    pos = torch.tensor([0.0, 0.0, DESPAWN_Z], device=device).unsqueeze(0)
                    yaw = torch.tensor([0.0], device=device)
                    eid = env_ids[env_idx:env_idx+1]
                    root_state = build_root_state(pos, yaw, env_origins, eid, asset.data.default_root_state)
                    _write_tabletop_root_state(asset, name, root_state, eid)
            print(
                f"[PLACEMENT_ERROR] env={_env_id_int(env_ids, env_idx)} "
                f"tabletop_objects skipped table_group_not_placed=true",
                flush=True,
            )
            continue

        desk_placed: List[OBB] = []
        for area in TABLE_RESERVED_AREAS:
            desk_placed.append(make_obb(area.center[0], area.center[1], area.bbox, area.yaw))
            wx, wy = offset_from_yaw(
                desk_pos[env_idx, 0].item(),
                desk_pos[env_idx, 1].item(),
                desk_yaw_rad[env_idx].item(),
                area.center[0],
                area.center[1],
            )
            debug_obbs[env_idx].append(
                (area.name, make_obb(wx, wy, area.bbox, desk_yaw_rad[env_idx].item() + area.yaw))
            )

        # The target is mandatory, but its pose is sampled on the current table
        # just like every other movable tabletop object.
        if "object" in table_prop_names and "object" in env.scene.keys():
            asset = env.scene["object"]
            meta = table_prop_meta["object"]
            placed = False
            last_reject_reason = "no_valid_candidate"
            for _ in range(100):
                x_min = (
                    tabletop_spawn_region.x_min
                    if tabletop_spawn_region is not None
                    else DESK_LOCAL_X_MIN
                )
                x_max = (
                    tabletop_spawn_region.x_max
                    if tabletop_spawn_region is not None
                    else DESK_LOCAL_X_MAX
                )
                y_min = (
                    tabletop_spawn_region.y_min
                    if tabletop_spawn_region is not None
                    else DESK_LOCAL_Y_MIN
                )
                y_max = (
                    tabletop_spawn_region.y_max
                    if tabletop_spawn_region is not None
                    else DESK_LOCAL_Y_MAX
                )
                lx = rng.uniform(x_min, x_max)
                ly = rng.uniform(y_min, y_max)
                obj_yaw = rng.uniform(0.0, 2.0 * math.pi)
                candidate = make_obb(lx, ly, meta.bbox, obj_yaw)
                if not _obb_inside_tabletop(candidate, tabletop_spawn_region):
                    last_reject_reason = "object outside_tabletop"
                    continue
                if obb_overlap_any(candidate, desk_placed, margin=tabletop_object_margin):
                    last_reject_reason = "object overlaps_tabletop_object"
                    continue
                wx, wy = offset_from_yaw(
                    desk_pos[env_idx, 0].item(),
                    desk_pos[env_idx, 1].item(),
                    desk_yaw_rad[env_idx].item(),
                    lx,
                    ly,
                )
                world_yaw = desk_yaw_rad[env_idx].item() + obj_yaw
                world_box = make_obb(wx, wy, meta.bbox, world_yaw)
                spawn_issue = _outside_spawn_region_issue("object", world_box, spawn_boundaries[env_idx])
                if spawn_issue is not None:
                    last_reject_reason = spawn_issue
                    continue
                object_z = _asset_room_z(env, "object", _env_id_int(env_ids, env_idx))
                pos = torch.tensor([wx, wy, object_z], device=device).unsqueeze(0)
                yaw = torch.tensor([world_yaw], device=device)
                eid = env_ids[env_idx:env_idx+1]
                root_state = build_root_state(
                    pos,
                    yaw,
                    env_origins,
                    eid,
                    asset.data.default_root_state,
                    base_orientation_wxyz=meta.base_orientation_wxyz,
                )
                asset.write_root_state_to_sim(root_state, env_ids=eid)
                desk_placed.append(candidate)
                debug_obbs[env_idx].append(("object", world_box))
                placement_records[env_idx]["object"] = TabletopPlacement(
                    name="object",
                    local_pose=(
                        lx,
                        ly,
                        object_z - desk_pos[env_idx, 2].item(),
                        obj_yaw,
                    ),
                    bbox=meta.bbox,
                )
                placed = True
                break
            if not placed:
                pos = torch.tensor([0.0, 0.0, DESPAWN_Z], device=device).unsqueeze(0)
                yaw = torch.tensor([0.0], device=device)
                eid = env_ids[env_idx:env_idx+1]
                root_state = build_root_state(pos, yaw, env_origins, eid, asset.data.default_root_state)
                asset.write_root_state_to_sim(root_state, env_ids=eid)
                print(
                    f"[PLACEMENT_ERROR] env={_env_id_int(env_ids, env_idx)} "
                    f"object=object tabletop_placement_failed despawning=true "
                    f"last_reject_reason={last_reject_reason}",
                    flush=True,
                )

        extra_names = [name for name in table_prop_names if name != "object"]
        if not extra_names:
            continue

        # How many extra tabletop props in this env.
        if not 0 <= min_table_objects <= len(extra_names):
            raise ValueError(
                f"min_table_objects must be within [0, {len(extra_names)}], got {min_table_objects}"
            )
        count = rng.randint(min_table_objects, len(extra_names))
        visible_extra_names = set(extra_names[:count])
        visible_extra_names.update(
            name for name in extra_names if table_prop_meta[name].mandatory
        )

        if "desk_lamp" in visible_extra_names and "desk_lamp" in env.scene.keys():
            asset = env.scene["desk_lamp"]
            meta = table_prop_meta["desk_lamp"]
            placed = False
            last_reject_reason = "no_valid_candidate"
            for _ in range(100):
                if tabletop_spawn_region is not None:
                    lamp_x_range = (
                        tabletop_spawn_region.x_min,
                        tabletop_spawn_region.x_max,
                    )
                    lamp_y_range = (
                        tabletop_spawn_region.y_min,
                        tabletop_spawn_region.y_max,
                    )
                else:
                    lamp_x_range = DESK_LAMP_LOCAL_X_RANGE
                    lamp_y_range = DESK_LAMP_LOCAL_Y_RANGE
                lx = rng.uniform(*lamp_x_range)
                ly = rng.uniform(*lamp_y_range)
                obj_yaw = DESK_LAMP_LOCAL_YAW

                candidate = make_obb(lx, ly, meta.bbox, obj_yaw)
                if not _obb_inside_tabletop(candidate, tabletop_spawn_region):
                    last_reject_reason = "desk_lamp outside_tabletop"
                    continue
                if obb_overlap_any(candidate, desk_placed, margin=tabletop_object_margin):
                    last_reject_reason = "desk_lamp overlaps_tabletop_object"
                    continue

                wx, wy = offset_from_yaw(
                    desk_pos[env_idx, 0].item(),
                    desk_pos[env_idx, 1].item(),
                    desk_yaw_rad[env_idx].item(),
                    lx, ly,
                )
                world_yaw = desk_yaw_rad[env_idx].item() + obj_yaw
                world_box = make_obb(wx, wy, meta.bbox, world_yaw)
                spawn_issue = _outside_spawn_region_issue("desk_lamp", world_box, spawn_boundaries[env_idx])
                if spawn_issue is not None:
                    last_reject_reason = spawn_issue
                    continue

                desk_placed.append(candidate)

                object_z = _asset_room_z(env, "desk_lamp", _env_id_int(env_ids, env_idx))
                pos = torch.tensor([wx, wy, object_z], device=device).unsqueeze(0)
                yaw = torch.tensor([world_yaw], device=device)
                eid = env_ids[env_idx:env_idx+1]

                root_state = build_root_state(pos, yaw, env_origins, eid, asset.data.default_root_state)
                _write_tabletop_root_state(asset, "desk_lamp", root_state, eid)
                debug_obbs[env_idx].append(
                    ("desk_lamp", world_box)
                )
                placement_records[env_idx]["desk_lamp"] = TabletopPlacement(
                    name="desk_lamp",
                    local_pose=(
                        lx,
                        ly,
                        object_z - desk_pos[env_idx, 2].item(),
                        obj_yaw,
                    ),
                    bbox=meta.bbox,
                )
                placed = True
                break

            if not placed:
                pos = torch.tensor([0.0, 0.0, DESPAWN_Z], device=device).unsqueeze(0)
                yaw = torch.tensor([0.0], device=device)
                eid = env_ids[env_idx:env_idx+1]
                root_state = build_root_state(pos, yaw, env_origins, eid, asset.data.default_root_state)
                _write_tabletop_root_state(asset, "desk_lamp", root_state, eid)
                print(
                    f"[PLACEMENT_ERROR] env={_env_id_int(env_ids, env_idx)} "
                    f"object=desk_lamp tabletop_placement_failed despawning=true "
                    f"last_reject_reason={last_reject_reason}",
                    flush=True,
                )
        elif "desk_lamp" in extra_names and "desk_lamp" in env.scene.keys():
            asset = env.scene["desk_lamp"]
            pos = torch.tensor([0.0, 0.0, DESPAWN_Z], device=device).unsqueeze(0)
            yaw = torch.tensor([0.0], device=device)
            eid = env_ids[env_idx:env_idx+1]
            root_state = build_root_state(pos, yaw, env_origins, eid, asset.data.default_root_state)
            _write_tabletop_root_state(asset, "desk_lamp", root_state, eid)

        for name in extra_names:
            if name == "desk_lamp":
                continue
            if name not in env.scene.keys():
                continue
            asset = env.scene[name]
            meta = table_prop_meta[name]
            visible = name in visible_extra_names

            if visible:
                placed = False
                last_reject_reason = "no_valid_candidate"
                for _ in range(300):
                    if tabletop_spawn_region is not None:
                        local_x_min = tabletop_spawn_region.x_min
                        local_x_max = tabletop_spawn_region.x_max
                        local_y_min = tabletop_spawn_region.y_min
                        local_y_max = tabletop_spawn_region.y_max
                    else:
                        local_x_min = TABLETOP_CUBE_LOCAL_X_MIN if name in TABLETOP_CUBE_PROP_NAMES else DESK_LOCAL_X_MIN
                        local_x_max = TABLETOP_CUBE_LOCAL_X_MAX if name in TABLETOP_CUBE_PROP_NAMES else DESK_LOCAL_X_MAX
                        local_y_min = DESK_LOCAL_Y_MIN
                        local_y_max = DESK_LOCAL_Y_MAX
                    lx = rng.uniform(local_x_min, local_x_max)
                    ly = rng.uniform(local_y_min, local_y_max)
                    obj_yaw = rng.uniform(0, 2 * math.pi)

                    candidate = make_obb(lx, ly, meta.bbox, obj_yaw)
                    if not _obb_inside_tabletop(candidate, tabletop_spawn_region):
                        last_reject_reason = f"{name} outside_tabletop"
                        continue
                    if obb_overlap_any(candidate, desk_placed, margin=tabletop_object_margin):
                        last_reject_reason = f"{name} overlaps_tabletop_object"
                        continue

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

                    object_z = _asset_room_z(env, name, _env_id_int(env_ids, env_idx))
                    pos = torch.tensor([wx, wy, object_z], device=device).unsqueeze(0)
                    yaw = torch.tensor([world_yaw], device=device)
                    eid = env_ids[env_idx:env_idx+1]

                    root_state = build_root_state(
                        pos,
                        yaw,
                        env_origins,
                        eid,
                        asset.data.default_root_state,
                        base_orientation_wxyz=meta.base_orientation_wxyz,
                    )
                    _write_tabletop_root_state(asset, name, root_state, eid)
                    debug_obbs[env_idx].append(
                        (name, world_box)
                    )
                    placement_records[env_idx][name] = TabletopPlacement(
                        name=name,
                        local_pose=(
                            lx,
                            ly,
                            object_z - desk_pos[env_idx, 2].item(),
                            obj_yaw,
                        ),
                        bbox=meta.bbox,
                    )
                    placed = True
                    break

                if not placed:
                    pos = torch.tensor([0.0, 0.0, DESPAWN_Z], device=device).unsqueeze(0)
                    yaw = torch.tensor([0.0], device=device)
                    eid = env_ids[env_idx:env_idx+1]
                    root_state = build_root_state(pos, yaw, env_origins, eid, asset.data.default_root_state)
                    _write_tabletop_root_state(asset, name, root_state, eid)
                    print(
                        f"[PLACEMENT_ERROR] env={_env_id_int(env_ids, env_idx)} "
                        f"object={name} tabletop_placement_failed despawning=true "
                        f"last_reject_reason={last_reject_reason}",
                        flush=True,
                    )
            else:
                pos = torch.tensor([0.0, 0.0, DESPAWN_Z], device=device).unsqueeze(0)
                yaw = torch.tensor([0.0], device=device)
                eid = env_ids[env_idx:env_idx+1]
                root_state = build_root_state(pos, yaw, env_origins, eid, asset.data.default_root_state)
                _write_tabletop_root_state(asset, name, root_state, eid)

    return debug_obbs, placement_records
