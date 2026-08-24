#!/usr/bin/env python3
"""Deterministic, Isaac-free regression tests for the production room planner.

The production constants and OBB helpers are loaded directly from
``tasks/utils/room_randomizer`` without importing Isaac Lab.  This keeps the
1,000-seed geometry test fast enough for ordinary CI while preventing the lab
copy of the planner constants from drifting away from runtime behavior.

Usage:
    python isaac-projects/room_randomizer_lab/test_placement.py
"""

from __future__ import annotations

import importlib.util
import math
import random
import sys
import types
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
PRODUCTION_DIR = PROJECT_ROOT / "tasks" / "utils" / "room_randomizer"
PACKAGE_NAME = "_production_room_randomizer_test"


def _load_production_module(name: str):
    package = sys.modules.get(PACKAGE_NAME)
    if package is None:
        package = types.ModuleType(PACKAGE_NAME)
        package.__path__ = [str(PRODUCTION_DIR)]
        sys.modules[PACKAGE_NAME] = package
    qualified_name = f"{PACKAGE_NAME}.{name}"
    spec = importlib.util.spec_from_file_location(
        qualified_name, PRODUCTION_DIR / f"{name}.py"
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load production module {name}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[qualified_name] = module
    spec.loader.exec_module(module)
    return module


C = _load_production_module("constants")
P = _load_production_module("placement_utils")

BBox = C.BBox
make_obb = P.make_obb
obb_corners = P.obb_corners
obb_inside_room = P.obb_inside_room
obb_overlap = P.obb_overlap
obb_overlap_any = P.obb_overlap_any
offset_from_yaw = P.offset_from_yaw

RIDGE_PREFIX = "ridgeback_"
TABLE_OBJECT_NAMES = (
    "object",
    "medical_bottle_a",
    "medical_bottle_b",
    "medical_bottle_c",
)
MEDICINE_BOTTLE_TABLE_OBJECT_NAMES = (
    "object",
    "medical_bottle_a",
    "medical_bottle_b",
    "medical_bottle_c",
)
MEDICINE_BOTTLE_TABLE_PROP_META_OVERRIDES = {
    "object": C.TablePropMeta(
        bbox=BBox(half_w=0.035, half_d=0.035), dynamic=True, mandatory=True
    ),
    "medical_bottle_a": C.TablePropMeta(
        bbox=BBox(half_w=0.024491, half_d=0.024491),
        dynamic=True,
        mandatory=True,
    ),
    "medical_bottle_b": C.TablePropMeta(
        bbox=BBox(half_w=0.024491, half_d=0.024491),
        dynamic=True,
        mandatory=True,
    ),
    "medical_bottle_c": C.TablePropMeta(
        bbox=BBox(half_w=0.024491, half_d=0.024490),
        dynamic=True,
        mandatory=True,
    ),
}
MEDICINE_BOTTLE_SPAWN_REGION = (-0.52, 0.18, -0.16, 0.28)
STATIC_LOGISTICS_CLUSTER = {
    "ridgeback_left": ((-0.70, 0.85), -math.pi / 2, C.RIDGEBACK_BBOX),
    "ridgeback_right": ((-0.70, -0.85), -math.pi / 2, C.RIDGEBACK_BBOX),
}


def _spawn_boundary_axis(box):
    return 0 if box[2] <= box[3] else 1


def _outside_spawn_region(box, boundaries):
    for _, boundary in boundaries:
        axis = _spawn_boundary_axis(boundary)
        center = boundary[axis]
        seed = C.SPAWN_REGION_SEED[axis]
        sign = 1.0 if seed >= center else -1.0
        if any(
            sign * (corner[axis] - center) < -C.SPAWN_BOUNDARY_TOLERANCE
            for corner in obb_corners(*box)
        ):
            return True
    return False


def _inside_group_bounds(box):
    return all(
        C.TABLE_GROUP_X_MIN <= x <= C.TABLE_GROUP_X_MAX
        and C.TABLE_GROUP_Y_MIN <= y <= C.TABLE_GROUP_Y_MAX
        for x, y in obb_corners(*box)
    )


def _inside_tabletop(box, spawn_region=None):
    if spawn_region is None:
        x_min, x_max = C.DESK_LOCAL_X_MIN, C.DESK_LOCAL_X_MAX
        y_min, y_max = C.DESK_LOCAL_Y_MIN, C.DESK_LOCAL_Y_MAX
    else:
        x_min, x_max, y_min, y_max = spawn_region
    return all(
        x_min <= x <= x_max and y_min <= y <= y_max
        for x, y in obb_corners(*box)
    )


def _wall_position(zone, meta, rng):
    along = rng.uniform(zone.sample_min, zone.sample_max)
    offsets = meta.wall_offsets
    offset = offsets.get(zone.wall, meta.wall_offset) if offsets else meta.wall_offset
    if zone.wall == "back":
        root_x, root_y = along, zone.fixed_coord + offset
    else:
        root_x, root_y = zone.fixed_coord - offset, along
    return root_x, root_y, zone.base_yaw + meta.yaw_offset


def _wall_box(root_x, root_y, yaw, meta):
    center_x, center_y = offset_from_yaw(
        root_x, root_y, yaw, meta.bbox_center[0], meta.bbox_center[1]
    )
    return make_obb(center_x, center_y, meta.bbox, yaw)


def _right_wall_surfaces(candidate, static_walls):
    corners = obb_corners(*candidate)
    low_y, high_y = min(y for _, y in corners), max(y for _, y in corners)
    surfaces = []
    for name, wall in static_walls:
        if _spawn_boundary_axis(wall) != 0 or C.SPAWN_REGION_SEED[0] >= wall[0]:
            continue
        wall_corners = obb_corners(*wall)
        if high_y < min(y for _, y in wall_corners) or low_y > max(y for _, y in wall_corners):
            continue
        surfaces.append((name, min(x for x, _ in wall_corners)))
    return surfaces


def _snap_right_wall(root_x, root_y, yaw, meta, static_walls):
    candidate = _wall_box(root_x, root_y, yaw, meta)
    surfaces = _right_wall_surfaces(candidate, static_walls)
    if not surfaces:
        return None
    target_face = min(face for _, face in surfaces) - C.RIGHT_WALL_CONTACT_CLEARANCE
    candidate_face = max(x for x, _ in obb_corners(*candidate))
    return root_x + target_face - candidate_face


def _wall_supports(zone, wall):
    axis = _spawn_boundary_axis(wall)
    center = wall[axis]
    seed = C.SPAWN_REGION_SEED[axis]
    return (zone.wall == "back" and axis == 1 and seed > center) or (
        zone.wall == "right" and axis == 0 and seed < center
    )


def _place_walls(rng, static_walls, boundaries, reserved=()):
    placed = []
    collision_boxes = list(reserved)
    records = []
    names = sorted(C.WALL_PROP_META, key=lambda name: not C.WALL_PROP_META[name].tall)
    for name in names:
        meta = C.WALL_PROP_META[name]
        zones = [zone for zone in C.WALL_ZONES if zone.wall in meta.allowed_walls]
        selected = None
        selected_wall = "despawned"
        for _ in range(100):
            zone = rng.choice(zones)
            root_x, root_y, yaw = _wall_position(zone, meta, rng)
            if zone.wall == "right":
                root_x = _snap_right_wall(root_x, root_y, yaw, meta, static_walls)
                if root_x is None:
                    continue
            candidate = _wall_box(root_x, root_y, yaw, meta)
            if not obb_inside_room(candidate) or _outside_spawn_region(candidate, boundaries):
                continue
            if any(
                not _wall_supports(zone, wall)
                and obb_overlap(candidate, wall, margin=C.OBB_PLACEMENT_MARGIN)
                for _, wall in static_walls
            ):
                continue
            if obb_overlap_any(candidate, collision_boxes, margin=C.OBB_PLACEMENT_MARGIN):
                continue
            selected = candidate
            selected_wall = zone.wall
            placed.append(candidate)
            collision_boxes.append(candidate)
            break
        records.append({"name": name, "wall": selected_wall, "box": selected})
    return placed, records


def _robot_local_xy(rx, ry, robot_yaw, local_xy):
    return offset_from_yaw(rx, ry, robot_yaw, local_xy[0], local_xy[1])


def _swept_box(start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    return make_obb(
        (start[0] + end[0]) * 0.5,
        (start[1] + end[1]) * 0.5,
        BBox(math.hypot(dx, dy) * 0.5 + C.RIDGEBACK_BBOX.half_w, C.RIDGEBACK_CORRIDOR_HALF_WIDTH),
        math.atan2(dy, dx),
    )


def _ridgeback_group(rx, ry, robot_yaw):
    waiting = _robot_local_xy(rx, ry, robot_yaw, C.RIDGEBACK_WAITING_ROBOT_LOCAL)
    result = {"ridgeback_waiting": make_obb(*waiting, C.RIDGEBACK_BBOX, robot_yaw)}
    for side, sign in (("left", 1.0), ("right", -1.0)):
        staging = _robot_local_xy(
            rx,
            ry,
            robot_yaw,
            (C.RIDGEBACK_STAGING_ROBOT_LOCAL[0], sign * C.RIDGEBACK_STAGING_ROBOT_LOCAL[1]),
        )
        delivery = _robot_local_xy(
            rx,
            ry,
            robot_yaw,
            (C.RIDGEBACK_DELIVERY_ROBOT_LOCAL[0], sign * C.RIDGEBACK_DELIVERY_ROBOT_LOCAL[1]),
        )
        result[f"ridgeback_staging_{side}"] = make_obb(*staging, C.RIDGEBACK_BBOX, robot_yaw)
        result[f"ridgeback_delivery_{side}"] = make_obb(*delivery, C.RIDGEBACK_BBOX, robot_yaw)
        result[f"ridgeback_corridor_waiting_{side}"] = _swept_box(waiting, staging)
        result[f"ridgeback_corridor_delivery_{side}"] = _swept_box(staging, delivery)
    return result


def _static_cluster_group(rx, ry, robot_yaw, static_cluster):
    result = {}
    for name, (local_xy, yaw_offset, bbox) in static_cluster.items():
        world_xy = _robot_local_xy(rx, ry, robot_yaw, local_xy)
        result[name] = make_obb(
            *world_xy, bbox, robot_yaw + yaw_offset
        )
    return result


def _is_moving_ridgeback_geometry(name):
    return name == "ridgeback_waiting" or name.startswith(
        ("ridgeback_staging_", "ridgeback_delivery_", "ridgeback_corridor_")
    )


def _sample_group(rng, include_mobile_ridgeback=True, static_cluster=None):
    layout = rng.choice(C.ROBOT_FACING_LAYOUTS)
    rx = rng.uniform(C.TABLE_SAMPLE_X_MIN, C.TABLE_SAMPLE_X_MAX)
    ry = rng.uniform(C.TABLE_SAMPLE_Y_MIN, C.TABLE_SAMPLE_Y_MAX)
    target = rng.uniform(layout.sample_min, layout.sample_max)
    target_x, target_y = (
        (target, layout.fixed_coord)
        if layout.target_axis == "y"
        else (layout.fixed_coord, target)
    )
    raw_yaw = math.atan2(target_y - ry, target_x - rx)
    delta = (raw_yaw - layout.yaw_center + math.pi) % (2.0 * math.pi) - math.pi
    delta = max(-C.ROBOT_FACING_MAX_YAW_OFFSET_RAD, min(C.ROBOT_FACING_MAX_YAW_OFFSET_RAD, delta))
    robot_yaw = layout.yaw_center + delta
    table_yaw = robot_yaw - math.pi / 2
    table_x, table_y = offset_from_yaw(
        rx, ry, table_yaw, -C.ROBOT_ORBIT_OFFSET[0], -C.ROBOT_ORBIT_OFFSET[1]
    )
    group = {
        "packing_table": make_obb(table_x, table_y, C.DESK_BBOX, table_yaw),
        "robot": make_obb(rx, ry, C.ROBOT_BBOX, robot_yaw),
    }
    if include_mobile_ridgeback:
        group.update(_ridgeback_group(rx, ry, robot_yaw))
    if static_cluster:
        group.update(_static_cluster_group(rx, ry, robot_yaw, static_cluster))
    return group, layout.name


def _valid_group(group, obstacles, boundaries):
    if any(
        not obb_inside_room(box)
        or not _inside_group_bounds(box)
        or _outside_spawn_region(box, boundaries)
        or obb_overlap_any(box, obstacles, margin=C.OBB_PLACEMENT_MARGIN)
        for box in group.values()
    ):
        return False
    items = list(group.items())
    for index, (name_a, box_a) in enumerate(items):
        for name_b, box_b in items[index + 1 :]:
            if _is_moving_ridgeback_geometry(name_a) and _is_moving_ridgeback_geometry(name_b):
                continue
            if {name_a, name_b} == {"packing_table", "robot"}:
                continue
            margin = C.RIDGEBACK_GROUP_MARGIN if name_a.startswith(RIDGE_PREFIX) or name_b.startswith(RIDGE_PREFIX) else C.OBB_PLACEMENT_MARGIN
            if obb_overlap(box_a, box_b, margin=margin):
                return False
    return True


def _place_tabletop(
    rng,
    table_box,
    boundaries,
    table_object_names=TABLE_OBJECT_NAMES,
    table_prop_meta_overrides=None,
    tabletop_spawn_region=None,
):
    table_x, table_y, _, _, table_yaw = table_box
    table_prop_meta = dict(C.TABLE_PROP_META)
    if table_prop_meta_overrides is not None:
        table_prop_meta.update(table_prop_meta_overrides)
    occupied = [make_obb(area.center[0], area.center[1], area.bbox, area.yaw) for area in C.TABLE_RESERVED_AREAS]
    placements = {}
    for name in table_object_names:
        meta = table_prop_meta[name]
        selected = None
        for _ in range(300):
            if name == "desk_lamp":
                if tabletop_spawn_region is None:
                    x_range = C.DESK_LAMP_LOCAL_X_RANGE
                    y_range = C.DESK_LAMP_LOCAL_Y_RANGE
                else:
                    x_range = tabletop_spawn_region[:2]
                    y_range = tabletop_spawn_region[2:]
                lx = rng.uniform(*x_range)
                ly = rng.uniform(*y_range)
                local_yaw = C.DESK_LAMP_LOCAL_YAW
            else:
                if tabletop_spawn_region is None:
                    low_x = C.TABLETOP_CUBE_LOCAL_X_MIN if name in C.TABLETOP_CUBE_PROP_NAMES else C.DESK_LOCAL_X_MIN
                    high_x = C.TABLETOP_CUBE_LOCAL_X_MAX if name in C.TABLETOP_CUBE_PROP_NAMES else C.DESK_LOCAL_X_MAX
                    low_y, high_y = C.DESK_LOCAL_Y_MIN, C.DESK_LOCAL_Y_MAX
                else:
                    low_x, high_x, low_y, high_y = tabletop_spawn_region
                lx, ly = rng.uniform(low_x, high_x), rng.uniform(low_y, high_y)
                local_yaw = rng.uniform(0.0, 2.0 * math.pi)
            candidate = make_obb(lx, ly, meta.bbox, local_yaw)
            if not _inside_tabletop(candidate, tabletop_spawn_region):
                continue
            if obb_overlap_any(candidate, occupied, margin=C.DESK_OBJECT_MARGIN):
                continue
            world_x, world_y = offset_from_yaw(table_x, table_y, table_yaw, lx, ly)
            world_box = make_obb(world_x, world_y, meta.bbox, table_yaw + local_yaw)
            if _outside_spawn_region(world_box, boundaries):
                continue
            selected = {
                "local": (lx, ly, local_yaw),
                "world": (world_x, world_y, table_yaw + local_yaw),
                "box": candidate,
            }
            occupied.append(candidate)
            break
        if selected is None:
            raise AssertionError(f"could not place mandatory tabletop object {name}")
        placements[name] = selected
    return placements


def randomize_one_room(
    seed_or_rng,
    table_object_names=TABLE_OBJECT_NAMES,
    table_prop_meta_overrides=None,
    tabletop_spawn_region=None,
    include_mobile_ridgeback=True,
    static_cluster=None,
):
    rng = seed_or_rng if isinstance(seed_or_rng, random.Random) else random.Random(seed_or_rng)
    static_walls = [
        (item.name, make_obb(item.center[0], item.center[1], item.bbox, item.yaw))
        for item in C.STATIC_ROOM_OBSTACLES
    ]
    boundaries = [
        (item.name, make_obb(item.center[0], item.center[1], item.bbox, item.yaw))
        for item in C.FALLBACK_NO_SPAWN_BOUNDARIES
    ]
    wall_boxes, wall_records = _place_walls(rng, static_walls, boundaries)
    obstacles = wall_boxes + [box for _, box in static_walls]
    group = None
    layout_name = None
    for _ in range(C.TABLE_GROUP_MAX_TRIES * 2):
        candidate, candidate_layout = _sample_group(
            rng,
            include_mobile_ridgeback=include_mobile_ridgeback,
            static_cluster=static_cluster,
        )
        if _valid_group(candidate, obstacles, boundaries):
            group, layout_name = candidate, candidate_layout
            break
    if group is None:
        raise AssertionError("could not place table/robot/Ridgeback group")
    tabletop = _place_tabletop(
        rng,
        group["packing_table"],
        boundaries,
        table_object_names,
        table_prop_meta_overrides,
        tabletop_spawn_region,
    )
    return {
        "layout": layout_name,
        "walls": wall_records,
        "static_walls": static_walls,
        "group": group,
        "tabletop": tabletop,
    }


def randomize_fixed_table_room(seed_or_rng):
    """Scramble props around the stable teleoperation table anchor."""
    rng = seed_or_rng if isinstance(seed_or_rng, random.Random) else random.Random(seed_or_rng)
    static_walls = [
        (item.name, make_obb(item.center[0], item.center[1], item.bbox, item.yaw))
        for item in C.STATIC_ROOM_OBSTACLES
    ]
    boundaries = [
        (item.name, make_obb(item.center[0], item.center[1], item.bbox, item.yaw))
        for item in C.FALLBACK_NO_SPAWN_BOUNDARIES
    ]
    table_x, table_y, table_yaw = C.TABLE_FALLBACK_X, C.TABLE_FALLBACK_Y, 0.0
    robot_x, robot_y = offset_from_yaw(
        table_x, table_y, table_yaw, *C.ROBOT_ORBIT_OFFSET
    )
    robot_yaw = table_yaw + math.pi / 2
    group = {
        "packing_table": make_obb(table_x, table_y, C.DESK_BBOX, table_yaw),
        "robot": make_obb(robot_x, robot_y, C.ROBOT_BBOX, robot_yaw),
    }
    group.update(_ridgeback_group(robot_x, robot_y, robot_yaw))
    if not _valid_group(group, [box for _, box in static_walls], boundaries):
        raise AssertionError("fixed teleoperation table group is invalid")
    wall_boxes, wall_records = _place_walls(
        rng, static_walls, boundaries, reserved=group.values()
    )
    tabletop = _place_tabletop(rng, group["packing_table"], boundaries)
    return {
        "layout": "fixed_teleop",
        "walls": wall_records,
        "static_walls": static_walls,
        "group": group,
        "tabletop": tabletop,
    }


def respawn_target(room, rng, table_prop_meta_overrides=None, tabletop_spawn_region=None):
    occupied = [
        placement["box"] for name, placement in room["tabletop"].items() if name != "object"
    ]
    occupied.extend(make_obb(area.center[0], area.center[1], area.bbox, area.yaw) for area in C.TABLE_RESERVED_AREAS)
    table_prop_meta = dict(C.TABLE_PROP_META)
    if table_prop_meta_overrides:
        table_prop_meta.update(table_prop_meta_overrides)
    meta = table_prop_meta["object"]
    table = room["group"]["packing_table"]
    for _ in range(300):
        if tabletop_spawn_region is None:
            x_min, x_max = C.DESK_LOCAL_X_MIN, C.DESK_LOCAL_X_MAX
            y_min, y_max = C.DESK_LOCAL_Y_MIN, C.DESK_LOCAL_Y_MAX
        else:
            x_min, x_max, y_min, y_max = tabletop_spawn_region
        lx, ly = rng.uniform(x_min, x_max), rng.uniform(y_min, y_max)
        yaw = rng.uniform(0.0, 2.0 * math.pi)
        candidate = make_obb(lx, ly, meta.bbox, yaw)
        if not _inside_tabletop(candidate, tabletop_spawn_region) or obb_overlap_any(candidate, occupied, margin=C.DESK_OBJECT_MARGIN):
            continue
        wx, wy = offset_from_yaw(table[0], table[1], table[4], lx, ly)
        room["tabletop"]["object"] = {
            "local": (lx, ly, yaw),
            "world": (wx, wy, table[4] + yaw),
            "box": candidate,
        }
        return
    raise AssertionError("target respawn did not find a table pose")


def _assert_valid(room, seed, tabletop_spawn_region=None):
    obstacles = [record["box"] for record in room["walls"] if record["box"] is not None]
    obstacles.extend(box for _, box in room["static_walls"])
    for name, box in room["group"].items():
        assert obb_inside_room(box), f"seed {seed}: {name} outside room"
        assert _inside_group_bounds(box), f"seed {seed}: {name} outside operating bounds"
        assert not obb_overlap_any(box, obstacles, margin=C.OBB_PLACEMENT_MARGIN), f"seed {seed}: {name} overlaps wall/furniture"

    local_boxes = []
    for name, placement in room["tabletop"].items():
        box = placement["box"]
        assert _inside_tabletop(box, tabletop_spawn_region), f"seed {seed}: {name} outside tabletop"
        for area in C.TABLE_RESERVED_AREAS:
            reserved = make_obb(area.center[0], area.center[1], area.bbox, area.yaw)
            assert not obb_overlap(box, reserved, margin=C.DESK_OBJECT_MARGIN), f"seed {seed}: {name} overlaps reserved area"
        for other_name, other_box in local_boxes:
            assert not obb_overlap(box, other_box, margin=C.DESK_OBJECT_MARGIN), f"seed {seed}: {name} overlaps {other_name}"
        local_boxes.append((name, box))


def run_geometry_tests(layout_count=1000):
    for seed in range(layout_count):
        room = randomize_one_room(0xA11CE + seed)
        _assert_valid(room, seed)
        before = room["tabletop"]["object"]["world"]
        respawn_target(room, random.Random(0xB0771E + seed))
        _assert_valid(room, seed)
        after = room["tabletop"]["object"]["world"]
        assert before != after, f"seed {seed}: target respawn did not move"

        fixed_room = randomize_fixed_table_room(0xF17ED + seed)
        _assert_valid(fixed_room, f"fixed-{seed}")
        assert fixed_room["group"]["packing_table"][:2] == (
            C.TABLE_FALLBACK_X,
            C.TABLE_FALLBACK_Y,
        ), f"seed {seed}: fixed teleoperation table moved"

        medicine_bottle_room = randomize_one_room(
            0xB10C0 + seed,
            MEDICINE_BOTTLE_TABLE_OBJECT_NAMES,
            MEDICINE_BOTTLE_TABLE_PROP_META_OVERRIDES,
            tabletop_spawn_region=MEDICINE_BOTTLE_SPAWN_REGION,
            include_mobile_ridgeback=False,
            static_cluster=STATIC_LOGISTICS_CLUSTER,
        )
        _assert_valid(
            medicine_bottle_room,
            f"medicine-bottle-{seed}",
            MEDICINE_BOTTLE_SPAWN_REGION,
        )
        assert set(medicine_bottle_room["tabletop"]) == set(MEDICINE_BOTTLE_TABLE_OBJECT_NAMES)
        assert {"ridgeback_left", "ridgeback_right"} <= set(medicine_bottle_room["group"])
        robot_box = medicine_bottle_room["group"]["robot"]
        for name, (local_xy, yaw_offset, bbox) in STATIC_LOGISTICS_CLUSTER.items():
            expected_xy = _robot_local_xy(robot_box[0], robot_box[1], robot_box[4], local_xy)
            actual = medicine_bottle_room["group"][name]
            assert math.dist(actual[:2], expected_xy) < 1.0e-9
            assert abs(actual[4] - (robot_box[4] + yaw_offset)) < 1.0e-9
            ridgeback_lateral_half_extent = (
                abs(math.sin(yaw_offset)) * bbox.half_w
                + abs(math.cos(yaw_offset)) * bbox.half_d
            )
            leg_gap = (
                abs(local_xy[1])
                - C.ROBOT_BBOX.half_d
                - ridgeback_lateral_half_extent
            )
            assert math.isclose(leg_gap, 0.10, abs_tol=1.0e-9)
        before = medicine_bottle_room["tabletop"]["object"]["world"]
        respawn_target(
            medicine_bottle_room,
            random.Random(0xB0771E + seed),
            MEDICINE_BOTTLE_TABLE_PROP_META_OVERRIDES,
            MEDICINE_BOTTLE_SPAWN_REGION,
        )
        _assert_valid(
            medicine_bottle_room,
            f"medicine-bottle-respawn-{seed}",
            MEDICINE_BOTTLE_SPAWN_REGION,
        )
        assert medicine_bottle_room["tabletop"]["object"]["world"] != before

    same_a = randomize_one_room(123456)
    same_b = randomize_one_room(123456)
    different = randomize_one_room(123457)
    assert same_a == same_b, "same seed produced different layouts"
    assert same_a != different, "different seeds produced the same layout"

    sequence = random.Random(987654)
    reset_a = randomize_one_room(sequence)
    reset_b = randomize_one_room(sequence)
    _assert_valid(reset_b, "full-reset")
    assert reset_a != reset_b, "full reset did not produce a new valid layout"

    fixed_a = randomize_fixed_table_room(123456)
    fixed_b = randomize_fixed_table_room(123457)
    assert fixed_a["group"] == fixed_b["group"], "fixed table group changed between resets"
    assert fixed_a["walls"] != fixed_b["walls"], "fixed mode did not scramble wall props"
    assert fixed_a["tabletop"] != fixed_b["tabletop"], "fixed mode did not scramble tabletop props"


def main():
    layout_count = 1000
    run_geometry_tests(layout_count)
    print(
        f"PASS: {layout_count} deterministic layouts, Ridgeback corridors, "
        "fixed-table teleop layouts, tabletop placements, respawn, reset, "
        "and seed reproducibility"
    )


if __name__ == "__main__":
    main()
