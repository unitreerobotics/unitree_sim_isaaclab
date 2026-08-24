# constants.py
# Room placement constants for the hospital room environment.
# Uses oriented bounding boxes (OBB) and continuous wall zones
# instead of circles and predefined slots.

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

# ============================================================
# Room geometry
# ============================================================

ROOM_X_MIN = -13.0
# The floor mesh in new_base_room.usda spans x=[-13, -1], but the
# room-facing right wall is authored at about x=-2.5.
ROOM_X_MAX = -0.5
# Authored safe back-wall props sit around y=-10.75..-10.96 and
# the back wall transform is farther back than the original -11.0 limit.
ROOM_Y_MIN = -11.25
ROOM_Y_MAX = -5.0
FLOOR_Z = 0.0
ROBOT_Z = 0.0328

# Wall surface positions (room-facing edge of each wall).
BACK_WALL_LINE_Y = -10.95
FRONT_WALL_LINE_Y = -5.47
# Composed bounds of Geo_M3_SideWall_40/41 in new_base_room.usda are
# x=[-2.570861, -2.468369]. The room interior is on the low-X side.
RIGHT_WALL_CENTER_X = -2.519615
RIGHT_WALL_HALF_THICKNESS = 0.051246
RIGHT_WALL_LINE_X = RIGHT_WALL_CENTER_X - RIGHT_WALL_HALF_THICKNESS
RIGHT_WALL_CONTACT_CLEARANCE = 0.005

# ============================================================
# Bounding box primitives
# ============================================================


@dataclass(frozen=True)
class BBox:
    """2D oriented bounding box footprint (local frame).

    half_w: half-extent along the object's local X axis (width).
    half_d: half-extent along the object's local Y axis (depth).
    """
    half_w: float
    half_d: float


@dataclass(frozen=True)
class StaticRoomObstacle:
    """Static room-shell footprint that the table group must avoid."""
    name: str
    center: Tuple[float, float]
    bbox: BBox
    yaw: float = 0.0


@dataclass(frozen=True)
class SpawnBoundary:
    """Half-plane boundary that randomized objects may not spawn beyond."""
    name: str
    center: Tuple[float, float]
    bbox: BBox
    yaw: float = 0.0


# ============================================================
# Wall zones — continuous strips where wall props can be placed
# ============================================================


@dataclass(frozen=True)
class WallZone:
    """A continuous strip along a wall where props sample positions.

    For the back wall:  the free axis is X, fixed axis is Y.
    For the right wall: the free axis is Y, fixed axis is X.
    """
    wall: str           # "back" or "right"
    sample_min: float   # min of the free axis (X for back, Y for right)
    sample_max: float   # max of the free axis
    fixed_coord: float  # centre position on the constrained axis
    base_yaw: float     # yaw to face into the room (radians)


WALL_ZONES: List[WallZone] = [
    # Back wall: props slide along X, fixed near back wall Y.
    WallZone(
        wall="back",
        sample_min=-12.0,
        sample_max=-4.0,
        fixed_coord=-10.75,  # prop center Y (matches authored back-wall props)
        base_yaw=0.0,        # face into room (+Y direction)
    ),
    # Right wall: props slide along Y, fixed near right wall X.
    WallZone(
        wall="right",
        sample_min=-10.0,
        sample_max=-7.0,
        fixed_coord=-3.0,    # prop center X (slightly off the right wall surface)
        base_yaw=math.pi / 2,  # face into room (-X direction)
    ),
]

# Conservative fallback RoomShell wall footprints used when USD stage bounds
# are unavailable. At runtime, the randomizer prefers the authored wall
# geometry from the spawned RoomShell so edits to new_base_room.usda are used.
STATIC_ROOM_OBSTACLES: List[StaticRoomObstacle] = [
    StaticRoomObstacle(
        name="static_front_wall",
        center=(-7.5, FRONT_WALL_LINE_Y),
        bbox=BBox(half_w=5.35, half_d=0.12),
    ),
    StaticRoomObstacle(
        name="static_front_wall_right_extension",
        center=(-1.35, FRONT_WALL_LINE_Y),
        bbox=BBox(half_w=0.85, half_d=0.12),
    ),
    StaticRoomObstacle(
        name="static_right_wall",
        center=(RIGHT_WALL_CENTER_X, -8.15),
        bbox=BBox(half_w=RIGHT_WALL_HALF_THICKNESS, half_d=3.15),
    ),
    StaticRoomObstacle(
        name="static_back_wall",
        center=(-8.35, BACK_WALL_LINE_Y),
        bbox=BBox(half_w=4.35, half_d=0.12),
    ),
]

# ============================================================
# Room interior sampling zone (for table group)
# ============================================================

TABLE_SAMPLE_X_MIN = -10.0
TABLE_SAMPLE_X_MAX = -5.0
TABLE_SAMPLE_Y_MIN = -9.0
TABLE_SAMPLE_Y_MAX = -6.0
TABLE_FALLBACK_X = -7.50
TABLE_FALLBACK_Y = -7.50
TABLE_GROUP_MAX_TRIES = 300

# No-spawn boundaries use the side containing this seed as the valid side.
SPAWN_REGION_SEED = (TABLE_FALLBACK_X, TABLE_FALLBACK_Y)
SPAWN_BOUNDARY_TOLERANCE = 0.08

# Conservative fallback no-spawn boundaries. Runtime stage geometry is preferred.
FALLBACK_NO_SPAWN_BOUNDARIES: List[SpawnBoundary] = [
    SpawnBoundary(
        name="no_spawn_front_wall",
        center=(-7.5, FRONT_WALL_LINE_Y),
        bbox=BBox(half_w=5.35, half_d=0.12),
    ),
    SpawnBoundary(
        name="no_spawn_front_wall_right_extension",
        center=(-1.35, FRONT_WALL_LINE_Y),
        bbox=BBox(half_w=0.85, half_d=0.12),
    ),
    SpawnBoundary(
        name="no_spawn_new_partition_wall",
        center=(RIGHT_WALL_LINE_X, -8.15),
        bbox=BBox(half_w=0.12, half_d=3.15),
    ),
    SpawnBoundary(
        name="no_spawn_back_wall",
        center=(-8.35, BACK_WALL_LINE_Y),
        bbox=BBox(half_w=4.35, half_d=0.12),
    ),
]

# Stricter usable area for the table group, including the virtual open side.
TABLE_GROUP_X_MIN = -12.35
TABLE_GROUP_X_MAX = RIGHT_WALL_LINE_X - 0.35
TABLE_GROUP_Y_MIN = BACK_WALL_LINE_Y + 0.35
TABLE_GROUP_Y_MAX = FRONT_WALL_LINE_Y - 0.35

# Robot-facing targets are intentionally narrower than the full room bounds.
# The open side is never a selectable facing layout; the robot may face the
# authored back wall or the new partition wall.
OPEN_WALL_NAMES = ("front_open", "left_open")
BACK_WALL_TARGET_X_MIN = -10.0
BACK_WALL_TARGET_X_MAX = -5.0
NEW_WALL_TARGET_Y_MIN = -9.0
NEW_WALL_TARGET_Y_MAX = -6.0
ROBOT_FACING_MAX_YAW_OFFSET_RAD = math.radians(15.0)


@dataclass(frozen=True)
class RobotFacingLayout:
    """Robot/table placement mode that keeps the robot facing a room wall."""
    name: str
    wall: str
    target_axis: str
    fixed_coord: float
    sample_min: float
    sample_max: float
    yaw_center: float


ROBOT_FACING_LAYOUTS: List[RobotFacingLayout] = [
    RobotFacingLayout(
        name="face_back_wall",
        wall="back",
        target_axis="y",
        fixed_coord=BACK_WALL_LINE_Y,
        sample_min=BACK_WALL_TARGET_X_MIN,
        sample_max=BACK_WALL_TARGET_X_MAX,
        yaw_center=-math.pi / 2,
    ),
    RobotFacingLayout(
        name="face_new_wall",
        wall="new_wall",
        target_axis="x",
        fixed_coord=RIGHT_WALL_LINE_X,
        sample_min=NEW_WALL_TARGET_Y_MIN,
        sample_max=NEW_WALL_TARGET_Y_MAX,
        yaw_center=0.0,
    ),
]


def _validate_robot_facing_layouts() -> None:
    """Guard against accidentally aiming the robot at the empty/open side."""
    for layout in ROBOT_FACING_LAYOUTS:
        if layout.wall in OPEN_WALL_NAMES:
            raise ValueError(f"robot facing layout targets open wall: {layout}")
        if layout.sample_min >= layout.sample_max:
            raise ValueError(f"invalid wall target span: {layout}")
        if layout.target_axis == "y":
            if not (ROOM_Y_MIN <= layout.fixed_coord <= ROOM_Y_MAX):
                raise ValueError(f"wall target y outside room: {layout}")
            if layout.sample_min < ROOM_X_MIN or layout.sample_max > ROOM_X_MAX:
                raise ValueError(f"wall target x span outside room: {layout}")
        elif layout.target_axis == "x":
            if not (ROOM_X_MIN <= layout.fixed_coord <= ROOM_X_MAX):
                raise ValueError(f"wall target x outside room: {layout}")
            if layout.sample_min < ROOM_Y_MIN or layout.sample_max > ROOM_Y_MAX:
                raise ValueError(f"wall target y span outside room: {layout}")
        else:
            raise ValueError(f"invalid wall target axis: {layout}")


_validate_robot_facing_layouts()

# Keep strict wall-facing placement unless training needs controlled jitter.
ROBOT_FACING_YAW_JITTER_RAD = 0.0

# ============================================================
# Desk geometry
# ============================================================

DESK_TOP_Z = 0.78
DESK_OBJECT_Z = DESK_TOP_Z + 0.04

# Desk surface sampling bounds (local to desk prim).
DESK_LOCAL_X_MIN = -0.38
DESK_LOCAL_X_MAX = 0.38
DESK_LOCAL_Y_MIN = -0.22
DESK_LOCAL_Y_MAX = 0.22
DESK_OBJECT_MARGIN = 0.03   # margin between tabletop OBBs

# ============================================================
# Orbital offsets (local-frame displacement from desk center)
# ============================================================

CHAIR_ORBIT_OFFSET = (0.0, -1.00)   # (local_x, local_y)
ROBOT_ORBIT_OFFSET = (-1.95, 1.10)

# ============================================================
# Source transforms (kept for reference)
# ============================================================

DESK_SOURCE_CENTER = (-4.40, -7.10, 0.0)
CHAIR_SOURCE_CENTER = (-5.80, -7.30, 0.0)
ROBOT_SOURCE_CENTER = (-6.1, -5.95, 0.0)

# Despawn height.
DESPAWN_Z = -100.0

# Margin added around every OBB during placement checks (metres).
OBB_PLACEMENT_MARGIN = 0.15

# ============================================================
# Wall prop metadata — bounding boxes replace spacing radii
# ============================================================


@dataclass(frozen=True)
class WallPropMeta:
    """Placement metadata for a wall prop."""
    usd_name: str
    bbox: BBox              # footprint in the object's local frame
    # Local XY offset from the asset root to the footprint center.
    bbox_center: Tuple[float, float] = (0.0, 0.0)
    tall: bool = False
    wall_offset: float = 0.0  # extra push away from wall surface (metres)
    wall_offsets: Dict[str, float] | None = None
    yaw_offset: float = 0.0   # yaw adjustment relative to wall base yaw (radians)
    allowed_walls: Tuple[str, ...] = ("back", "right")


WALL_PROP_META: Dict[str, WallPropMeta] = {
    "medical_cabinet": WallPropMeta(
        "SM_MedicalCabinet_01a",
        bbox=BBox(half_w=0.436, half_d=0.328),
        bbox_center=(0.415679, 0.303706),
        tall=True,
        wall_offset=0.25,
        wall_offsets={"back": 0.431706, "right": 0.131706},
        yaw_offset=math.pi,
        allowed_walls=("back", "right"),
    ),
    "shelf_set": WallPropMeta(
        "SM_ShelfSet_01a",
        bbox=BBox(half_w=0.861, half_d=0.280),
        tall=True,
        wall_offset=-0.220,
        wall_offsets={"back": 0.080, "right": -0.220},
        yaw_offset=math.pi,
        allowed_walls=("back", "right"),
    ),
    "supply_cabinet": WallPropMeta(
        "SM_SupplyCabinet_01c",
        bbox=BBox(half_w=0.367, half_d=0.737),
        tall=True,
        wall_offset=0.167,
        wall_offsets={"back": 0.167, "right": -0.133},
        yaw_offset=math.pi / 2,
        allowed_walls=("back", "right"),
    ),
    "trash_can": WallPropMeta(
        "SM_TrashCan",
        bbox=BBox(half_w=0.150, half_d=0.150),
    ),
    "plant_a": WallPropMeta(
        "SM_Plant01",
        bbox=BBox(half_w=0.352, half_d=0.404),
    ),
    "plant_b": WallPropMeta(
        "SM_Plant02",
        bbox=BBox(half_w=0.252, half_d=0.3),
    ),
    "supply_cart_a": WallPropMeta(
        "SM_SupplyCart_02a",
        bbox=BBox(half_w=0.421, half_d=0.228),
    ),
    "supply_cart_b": WallPropMeta(
        "SM_SupplyCart_03a",
        bbox=BBox(half_w=0.298, half_d=0.556),
        yaw_offset=math.pi / 2,
    ),
}

# ============================================================
# Table group bounding boxes
# ============================================================

DESK_BBOX = BBox(half_w=0.745, half_d=0.227)
CHAIR_BBOX = BBox(half_w=0.347, half_d=0.343)
ROBOT_BBOX = BBox(half_w=0.65, half_d=0.50)

# ============================================================
# Tabletop object metadata
# ============================================================


@dataclass(frozen=True)
class TablePropMeta:
    """Placement metadata for a tabletop object."""
    bbox: BBox


TABLE_PROP_META: Dict[str, TablePropMeta] = {
    "coffee_cup":   TablePropMeta(bbox=BBox(half_w=0.043, half_d=0.043)),
    "desk_lamp":    TablePropMeta(bbox=BBox(half_w=0.241, half_d=0.134)),
    "box_portable": TablePropMeta(bbox=BBox(half_w=0.195, half_d=0.145)),
}

# ============================================================
# Wall yaw lookup (kept for backward compat, but yaw_offset
# on WallPropMeta is the primary source now)
# ============================================================

WALL_PROP_YAW_BY_WALL: Dict[Tuple[str, str], float] = {
    ("SM_MedicalCabinet_01a", "back"):  180.0,
    ("SM_MedicalCabinet_01a", "right"): 180.0,
    ("SM_ShelfSet_01a",       "back"):  180.0,
    ("SM_ShelfSet_01a",       "right"): 180.0,
    ("SM_SupplyCabinet_01c",  "back"):  90.0,
    ("SM_SupplyCabinet_01c",  "right"): 90.0,
    ("SM_TrashCan",           "back"):  0.0,
    ("SM_TrashCan",           "right"): 0.0,
    ("SM_Plant01",            "back"):  0.0,
    ("SM_Plant01",            "right"): 0.0,
    ("SM_Plant02",            "back"):  0.0,
    ("SM_Plant02",            "right"): 0.0,
    ("SM_SupplyCart_02a",     "back"):  0.0,
    ("SM_SupplyCart_02a",     "right"): 0.0,
    ("SM_SupplyCart_03a",     "back"):  0.0,
    ("SM_SupplyCart_03a",     "right"): 0.0,
}

# ============================================================
# Asset USD paths (Omniverse S3 CDN) — kept for reference
# ============================================================

_HOSPITAL = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Environments/Hospital/Props"
_OFFICE = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Environments/Office/Props"
_WAREHOUSE = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Environments/Simple_Warehouse/Props"

ASSET_PATHS: Dict[str, str] = {
    "SM_MedicalCabinet_01a": f"{_HOSPITAL}/SM_MedicalCabinet_01a.usd",
    "SM_ShelfSet_01a":       f"{_HOSPITAL}/SM_ShelfSet_01a.usd",
    "SM_SupplyCabinet_01c":  f"{_HOSPITAL}/SM_SupplyCabinet_01c.usd",
    "SM_TrashCan":           f"{_HOSPITAL}/SM_TrashCan.usd",
    "SM_SupplyCart_02a":     f"{_HOSPITAL}/SM_SupplyCart_02a.usd",
    "SM_SupplyCart_03a":     f"{_HOSPITAL}/SM_SupplyCart_03a.usd",
    "SM_Desk_04a":           f"{_HOSPITAL}/SM_Desk_04a.usd",
    "SM_Chair_04a":          f"{_HOSPITAL}/SM_Chair_04a.usd",
    "SM_Plant01":            f"{_OFFICE}/SM_Plant01.usd",
    "SM_Plant02":            f"{_OFFICE}/SM_Plant02.usd",
    "SM_CoffeeToGo":         f"{_OFFICE}/SM_CoffeeToGo.usd",
    "SM_Lamp02":             f"{_OFFICE}/SM_Lamp02.usd",
    "SM_BoxPortableC":       f"{_OFFICE}/SM_BoxPortableC.usd",
    "SM_CratePlastic_D_01":  f"{_WAREHOUSE}/SM_CratePlastic_D_01.usd",
    "RidgebackUr":           "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Robots/Clearpath/RidgebackUr/ridgeback_ur5.usd",
}

TEMPLATE_ROOM_USD = "/World/Environment"
