# constants.py
# Room placement constants for the hospital room environment.
# Uses oriented bounding boxes (OBB) and continuous wall zones.

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Dict, List, Tuple

# ============================================================
# Paths
# ============================================================

PROJECT_ROOT = os.environ.get("PROJECT_ROOT", os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
ROOM_SHELL_USD = os.path.expanduser(
    os.environ.get(
        "ROOM_RANDOMIZER_ROOM_SHELL_USD",
        os.path.join(PROJECT_ROOT, "isaac-projects", "new_base_room.usda"),
    )
)

# ============================================================
# Room geometry
# ============================================================

ROOM_X_MIN = -13.0
ROOM_X_MAX = -0.5
ROOM_Y_MIN = -11.25
ROOM_Y_MAX = -5.0
FLOOR_Z = 0.0

# Default stand positions
ROBOT_Z = 0.76         # pelvis default height for base-fixed G1
TABLE_Z = -0.2         # packing table Z position
DESK_OBJECT_Z = 0.84   # object height on desk

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
        fixed_coord=-10.75,  # prop center Y
        base_yaw=0.0,        # face into room (+Y direction)
    ),
    # Right wall: props slide along Y, fixed near right wall X.
    WallZone(
        wall="right",
        sample_min=-10.0,
        sample_max=-7.0,
        fixed_coord=-3.0,    # prop center X
        base_yaw=math.pi / 2,  # face into room (-X direction)
    ),
]

# Conservative fallback RoomShell wall footprints used when USD stage bounds
# are unavailable. At runtime, the Isaac Lab event prefers the authored wall
# geometry from the spawned RoomShell so edits to new_base_room.usda are used.
STATIC_ROOM_OBSTACLES: List[StaticRoomObstacle] = [
    StaticRoomObstacle(
        name="static_front_wall",
        center=(-7.5, -5.47),
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
        center=(-8.35, -10.95),
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

# Stricter usable area for the table group. Unlike ROOM_* these bounds model
# where the robot/table may operate, including virtual limits on open sides.
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
    """Robot/table placement mode that keeps the robot facing a room wall.

    If target_axis is "y", wall points are sampled as (sample, fixed_coord).
    If target_axis is "x", wall points are sampled as (fixed_coord, sample).
    """
    name: str
    wall: str
    target_axis: str
    fixed_coord: float
    sample_min: float
    sample_max: float
    yaw_center: float


ROBOT_FACING_LAYOUTS: List[RobotFacingLayout] = [
    # Robot sees the back wall, with the front wall behind it.
    RobotFacingLayout(
        name="face_back_wall",
        wall="back",
        target_axis="y",
        fixed_coord=BACK_WALL_LINE_Y,
        sample_min=BACK_WALL_TARGET_X_MIN,
        sample_max=BACK_WALL_TARGET_X_MAX,
        yaw_center=-math.pi / 2,
    ),
    # Robot sees the new partition wall at the high-X edge of the valid room.
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

# Keep this at zero for strict wall-facing placement. Increase slightly only if
# training needs a controlled yaw perturbation around the selected wall layout.
ROBOT_FACING_YAW_JITTER_RAD = 0.0

# ============================================================
# Packing Table geometry (cloned desk)
# ============================================================

# Surface bounds relative to packing table center
DESK_LOCAL_X_MIN = -0.85
DESK_LOCAL_X_MAX = 0.18
DESK_LOCAL_Y_MIN = -0.28
DESK_LOCAL_Y_MAX = 0.28
DESK_OBJECT_MARGIN = 0.03   # margin between tabletop OBBs
DESK_LAMP_LOCAL_X_RANGE = (-0.60, -0.35)
DESK_LAMP_LOCAL_Y_RANGE = (0.10, 0.14)
DESK_LAMP_LOCAL_YAW = 0.0
DESK_LAMP_Z = 0.82
TABLETOP_CUBE_PROP_NAMES = {"blue_cube", "yellow_cube"}
TABLETOP_CUBE_LOCAL_X_MIN = DESK_LOCAL_X_MIN
TABLETOP_CUBE_LOCAL_X_MAX = 0.18

# Regular pick/place local transforms relative to the packing table center.
# These match TableCylinderSceneCfg: robot=(-0.15, 0.0), table=(0.0, 0.55),
# object=(-0.35, 0.40) when the table yaw is zero.
ROBOT_ORBIT_OFFSET = (-0.15, -0.55)
OBJECT_TABLE_LOCAL_OFFSET = (-0.35, -0.15)

# Ridgeback poses are expressed in the randomized G1 frame: local X points
# forward and local Y points left.  The Ridgeback articulation root is rotated
# by -90 degrees from G1 so its planar X/Y joints retain their existing
# lateral/forward meaning for every randomized robot yaw.
RIDGEBACK_WAITING_ROBOT_LOCAL = (-1.70, 0.0)
RIDGEBACK_STAGING_ROBOT_LOCAL = (-1.10, 0.75)
RIDGEBACK_DELIVERY_ROBOT_LOCAL = (-0.70, 0.78)
RIDGEBACK_ROOT_YAW_OFFSET = -math.pi / 2
RIDGEBACK_PLANAR_YAW = math.pi / 2

# Clearpath Ridgeback is approximately 0.96 x 0.79 m.  The footprint includes
# a small allowance for the collision basket mounted to the chassis.
RIDGEBACK_BBOX = BBox(half_w=0.50, half_d=0.42)
RIDGEBACK_CORRIDOR_HALF_WIDTH = RIDGEBACK_BBOX.half_d
RIDGEBACK_GROUP_MARGIN = 0.0
# Height of the dark rectangular support directly under the Ridgeback crate.
# Edit this value to raise or lower that support and its parented container.
RIDGEBACK_CONTAINER_RISER_HEIGHT = 0.15

# Static hospital Ridgeback positions form a rear circular arc around G1's
# feet.  The distance that matters is the visible edge gap in
# Screenshot-2026-08-24_14-47-43: from G1's nearest foot to the Ridgeback's
# G1-facing tip, not root-to-root.  The 0.85 m root radius is therefore the
# 0.50 m Ridgeback half-length, 0.25 m G1 foot footprint, and 0.10 m gap.
# The platform's local +X axis (and therefore its crate) is turned toward G1,
# so every retained point has the same edge clearance.
RIDGEBACK_STATIC_G1_FOOT_TIP_RADIUS = 0.25
RIDGEBACK_STATIC_TIP_CLEARANCE_MAX = 0.03
RIDGEBACK_STATIC_ARC_RADIUS = (
    RIDGEBACK_STATIC_G1_FOOT_TIP_RADIUS
    + RIDGEBACK_BBOX.half_w
    + RIDGEBACK_STATIC_TIP_CLEARANCE_MAX
)
# Manual angle control: edit this list (degrees). Keep the values non-zero so
# the Ridgeback is never spawned directly behind G1.
RIDGEBACK_STATIC_ARC_ANGLES = tuple(
    math.radians(angle)
    for angle in (-25, -30, -35, -40, -45, -50, -55, -60, 25, 30, 35, 40, 45, 50, 55, 60)
)


def ridgeback_static_arc_pose(index: int) -> tuple[tuple[float, float], float]:
    """Return one rear-semicircle Ridgeback position and its G1-facing yaw.

    Local X points forward from G1 and local Y points to its left.  The arc is
    centred behind G1, while the returned yaw offset turns the platform's
    crate-facing local +X direction back toward G1.
    """
    angle = RIDGEBACK_STATIC_ARC_ANGLES[index % len(RIDGEBACK_STATIC_ARC_ANGLES)]
    local_xy = (
        -RIDGEBACK_STATIC_ARC_RADIUS * math.cos(angle),
        RIDGEBACK_STATIC_ARC_RADIUS * math.sin(angle),
    )
    return local_xy, -angle


def ridgeback_static_arc_tip_clearance(index: int) -> float:
    """Return the calibrated G1-feet-to-Ridgeback-tip clearance in metres."""
    local_xy, _ = ridgeback_static_arc_pose(index)
    return (
        math.hypot(*local_xy)
        - RIDGEBACK_STATIC_G1_FOOT_TIP_RADIUS
        - RIDGEBACK_BBOX.half_w
    )

# Despawn height
DESPAWN_Z = -100.0

# Margin added around every OBB during placement checks (metres)
OBB_PLACEMENT_MARGIN = 0.15

# ============================================================
# Wall prop metadata
# ============================================================

@dataclass(frozen=True)
class WallPropMeta:
    """Placement metadata for a wall prop."""
    usd_name: str
    bbox: BBox              # footprint in the object's local frame
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
        wall_offset=0,
        wall_offsets={"back": 0.431706, "right": 0.131706},
        yaw_offset=math.pi,
        allowed_walls=("back", "right"),
    ),
    "shelf_set": WallPropMeta(
        "SM_ShelfSet_01a",
        bbox=BBox(half_w=0.861, half_d=0.280),
        tall=True,
        wall_offset=0,
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

DESK_BBOX = BBox(half_w=1.10, half_d=0.65)
ROBOT_BBOX = BBox(half_w=0.25, half_d=0.25)
ROBOT_TABLE_MARGIN = 0.0

# ============================================================
# Tabletop object metadata
# ============================================================

@dataclass(frozen=True)
class TablePropMeta:
    """Placement metadata for a tabletop object."""
    bbox: BBox
    dynamic: bool = False
    mandatory: bool = False
    # Fixed object-frame orientation composed after the sampled world yaw.
    # This lets physics-ready assets whose authored vertical axis is not +Z
    # remain upright without changing placement OBB semantics.
    base_orientation_wxyz: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class TableReservedArea:
    """Table-local footprint occupied by built-in table geometry."""
    name: str
    center: Tuple[float, float]
    bbox: BBox
    yaw: float = 0.0


TABLE_RESERVED_AREAS: List[TableReservedArea] = [
    # Grey wire tray/container built into PackingTable.usd.
    # Local center and footprint come from the container_h20 notes in the
    # fixed hospital red-block task.
    TableReservedArea(
        name="container_h20",
        center=(0.625, -0.094),
        bbox=BBox(half_w=0.365, half_d=0.245),
    ),
]


TABLE_PROP_META: Dict[str, TablePropMeta] = {
    "coffee_cup":   TablePropMeta(bbox=BBox(half_w=0.043, half_d=0.043)),
    "cup_half_full": TablePropMeta(
        bbox=BBox(half_w=0.045000, half_d=0.059930), dynamic=True, mandatory=True
    ),
    "plastic_cup": TablePropMeta(
        bbox=BBox(half_w=0.035000, half_d=0.035000), dynamic=True, mandatory=True
    ),
    "felt_pen_pink": TablePropMeta(
        bbox=BBox(half_w=0.059126, half_d=0.007000), dynamic=True, mandatory=True
    ),
    "desk_lamp":    TablePropMeta(bbox=BBox(half_w=0.241, half_d=0.134)),
    "box_portable": TablePropMeta(bbox=BBox(half_w=0.195, half_d=0.145)),
    "blue_cube":    TablePropMeta(bbox=BBox(half_w=0.05, half_d=0.05), dynamic=True, mandatory=True),
    "yellow_cube":  TablePropMeta(bbox=BBox(half_w=0.05, half_d=0.05), dynamic=True, mandatory=True),
    # Measured authored XY extents for Shidan's hospital props.  All four are
    # required in the teleoperation scene; none is randomly omitted.
    "object":       TablePropMeta(bbox=BBox(half_w=0.040, half_d=0.040), dynamic=True, mandatory=True),
    "hand_sanitizer": TablePropMeta(
        bbox=BBox(half_w=0.045, half_d=0.035), dynamic=True, mandatory=True
    ),
    "medicine_bottle_a": TablePropMeta(
        bbox=BBox(half_w=0.040, half_d=0.040), dynamic=True, mandatory=True
    ),
    "medicine_bottle_b": TablePropMeta(
        bbox=BBox(half_w=0.040, half_d=0.040), dynamic=True, mandatory=True
    ),
    "gauze_box": TablePropMeta(
        bbox=BBox(half_w=0.085, half_d=0.055), dynamic=True, mandatory=True
    ),
    "specimen_cup": TablePropMeta(
        bbox=BBox(half_w=0.040, half_d=0.040), dynamic=True, mandatory=True
    ),
    "medical_bottle_a": TablePropMeta(
        bbox=BBox(half_w=0.024491, half_d=0.024491), dynamic=True, mandatory=True
    ),
    "medical_bottle_b": TablePropMeta(
        bbox=BBox(half_w=0.024491, half_d=0.024491), dynamic=True, mandatory=True
    ),
    "medical_bottle_c": TablePropMeta(
        bbox=BBox(half_w=0.024491, half_d=0.024490), dynamic=True, mandatory=True
    ),
    # Natural-scale screenshot props used by the two-pill hospital task.
    "pill_bottle_t": TablePropMeta(
        bbox=BBox(half_w=0.014611, half_d=0.014611), dynamic=True, mandatory=True
    ),
    "pill_bottle_v": TablePropMeta(
        bbox=BBox(half_w=0.013665, half_d=0.013665), dynamic=True, mandatory=True
    ),
    "medical_bottle_f": TablePropMeta(
        bbox=BBox(half_w=0.030050, half_d=0.030050), dynamic=True, mandatory=True
    ),
    "marker_blue": TablePropMeta(
        bbox=BBox(half_w=0.058500, half_d=0.012140), dynamic=True, mandatory=True
    ),
    "marker_yellow": TablePropMeta(
        bbox=BBox(half_w=0.058500, half_d=0.012140), dynamic=True, mandatory=True
    ),
    "mustard_bottle": TablePropMeta(
        bbox=BBox(half_w=0.048013, half_d=0.029125), dynamic=True, mandatory=True
    ),
    "cracker_box": TablePropMeta(
        bbox=BBox(half_w=0.082018, half_d=0.035900), dynamic=True, mandatory=True
    ),
    "tomato_soup_can": TablePropMeta(
        bbox=BBox(half_w=0.033830, half_d=0.033858), dynamic=True, mandatory=True
    ),
}

# ============================================================
# Asset USD paths (Omniverse S3 CDN)
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
}
