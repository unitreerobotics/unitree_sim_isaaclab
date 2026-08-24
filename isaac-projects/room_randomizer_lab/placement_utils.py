# placement_utils.py
# Geometry utilities for room randomization.
# Uses Oriented Bounding Boxes (OBB) with the Separating Axis Theorem
# for collision detection, and continuous zone sampling.

from __future__ import annotations

import math
from typing import List, Tuple

try:
    import torch
    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False

try:
    from .constants import (
        BBox,
        ROOM_X_MIN,
        ROOM_X_MAX,
        ROOM_Y_MIN,
        ROOM_Y_MAX,
        OBB_PLACEMENT_MARGIN,
    )
except ImportError:
    from constants import (
        BBox,
        ROOM_X_MIN,
        ROOM_X_MAX,
        ROOM_Y_MIN,
        ROOM_Y_MAX,
        OBB_PLACEMENT_MARGIN,
    )


# =====================================================================
# OBB representation: (cx, cy, half_w, half_d, yaw_rad)
# Stored as 5-element tuples for the pure-Python path,
# or (N, 5) tensors for the batched path.
# =====================================================================

OBB = Tuple[float, float, float, float, float]  # cx, cy, hw, hd, yaw


def make_obb(cx: float, cy: float, bbox: BBox, yaw_rad: float) -> OBB:
    """Create an OBB tuple from a centre position, bbox, and yaw."""
    return (cx, cy, bbox.half_w, bbox.half_d, yaw_rad)


# =====================================================================
# OBB corner computation
# =====================================================================

def obb_corners(cx: float, cy: float, hw: float, hd: float, yaw: float) -> List[Tuple[float, float]]:
    """Compute the 4 world-space corners of an oriented bounding box.

    Returns corners in order: (+w,+d), (-w,+d), (-w,-d), (+w,-d).
    """
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)

    # Local corners → world
    corners = []
    for sx, sy in [(hw, hd), (-hw, hd), (-hw, -hd), (hw, -hd)]:
        wx = cx + sx * cos_y - sy * sin_y
        wy = cy + sx * sin_y + sy * cos_y
        corners.append((wx, wy))
    return corners


# =====================================================================
# Separating Axis Theorem (SAT) for OBB-OBB overlap
# =====================================================================

def _project_corners_onto_axis(corners: List[Tuple[float, float]], axis: Tuple[float, float]) -> Tuple[float, float]:
    """Project 4 corners onto a 1D axis, return (min, max) scalar projection."""
    dots = [c[0] * axis[0] + c[1] * axis[1] for c in corners]
    return min(dots), max(dots)


def obb_overlap(a: OBB, b: OBB, margin: float = 0.0) -> bool:
    """Test whether two OBBs overlap using the Separating Axis Theorem.

    Args:
        a, b:   OBB tuples (cx, cy, half_w, half_d, yaw_rad).
        margin: extra clearance added to each box before testing.

    Returns:
        True if the (possibly inflated) boxes overlap.
    """
    # Inflate boxes by margin.
    a_inflated = (a[0], a[1], a[2] + margin, a[3] + margin, a[4])
    b_inflated = (b[0], b[1], b[2] + margin, b[3] + margin, b[4])

    corners_a = obb_corners(*a_inflated)
    corners_b = obb_corners(*b_inflated)

    # 4 candidate separating axes: 2 edge normals from each box.
    axes = []
    for box_yaw in (a[4], b[4]):
        cos_y = math.cos(box_yaw)
        sin_y = math.sin(box_yaw)
        axes.append((cos_y, sin_y))        # along local X
        axes.append((-sin_y, cos_y))       # along local Y

    for axis in axes:
        min_a, max_a = _project_corners_onto_axis(corners_a, axis)
        min_b, max_b = _project_corners_onto_axis(corners_b, axis)
        # If projections don't overlap on this axis → no collision.
        if max_a < min_b or max_b < min_a:
            return False

    # All axes overlapped → collision.
    return True


# =====================================================================
# Room bounds check (OBB corners must all be inside)
# =====================================================================

def obb_inside_room(box: OBB) -> bool:
    """Check that all 4 corners of an OBB are inside the room bounds."""
    for wx, wy in obb_corners(*box):
        if wx < ROOM_X_MIN or wx > ROOM_X_MAX:
            return False
        if wy < ROOM_Y_MIN or wy > ROOM_Y_MAX:
            return False
    return True


# =====================================================================
# Overlap-any check
# =====================================================================

def obb_overlap_any(candidate: OBB, placed: List[OBB], margin: float = OBB_PLACEMENT_MARGIN) -> bool:
    """Check if a candidate OBB overlaps any OBB in the placed list."""
    for p in placed:
        if obb_overlap(candidate, p, margin=margin):
            return True
    return False


# =====================================================================
# Quaternion / coordinate helpers (unchanged from before)
# =====================================================================

def yaw_to_quat(yaw_rad: torch.Tensor) -> torch.Tensor:
    """Convert Z-axis yaw angles to (w, x, y, z) quaternions.

    Args:
        yaw_rad: (N,) yaw angles in radians.

    Returns:
        (N, 4) quaternion tensor in (w, x, y, z) order.
    """
    half = yaw_rad * 0.5
    quat = torch.zeros(yaw_rad.shape[0], 4, device=yaw_rad.device, dtype=yaw_rad.dtype)
    quat[:, 0] = torch.cos(half)
    quat[:, 3] = torch.sin(half)
    return quat


def offset_from_yaw(
    origin_x: float,
    origin_y: float,
    yaw_rad: float,
    local_x: float,
    local_y: float,
) -> Tuple[float, float]:
    """Apply a local offset rotated by yaw around an origin (scalar version)."""
    cos_y = math.cos(yaw_rad)
    sin_y = math.sin(yaw_rad)
    return (
        origin_x + local_x * cos_y - local_y * sin_y,
        origin_y + local_x * sin_y + local_y * cos_y,
    )


def offset_from_yaw_batched(
    origin: torch.Tensor,
    yaw_rad: torch.Tensor,
    local_x: float,
    local_y: float,
    z: float,
) -> torch.Tensor:
    """Apply a local offset rotated by yaw around an origin (batched).

    Args:
        origin:  (N, 3) world-space origin positions.
        yaw_rad: (N,) yaw angles in radians.
        local_x, local_y: scalar local offset.
        z: scalar Z value.

    Returns:
        (N, 3) world-space positions.
    """
    cos_yaw = torch.cos(yaw_rad)
    sin_yaw = torch.sin(yaw_rad)

    result = torch.zeros_like(origin)
    result[:, 0] = origin[:, 0] + local_x * cos_yaw - local_y * sin_yaw
    result[:, 1] = origin[:, 1] + local_x * sin_yaw + local_y * cos_yaw
    result[:, 2] = z
    return result


def local_to_world_xy(
    desk_pos: torch.Tensor,
    desk_yaw_rad: torch.Tensor,
    local_xy: torch.Tensor,
) -> torch.Tensor:
    """Transform local desk-surface (x, y) to world (x, y).

    Args:
        desk_pos:     (N, 3) desk world position.
        desk_yaw_rad: (N,) desk yaw in radians.
        local_xy:     (N, 2) local coordinates on the desk surface.

    Returns:
        (N, 2) world-space XY positions.
    """
    cos_yaw = torch.cos(desk_yaw_rad)
    sin_yaw = torch.sin(desk_yaw_rad)

    world_x = desk_pos[:, 0] + local_xy[:, 0] * cos_yaw - local_xy[:, 1] * sin_yaw
    world_y = desk_pos[:, 1] + local_xy[:, 0] * sin_yaw + local_xy[:, 1] * cos_yaw

    return torch.stack([world_x, world_y], dim=-1)


def build_root_state(
    pos: torch.Tensor,
    yaw_rad: torch.Tensor,
    env_origins: torch.Tensor,
    env_ids: torch.Tensor,
    default_state: torch.Tensor,
) -> torch.Tensor:
    """Build a (len(env_ids), 13) root-state tensor for write_root_state_to_sim."""
    state = default_state[env_ids].clone()

    state[:, 0] = pos[:, 0] + env_origins[env_ids, 0]
    state[:, 1] = pos[:, 1] + env_origins[env_ids, 1]
    state[:, 2] = pos[:, 2] + env_origins[env_ids, 2]

    quat = yaw_to_quat(yaw_rad)
    state[:, 3:7] = quat
    state[:, 7:] = 0.0

    return state
