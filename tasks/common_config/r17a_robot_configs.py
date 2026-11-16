# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Robot, mounting and camera presets for the Unitree R1-7A arm robots (robots/unitree_r17a.py).

Mountings (``mount``):

* "table":  the arm base flange stands flush on the table top, ``R17A_TABLE_EDGE_INSET`` behind the robot-side
  table edge (the 15 x 15 cm flange ends 5 mm inside the edge)
* "column": the arm base stands on a column (static cylinder from the floor) right in front of the robot-side table
  edge (``R17A_COLUMN_EDGE_GAP`` between column and table), its flange ``R17A_COLUMN_HEIGHT`` above the table top

``r17a_anchor`` turns the table edge into the mount point of either mounting, so every task can use both.

Cameras: ``front_camera`` (image ``head``) is fixed in the world behind the arms and looks at the table (the arm
has no head camera); the Dex1 robot also has ``left_wrist_camera`` at the gripper's camera link (as on the R1-A7-D).
"""

from __future__ import annotations

import math

import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.sensors import CameraCfg

from robots.robot_assets import robot_link_prim_path
from robots.unitree_r17a import R17A_ARMS, R17A_ASSET, R17A_CFG, R17A_COLUMN_RADIUS

from .camera_calibration import log_once, rotmat_to_quat_xyzw
from .camera_configs import CameraBaseCfg
from .r1a7d_camera_configs import _DEX1_CAMERA_LENS_OFFSET, _ROS_CAM_IN_DEX1_CAMERA_LINK, _WRIST_CLIPPING_RANGE

R17A_BASE_BOTTOM = 0.026  # [m] the arm base flange reaches this far below the base_link origin (base_link.STL)
R17A_BASE_HALF = 0.075  # [m] half width of the square arm base flange (base_link.STL: 0.15 x 0.15 m)
R17A_MOUNTS = ("table", "column")
R17A_TABLE_EDGE_INSET = R17A_BASE_HALF + 0.005  # [m] table mounting: flange center behind the table edge
R17A_COLUMN_EDGE_GAP = 0.01  # [m] column mounting: free space between the column and the table edge
R17A_COLUMN_HEIGHT = 0.15  # [m] column mounting: flange above the table top
_ROBOT_PRIM = "/World/envs/env_.*/Robot"


def r17a_anchor(mount: str, edge_point, yaw: float) -> tuple:
    """(x, y) of ``mount_link`` for a mounting at a straight table edge.

    ``edge_point``: (x, y) on the robot-side table edge where the arm goes, ``yaw``: heading of the robot, which
    faces the table (0 = the arms reach towards +x, perpendicular to the edge). Table: the flange sits on the table
    top just inside the edge; column: the column stands on the floor just in front of the edge.
    """
    if mount not in R17A_MOUNTS:
        raise ValueError(f"unknown mount '{mount}' ({' | '.join(R17A_MOUNTS)})")
    d = R17A_TABLE_EDGE_INSET if mount == "table" else -(R17A_COLUMN_RADIUS + R17A_COLUMN_EDGE_GAP)
    return (edge_point[0] + d * math.cos(yaw), edge_point[1] + d * math.sin(yaw))


def r17a_mount(hand: str, mount: str, anchor, yaw: float, surface_z: float, column_height: float = R17A_COLUMN_HEIGHT):
    """(pos, rot xyzw) of ``mount_link`` and the columns [(x, y, top z)] for a mounting.

    ``anchor``: (x, y) of the mount point (between the arms), ``yaw``: heading (0 = the arms reach towards +x),
    ``surface_z``: table top height.
    """
    if mount not in R17A_MOUNTS:
        raise ValueError(f"unknown mount '{mount}' ({' | '.join(R17A_MOUNTS)})")
    top = surface_z + (column_height if mount == "column" else 0.0)
    rot = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    c, s = math.cos(yaw), math.sin(yaw)
    columns = [(anchor[0] + c * x - s * y, anchor[1] + s * x + c * y, top) for x, y in R17A_ARMS[hand].values()] if mount == "column" else []
    return (anchor[0], anchor[1], top + R17A_BASE_BOTTOM), rot, columns


def r17a_robot(hand: str, pos, rot, prim_path: str = _ROBOT_PRIM) -> ArticulationCfg:
    base = R17A_CFG[hand]
    return base.replace(prim_path=prim_path, init_state=base.init_state.replace(pos=tuple(pos), rot=tuple(rot)))


def r17a_column(x: float, y: float, top: float, name: str) -> AssetBaseCfg:
    """Grey column (static collider) from the floor up to ``top``."""
    return AssetBaseCfg(
        prim_path=f"/World/envs/env_.*/{name}",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(x, y, top / 2), rot=(0.0, 0.0, 0.0, 1.0)),
        spawn=sim_utils.CylinderCfg(
            radius=R17A_COLUMN_RADIUS, height=top, axis="Z",
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.35, 0.36, 0.38), metallic=0.6, roughness=0.4),
        ),
    )


def look_at_ros(eye, target) -> tuple:
    """Quaternion (xyzw) of a ROS camera (x right, y down, z forward) at ``eye`` looking at ``target``."""
    f = np.asarray(target, float) - np.asarray(eye, float)
    f /= np.linalg.norm(f)
    r = np.cross(f, [0.0, 0.0, 1.0])
    r /= np.linalg.norm(r)
    d = np.cross(f, r)
    return rotmat_to_quat_xyzw(np.stack([r, d, f], axis=1).tolist())


def r17a_front_camera(eye, target) -> CameraCfg:
    """World-fixed camera (scene key ``front_camera`` -> image ``head``); calibrated head_left intrinsics if usable."""
    return CameraBaseCfg.get_camera_config(
        prim_path="/World/envs/env_.*/r17a_front_cam", pos_offset=tuple(eye), rot_offset=look_at_ros(eye, target), calib_name="head_left"
    )


def r17a_dex1_wrist_camera() -> CameraCfg:
    """Wrist camera of the Dex1 gripper (its camera link, same mount as on the R1-A7-D)."""
    link = "left_dex1_camera_link"
    log_once(f"[r17a_robot_configs] left wrist camera: robot asset mount ({link})")
    return CameraBaseCfg.get_camera_config(
        prim_path=f"{_ROBOT_PRIM}/{robot_link_prim_path(R17A_ASSET['dex1'], link)}/left_wrist_camera",
        pos_offset=_DEX1_CAMERA_LENS_OFFSET, rot_offset=_ROS_CAM_IN_DEX1_CAMERA_LINK, calib_name="left_wrist",
        clipping_range=_WRIST_CLIPPING_RANGE,
    )
