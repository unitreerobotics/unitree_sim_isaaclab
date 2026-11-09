# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
Camera presets for the Unitree G1-D (independent of the G1 presets in camera_configs.py).

Mounts come from the G1-D asset and the real-robot calibration (see camera_calibration.py):

* head stereo: attached to ``Body_Depth_Camera_Link`` of the G1-D URDF. The camera module mesh spans
  y in [-0.0709, 0.0059] of that link (front face at x = 0), i.e. the stereo pair is centered at
  y = -0.0325. The left eye sits half a calibrated baseline to the robot's left of that center, the
  right eye is placed with the calibrated head_stereo extrinsic. Intrinsics: head_left / head_right.
* wrist: the hand-eye result ``T_wrist_cam`` (relative to its ``reference_link``) is used if it is usable
  (present, ``usable`` and calibrated with this robot, see ``camera_calibration.get_handeye``). Otherwise -
  e.g. no calibration file, or a hand-eye result of a Dex3-hand G1-D (g1_d.urdf) while simulating the Dex1
  gripper - the camera is placed at the robot asset's (URDF / USD) ``<side>_dex1_camera_link``: on top of the gripper, looking 30 deg down at the
  fingers (optical axis = link +x, image up = link +y, lens 2.4 mm in front of the link origin).
  Intrinsics: left_wrist / right_wrist.

Link prim paths are resolved from the asset (URDF importer nests links), so the presets work for
the URDF and a converted USD alike.
"""

from __future__ import annotations

from isaaclab.sensors import CameraCfg
from isaaclab.utils import configclass

from robots.robot_assets import robot_link_prim_path
from robots.unitree_g1d import G1D_DEX1_ASSET

from .camera_calibration import log_once, compose_pose, get_handeye, get_stereo_extrinsic, rotmat_to_quat_xyzw
from .camera_configs import CameraBaseCfg

G1D_HEAD_CAMERA_LINK = "Body_Depth_Camera_Link"
G1D_HEAD_MODULE_CENTER_Y = -0.0325
G1D_DEFAULT_STEREO_BASELINE = 0.060
# ROS camera frame (x right, y down, z forward) in a link frame with x forward / z up
_ROS_CAM_IN_XFWD_LINK = (-0.5, 0.5, -0.5, 0.5)
_ROBOT_PRIM = "/World/envs/env_.*/Robot"
# ROS camera in <side>_dex1_camera_link: z (forward) = link x, y (down) = -link y, x (right) = link z
_ROS_CAM_IN_DEX1_CAMERA_LINK = rotmat_to_quat_xyzw([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]])
_DEX1_CAMERA_LENS_OFFSET = (0.0024, 0.0, 0.0)
# the gripper fingers are only ~9 cm in front of the wrist camera: near plane 1 cm (default 0.1 m clips them)
_WRIST_CLIPPING_RANGE = (0.01, 1.0e5)


def _link_path(link: str, asset: str = G1D_DEX1_ASSET) -> str:
    return f"{_ROBOT_PRIM}/{robot_link_prim_path(asset, link)}"


def _head_stereo_poses():
    """(left, right) eye poses in the head camera link; each (pos, quat xyzw), ROS camera frame."""
    rel = get_stereo_extrinsic("head_right")
    if rel is None:
        log_once(f"[g1d_camera_configs] no head_stereo extrinsic, using a parallel {G1D_DEFAULT_STEREO_BASELINE * 1000:.0f} mm baseline")
        rel = ((G1D_DEFAULT_STEREO_BASELINE, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    left = ((0.0, G1D_HEAD_MODULE_CENTER_Y + 0.5 * rel[0][0], 0.0), _ROS_CAM_IN_XFWD_LINK)
    return left, compose_pose(*left, *rel)


@configclass
class G1DCameraPresets:
    """Calibrated cameras of the G1-D."""

    @classmethod
    def head_left_camera(cls) -> CameraCfg:
        """head stereo left eye (scene key ``front_camera`` -> image ``head``)."""
        (pos, rot), _ = _head_stereo_poses()
        return CameraBaseCfg.get_camera_config(
            prim_path=f"{_link_path(G1D_HEAD_CAMERA_LINK)}/front_cam", pos_offset=pos, rot_offset=rot, calib_name="head_left"
        )

    @classmethod
    def head_right_camera(cls) -> CameraCfg:
        """head stereo right eye (scene key ``front_right_camera`` -> image ``head_right``)."""
        _, (pos, rot) = _head_stereo_poses()
        return CameraBaseCfg.get_camera_config(
            prim_path=f"{_link_path(G1D_HEAD_CAMERA_LINK)}/front_right_cam", pos_offset=pos, rot_offset=rot, calib_name="head_right"
        )

    @classmethod
    def wrist_camera(cls, side: str) -> CameraCfg:
        """``side`` = "left" / "right"."""
        he = get_handeye(f"{side}_arm", robot_asset=G1D_DEX1_ASSET)
        if he is not None:  # usable calibration of this robot
            link, pos, rot = he
            log_once(f"[g1d_camera_configs] {side} wrist camera: hand-eye calibration ({link})")
        else:  # camera link of the robot asset (URDF or USD)
            link, pos, rot = f"{side}_dex1_camera_link", _DEX1_CAMERA_LENS_OFFSET, _ROS_CAM_IN_DEX1_CAMERA_LINK
            log_once(f"[g1d_camera_configs] {side} wrist camera: robot asset mount ({link})")
        return CameraBaseCfg.get_camera_config(
            prim_path=f"{_link_path(link)}/{side}_wrist_camera", pos_offset=pos, rot_offset=rot, calib_name=f"{side}_wrist",
            clipping_range=_WRIST_CLIPPING_RANGE,
        )

    @classmethod
    def left_wrist_camera(cls) -> CameraCfg:
        return cls.wrist_camera("left")

    @classmethod
    def right_wrist_camera(cls) -> CameraCfg:
        return cls.wrist_camera("right")
