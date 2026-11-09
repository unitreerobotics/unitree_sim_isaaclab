# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
Camera presets for the Unitree R1-D (calibration where usable, otherwise the robot asset).

* head stereo: the R1-D URDF has no head camera link, so the pair is attached to ``head_yaw_Link`` at an
  ESTIMATED position from the head mesh: lenses on the face at x = 0.085 m, z = 0, centered (left eye at
  +baseline/2), looking straight forward (the head_pitch / head_yaw joints tilt it). Right eye = left eye
  + calibrated head_stereo extrinsic (else a parallel 60 mm baseline). Replace R1D_HEAD_LEFT_* once the
  real mount is known. Intrinsics: calibration head_left / head_right if usable.
* wrist: hand-eye result if it was calibrated with this robot (``handeye.<arm>.urdf.name`` ==
  r1_d_with_dex1_1), else the asset's ``<side>_dex1_camera_link`` (on top of the gripper, 30 deg down).
  Intrinsics: left_wrist / right_wrist if usable.
"""

from __future__ import annotations

from isaaclab.sensors import CameraCfg
from isaaclab.utils import configclass

from robots.robot_assets import robot_link_prim_path
from robots.unitree_r1d import R1D_DEX1_ASSET

from .camera_calibration import log_once, compose_pose, get_handeye, get_stereo_extrinsic, rotmat_to_quat_xyzw
from .camera_configs import CameraBaseCfg

R1D_HEAD_CAMERA_LINK = "head_yaw_Link"
R1D_HEAD_CENTER = (0.085, 0.0, 0.0)  # estimated stereo-pair center in head_yaw_Link (x forward, z up)
R1D_DEFAULT_STEREO_BASELINE = 0.060
_ROS_CAM_IN_XFWD_LINK = (-0.5, 0.5, -0.5, 0.5)
_ROS_CAM_IN_DEX1_CAMERA_LINK = rotmat_to_quat_xyzw([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]])
_DEX1_CAMERA_LENS_OFFSET = (0.0024, 0.0, 0.0)
_WRIST_CLIPPING_RANGE = (0.01, 1.0e5)
_ROBOT_PRIM = "/World/envs/env_.*/Robot"


def _link_path(link: str) -> str:
    return f"{_ROBOT_PRIM}/{robot_link_prim_path(R1D_DEX1_ASSET, link)}"


def _head_stereo_poses():
    rel = get_stereo_extrinsic("head_right")
    if rel is None:
        log_once(f"[r1d_camera_configs] no usable head_stereo extrinsic, using a parallel {R1D_DEFAULT_STEREO_BASELINE * 1000:.0f} mm baseline")
        rel = ((R1D_DEFAULT_STEREO_BASELINE, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    x, y, z = R1D_HEAD_CENTER
    left = ((x, y + 0.5 * rel[0][0], z), _ROS_CAM_IN_XFWD_LINK)
    return left, compose_pose(*left, *rel)


@configclass
class R1DCameraPresets:
    @classmethod
    def head_left_camera(cls) -> CameraCfg:
        """head stereo left eye (scene key ``front_camera`` -> image ``head``)."""
        (pos, rot), _ = _head_stereo_poses()
        return CameraBaseCfg.get_camera_config(
            prim_path=f"{_link_path(R1D_HEAD_CAMERA_LINK)}/front_cam", pos_offset=pos, rot_offset=rot, calib_name="head_left"
        )

    @classmethod
    def head_right_camera(cls) -> CameraCfg:
        """head stereo right eye (scene key ``front_right_camera`` -> image ``head_right``)."""
        _, (pos, rot) = _head_stereo_poses()
        return CameraBaseCfg.get_camera_config(
            prim_path=f"{_link_path(R1D_HEAD_CAMERA_LINK)}/front_right_cam", pos_offset=pos, rot_offset=rot, calib_name="head_right"
        )

    @classmethod
    def wrist_camera(cls, side: str) -> CameraCfg:
        he = get_handeye(f"{side}_arm", robot_asset=R1D_DEX1_ASSET)
        if he is not None:
            link, pos, rot = he
            log_once(f"[r1d_camera_configs] {side} wrist camera: hand-eye calibration ({link})")
        else:
            link, pos, rot = f"{side}_dex1_camera_link", _DEX1_CAMERA_LENS_OFFSET, _ROS_CAM_IN_DEX1_CAMERA_LINK
            log_once(f"[r1d_camera_configs] {side} wrist camera: robot asset mount ({link})")
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
