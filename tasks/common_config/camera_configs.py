# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""
public camera configuration
include the basic configuration for different types of cameras, support scene-specific parameter customization
"""

import isaaclab.sim as sim_utils
from isaaclab.sensors import CameraCfg
from isaaclab.utils import configclass
import os

from .camera_calibration import log_once, compose_pose, get_handeye, get_intrinsics, get_stereo_extrinsic, monotonic_radial_fit

# ---------------------------------------------------------------------------
# G1 cameras: the real-robot calibration is used where it is usable, otherwise the robot asset (USD)
# defines the camera (see camera_calibration.py for what "usable" means):
#   head stereo : left eye = the G1 head camera of the asset (d435_link, original sim pose); right eye =
#                 left eye + calibrated head_stereo extrinsic (else a parallel 60 mm baseline).
#                 Intrinsics: calibration head_left / head_right (else the default pinhole camera).
#   wrist       : calibrated hand-eye pose if it was calibrated with this robot asset, else the wrist
#                 camera mount of the asset (hand-specific offsets below). Intrinsics: left/right_wrist.
# ---------------------------------------------------------------------------
G1_HEAD_CAMERA_PARENT = "d435_link"
G1_HEAD_LEFT_POS = (0.0, 0.0, 0.0)
G1_DEFAULT_STEREO_BASELINE = 0.060  # m, used only if the calibration has no usable head_stereo extrinsic
# ROS camera frame (x right, y down, z forward) in a link frame with x forward / z up
_ROS_CAM_IN_XFWD_LINK = (-0.5, 0.5, -0.5, 0.5)
G1_HEAD_LEFT_ROT = _ROS_CAM_IN_XFWD_LINK
_ROBOT_PRIM = "/World/envs/env_.*/Robot"


def _g1_robot_asset(hand: str) -> str:
    """USD of the fixed-base G1 with the given hand (the hand-eye calibration must name this robot)."""
    from robots.unitree import G129_CFG_WITH_DEX1_BASE_FIX, G129_CFG_WITH_DEX3_BASE_FIX, G129_CFG_WITH_INSPIRE_HAND

    cfg = {"gripper": G129_CFG_WITH_DEX1_BASE_FIX, "dex3": G129_CFG_WITH_DEX3_BASE_FIX, "inspire": G129_CFG_WITH_INSPIRE_HAND}[hand]
    return cfg.spawn.usd_path


def _head_stereo_poses(left_pos, left_rot):
    """(left_pose, right_pose) of the head eyes in the head camera link; each pose is (pos, quat xyzw), ROS frame.
    The right eye is the left eye + the calibrated head_stereo extrinsic (else a parallel 60 mm baseline)."""
    rel = get_stereo_extrinsic("head_right")
    if rel is None:
        log_once(f"[camera_configs] no usable head_stereo extrinsic, using a parallel {G1_DEFAULT_STEREO_BASELINE * 1000:.0f} mm baseline")
        rel = ((G1_DEFAULT_STEREO_BASELINE, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    left = (left_pos, left_rot)
    return left, compose_pose(*left, *rel)


def _g1_head_stereo_poses():
    """(left_pose, right_pose) of the G1 head eyes in ``d435_link``."""
    return _head_stereo_poses(G1_HEAD_LEFT_POS, G1_HEAD_LEFT_ROT)


# H1-2: same scheme as the G1; the head camera of the asset is ``camera_link`` (x forward), the left eye keeps
# the original sim pose of the H1-2 front camera.
H12_HEAD_CAMERA_PARENT = "camera_link"
H12_HEAD_LEFT_POS = (0.0, 0.0, 0.0)
H12_HEAD_LEFT_ROT = _ROS_CAM_IN_XFWD_LINK


def _h12_robot_asset() -> str:
    from robots.unitree import H12_CFG_WITH_INSPIRE_HAND

    return H12_CFG_WITH_INSPIRE_HAND.spawn.usd_path


def calibrated_pinhole_spawn(
    calib_name: str,
    focus_distance: float = 400.0,
    clipping_range: tuple = (0.1, 1.0e5),
    horizontal_aperture: float = 20.955,
) -> tuple[sim_utils.PinholeCameraCfg, int, int]:
    """Pinhole camera spawn cfg reproducing a real camera's calibrated intrinsics and lens distortion.

    The OpenCV intrinsics (fx, fy, cx, cy) and distortion coefficients are authored as the RTX
    ``omni:lensdistortion`` API, which takes precedence over the focal-length / aperture projection.
    The focal length / aperture are still set consistently (square pixels, centered principal point)
    so that viewport previews and ``CameraData.intrinsic_matrices`` are close to the calibration.

    Returns:
        (spawn cfg, height, width) - the image size the calibration was done at,
        or None if the calibration file has no intrinsics for ``calib_name``.
    """
    intr = get_intrinsics(calib_name)
    if intr is None:
        return None
    width, height = intr["width"], intr["height"]
    f_avg = 0.5 * (intr["fx"] + intr["fy"])
    common = dict(fx=intr["fx"], fy=intr["fy"], cx=intr["cx"], cy=intr["cy"], image_size=(width, height))
    model = intr["camera_model"].lower()
    radial_fit = monotonic_radial_fit(intr) if model == "pinhole" else None
    if radial_fit is not None:
        # the calibrated radial polynomial folds back before the image corners: keep the pinhole model
        # (fx, fy, cx, cy, p1, p2) with the radial part re-fitted in the monotonic rational form
        log_once(
            f"[camera_configs] {calib_name}: radial distortion polynomial folds inside the image; using the rational "
            f"pinhole form (max deviation {radial_fit['max_error_px']:.3f} px for r_d <= 0.7)"
        )
        coeffs = {**{k: radial_fit[k] for k in ("k1", "k2", "k3", "k4", "k5", "k6")}, "p1": float(intr.get("p1", 0.0)), "p2": float(intr.get("p2", 0.0))}
        distortion = sim_utils.OpenCvPinholeDistortionCfg(**common, **coeffs)
    elif model == "pinhole":
        distortion = sim_utils.OpenCvPinholeDistortionCfg(
            **common, **{k: float(intr.get(k, 0.0)) for k in ("k1", "k2", "k3", "k4", "k5", "k6", "p1", "p2")}
        )
    elif model == "fisheye":
        distortion = sim_utils.OpenCvFisheyeDistortionCfg(**common, **{k: float(intr.get(k, 0.0)) for k in ("k1", "k2", "k3", "k4")})
    else:
        raise ValueError(f"Unsupported camera_model '{intr['camera_model']}' for calibrated camera '{calib_name}'")

    spawn = sim_utils.PinholeCameraCfg(
        focal_length=f_avg * horizontal_aperture / width,
        focus_distance=focus_distance,
        horizontal_aperture=horizontal_aperture,
        vertical_aperture=horizontal_aperture * height / width,
        clipping_range=clipping_range,
        distortion=distortion,
    )
    return spawn, height, width


@configclass
class CameraBaseCfg:
    """camera base configuration class
    
    provide the default configuration for different types of cameras, support scene-specific parameter customization
    """
    
    @classmethod
    def get_camera_config(
        cls,
        prim_path: str = "/World/envs/env_.*/Robot/d435_link/front_cam",
        update_period: float = 0.02,
        height: int = 480,
        width: int =  640,
        focal_length: float = 7.6,
        focus_distance: float = 400.0,
        horizontal_aperture: float = 20.0,
        clipping_range: tuple = (0.1, 1.0e5),
        pos_offset: tuple = (0, 0.0, 0),
        rot_offset: tuple = (-0.5, 0.5, -0.5, 0.5),
        data_types: list = None,
        calib_name: str | None = None,
    ) -> CameraCfg:
        """get the front camera configuration
        
        Args:
            prim_path: the path of the camera in the scene
            update_period: update period (seconds)
            height: image height (pixels)
            width: image width (pixels)
            focal_length: focal length
            focus_distance: focus distance
            horizontal_aperture: horizontal aperture
            clipping_range: clipping range (near clipping plane, far clipping plane)
            pos_offset: position offset (x, y, z)
            rot_offset: rotation offset quaternion
            data_types: data type list
            calib_name: name of a camera in the real-robot calibration file (see camera_calibration.py).
                If given, its calibrated intrinsics, lens distortion and image size replace
                height / width / focal_length / horizontal_aperture.

        Returns:
            CameraCfg: camera configuration
        """
        if data_types is None:
            data_types = ["rgb"]

        calibrated = None
        if calib_name is not None:
            calibrated = calibrated_pinhole_spawn(calib_name, focus_distance=focus_distance, clipping_range=clipping_range)
        if calibrated is not None:
            spawn, height, width = calibrated
        else:
            spawn = sim_utils.PinholeCameraCfg(
                focal_length=focal_length,
                focus_distance=focus_distance,
                horizontal_aperture=horizontal_aperture,
                clipping_range=clipping_range
            )

        return CameraCfg(
            prim_path=prim_path,
            update_period=update_period,
            height=height,
            width=width,
            data_types=data_types,
            spawn=spawn,
            offset=CameraCfg.OffsetCfg(
                pos=pos_offset,
                rot=rot_offset,
                convention="ros"
            )
        )
    



@configclass
class CameraPresets:
    """camera preset configuration collection
    
    include the common camera configuration preset for different scenes
    """
    
    @classmethod
    def g1_front_camera(cls) -> CameraCfg:
        """head stereo LEFT eye (kept as ``front_camera`` / image ``head`` for compatibility).

        Mount: the G1 head camera of the asset (d435_link). Intrinsics: calibration ``head_left`` if usable.
        """
        (pos, rot), _ = _g1_head_stereo_poses()
        return CameraBaseCfg.get_camera_config(
            prim_path=f"{_ROBOT_PRIM}/{G1_HEAD_CAMERA_PARENT}/front_cam",
            pos_offset=pos, rot_offset=rot, calib_name="head_left",
        )

    @classmethod
    def g1_front_right_camera(cls) -> CameraCfg:
        """head stereo RIGHT eye: left eye + calibrated stereo extrinsic (else parallel 60 mm baseline).
        Intrinsics: calibration ``head_right`` if usable."""
        _, (pos, rot) = _g1_head_stereo_poses()
        return CameraBaseCfg.get_camera_config(
            prim_path=f"{_ROBOT_PRIM}/{G1_HEAD_CAMERA_PARENT}/front_right_cam",
            pos_offset=pos, rot_offset=rot, calib_name="head_right",
        )

    @classmethod
    def g1_calibrated_wrist_camera(cls, side: str, fallback: CameraCfg, hand: str) -> CameraCfg:
        """G1 wrist camera: the calibrated ``<side>_arm`` hand-eye pose if it is usable for this robot
        (``hand``: "gripper" / "dex3" / "inspire"), else the asset mount given by ``fallback``;
        ``<side>_wrist`` intrinsics if usable, else those of ``fallback``."""
        return cls.calibrated_wrist_camera(side, fallback, _g1_robot_asset(hand), robot_name="G1")

    @classmethod
    def calibrated_wrist_camera(cls, side: str, fallback: CameraCfg, asset: str, robot_name: str) -> CameraCfg:
        """Wrist camera of the robot ``asset``: the calibrated ``<side>_arm`` hand-eye pose if it was calibrated
        with this asset, else the asset mount given by ``fallback``; ``<side>_wrist`` intrinsics if usable."""
        from robots.robot_assets import robot_link_prim_path

        cfg = fallback
        he = get_handeye(f"{side}_arm", robot_asset=asset)
        if he is not None:
            link, pos, rot = he
            cfg = cfg.replace(
                prim_path=f"{_ROBOT_PRIM}/{robot_link_prim_path(asset, link)}/{side}_wrist_camera",
                offset=CameraCfg.OffsetCfg(pos=pos, rot=rot, convention="ros"),
            )
            log_once(f"[camera_configs] {robot_name} {side} wrist camera: hand-eye calibration ({link})")
        else:
            log_once(f"[camera_configs] {robot_name} {side} wrist camera: robot asset mount ({cfg.prim_path.split('/')[-2]})")
        calibrated = calibrated_pinhole_spawn(f"{side}_wrist", focus_distance=400.0, clipping_range=(0.1, 1.0e5))
        if calibrated is not None:
            spawn, height, width = calibrated
            cfg = cfg.replace(spawn=spawn, height=height, width=width)
        return cfg

    @classmethod
    def h12_front_camera(cls) -> CameraCfg:
        """H1-2 head stereo LEFT eye (``front_camera`` / image ``head``): the original front camera pose in
        ``camera_link``; intrinsics: calibration ``head_left`` if usable."""
        (pos, rot), _ = _head_stereo_poses(H12_HEAD_LEFT_POS, H12_HEAD_LEFT_ROT)
        return CameraBaseCfg.get_camera_config(
            prim_path=f"{_ROBOT_PRIM}/{H12_HEAD_CAMERA_PARENT}/front_cam",
            pos_offset=pos, rot_offset=rot, calib_name="head_left",
        )

    @classmethod
    def h12_front_right_camera(cls) -> CameraCfg:
        """H1-2 head stereo RIGHT eye: left eye + calibrated stereo extrinsic (else parallel 60 mm baseline);
        intrinsics: calibration ``head_right`` if usable."""
        _, (pos, rot) = _head_stereo_poses(H12_HEAD_LEFT_POS, H12_HEAD_LEFT_ROT)
        return CameraBaseCfg.get_camera_config(
            prim_path=f"{_ROBOT_PRIM}/{H12_HEAD_CAMERA_PARENT}/front_right_cam",
            pos_offset=pos, rot_offset=rot, calib_name="head_right",
        )

    @classmethod
    def h12_left_inspire_wrist_camera(cls) -> CameraCfg:
        """H1-2 left wrist camera: hand-eye calibration if made with the H1-2 asset, else the asset mount;
        calibrated ``left_wrist`` intrinsics if usable."""
        return cls.calibrated_wrist_camera("left", cls.left_inspire_wrist_camera(), _h12_robot_asset(), robot_name="H1-2")

    @classmethod
    def h12_right_inspire_wrist_camera(cls) -> CameraCfg:
        """H1-2 right wrist camera (see h12_left_inspire_wrist_camera)."""
        return cls.calibrated_wrist_camera("right", cls.right_inspire_wrist_camera(), _h12_robot_asset(), robot_name="H1-2")
    @classmethod
    def g1_world_camera(cls) -> CameraCfg:
        """front camera configuration"""
        return CameraBaseCfg.get_camera_config(prim_path="/World/envs/env_.*/Robot/d435_link/PerspectiveCamera_robot",
                                                    pos_offset=(-0.9, 0.0, 0.0),
                                                    rot_offset=(0.51292, -0.48674, 0.48674, -0.51292),
                                                    focal_length = 12,
                                                    horizontal_aperture=27)
    @classmethod
    def h12_world_camera(cls) -> CameraCfg:
        """front camera configuration"""
        return CameraBaseCfg.get_camera_config(prim_path="/World/envs/env_.*/Robot/camera_link/PerspectiveCamera_robot",
                                                    pos_offset=(-0.9, 0.0, 0.0),
                                                    rot_offset=(0.51292, -0.48674, 0.48674, -0.51292),
                                                    focal_length = 12,
                                                    horizontal_aperture=27)
    @classmethod
    def left_gripper_wrist_camera(cls) -> CameraCfg:
        """left wrist camera (G1): real calibration intrinsics + hand-eye pose"""
        return CameraPresets.g1_calibrated_wrist_camera("left", CameraBaseCfg.get_camera_config(
            prim_path="/World/envs/env_.*/Robot/left_hand_base_link/left_wrist_camera",
            height=480,
            width=640,
            update_period=0.02,
            data_types=["rgb"],
            focal_length=12,
            focus_distance=400.0,
            horizontal_aperture=20.0,
            clipping_range=(0.1, 1.0e5),
            pos_offset=(0.02541028, 0.045, 0.135),
            rot_offset=(0.93969, 0, 0, -0.34202),
        ), hand="gripper")
    @classmethod
    def right_gripper_wrist_camera(cls) -> CameraCfg:
        """right wrist camera (G1): real calibration intrinsics + hand-eye pose"""
        return CameraPresets.g1_calibrated_wrist_camera("right", CameraBaseCfg.get_camera_config(
            prim_path="/World/envs/env_.*/Robot/right_hand_base_link/right_wrist_camera",
            height=480,
            width=640,
            update_period=0.02,
            data_types=["rgb"],
            focal_length=12,
            focus_distance=400.0,
            horizontal_aperture=20.0,
            clipping_range=(0.1, 1.0e5),
            pos_offset=(-0.02541028, 0.045, 0.135),
            rot_offset=(0.93969, 0, 0, -0.34202),
        ), hand="gripper") 
    @classmethod
    def left_dex3_wrist_camera(cls) -> CameraCfg:
        """left wrist camera (G1): real calibration intrinsics + hand-eye pose"""
        return CameraPresets.g1_calibrated_wrist_camera("left", CameraBaseCfg.get_camera_config(
            prim_path="/World/envs/env_.*/Robot/left_hand_camera_base_link/left_wrist_camera",
            height=480,
            width=640,
            update_period=0.02,
            data_types=["rgb"],
            focal_length=12.0,
            focus_distance=400.0,
            horizontal_aperture=20.0,
            clipping_range=(0.1, 1.0e5),
            pos_offset=(-0.04012, -0.07441 ,0.15711),
            rot_offset=(0.86024, 0.0424, 0.50809, 0.00539),
        ), hand="dex3")
    @classmethod
    def right_dex3_wrist_camera(cls) -> CameraCfg:
        """right wrist camera (G1): real calibration intrinsics + hand-eye pose"""
        return CameraPresets.g1_calibrated_wrist_camera("right", CameraBaseCfg.get_camera_config(
            prim_path="/World/envs/env_.*/Robot/right_hand_camera_base_link/right_wrist_camera",
            height=480,
            width=640,
            update_period=0.02,
            data_types=["rgb"],
            focal_length=12.0,
            focus_distance=400.0,
            horizontal_aperture=20.0,
            clipping_range=(0.1, 1.0e5),
            pos_offset=(-0.04012, 0.07441 ,0.15711),
            rot_offset=(0.86024, 0.0424, 0.50809, 0.00539),
        ), hand="dex3") 
    
    @classmethod
    def left_inspire_wrist_camera(cls, calibrated: bool = False) -> CameraCfg:
        """left wrist camera configuration (calibrated=True: G1, use the real left_wrist intrinsics + hand-eye pose)"""
        cfg = CameraBaseCfg.get_camera_config(
            prim_path="/World/envs/env_.*/Robot/left_hand_camera_base_link/left_wrist_camera",
            height=480,
            width=640,
            update_period=0.02,
            data_types=["rgb"],
            focal_length=12.0,
            focus_distance=400.0,
            horizontal_aperture=20.0,
            clipping_range=(0.1, 1.0e5),
            pos_offset=(-0.04012, -0.07441 ,0.15711),
            rot_offset=(0.86024, 0.0424, 0.50809, 0.00539),
        )
        return CameraPresets.g1_calibrated_wrist_camera("left", cfg, hand="inspire") if calibrated else cfg
    @classmethod
    def right_inspire_wrist_camera(cls, calibrated: bool = False) -> CameraCfg:
        """right wrist camera configuration (calibrated=True: G1, use the real right_wrist intrinsics + hand-eye pose)"""
        cfg = CameraBaseCfg.get_camera_config(
            prim_path="/World/envs/env_.*/Robot/right_hand_camera_base_link/right_wrist_camera",
            height=480,
            width=640,
            update_period=0.02,
            data_types=["rgb"],
            focal_length=12.0,
            focus_distance=400.0,
            horizontal_aperture=20.0,
            clipping_range=(0.1, 1.0e5),
            pos_offset=(-0.04012, 0.07441 ,0.15711),
            rot_offset=(0.86024, 0.0424, 0.50809, 0.00539),
        )
        return CameraPresets.g1_calibrated_wrist_camera("right", cfg, hand="inspire") if calibrated else cfg 