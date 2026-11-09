# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
Real-robot camera calibration (``uniclib_calibration`` format) for the simulated cameras.

The calibration file provides:

* ``intrinsics.<camera>``: OpenCV pinhole/fisheye intrinsics + distortion.
* ``extrinsics.head_stereo``: right eye relative to the left eye,
  ``p_head_left = R(rvec) * p_head_right + tvec`` (tvec in mm).
* ``handeye.<left_arm|right_arm>``: eye-in-hand transform of the wrist camera,
  ``p_<reference_link> = T_wrist_cam * p_cam`` (translation in m).

All camera frames are OpenCV frames (x right, y down, z forward), which is Isaac Lab's ``"ros"``
camera convention, so the transforms can be used directly as ``CameraCfg.OffsetCfg(convention="ros")``.

Which file is used (first match wins):

1. ``set_calibration_path(path)`` (``sim_main.py --camera_calib PATH`` calls this),
2. the ``UNITREE_CAMERA_CALIB`` environment variable,
3. ``robots/uniclib_calibration.json`` in this repository.
"""

from __future__ import annotations

import json
import math
import os
import warnings
from functools import lru_cache

_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_CALIB_PATH = os.path.join(_PROJECT_ROOT, "robots", "uniclib_calibration.json")


_logged: set[str] = set()


def log_once(msg: str) -> None:
    """Print a message only once per process (camera presets are evaluated once per task class)."""
    if msg not in _logged:
        _logged.add(msg)
        print(msg)


def set_calibration_path(path: str | None) -> None:
    """Select the calibration file. Must be called before the task configs are created."""
    if path:
        path = os.path.abspath(os.path.expanduser(path))
        if not os.path.isfile(path):
            raise FileNotFoundError(f"Camera calibration file not found: {path}")
        os.environ["UNITREE_CAMERA_CALIB"] = path


def calib_path() -> str:
    return os.environ.get("UNITREE_CAMERA_CALIB") or DEFAULT_CALIB_PATH


@lru_cache(maxsize=None)
def _load(path: str) -> dict:
    """The calibration file; ``{}`` (everything falls back to the robot asset) if it is missing or invalid."""
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        if data.get("format") != "uniclib_calibration":
            raise ValueError(f"unsupported format {data.get('format')!r}")
    except Exception as e:
        print(f"[camera_calibration] calibration not usable ({path}: {e}); cameras use the robot asset / defaults")
        return {}
    print(f"[camera_calibration] using {path}")
    return data


def load_calibration() -> dict:
    return _load(calib_path())


def _usable(entry) -> bool:
    return entry is not None and entry.get("usable", True) is not False


def get_intrinsics(name: str) -> dict | None:
    """``{"camera_model", "height", "width", "fx", "fy", "cx", "cy", <distortion>}``, or None if not usable."""
    cam = load_calibration().get("intrinsics", {}).get(name)
    if not _usable(cam):
        log_once(f"[camera_calibration] no usable intrinsics for '{name}'; using the default camera")
        return None
    height, width = cam["image_size"]  # [H, W]
    return {"camera_model": cam["camera_model"], "height": int(height), "width": int(width), **cam["intrinsic"]}


def get_stereo_extrinsic(name: str, group: str = "head_stereo"):
    """Pose of camera ``name`` in its stereo reference camera frame: (pos [m], quat xyzw), or None."""
    stereo = load_calibration().get("extrinsics", {}).get(group)
    if not _usable(stereo):
        return None
    for cam in stereo.get("cameras", []):
        if cam["name"] == name:
            return tuple(float(v) / 1000.0 for v in cam["tvec"]), rotvec_to_quat_xyzw(cam["rvec"])
    return None


def get_handeye(arm: str, robot_asset: str | None = None):
    """Eye-in-hand pose of the wrist camera: (reference_link, pos [m], quat xyzw), or None if not usable.

    ``arm`` is ``"left_arm"`` or ``"right_arm"``. The rotation is taken from the 4x4 ``T_wrist_cam``.
    If ``robot_asset`` (URDF / USD path of the simulated robot) is given, the result is only used if it
    was calibrated with that robot (``urdf.name``, compared without extension) and its reference link
    exists in the asset - a hand-eye result of another hand / robot would put the camera somewhere wrong.
    """
    he = load_calibration().get("handeye", {}).get(arm)
    if not _usable(he):
        return None
    if robot_asset is not None:
        calib_urdf = (he.get("urdf") or {}).get("name")
        asset_name = os.path.splitext(os.path.basename(robot_asset))[0]
        if not calib_urdf or os.path.splitext(calib_urdf)[0] != asset_name:
            log_once(f"[camera_calibration] hand-eye '{arm}' was calibrated with '{calib_urdf}', not "
                     f"'{os.path.basename(robot_asset)}': not used")
            return None
        try:
            from robots.robot_assets import robot_link_prim_path

            robot_link_prim_path(robot_asset, he["reference_link"])
        except Exception as e:
            log_once(f"[camera_calibration] hand-eye '{arm}': reference link not in the robot asset ({e}): not used")
            return None
    T = he["T_wrist_cam"]
    pos = (float(T[0][3]), float(T[1][3]), float(T[2][3]))
    quat = rotmat_to_quat_xyzw([row[:3] for row in T[:3]])
    if he.get("joint_offsets_rad"):
        # Real-robot encoder zero offsets; the simulator has exact joint zeros, so they are not applied.
        pass
    return he["reference_link"], pos, quat


def monotonic_radial_fit(intr: dict):
    """Re-fit a pinhole radial distortion polynomial that folds back inside the image (still pinhole).

    Strong barrel distortion (e.g. the wrist cameras, k1 ~ -0.4) calibrated with targets near the image
    center gives r_d(r) = r (1 + k1 r^2 + k2 r^4 + k3 r^6) whose maximum lies below the radius of the image
    corners: the polynomial is only valid inside the calibrated area and the renderer would show folded
    garbage in the corners. In that case the same OpenCV *pinhole* model is kept, but its radial part is
    re-fitted in the rational form r (1 + k1 r^2 + k2 r^4 + k3 r^6) / (1 + k4 r^2 + k5 r^4 + k6 r^6), which matches
    the calibration in the trustworthy region (r_d <= 0.7) and stays monotonic up to the corners.
    fx, fy, cx, cy and the tangential terms p1, p2 are unchanged.

    Returns:
        None if the calibrated polynomial is fine for the whole image, else
        {"k1".."k6", "max_error_px"} (the new radial coefficients).
    """
    import numpy as np

    k1, k2, k3 = (float(intr.get(k, 0.0)) for k in ("k1", "k2", "k3"))
    fx, fy, cx, cy, W, H = intr["fx"], intr["fy"], intr["cx"], intr["cy"], intr["width"], intr["height"]
    if any(float(intr.get(k, 0.0)) for k in ("k4", "k5", "k6")):
        return None  # already a rational model

    def rd(r):
        return r * (1 + k1 * r**2 + k2 * r**4 + k3 * r**6)

    rd_need = max(math.hypot((u - cx) / fx, (v - cy) / fy) for u, v in ((0, 0), (W, 0), (0, H), (W, H)))
    r = np.linspace(0.0, 3.0, 30001)
    falling = np.nonzero(np.diff(rd(r)) <= 0)[0]
    if len(falling) == 0 or rd(r[falling[0]]) >= rd_need:
        return None
    r_fold = r[falling[0]]
    r_ok = r[: falling[0]]
    rr = np.linspace(0.0, float(np.interp(0.7, rd(r_ok), r_ok)), 200)  # error check region (r_d <= 0.7)
    t = np.linspace(0.0, 6.0, 60001)
    best = None
    for frac in (0.6, 0.7, 0.8, 0.9):
        x = np.linspace(1e-4, frac * r_fold, 400)
        y = rd(x)
        # y (1 + a4 x^2 + a5 x^4 + a6 x^6) = x (1 + a1 x^2 + a2 x^4 + a3 x^6): linear least squares in a1..a6
        A = np.stack([x**3, x**5, x**7, -y * x**2, -y * x**4, -y * x**6], 1)
        for keep in ((0, 3), (0, 1, 3, 4), (0, 1, 2, 3), (0, 1, 2, 3, 4, 5)):
            a = np.zeros(6)
            a[list(keep)] = np.linalg.lstsq(A[:, keep], y - x, rcond=None)[0]
            num = t * (1 + a[0] * t**2 + a[1] * t**4 + a[2] * t**6)
            den = 1 + a[3] * t**2 + a[4] * t**4 + a[5] * t**6
            ok = den > 0
            yy = np.where(ok, num / np.where(ok, den, 1.0), -np.inf)
            if yy.max() < 1.02 * rd_need:
                continue
            reach = int(np.argmax(yy >= 1.02 * rd_need))
            if not (np.all(ok[: reach + 1]) and np.all(np.diff(yy[: reach + 1]) > 0)):
                continue
            f = rr * (1 + a[0] * rr**2 + a[1] * rr**4 + a[2] * rr**6) / (1 + a[3] * rr**2 + a[4] * rr**4 + a[5] * rr**6)
            err = float(np.max(np.abs(f - rd(rr))) * fx)
            if best is None or err < best[0]:
                best = (err, a.copy())
    if best is None:
        log_once(
            f"[camera_calibration] WARNING: radial distortion (k1={k1:.3f}, k2={k2:.3f}, k3={k3:.3f}) folds back at "
            f"r={r_fold:.2f} inside the image and no monotonic re-fit was found: the image corners will show artifacts"
        )
        return None
    err, a = best
    return {**{f"k{i + 1}": float(a[i]) for i in range(6)}, "max_error_px": err}


# ---------------------------------------------------------------------------
# quaternion helpers, (x, y, z, w) order as used by Isaac Lab 3.0
# ---------------------------------------------------------------------------
def rotvec_to_quat_xyzw(rvec) -> tuple[float, float, float, float]:
    rx, ry, rz = (float(v) for v in rvec)
    angle = math.sqrt(rx * rx + ry * ry + rz * rz)
    if angle < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    s = math.sin(angle / 2.0) / angle
    return (rx * s, ry * s, rz * s, math.cos(angle / 2.0))


def rpy_to_quat_xyzw(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """URDF ``rpy`` (fixed-axis X, then Y, then Z) to quaternion."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def rotmat_to_quat_xyzw(R) -> tuple[float, float, float, float]:
    (m00, m01, m02), (m10, m11, m12), (m20, m21, m22) = R
    tr = m00 + m11 + m22
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = ((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s)
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2
        q = (0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2
        q = ((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s)
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2
        q = ((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s)
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def quat_mul_xyzw(q1, q2) -> tuple[float, float, float, float]:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quat_rotate_xyzw(q, v) -> tuple[float, float, float]:
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def compose_pose(parent_pos, parent_quat_xyzw, child_pos, child_quat_xyzw):
    """T_parent * T_child, poses given as (pos, quat xyzw)."""
    rotated = quat_rotate_xyzw(parent_quat_xyzw, child_pos)
    pos = tuple(p + r for p, r in zip(parent_pos, rotated))
    quat = quat_mul_xyzw(parent_quat_xyzw, child_quat_xyzw)
    norm = math.sqrt(sum(c * c for c in quat))
    return pos, tuple(c / norm for c in quat)
