# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Viewport camera that follows the robot (chase view).

A plain USD camera (no camera sensor, so it costs nothing unless the viewport looks through it) placed behind
and above the robot root. It follows the robot position and heading (yaw only - pitch / roll and the bobbing of
walking are not followed, which keeps the view steady); the heading is low-pass filtered.
"""

from __future__ import annotations

import math

FOLLOW_CAMERA_PATH = "/World/FollowCamera"


class FollowCamera:
    def __init__(self, env, asset_name: str = "robot", prim_path: str = FOLLOW_CAMERA_PATH,
                 distance: float = 2.2, height: float = 1.6, look_height: float = 0.9, look_ahead: float = 0.4,
                 focal_length: float = 16.5, yaw_smoothing: float = 0.15):
        import omni.usd
        from pxr import Gf, UsdGeom

        self._env = env
        self._robot = env.scene[asset_name]
        self._distance, self._height = distance, height
        self._look_height, self._look_ahead = look_height, look_ahead
        self._alpha = yaw_smoothing
        self._yaw = None
        self._last = None
        stage = omni.usd.get_context().get_stage()
        cam = UsdGeom.Camera.Define(stage, prim_path)
        cam.CreateFocalLengthAttr(focal_length)
        cam.CreateHorizontalApertureAttr(20.955)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 1.0e5))
        self._op = UsdGeom.Xformable(cam.GetPrim()).MakeMatrixXform()
        self.path = prim_path
        self.update(force=True)

    def _root_pose(self):
        d = self._robot.data
        pos = d.root_pos_w.torch[0] if hasattr(d.root_pos_w, "torch") else d.root_pos_w[0]
        quat = d.root_quat_w.torch[0] if hasattr(d.root_quat_w, "torch") else d.root_quat_w[0]
        x, y, z, w = (float(v) for v in quat)  # xyzw
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return float(pos[0]), float(pos[1]), yaw

    def update(self, force: bool = False) -> None:
        from pxr import Gf

        px, py, yaw = self._root_pose()
        if self._yaw is None or force:
            self._yaw = yaw
        else:  # low-pass the heading (shortest angle)
            self._yaw += self._alpha * math.atan2(math.sin(yaw - self._yaw), math.cos(yaw - self._yaw))
        key = (round(px, 3), round(py, 3), round(self._yaw, 3))
        if key == self._last and not force:
            return  # robot did not move: no USD write
        self._last = key
        c, s = math.cos(self._yaw), math.sin(self._yaw)
        eye = Gf.Vec3d(px - self._distance * c, py - self._distance * s, self._height)
        target = Gf.Vec3d(px + self._look_ahead * c, py + self._look_ahead * s, self._look_height)
        view = Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0, 0, 1))  # world -> camera (USD / OpenGL camera)
        self._op.Set(view.GetInverse())
