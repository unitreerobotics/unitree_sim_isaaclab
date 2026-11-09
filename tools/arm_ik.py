# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
Cartesian end-effector control for the G1 / G1-D arms (numpy only), used by the keyboard teleop scripts.

The arm kinematics (torso_link -> <side>_wrist_yaw_link, 7 revolute joints) are read from a URDF.
G1 29dof and G1-D have identical arms, so the G1-D URDF of this repository is used by default.
Poses are expressed in the torso frame (x forward, y left, z up); the tool center point (TCP) is a
point in front of the wrist (between the gripper fingers).
"""

from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET

import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_ARM_URDF = os.path.join(_REPO, "assets", "urdf", "g1_d_description", "g1_d_with_dex1_1.urdf")


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    x, y, z = axis
    c, s, C = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def _rot_error(R_target, R):
    """Rotation vector taking R to R_target (world frame)."""
    E = R_target @ R.T
    angle = math.acos(max(-1.0, min(1.0, (np.trace(E) - 1.0) / 2.0)))
    if angle < 1e-9:
        return np.zeros(3)
    v = np.array([E[2, 1] - E[1, 2], E[0, 2] - E[2, 0], E[1, 0] - E[0, 1]])
    return v * (angle / (2.0 * math.sin(angle))) if angle < math.pi - 1e-6 else v


class ArmKinematics:
    """FK / Jacobian / damped-least-squares IK of one arm."""

    def __init__(self, side: str, tcp_offset=(0.15, 0.0, 0.0), urdf: str = DEFAULT_ARM_URDF, base_link: str = "torso_link"):
        root = ET.parse(urdf).getroot()
        by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
        chain, link = [], f"{side}_wrist_yaw_link"
        while link != base_link:
            j = by_child[link]
            chain.append(j)
            link = j.find("parent").get("link")
        self.joints = []
        for j in reversed(chain):
            o = j.find("origin")
            T = np.eye(4)
            T[:3, :3] = _rpy(*[float(v) for v in (o.get("rpy") or "0 0 0").split()])
            T[:3, 3] = [float(v) for v in o.get("xyz").split()]
            axis = np.array([float(v) for v in j.find("axis").get("xyz").split()]) if j.get("type") != "fixed" else None
            lim = j.find("limit")
            limits = (float(lim.get("lower")), float(lim.get("upper"))) if axis is not None else None
            self.joints.append((j.get("name"), T, axis, limits))
        self.names = [n for n, _, a, _ in self.joints if a is not None]
        assert len(self.names) == 7, f"expected 7 arm joints, got {self.names}"
        self.lower = np.array([lim[0] for _, _, a, lim in self.joints if a is not None])
        self.upper = np.array([lim[1] for _, _, a, lim in self.joints if a is not None])
        self.tcp = np.asarray(tcp_offset, dtype=float)

    def _frames(self, q):
        """Joint origins / axes (world) and the TCP pose."""
        T = np.eye(4)
        origins, axes, k = [], [], 0
        for _, Tj, axis, _ in self.joints:
            T = T @ Tj
            if axis is not None:
                origins.append(T[:3, 3].copy())
                axes.append(T[:3, :3] @ axis)
                R = np.eye(4)
                R[:3, :3] = _axis_angle(axis, q[k])
                T = T @ R
                k += 1
        p = T[:3, 3] + T[:3, :3] @ self.tcp
        return origins, axes, p, T[:3, :3]

    def fk(self, q):
        _, _, p, R = self._frames(q)
        return p, R

    def ik(self, p_target, R_target, q0, iters=15, rot_weight=0.5, damping=0.03, q_rest=None, rest_gain=0.1):
        """Damped least squares from q0; returns (q, position error [m], rotation error [rad]).

        ``q_rest``: posture the redundant (null-space) motion is pulled towards, keeps the 7-dof arm natural.
        """
        q = np.clip(np.asarray(q0, dtype=float), self.lower, self.upper)
        for _ in range(iters):
            origins, axes, p, R = self._frames(q)
            e = np.concatenate([p_target - p, rot_weight * _rot_error(R_target, R)])
            if np.linalg.norm(e[:3]) < 1e-4 and np.linalg.norm(e[3:]) < 1e-3:
                break
            J = np.zeros((6, 7))
            for i, (o, a) in enumerate(zip(origins, axes)):
                J[:3, i] = np.cross(a, p - o)
                J[3:, i] = rot_weight * a
            JJ = np.linalg.solve(J @ J.T + damping ** 2 * np.eye(6), np.eye(6))
            dq = J.T @ JJ @ e
            if q_rest is not None:
                dq += (np.eye(7) - J.T @ JJ @ J) @ (rest_gain * (np.asarray(q_rest) - q))
            q = np.clip(q + np.clip(dq, -0.2, 0.2), self.lower, self.upper)
        p, R = self.fk(q)
        return q, float(np.linalg.norm(p_target - p)), float(np.linalg.norm(_rot_error(R_target, R)))


_MIRROR = np.diag([1.0, -1.0, 1.0])


class CartesianArms:
    """Keyboard-friendly Cartesian control of both arms (targets in the torso frame).

    ``q`` is the 14-vector of arm joint targets (left 7, right 7; lowcmd motors 15..28).
    """

    POS_RATE = 0.15  # m/s
    ROT_RATE = 0.8  # rad/s

    def __init__(self, q_arms, tcp_offset=(0.15, 0.0, 0.0), urdf: str = DEFAULT_ARM_URDF, base_link: str = "torso_link"):
        self.arm = {s: ArmKinematics(s, tcp_offset, urdf, base_link) for s in ("left", "right")}
        self.q = {"left": np.array(q_arms[:7], dtype=float), "right": np.array(q_arms[7:14], dtype=float)}
        self.q_start = {s: v.copy() for s, v in self.q.items()}
        self.side = "left"
        self._sync_targets()

    def _sync_targets(self):
        self.target = {s: self.arm[s].fk(self.q[s]) for s in ("left", "right")}

    def reset(self):
        self.q = {s: v.copy() for s, v in self.q_start.items()}
        self._sync_targets()

    def cycle_side(self):
        self.side = {"left": "right", "right": "both", "both": "left"}[self.side]
        self._sync_targets()

    def step(self, dpos, drot, dt):
        """dpos / drot: (x, y, z) direction in {-1, 0, 1} for translation / rotation about torso axes."""
        dpos = np.asarray(dpos, dtype=float) * self.POS_RATE * dt
        drot = np.asarray(drot, dtype=float) * self.ROT_RATE * dt
        if not dpos.any() and not drot.any():
            return
        sides = ("left", "right") if self.side == "both" else (self.side,)
        for s in sides:
            dp, dr = (dpos, drot) if not (self.side == "both" and s == "right") else (_MIRROR @ dpos, -(_MIRROR @ drot))
            p, R = self.target[s]
            angle = np.linalg.norm(dr)
            R_new = _axis_angle(dr, angle) @ R if angle > 0 else R
            p_new = p + dp
            q, ep, er = self.arm[s].ik(p_new, R_new, self.q[s], q_rest=self.q_start[s])
            self.q[s] = q
            # unreachable: follow what the arm achieved instead of winding the target up
            self.target[s] = (p_new, R_new) if (ep < 0.01 and er < 0.1) else self.arm[s].fk(q)

    def q_arms(self):
        return list(self.q["left"]) + list(self.q["right"])

    def tcp_position(self, side):
        return self.target[side][0]
