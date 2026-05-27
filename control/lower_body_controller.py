# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""In-process GR00T-WBC lower-body controller.

Wraps NVIDIA's ``GR00T-WholeBodyControl-Balance.onnx`` +
``GR00T-WholeBodyControl-Walk.onnx`` checkpoints and exposes a single
``step()`` that consumes the locomotion command (vx, vy, omega, height)
and current robot state, then emits 15-D absolute joint targets
(12 legs + 3 waist) ready to write into ``rt/lowcmd`` slots 0..14 or
directly into the IsaacLab joint-target buffer.

Inference is pure NumPy + onnxruntime; no torch dependency.

Observation layout (86-D per frame, 6-frame history → 516-D input):
    0:3    nav_cmd * cmd_scale
    3      base_height_cmd
    4:7    torso_rpy_cmd
    7:10   base_ang_vel * ang_vel_scale
    10:13  projected_gravity (body frame)
    13:42  (qj - default_angles_29) * dof_pos_scale   (29-D body order)
    42:71  dqj * dof_vel_scale                         (29-D body order)
    71:86  last raw action (15-D)
"""

from __future__ import annotations

import collections
from pathlib import Path
from typing import Sequence

import numpy as np
import onnxruntime as ort


def _project_gravity(quat_xyzw: np.ndarray) -> np.ndarray:
    """Rotate world gravity [0, 0, -1] into the body frame using a
    quaternion in (x, y, z, w) order — the convention published on
    ``rt/lowstate`` by ``dds/g1_robot_dds.py``.

    Standing upright with identity quaternion the result is [0, 0, -1].
    """
    x, y, z, w = quat_xyzw.astype(np.float32)
    return np.array(
        [
            -2.0 * (x * z - w * y),
            -2.0 * (y * z + w * x),
            -(1.0 - 2.0 * (x * x + y * y)),
        ],
        dtype=np.float32,
    )


def _quat_inverse_xyzw(q: np.ndarray) -> np.ndarray:
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=np.float32)


def _quat_multiply_xyzw(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ],
        dtype=np.float32,
    )


class LowerBodyController:
    """GR00T-WBC Balance + Walk wrapper.

    Joint order (15-D output, matches Balance/Walk ONNX export):
        0..5   left  leg (hip_pitch, hip_roll, hip_yaw, knee, ank_pitch, ank_roll)
        6..11  right leg (same order)
        12..14 waist (yaw, roll, pitch)

    29-D body order used for obs assembly (matches DDS slot order):
        0..11  legs (left then right)
        12..14 waist
        15..21 left  arm (shoulder_pitch, _roll, _yaw, elbow,
                           wrist_roll, _pitch, _yaw)
        22..28 right arm (same order)
    """

    BALANCE_PATH = "GR00T-WholeBodyControl-Balance.onnx"
    WALK_PATH = "GR00T-WholeBodyControl-Walk.onnx"
    SWITCH_THRESH = 0.05  # |nav_cmd| < this → Balance; else Walk.

    # GR00T-WBC default angles (15-D lower body only).
    DEFAULT_ANGLES_15 = np.array(
        [
            -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,   # left leg
            -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,   # right leg
             0.0, 0.0, 0.0,                   # waist
        ],
        dtype=np.float32,
    )

    NUM_BODY_DOF = 29
    NUM_LOWER = 15
    SINGLE_OBS_DIM = 86
    OBS_HISTORY = 6
    NUM_OBS = SINGLE_OBS_DIM * OBS_HISTORY  # 516

    ACTION_SCALE = 0.25
    CMD_SCALE = np.array([2.0, 2.0, 0.5], dtype=np.float32)
    ANG_VEL_SCALE = 0.5
    DOF_POS_SCALE = 1.0
    DOF_VEL_SCALE = 0.05

    def __init__(
        self,
        onnx_dir: str | Path,
        providers: Sequence[str] | None = None,
    ) -> None:
        onnx_dir = Path(onnx_dir)
        balance_path = onnx_dir / self.BALANCE_PATH
        walk_path = onnx_dir / self.WALK_PATH
        if not balance_path.exists():
            raise FileNotFoundError(f"Balance ONNX not found: {balance_path}")
        if not walk_path.exists():
            raise FileNotFoundError(f"Walk ONNX not found: {walk_path}")
        if providers is None:
            providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
        self._balance = ort.InferenceSession(str(balance_path), providers=list(providers))
        self._walk = ort.InferenceSession(str(walk_path), providers=list(providers))
        self._input_name = self._balance.get_inputs()[0].name

        self._history: collections.deque[np.ndarray] = collections.deque(
            maxlen=self.OBS_HISTORY
        )
        # "last action" slot of the obs (positions 71..85) is the
        # previous raw output of whichever WBC ran last tick.
        self._last_action = np.zeros(self.NUM_LOWER, dtype=np.float32)
        # Optional spawn-quat calibration: maps the USD's raw root quat at
        # spawn to identity, so projected_gravity computed from quat is
        # canonical even if the USD's root link has a non-anatomical basis.
        self._calibration_inv: np.ndarray | None = None
        # Pre-allocated buffers reused per tick.
        self._single_obs = np.zeros(self.SINGLE_OBS_DIM, dtype=np.float32)
        self._obs_buf = np.zeros(self.NUM_OBS, dtype=np.float32)
        self._padded_defaults = np.zeros(self.NUM_BODY_DOF, dtype=np.float32)
        self._padded_defaults[: self.NUM_LOWER] = self.DEFAULT_ANGLES_15

    # ------------------------------------------------------------------ public
    def reset(self) -> None:
        self._history.clear()
        self._last_action[:] = 0.0

    def calibrate_spawn_quat(self, q_spawn_xyzw: np.ndarray) -> None:
        """Treat the given quaternion as the body-frame identity going
        forward. Idempotent."""
        self._calibration_inv = _quat_inverse_xyzw(
            np.asarray(q_spawn_xyzw, dtype=np.float32)
        )

    def step(
        self,
        legs_q: np.ndarray,           # (12,)
        legs_dq: np.ndarray,          # (12,)
        waist_q: np.ndarray,          # (3,)
        waist_dq: np.ndarray,         # (3,)
        arms_q: np.ndarray,           # (14,)
        arms_dq: np.ndarray,          # (14,)
        base_quat_xyzw: np.ndarray,   # (4,) x, y, z, w
        base_ang_vel: np.ndarray,     # (3,) rad/s, body frame
        navigate_cmd: np.ndarray,     # (3,) vx, vy, omega
        base_height_cmd: float,
        torso_rpy_cmd: Sequence[float] = (0.0, 0.0, 0.0),
        projected_gravity: np.ndarray | None = None,
    ) -> np.ndarray:
        """Run one inference step. Returns (15,) float32 absolute joint
        position targets for the lower body in WBC joint order.

        The returned values are already in radians (action_scale +
        default_angles applied here), so the caller can write them
        directly into joint targets without further scaling.
        """
        nav = np.asarray(navigate_cmd, dtype=np.float32).reshape(-1)[:3]
        pg = (
            np.asarray(projected_gravity, dtype=np.float32).reshape(-1)[:3]
            if projected_gravity is not None else None
        )
        single = self._build_single_obs(
            nav_cmd=nav,
            height_cmd=float(base_height_cmd),
            rpy_cmd=np.asarray(torso_rpy_cmd, dtype=np.float32),
            base_ang_vel=np.asarray(base_ang_vel, dtype=np.float32),
            base_quat_xyzw=np.asarray(base_quat_xyzw, dtype=np.float32),
            projected_gravity=pg,
            legs_q=np.asarray(legs_q, dtype=np.float32),
            waist_q=np.asarray(waist_q, dtype=np.float32),
            arms_q=np.asarray(arms_q, dtype=np.float32),
            legs_dq=np.asarray(legs_dq, dtype=np.float32),
            waist_dq=np.asarray(waist_dq, dtype=np.float32),
            arms_dq=np.asarray(arms_dq, dtype=np.float32),
        )
        # Pre-warm: on the first tick, fill the history with copies of
        # this frame rather than letting `_build_full_obs` zero-pad
        # (WBC was trained on continuous trajectories).
        if not self._history:
            for _ in range(self.OBS_HISTORY):
                self._history.append(single.copy())
        else:
            self._history.append(single.copy())
        full_obs = self._build_full_obs()

        sess = self._balance if np.linalg.norm(nav) < self.SWITCH_THRESH else self._walk
        action = sess.run(None, {self._input_name: full_obs[None, :]})[0][0]  # (15,)
        self._last_action[:] = action  # raw, pre-scale, for next tick's obs
        return action * self.ACTION_SCALE + self.DEFAULT_ANGLES_15

    # ----------------------------------------------------------------- helpers
    def _build_single_obs(
        self,
        *,
        nav_cmd: np.ndarray,
        height_cmd: float,
        rpy_cmd: np.ndarray,
        base_ang_vel: np.ndarray,
        base_quat_xyzw: np.ndarray,
        projected_gravity: np.ndarray | None,
        legs_q: np.ndarray,
        waist_q: np.ndarray,
        arms_q: np.ndarray,
        legs_dq: np.ndarray,
        waist_dq: np.ndarray,
        arms_dq: np.ndarray,
    ) -> np.ndarray:
        o = self._single_obs
        o[0:3] = nav_cmd * self.CMD_SCALE
        o[3] = height_cmd
        o[4:7] = rpy_cmd
        o[7:10] = base_ang_vel * self.ANG_VEL_SCALE
        # Prefer the explicit projected_gravity (from IsaacLab's
        # data.projected_gravity_b) over the quat-based computation.
        if projected_gravity is not None:
            o[10:13] = projected_gravity
        else:
            q_eff = (
                _quat_multiply_xyzw(base_quat_xyzw, self._calibration_inv)
                if self._calibration_inv is not None
                else base_quat_xyzw
            )
            o[10:13] = _project_gravity(q_eff)

        # 29-D qj = legs (12) + waist (3) + arms (14), minus padded defaults.
        qj = np.concatenate([legs_q, waist_q, arms_q]).astype(np.float32)
        o[13 : 13 + self.NUM_BODY_DOF] = (qj - self._padded_defaults) * self.DOF_POS_SCALE

        dqj = np.concatenate([legs_dq, waist_dq, arms_dq]).astype(np.float32)
        o[13 + self.NUM_BODY_DOF : 13 + 2 * self.NUM_BODY_DOF] = dqj * self.DOF_VEL_SCALE

        o[13 + 2 * self.NUM_BODY_DOF :] = self._last_action
        return o

    def _build_full_obs(self) -> np.ndarray:
        n = len(self._history)
        if n < self.OBS_HISTORY:
            pad = self.OBS_HISTORY - n
            self._obs_buf[: pad * self.SINGLE_OBS_DIM] = 0.0
            offset = pad * self.SINGLE_OBS_DIM
        else:
            offset = 0
        for frame in self._history:
            self._obs_buf[offset : offset + self.SINGLE_OBS_DIM] = frame
            offset += self.SINGLE_OBS_DIM
        return self._obs_buf
