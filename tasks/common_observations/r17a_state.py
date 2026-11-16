# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
R1-7A arm robot state observations (see robots/unitree_r17a.py).

rt/lowstate (``g129`` DDS object, unitree_hg LowState_, 35 motor slots):

* 0..6   : left arm J1..J7
* 7..13  : right arm J1..J7 (no R1-7A robot has one: 0)
* 31     : Dex1 gripper motor of the (left) arm, q 0 closed .. 5.4 open (like the G1-D / R1-A7-D "internal" wiring)
* 31 / 32: Dex2-5 motors of the left hand (fingers, thumb), see dex2_5_state.py

Hand observations: Dex1 finger position (also on rt/dex1/left/state), Dex2-5 joints (dex2_5_state.py), LinkerHand
O6 joints (also on rt/linkerhand/left/state as raw 0..255 angles).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from robots.linkerhand_o6 import O6_MIMIC, O6_RAW_MAX, O6_RAW_OPEN, o6_all_joints, o6_motor_joints
from robots.unitree_r17a import r17a_arm_joints
from tasks.common_observations.dex2_5_state import dex2_5_motor_states, has_dex2_5
from tasks.common_observations.dex2_5_state import get_robot_dex2_5_joint_states as get_robot_r17a_dex2_5_joint_states  # noqa: F401
from tasks.common_observations.g1_29dof_state import _get_g1_robot_dds_instance, get_robot_imu_data
from tasks.common_observations.g1d_state import (
    _DEX1_MOTOR_PER_M,
    DEX1_LOWCMD_SLOTS,
    LOWSTATE_SLOTS,
    _dds_due,
    _gather_states,
    g1d_finger_to_g1_dex1,
)
from tasks.common_observations.gripper_state import _get_gripper_dds_instance
from robots.unitree_g1d import G1D_DEX1_FINGER_CLOSED, G1D_DEX1_FINGER_OPEN

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

R17A_LOWSTATE_JOINTS = r17a_arm_joints("left") + r17a_arm_joints("right")  # slots 0..13
R17A_DEX1_JOINTS = ["left_dex1_finger_joint_1"]


def _lowstate_arrays(env: ManagerBasedRLEnv, pos, vel, torque):
    n = pos.shape[1]
    p, v, t = (torch.zeros(LOWSTATE_SLOTS, device=pos.device, dtype=pos.dtype) for _ in range(3))
    p[:n], v[:n], t[:n] = pos[0], vel[0], torque[0]
    names = env.scene["robot"].data.joint_names
    if has_dex2_5(names):
        slots, q, dq, tau = dex2_5_motor_states(env)
        for i, s in enumerate(slots):
            p[s], v[s], t[s] = q[i], dq[i], tau[i]
    elif R17A_DEX1_JOINTS[0] in names:
        _, fp, fv, ft = _gather_states(env, R17A_DEX1_JOINTS, "r17a_dex1_lowstate")
        s = DEX1_LOWCMD_SLOTS["left"]
        p[s] = (fp[0, 0] - G1D_DEX1_FINGER_CLOSED) * _DEX1_MOTOR_PER_M
        v[s] = fv[0, 0] * _DEX1_MOTOR_PER_M
        t[s] = ft[0, 0]
    return p, v, t


def get_robot_r17a_body_joint_states(env: ManagerBasedRLEnv, enable_dds: bool = True) -> torch.Tensor:
    """[batch, 42] = positions | velocities | torques of lowstate slots 0..13 (arms)."""
    st, pos, vel, torque = _gather_states(env, R17A_LOWSTATE_JOINTS, "r17a_body")
    if enable_dds and pos.shape[0] > 0 and _dds_due(st):
        try:
            dds = _get_g1_robot_dds_instance()
            if dds:
                imu = get_robot_imu_data(env)
                p, v, t = _lowstate_arrays(env, pos, vel, torque)
                dds.write_robot_state(p.cpu().numpy(), v.cpu().numpy(), t.cpu().numpy(), imu[0].cpu().numpy())
        except Exception as e:
            print(f"[r17a_state] Error writing robot state to DDS: {e}")
    return torch.cat([pos, vel, torque], dim=1)


def get_robot_r17a_dex1_joint_states(env: ManagerBasedRLEnv, enable_dds: bool = True) -> torch.Tensor:
    """[batch, 1] Dex1 finger position [0 closed .. 0.041 m open]; also on rt/dex1/left/state (G1 Dex1 range)."""
    st, pos, vel, torque = _gather_states(env, R17A_DEX1_JOINTS, "r17a_dex1")
    if enable_dds and pos.shape[0] > 0 and _dds_due(st):
        try:
            dds = _get_gripper_dds_instance()
            if dds:
                p = g1d_finger_to_g1_dex1(pos[0]).cpu().numpy()
                v, t = vel[0].cpu().numpy(), torque[0].cpu().numpy()
                dds.write_gripper_state(p, v, t, p * 0, v * 0, t * 0)
        except Exception as e:
            print(f"[r17a_state] Failed to write Dex1 state to DDS: {e}")
    return pos


_linkerhand_dds = []


def _get_linkerhand_dds():
    if not _linkerhand_dds:
        try:
            from dds.dds_master import dds_manager

            obj = dds_manager.objects.get("linkerhand")
        except Exception:
            obj = None
        if obj is None:
            return None
        _linkerhand_dds.append(obj)
    return _linkerhand_dds[0]


_o6_limits = {}


def o6_raw_from_joints(env: ManagerBasedRLEnv, side: str, pos, vel):
    """Joint positions / velocities of the 6 actuated O6 joints -> raw angles 0..255 / raw per second."""
    robot = env.scene["robot"]
    lim = _o6_limits.get(id(robot))
    if lim is None:
        names = list(robot.data.joint_names)
        ids = [names.index(j) for j in o6_motor_joints(side)]
        lo, hi = robot.data.joint_pos_limits.torch[0][ids].unbind(-1)
        open_ = torch.where(lo.abs() <= hi.abs(), lo, hi)
        closed = lo + hi - open_
        lim = _o6_limits[id(robot)] = (open_.to(pos.device), closed.to(pos.device))
    open_, closed = lim
    c = ((pos - open_) / (closed - open_)).clamp(0.0, 1.0)  # 0 open .. 1 closed
    raw = (O6_RAW_OPEN - c * O6_RAW_MAX).abs()
    sign = -1.0 if O6_RAW_OPEN > 0 else 1.0
    return raw, sign * vel / (closed - open_) * O6_RAW_MAX


def get_robot_r17a_o6_joint_states(env: ManagerBasedRLEnv, enable_dds: bool = True) -> torch.Tensor:
    """[batch, 11] O6 joint positions [rad]: the 6 actuated joints in the SDK order, then the 5 distal joints.

    Published on rt/linkerhand/left/state as raw angles (0..255, 255 = open).
    """
    st, pos, vel, torque = _gather_states(env, o6_all_joints("left"), "r17a_o6")
    if enable_dds and pos.shape[0] > 0 and _dds_due(st):
        try:
            dds = _get_linkerhand_dds()
            if dds:
                raw, raw_v = o6_raw_from_joints(env, "left", pos[0, :6], vel[0, :6])
                dds.write_hand_states({"left": {"positions": raw.tolist(), "velocities": raw_v.tolist(), "torques": torque[0, :6].tolist()}})
        except Exception as e:
            print(f"[r17a_state] Failed to write LinkerHand state to DDS: {e}")
    return pos


__all__ = [
    "get_robot_r17a_body_joint_states",
    "get_robot_r17a_dex1_joint_states",
    "get_robot_r17a_dex2_5_joint_states",
    "get_robot_r17a_o6_joint_states",
    "R17A_LOWSTATE_JOINTS",
    "O6_MIMIC",
    "G1D_DEX1_FINGER_OPEN",
]
