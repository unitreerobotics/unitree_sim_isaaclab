# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
G1-D robot state observations (independent of the G1 29dof state functions).

Joints are looked up by name (``joint_index.JointGather``), so the functions work for the URDF- or
USD-imported G1-D whatever the joint order is.

DDS: the state is published through the same ``g129`` (rt/lowstate, 29 motors) and ``dex1`` DDS
objects as the G1, so the existing teleoperation clients work unchanged. Motor slots:

* 12 / 13: waist yaw (torso_Joint) / waist pitch (Yaw_Joint)
* 15..28 : arms (same joints as G1, same order)
* others  : not present on the G1-D (no legs), read 0. Lift and base go through g1d_mobile_dds.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

import torch

from tasks.common_observations.joint_index import JointGather

# DDS publishers / IMU helper are reused read-only from the G1 modules (they are not robot specific)
from tasks.common_observations.g1_29dof_state import _get_g1_robot_dds_instance, get_robot_imu_data
from tasks.common_observations.gripper_state import _get_gripper_dds_instance
from robots.unitree_g1d import G1D_DEX1_FINGER_CLOSED, G1D_DEX1_FINGER_OPEN, G1D_WAIST_PITCH_JOINT, G1D_WAIST_YAW_JOINT

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

ARM_JOINTS = [
    f"{side}_{j}_joint"
    for side in ("left", "right")
    for j in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")
]
# 29 lowstate motor slots (real G1-D layout, see xr_teleoperate mobile-teleop G1_29_JointWaistIndex):
# 12 = waist yaw, 13 = waist pitch, 15..28 = arms; "" = not present on G1-D (reported as 0)
G1D_LOWSTATE_JOINTS = [""] * 12 + [G1D_WAIST_YAW_JOINT, G1D_WAIST_PITCH_JOINT, ""] + ARM_JOINTS

# Dex1 gripper, DDS order: right, left (one finger joint per gripper is reported)
G1D_DEX1_JOINTS = ["right_dex1_finger_joint_1", "left_dex1_finger_joint_1"]

# The dex1 DDS object converts motor values from/to the G1 Dex1 joint range
# [-0.02 (open), 0.024 (closed)]; map the G1-D finger travel onto that range.
_G1_DEX1_OPEN, _G1_DEX1_CLOSED = -0.02, 0.024


def g1d_finger_to_g1_dex1(x):
    c = (x - G1D_DEX1_FINGER_OPEN) / (G1D_DEX1_FINGER_CLOSED - G1D_DEX1_FINGER_OPEN)
    return _G1_DEX1_OPEN + c * (_G1_DEX1_CLOSED - _G1_DEX1_OPEN)


def g1_dex1_to_g1d_finger(x):
    c = (x - _G1_DEX1_OPEN) / (_G1_DEX1_CLOSED - _G1_DEX1_OPEN)
    return G1D_DEX1_FINGER_OPEN + c * (G1D_DEX1_FINGER_CLOSED - G1D_DEX1_FINGER_OPEN)

_DDS_MIN_INTERVAL_MS = 20
_state: dict[str, dict] = {}


def _gather_states(env: ManagerBasedRLEnv, names: list[str], tag: str):
    robot = env.scene["robot"]
    joint_pos = robot.data.joint_pos.torch
    st = _state.get(tag)
    if st is None or st["robot"] is not robot:
        st = {"robot": robot, "jg": JointGather(list(robot.data.joint_names), names, joint_pos.device), "dds_last_ms": 0}
        missing = [n for n in st["jg"].missing if n]
        if missing:
            print(f"[g1d_state] {tag}: robot has no joints {missing}; they are reported as 0")
        _state[tag] = st
    jg = st["jg"]
    return st, jg.gather(joint_pos), jg.gather(robot.data.joint_vel.torch), jg.gather(robot.actuators.applied_effort.torch)


def _dds_due(st) -> bool:
    now_ms = int(time.time() * 1000)
    if now_ms - st["dds_last_ms"] >= _DDS_MIN_INTERVAL_MS:
        st["dds_last_ms"] = now_ms
        return True
    return False


def get_robot_g1d_body_joint_states(env: ManagerBasedRLEnv, enable_dds: bool = True) -> torch.Tensor:
    """[batch, 87] = positions | velocities | torques of the 29 lowstate motor slots."""
    st, pos, vel, torque = _gather_states(env, G1D_LOWSTATE_JOINTS, "g1d_body")
    if enable_dds and pos.shape[0] > 0 and _dds_due(st):
        try:
            dds = _get_g1_robot_dds_instance()
            if dds:
                imu = get_robot_imu_data(env)
                dds.write_robot_state(pos[0].cpu().numpy(), vel[0].cpu().numpy(), torque[0].cpu().numpy(), imu[0].cpu().numpy())
        except Exception as e:
            print(f"[g1d_state] Error writing robot state to DDS: {e}")
    return torch.cat([pos, vel, torque], dim=1)


def get_robot_g1d_dex1_joint_states(env: ManagerBasedRLEnv, enable_dds: bool = True) -> torch.Tensor:
    """[batch, 2] gripper finger positions (right, left) in the G1-D joint range [0 closed, 0.041 open]."""
    st, pos, vel, torque = _gather_states(env, G1D_DEX1_JOINTS, "g1d_dex1")
    if enable_dds and pos.shape[0] > 0 and _dds_due(st):
        try:
            dds = _get_gripper_dds_instance()
            if dds:
                scale = (_G1_DEX1_CLOSED - _G1_DEX1_OPEN) / (G1D_DEX1_FINGER_CLOSED - G1D_DEX1_FINGER_OPEN)
                p = g1d_finger_to_g1_dex1(pos[0]).cpu().numpy()
                v = (vel[0] * scale).cpu().numpy()
                t = torque[0].cpu().numpy()
                dds.write_gripper_state(p[1:], v[1:], t[1:], p[:1], v[:1], t[:1])
        except Exception as e:
            print(f"[g1d_state] Failed to write Dex1 state to DDS: {e}")
    return pos
