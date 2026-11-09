# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
R1-D robot state observations.

Joints are looked up by name, the state is published through the same ``g129`` (rt/lowstate) and ``dex1``
DDS objects as the G1 / G1-D. lowstate motor slots (xr_teleoperate R1_A7_JointIndex):

* 0 / 1   : left / right drive wheel
* 12      : lift column motor, q = height / kColumnRadius_m (0.00382 m/rad), sum of the two lift stages
* 13      : waist yaw
* 15..28  : arms (same joint names / order as the G1)
* 29 / 30 : head pitch / head yaw
* others  : not present on the R1-D (read 0)

The Dex1 grippers have the same joints / travel as on the G1-D, so the G1-D gripper state function is reused.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from robots.unitree_r1d import R1D_COLUMN_RADIUS, R1D_HEAD_PITCH_JOINT, R1D_HEAD_YAW_JOINT, R1D_LIFT_JOINTS, R1D_WAIST_YAW_JOINT
from tasks.common_observations.joint_index import JointGather
from tasks.common_observations.g1_29dof_state import _get_g1_robot_dds_instance, get_robot_imu_data
from tasks.common_observations.g1d_state import ARM_JOINTS, _dds_due, _gather_states
from tasks.common_observations.g1d_state import get_robot_g1d_dex1_joint_states as get_robot_r1d_dex1_joint_states  # noqa: F401

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

R1D_LOWSTATE_JOINTS = (
    ["Left_Wheel_joint", "Right_Wheel_joint"] + [""] * 11 + [R1D_WAIST_YAW_JOINT, ""] + ARM_JOINTS + [R1D_HEAD_PITCH_JOINT, R1D_HEAD_YAW_JOINT]
)
_COLUMN_SLOT = 12
_lift = {}


def get_robot_r1d_body_joint_states(env: ManagerBasedRLEnv, enable_dds: bool = True) -> torch.Tensor:
    """[batch, 93] = positions | velocities | torques of lowstate motor slots 0..30."""
    st, pos, vel, torque = _gather_states(env, R1D_LOWSTATE_JOINTS, "r1d_body")
    # column motor (slot 12) from the two simulated lift stages
    robot = env.scene["robot"]
    lg = _lift.get(id(robot))
    if lg is None:
        lg = _lift[id(robot)] = JointGather(list(robot.data.joint_names), list(R1D_LIFT_JOINTS), pos.device)
    pos[:, _COLUMN_SLOT] = lg.gather(robot.data.joint_pos.torch).sum(dim=1) / R1D_COLUMN_RADIUS
    vel[:, _COLUMN_SLOT] = lg.gather(robot.data.joint_vel.torch).sum(dim=1) / R1D_COLUMN_RADIUS
    if enable_dds and pos.shape[0] > 0 and _dds_due(st):
        try:
            dds = _get_g1_robot_dds_instance()
            if dds:
                imu = get_robot_imu_data(env)
                dds.write_robot_state(pos[0].cpu().numpy(), vel[0].cpu().numpy(), torque[0].cpu().numpy(), imu[0].cpu().numpy())
        except Exception as e:
            print(f"[r1d_state] Error writing robot state to DDS: {e}")
    return torch.cat([pos, vel, torque], dim=1)
