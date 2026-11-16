# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
Unitree Dex2-5 tendon hand state (see robots/dex2_5_hand.py for the hand / the simulated transmission).

* ``get_robot_dex2_5_joint_states``: observation, the 20 simulated finger joint positions (left 10, right 10).
* ``dex2_5_motor_states``: the four motors as published on rt/lowstate, slots 31 / 32 (left fingers, thumb) and
  33 / 34 (right fingers, thumb), like the real "internal" wiring. The real motor ``q`` / ``dq`` are uncalibrated
  diagnostic values, so the simulated ones are defined here:

      q       mean flexion angle [rad] of the joints driven by the motor (0 = open, positive = closed)
      dq      its rate [rad/s]
      tau_est the motor torque applied by the action provider [Nm]: the commanded torque after the +-0.3 Nm limit,
              0 for a disabled slot / without a command (also while the hand keeps holding an object)
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from robots.dex2_5_hand import (
    DEX2_5_JOINT_TORQUE_GAIN,
    DEX2_5_JOINTS,
    DEX2_5_LOWCMD_SLOTS,
    DEX2_5_MOTORS,
    DEX2_5_SIDES,
    dex2_5_motor_joints,
)
from tasks.common_observations.joint_index import JointGather

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def has_dex2_5(joint_names) -> bool:
    return DEX2_5_JOINTS[0] in joint_names


class Dex2_5Layout:
    """Joint tables of the Dex2-5 hands of a robot, grouped by motor.

    Motor order: left fingers, left thumb, right fingers, right thumb (= ``slots``); a hand the robot does not have
    (one-hand robots, e.g. the R1-7A arm) is left out. The joint tensors list the joints of all motors in that order;
    ``motor`` gives the motor of each joint.
    """

    def __init__(self, robot):
        names = list(robot.data.joint_names)
        index = {n: i for i, n in enumerate(names)}
        limits = robot.data.joint_pos_limits.torch[0]
        device = limits.device
        ids, motor, gain, self.slots = [], [], [], []
        self.sides = [s for s in DEX2_5_SIDES if dex2_5_motor_joints(s, "fingers")[0] in index]
        for side in self.sides:
            for m, kind in enumerate(DEX2_5_MOTORS):
                joints = dex2_5_motor_joints(side, kind)
                ids += [index[j] for j in joints]
                motor += [len(self.slots)] * len(joints)
                gain += [DEX2_5_JOINT_TORQUE_GAIN[kind]] * len(joints)
                self.slots.append(DEX2_5_LOWCMD_SLOTS[side][m])
        self.num_motors = len(self.slots)
        self.ids = torch.tensor(ids, dtype=torch.long, device=device)
        self.motor = torch.tensor(motor, dtype=torch.long, device=device)
        self.gain = torch.tensor(gain, dtype=torch.float32, device=device)  # joint torque per motor torque
        self.lower = limits[self.ids, 0].float().clone()
        self.upper = limits[self.ids, 1].float().clone()
        # 0 rad is open, the closed pose is the other (non-zero) limit
        closed = torch.where(self.lower.abs() > self.upper.abs(), self.lower, self.upper)
        self.open = self.lower + self.upper - closed
        self.dir = torch.sign(closed - self.open)  # closing direction of each joint
        self.joints_per_motor = torch.zeros(self.num_motors, device=device).index_add_(0, self.motor, torch.ones_like(self.gain))
        self.tau_cmd = torch.zeros(self.num_motors, device=device)  # motor torques applied (action_provider/dex2_5_torque.py)

    def motor_mean(self, per_joint: torch.Tensor) -> torch.Tensor:
        """Mean of a per-joint quantity over the joints of each motor -> [num_motors]."""
        return torch.zeros(self.num_motors, device=per_joint.device).index_add_(0, self.motor, per_joint.float()) / self.joints_per_motor


_layouts: dict[int, tuple] = {}


def dex2_5_layout(robot) -> Dex2_5Layout:
    cached = _layouts.get(id(robot))
    if cached is None or cached[0] is not robot:
        cached = _layouts[id(robot)] = (robot, Dex2_5Layout(robot))
    return cached[1]


def dex2_5_motor_states(env: ManagerBasedRLEnv):
    """(lowstate slots, q, dq, tau_est) of the four hand motors of env 0 (definitions: module docstring)."""
    robot = env.scene["robot"]
    lay = dex2_5_layout(robot)
    data = robot.data
    q = data.joint_pos.torch[0].index_select(0, lay.ids)
    dq = data.joint_vel.torch[0].index_select(0, lay.ids)
    return (
        lay.slots,
        lay.motor_mean((q - lay.open) * lay.dir),
        lay.motor_mean(dq * lay.dir),
        lay.tau_cmd,
    )


_gather: dict[int, tuple] = {}


def get_robot_dex2_5_joint_states(env: ManagerBasedRLEnv, enable_dds: bool = True) -> torch.Tensor:
    """[batch, 10 per hand] Dex2-5 joint positions [rad] (left 10: thumb 11 12, fingers 21 22 .. 51 52; then right 10;
    a hand the robot does not have is left out).

    The hand has no DDS topic of its own: its motor state is part of rt/lowstate, published with the body state
    (``enable_dds`` is accepted for the common observation signature).
    """
    robot = env.scene["robot"]
    joint_pos = robot.data.joint_pos.torch
    cached = _gather.get(id(robot))
    if cached is None or cached[0] is not robot:
        names = list(robot.data.joint_names)
        cached = _gather[id(robot)] = (robot, JointGather(names, [j for j in DEX2_5_JOINTS if j in names], joint_pos.device))
    return cached[1].gather(joint_pos)
