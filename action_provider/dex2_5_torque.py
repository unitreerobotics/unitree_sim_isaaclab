# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Torque control of the simulated Unitree Dex2-5 tendon hands (see robots/dex2_5_hand.py).

Like the real hand (unirobot ``Dex2_5Interface``) the two motors of a hand - four fingers, thumb - are commanded
with a torque only: rt/lowcmd motors 31 / 32 (left) and 33 / 34 (right), ``mode = 1`` and ``tau`` in +-0.3 Nm
(``q`` / ``dq`` / ``kp`` / ``kd`` are ignored: the hand has no position control). Positive closes, negative opens.

The environment is driven with joint position targets, so the motor torque is applied through the joint drive:
every joint of a motor gets the target ``q + gain * tau / stiffness``, i.e. the drive torque ``gain * tau`` in the
closing direction minus the drive damping - the fingers close until they are stopped by an object or their joint
limit and then press with that torque.

Zero torque (or a disabled slot) does not open the hand. With ``DEX2_5_HOLD_GRIP`` (default) a joint that was
stopped - by an object or its limit - when the torque was released keeps its last target, i.e. it keeps pressing
with the last torque: a grasped object stays in the hand until an opening torque is commanded. A joint that was
still moving stops where it is. Without ``DEX2_5_HOLD_GRIP`` every joint just keeps its pose (no grip force).
"""

from __future__ import annotations

import torch

from robots.dex2_5_hand import DEX2_5_HOLD_GRIP, DEX2_5_MAX_TORQUE, DEX2_5_STALL_SPEED
from tasks.common_observations.dex2_5_state import dex2_5_layout


class Dex2_5TorqueHands:
    def __init__(self, robot, name: str = "dex2_5"):
        self._robot = robot
        self._name = name
        self._lay = lay = dex2_5_layout(robot)
        stiffness = robot.data.joint_stiffness.torch[0].index_select(0, lay.ids).float()
        self._lead = lay.dir * lay.gain / stiffness  # target offset [rad] per Nm of motor torque
        self._active = torch.zeros(len(lay.ids), dtype=torch.bool, device=lay.ids.device)
        self._warned = False

    def reset(self) -> None:
        """Forget the latched pose: the targets (default pose after an environment reset) are held."""
        self._active.zero_()
        self._lay.tau_cmd.zero_()

    def apply(self, targets: torch.Tensor, modes=None, torques=None) -> None:
        """Write the hand joint targets for the lowcmd ``modes`` / ``torques`` (None: no command, pose held)."""
        lay = self._lay
        tau = [0.0] * lay.num_motors
        if modes is not None and torques is not None:
            for m, slot in enumerate(lay.slots):
                if slot < len(torques) and slot < len(modes) and modes[slot] == 1:
                    t = float(torques[slot])
                    if abs(t) > DEX2_5_MAX_TORQUE and not self._warned:
                        self._warned = True
                        print(f"[{self._name}] Dex2-5 torque {t:+.2f} Nm on lowcmd motor {slot} limited to +-{DEX2_5_MAX_TORQUE} Nm")
                    tau[m] = max(-DEX2_5_MAX_TORQUE, min(DEX2_5_MAX_TORQUE, t))
        lay.tau_cmd = torch.tensor(tau, dtype=torch.float32, device=lay.ids.device)  # reported as lowstate tau_est
        tau_j = lay.tau_cmd.index_select(0, lay.motor)
        q = self._robot.data.joint_pos.torch[0].index_select(0, lay.ids).float()
        active = tau_j != 0.0
        # torque released in this step: a moving joint stops at its current pose, a stopped one keeps its target
        # (= keeps pressing); in every later step the target is kept
        stop_here = self._active & ~active
        if DEX2_5_HOLD_GRIP:
            dq = self._robot.data.joint_vel.torch[0].index_select(0, lay.ids)
            stop_here = stop_here & (dq.abs() > DEX2_5_STALL_SPEED)
        held = torch.where(stop_here, q, targets[lay.ids])
        targets[lay.ids] = torch.where(active, q + self._lead * tau_j, held)
        self._active = active
