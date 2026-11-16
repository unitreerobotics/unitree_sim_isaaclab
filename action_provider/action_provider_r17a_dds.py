# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""DDS action provider for the Unitree R1-7A arm robots (fixed base, see robots/unitree_r17a.py).

Same DDS interface style as the G1-D / R1-A7-D providers:

* rt/lowcmd (``g129`` DDS object, unitree_hg LowCmd_): motors 0..6 -> left arm J1..J7 (7..13 would drive a right arm;
  all R1-7A robots are one left arm). The arm targets follow the lowcmd positions once a lowcmd was received.
* Dex1 (``--enable_dex1_dds`` for rt/dex1/left/cmd): lowcmd motor 31 ("internal" wiring, motor q 0 closed .. 5.4 open,
  used when the slot has mode 1 and kp > 0; it then takes precedence over rt/dex1/left/cmd), exactly as on the
  G1-D / R1-A7-D left gripper.
* Dex2-5 (left hand): torque-only lowcmd motors 31 / 32 (fingers, thumb), like the R1-A7-D left Dex2-5 hand
  (action_provider/dex2_5_torque.py); no extra flag.
* LinkerHand O6 (``--enable_linkerhand_dds``): rt/linkerhand/left/cmd, 6 raw angles 0..255 (255 = open) in the
  SDK order; the distal joints follow their proximal joint (URDF mimic factors).

Actions are sent relative to the action term's default offset (``use_default_offset=True``).
"""

import time
from typing import Optional

import torch

from action_provider.action_base import ActionProvider
from action_provider.dex2_5_torque import Dex2_5TorqueHands
from dds.dds_master import dds_manager
from robots.linkerhand_o6 import O6_MIMIC, o6_joint, o6_motor_joints, raw_to_fraction_closed
from robots.unitree_r17a import r17a_arm_joints
from tasks.common_observations.dex2_5_state import has_dex2_5
from tasks.common_observations.g1d_state import DEX1_LOWCMD_SLOTS, dex1_motor_to_g1d_finger, g1_dex1_to_g1d_finger

ARM_SLOTS = {"left": 0, "right": 7}  # first lowcmd motor of each arm


class R17ADDSActionProvider(ActionProvider):
    ROBOT = "R1-7A"

    def __init__(self, env, args_cli):
        super().__init__(type(self).__name__)
        self.env = env
        self._device = device = str(env.device)
        self.robot = robot = env.scene["robot"]
        names = list(robot.data.joint_names)
        idx = {n: i for i, n in enumerate(names)}
        self.sides = [s for s in ("left", "right") if f"{s}_J1" in idx]
        self._arm_ids = {s: torch.tensor([idx[j] for j in r17a_arm_joints(s)], dtype=torch.long, device=device) for s in self.sides}

        self._dex2_5 = has_dex2_5(names)
        self._o6 = o6_joint("left", "index_mcp_pitch") in idx
        self._dex1 = "left_dex1_finger_joint_1" in idx
        hand = "Dex2-5 hands" if self._dex2_5 else "LinkerHand O6" if self._o6 else "Dex1 gripper"

        self.robot_dds = dds_manager.get_object("g129")
        self.gripper_dds = dds_manager.get_object("dex1") if (self._dex1 and args_cli.enable_dex1_dds) else None
        self.o6_dds = dds_manager.get_object("linkerhand") if (self._o6 and getattr(args_cli, "enable_linkerhand_dds", False)) else None
        for flag, needed in (("enable_dex1_dds", self._dex1), ("enable_dex3_dds", False), ("enable_inspire_dds", False),
                             ("enable_linkerhand_dds", self._o6)):
            if getattr(args_cli, flag, False) and not needed:
                print(f"[{self.name}] {self.ROBOT} has {hand}; --{flag} is ignored")

        if self._dex1:
            self._fingers = torch.tensor([idx["left_dex1_finger_joint_1"], idx["left_dex1_finger_joint_2"]], dtype=torch.long, device=device)
        self._torque_hands = Dex2_5TorqueHands(robot, self.name) if self._dex2_5 else None
        if self._o6:
            ids = [idx[j] for j in o6_motor_joints("left")]
            lo, hi = robot.data.joint_pos_limits.torch[0][ids].unbind(-1)
            self._o6_ids = torch.tensor(ids, dtype=torch.long, device=device)
            self._o6_open = torch.where(lo.abs() <= hi.abs(), lo, hi).float().to(device)
            self._o6_closed = (lo + hi).float().to(device) - self._o6_open
            self._o6_mimic = [(idx[o6_joint("left", d)], idx[o6_joint("left", p)], m) for d, (p, m) in O6_MIMIC.items()]

        term = env.action_manager.get_term("joint_pos")
        self._offset = torch.as_tensor(term._offset, device=device).reshape(-1, len(names))[0].float()
        self._scale = torch.as_tensor(term._scale, device=device).float()
        self._start_time = time.time()  # ignore commands left in the (named) shared memory by an earlier run
        self._last_episode_step = None
        print(f"[{self.name}] DDS initialized: {len(self.sides)} arm(s) (lowcmd motors "
              f"{', '.join(f'{ARM_SLOTS[s]}..{ARM_SLOTS[s] + 6}' for s in self.sides)}) + {hand}"
              + (" (lowcmd 31" + (" + rt/dex1/left)" if self.gripper_dds else ")") if self._dex1 else "")
              + (" (torque, lowcmd " + " + ".join(f"{a}/{b}" for a, b in zip(self._torque_hands._lay.slots[::2], self._torque_hands._lay.slots[1::2])) + ")" if self._dex2_5 else "")
              + ((" (rt/linkerhand/left)" if self.o6_dds else " (no --enable_linkerhand_dds: held open)") if self._o6 else ""))

    def _reset_targets(self):
        self._targets = self.robot.data.default_joint_pos.torch[0].clone().float().to(self._device)
        if self._torque_hands:
            self._torque_hands.reset()

    def get_action(self, env) -> Optional[torch.Tensor]:
        try:
            step = int(env.episode_length_buf[0])
            if self._last_episode_step is None or step < self._last_episode_step:
                self._reset_targets()
            self._last_episode_step = step

            internal_gripper = None
            hand_modes = hand_torques = None
            cmd = self.robot_dds.get_robot_command() if self.robot_dds else None
            if cmd and "motor_cmd" in cmd:
                mc = cmd["motor_cmd"]
                positions = mc["positions"]
                modes = mc.get("modes") or [1] * len(positions)
                kps = mc.get("kp") or [0.0] * len(positions)
                for side in self.sides:
                    s0 = ARM_SLOTS[side]
                    if len(positions) >= s0 + 7:
                        self._targets[self._arm_ids[side]] = torch.tensor(positions[s0:s0 + 7], dtype=torch.float32, device=self._device)
                slot = DEX1_LOWCMD_SLOTS["left"]
                if self._dex1 and slot < len(positions) and modes[slot] == 1 and kps[slot] > 0.0:
                    internal_gripper = positions[slot]
                hand_modes, hand_torques = modes, mc.get("torques")

            if self._dex1:
                if internal_gripper is not None:
                    self._targets[self._fingers] = dex1_motor_to_g1d_finger(internal_gripper)
                elif self.gripper_dds:
                    g = self.gripper_dds.get_gripper_command()
                    p = (g or {}).get("left_gripper_cmd", {}).get("positions", [])
                    if p:
                        self._targets[self._fingers] = float(g1_dex1_to_g1d_finger(float(p[0])))
            if self._torque_hands:
                self._torque_hands.apply(self._targets, hand_modes, hand_torques)
            if self._o6 and self.o6_dds:
                h = (self.o6_dds.get_hand_commands() or {}).get("left_hand_cmd", {})
                raw = h.get("positions", [])
                if len(raw) >= 6 and float(h.get("stamp", 0.0)) >= self._start_time:
                    c = torch.tensor([raw_to_fraction_closed(v) for v in raw[:6]], dtype=torch.float32, device=self._device)
                    self._targets[self._o6_ids] = self._o6_open + c * (self._o6_closed - self._o6_open)
            if self._o6:
                for dip, prox, m in self._o6_mimic:
                    self._targets[dip] = self._targets[prox] * m
            return ((self._targets - self._offset) / self._scale).unsqueeze(0)
        except Exception as e:
            print(f"[{self.name}] Get DDS action failed: {e}")
            return None

    def cleanup(self):
        try:
            for dds in (self.robot_dds, self.gripper_dds, self.o6_dds):
                if dds:
                    dds.stop_communication()
        except Exception as e:
            print(f"[{self.name}] Clean up DDS resources failed: {e}")
