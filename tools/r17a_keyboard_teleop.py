#!/usr/bin/env python3
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
Keyboard teleoperation test for the simulated Unitree R1-7A arm robots: one left arm with a Dex1 gripper, a Dex2-5
hand or a LinkerHand O6. The robot is detected from the running task (rt/sim_state);
``--hand dex1|dex2_5|o6`` selects it by hand.

DDS topics (domain 1), as in robots/unitree_r17a.py:

    arm       rt/lowcmd   motors 0..6 (left arm J1..J7), unitree_hg/LowCmd
    Dex1      rt/dex1/left/cmd (unitree_go/MotorCmds: q in [0 closed, 5.4 open]); start the simulator with --enable_dex1_dds
    Dex2-5    rt/lowcmd   motors 31 / 32 (fingers, thumb): torque only (mode 1, q = kp = kd = 0,
              tau +-0.3 Nm, ramped with 0.6 Nm/s)
    O6        rt/linkerhand/left/cmd (unitree_go/MotorCmds, 6 raw angles 0..255, 255 = open); start the simulator
              with --enable_linkerhand_dds
    state     rt/lowstate, rt/dex1/left/state, rt/linkerhand/left/state, rt/sim_state (task name)

Start the simulator first, e.g.

    python sim_main.py --device cpu --task Isaac-PickPlace-RedBlock-R17A-Dex1-Joint --enable_dex1_dds --robot_type r17a

then, in another terminal (same conda env, it needs unitree_sdk2py and pynput):

    python tools/r17a_keyboard_teleop.py

Keys (hold to move). The hand (tool point) is moved in the robot mount frame (x forward, y left, z up),
the arm joints are solved by inverse kinematics (tools/arm_ik.py):
    I / K        hand forward / backward          J / L   hand left / right
    U / O        hand up / down
    [ / ]        roll  (about forward axis)       ; / '   pitch (about left axis)
    , / .        yaw   (about up axis)
    P            arm back to the start pose
    Z / X        hand close / open
                 Dex1 / O6: the position follows while a key is held
                 Dex2-5: closing / opening torque while a key is held; released = zero torque (the hand keeps its grip)
    ESC          quit
"""

import argparse
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(1, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from arm_ik import CartesianArms  # noqa: E402
from g1d_keyboard_teleop import GRIPPER_RANGE, GRIPPER_RATE, RATE_HZ, Keyboard, cartesian_keys, clamp  # noqa: E402
from robots.dex2_5_hand import DEX2_5_LOWCMD_SLOTS, DEX2_5_MAX_TORQUE  # noqa: E402  (no simulator imports)
from robots.linkerhand_o6 import O6_NUM_MOTORS, O6_RAW_MAX, fraction_closed_to_raw, raw_to_fraction_closed  # noqa: E402
from robots.r17a_urdf import R17A_ARMS, R17A_TCP, r17a_asset  # noqa: E402
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber  # noqa: E402
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_, unitree_hg_msg_dds__LowCmd_  # noqa: E402
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_  # noqa: E402
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_  # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_  # noqa: E402
from unitree_sdk2py.utils.crc import CRC  # noqa: E402

HANDS = ("dex1", "dex2_5", "o6")
ARM_SLOTS = {"left": 0, "right": 7}
NUM_MOTORS = 14
DEX2_5_TORQUE_RATE = 0.6  # Nm/s (unirobot qtau_rate_limit)
O6_RATE = 1.0  # closure fraction per second while Z / X is held
_TASK_NAME = re.compile(r'"task_name"\s*:\s*"([^"]*)"')


def hand_of_task(task_name: str | None) -> str | None:
    name = (task_name or "").lower().replace("-", "_")
    if "r17a" not in name:
        return None
    return "dex2_5" if "dex2_5" in name else "o6" if "_o6_" in name else "dex1" if "dex1" in name else None


class R17AKeyboardTeleop:
    def __init__(self, domain: int, interface: str | None, hand: str = "auto"):
        self.hand = hand
        self.task_name = None
        if interface:
            ChannelFactoryInitialize(domain, interface)
        else:
            ChannelFactoryInitialize(domain)
        self.crc = CRC()
        self.lowcmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_pub.Init()
        self.grip_pub = ChannelPublisher("rt/dex1/left/cmd", MotorCmds_)
        self.grip_pub.Init()
        self.o6_pub = ChannelPublisher("rt/linkerhand/left/cmd", MotorCmds_)
        self.o6_pub.Init()
        self.lowstate = self.grip_state = self.o6_state = None
        ChannelSubscriber("rt/lowstate", LowState_).Init(lambda m: setattr(self, "lowstate", m), 1)
        ChannelSubscriber("rt/dex1/left/state", MotorStates_).Init(lambda m: setattr(self, "grip_state", m), 1)
        ChannelSubscriber("rt/linkerhand/left/state", MotorStates_).Init(lambda m: setattr(self, "o6_state", m), 1)
        self._sim_state_sub = ChannelSubscriber("rt/sim_state", String_)
        self._sim_state_sub.Init(self._on_sim_state, 1)
        self.q = [0.0] * NUM_MOTORS
        self.grip = GRIPPER_RANGE[1]  # Dex1 motor q (open)
        self.o6_close = 0.0  # O6 closure fraction 0 open .. 1 closed (all 6 joints)
        self.hand_tau = {"left": 0.0, "right": 0.0}  # Dex2-5 motor torque (fingers and thumb of a hand alike)
        self.arms = None

    def _on_sim_state(self, msg):
        if self.task_name is None:
            m = _TASK_NAME.search(msg.data)
            if m:
                self.task_name = m.group(1)

    def wait_for_sim(self, timeout=10.0):
        print("waiting for the simulator (rt/lowstate, rt/sim_state) ...")
        t0 = time.time()
        while (self.lowstate is None or (self.hand not in HANDS and self.task_name is None)) and time.time() - t0 < timeout:
            time.sleep(0.05)
        named = hand_of_task(self.task_name)
        if self.hand not in HANDS:
            self.hand = named or "dex1"
            reason = f"task {self.task_name}" if named else f"{'task ' + str(self.task_name) if self.task_name else 'no rt/sim_state'}: assuming Dex1, use --hand"
        else:
            reason = "--hand"
            if named and named != self.hand:
                print(f"WARNING: --hand {self.hand}, but the simulator runs {self.task_name} ({named}).")
        if self.task_name and "r17a" not in self.task_name.lower():
            print(f"WARNING: the simulator runs {self.task_name}, which is not an R1-7A task.")
        self.sides = tuple(R17A_ARMS[self.hand])
        print(f"robot: {len(self.sides)} arm(s) with {self.hand} [{reason}]")
        if self.lowstate is None:
            print("no rt/lowstate received - is sim_main.py running with --robot_type r17a? Starting from zero targets.")
        else:
            for i in range(NUM_MOTORS):  # start from the current pose so nothing jumps
                self.q[i] = float(self.lowstate.motor_state[i].q)
            if self.hand == "dex1" and self.grip_state is not None and len(self.grip_state.states):
                self.grip = clamp(float(self.grip_state.states[0].q), *GRIPPER_RANGE)
            if self.hand == "o6" and self.o6_state is not None and len(self.o6_state.states) >= O6_NUM_MOTORS:
                self.o6_close = sum(raw_to_fraction_closed(s.q) for s in self.o6_state.states[:O6_NUM_MOTORS]) / O6_NUM_MOTORS
            print("connected.")
        tips = {s: R17A_TCP[self.hand][s][0] for s in self.sides}
        tcp = R17A_TCP[self.hand][self.sides[0]][1]
        q_arms = [v for s in self.sides for v in self.q[ARM_SLOTS[s]:ARM_SLOTS[s] + 7]]
        self.arms = CartesianArms(q_arms, tcp, r17a_asset(self.hand), base_link="mount_link", sides=self.sides, tip_links=tips)
        if len(self.sides) == 2:  # the right hand's tool point is mirrored (palm on the other side)
            from arm_ik import ArmKinematics

            self.arms.arm["right"] = ArmKinematics("right", R17A_TCP[self.hand]["right"][1], r17a_asset(self.hand), "mount_link", tips["right"])
            self.arms._sync_targets()

    def publish(self):
        msg = unitree_hg_msg_dds__LowCmd_()
        msg.mode_pr = 0
        msg.mode_machine = int(self.lowstate.mode_machine) if self.lowstate is not None else 0
        for s in self.sides:
            for i in range(ARM_SLOTS[s], ARM_SLOTS[s] + 7):
                m = msg.motor_cmd[i]
                m.mode, m.q, m.dq, m.tau, m.kp, m.kd = 1, float(self.q[i]), 0.0, 0.0, 60.0, 1.5
        if self.hand == "dex2_5":
            for s, slots in ((s, DEX2_5_LOWCMD_SLOTS[s]) for s in self.sides):
                for slot in slots:
                    m = msg.motor_cmd[slot]
                    m.mode, m.q, m.dq, m.tau, m.kp, m.kd = 1, 0.0, 0.0, float(self.hand_tau[s]), 0.0, 0.0
        msg.crc = self.crc.Crc(msg)
        self.lowcmd_pub.Write(msg)
        if self.hand == "dex1":
            g = MotorCmds_()
            g.cmds = [unitree_go_msg_dds__MotorCmd_()]
            g.cmds[0].q, g.cmds[0].kp, g.cmds[0].kd = float(self.grip), 5.0, 0.05
            self.grip_pub.Write(g)
        elif self.hand == "o6":
            h = MotorCmds_()
            h.cmds = [unitree_go_msg_dds__MotorCmd_() for _ in range(O6_NUM_MOTORS)]
            for c in h.cmds:
                c.q = float(fraction_closed_to_raw(self.o6_close))
            self.o6_pub.Write(h)

    def step(self, held, once, dt):
        for k in once:
            if k == "tab":
                self.arms.cycle_side()
            elif k == "p":
                self.arms.reset()
        dpos, drot = cartesian_keys(held)
        self.arms.step(dpos, drot, dt)
        q_arms = self.arms.q_arms()
        for i, s in enumerate(self.sides):
            self.q[ARM_SLOTS[s]:ARM_SLOTS[s] + 7] = q_arms[7 * i:7 * i + 7]

        def axis(pos, neg):
            return (1 if pos in held else 0) - (1 if neg in held else 0)

        if self.hand == "dex2_5":
            for s, close_k, open_k in (("left", "z", "x"), ("right", "c", "v"))[:len(self.sides)]:
                goal = axis(close_k, open_k) * DEX2_5_MAX_TORQUE  # like the unirobot runtime: ramp up, release at once
                step = DEX2_5_TORQUE_RATE * dt
                self.hand_tau[s] = 0.0 if goal == 0.0 else clamp(goal, self.hand_tau[s] - step, self.hand_tau[s] + step)
        elif self.hand == "dex1":
            self.grip = clamp(self.grip - axis("z", "x") * GRIPPER_RATE * dt, *GRIPPER_RANGE)
        else:
            self.o6_close = clamp(self.o6_close + axis("z", "x") * O6_RATE * dt, 0.0, 1.0)

    def status(self):
        tcp = " ".join(f"{s[0].upper()}=({p[0]:+.2f},{p[1]:+.2f},{p[2]:+.2f})" for s in self.sides for p in [self.arms.tcp_position(s)])
        if self.hand == "dex2_5":
            hand = "hand tau " + " ".join(f"{s[0].upper()}={self.hand_tau[s]:+.2f}" for s in self.sides) + " Nm"
        elif self.hand == "dex1":
            hand = f"grip {self.grip:.1f}"
        else:
            hand = f"O6 raw {fraction_closed_to_raw(self.o6_close):.0f}/{O6_RAW_MAX:.0f}"
        return f"\r arm {self.arms.side:5s} {tcp} | {hand}   "


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--domain", type=int, default=1, help="DDS domain id (the simulator uses 1)")
    ap.add_argument("--interface", type=str, default=None, help="network interface for DDS (default: auto)")
    ap.add_argument("--hand", choices=["auto", *HANDS], default="auto",
                    help="end effector of the simulated robot; auto (default): detected from the running task (rt/sim_state)")
    args = ap.parse_args()

    print("Keys (hold to move)" + __doc__.partition("Keys (hold to move)")[2])
    teleop = R17AKeyboardTeleop(args.domain, args.interface, args.hand)
    teleop.wait_for_sim()
    kb = Keyboard()
    print(f"keyboard backend: {kb.backend}. Press ESC to quit.\n")
    dt = 1.0 / RATE_HZ
    try:
        last = time.time()
        while not kb.quit:
            now = time.time()
            held, once = kb.snapshot()
            teleop.step(held, once, now - last)
            last = now
            teleop.publish()
            sys.stdout.write(teleop.status())
            sys.stdout.flush()
            time.sleep(max(0.0, dt - (time.time() - now)))
    except KeyboardInterrupt:
        pass
    finally:
        kb.close()
        print("\nbye")


if __name__ == "__main__":
    main()
