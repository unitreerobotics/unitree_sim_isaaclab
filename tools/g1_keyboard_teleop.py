#!/usr/bin/env python3
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
Keyboard teleoperation test for the simulated Unitree G1 (29 dof, fixed base): arms + hands.

Same DDS topics as the real robot / xr_teleoperate (DDS domain 1):

    arms      rt/lowcmd                 (unitree_hg/LowCmd, motors 15..28; the other motors keep their state)
    dex1      rt/dex1/{left,right}/cmd  (unitree_go/MotorCmds, q: 0 closed .. 5.4 open)
    dex3      rt/dex3/{left,right}/cmd  (unitree_hg/HandCmd, 7 joints in rad)
    inspire   rt/inspire/cmd            (unitree_go/MotorCmds, 12 values, 1 open .. 0 closed)

Start the simulator first, e.g.

    python sim_main.py --device cpu --task Isaac-PickPlace-Cylinder-G129-Dex1-Joint --enable_dex1_dds --robot_type g129
    python sim_main.py --device cpu --task Isaac-PickPlace-Cylinder-G129-Dex3-Joint --enable_dex3_dds --robot_type g129
    python sim_main.py --device cpu --task Isaac-PickPlace-Cylinder-G129-Inspire-Joint --enable_inspire_dds --robot_type g129

then, in another terminal (same conda env, it needs unitree_sdk2py and pynput):

    python tools/g1_keyboard_teleop.py --hand dex1      # or dex3 / inspire

Keys (hold to move). The hand (tool point between the fingers) is moved in the torso frame,
the arm joints are solved by inverse kinematics (tools/arm_ik.py):
    I / K        hand forward / backward          J / L   hand left / right
    U / O        hand up / down
    [ / ]        roll  (about forward axis)       ; / '   pitch (about left axis)
    , / .        yaw   (about up axis)
    TAB          switch hand: left / right / both (mirrored)     P   arms back to the start pose
    Z / X        left hand close / open           C / V   right hand close / open
    ESC          quit
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arm_ik import CartesianArms  # noqa: E402
from g1d_keyboard_teleop import RATE_HZ, Keyboard, cartesian_keys, clamp  # noqa: E402  (shared keyboard)
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber  # noqa: E402
from unitree_sdk2py.idl.default import (  # noqa: E402
    unitree_go_msg_dds__MotorCmd_,
    unitree_hg_msg_dds__HandCmd_,
    unitree_hg_msg_dds__LowCmd_,
)
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_  # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, LowCmd_, LowState_  # noqa: E402
from unitree_sdk2py.utils.crc import CRC  # noqa: E402

GRIP_RATE = 1.0  # full open <-> closed in 1 s
DEX1_OPEN_Q = 5.4  # motor q of a fully open Dex1 (0 = closed)
TCP_OFFSET = {"dex1": (0.15, 0.0, 0.0), "dex3": (0.10, 0.0, 0.0), "inspire": (0.13, 0.0, 0.0)}  # in wrist_yaw_link
# Dex3 joint order: thumb_0, thumb_1, thumb_2, middle_0, middle_1, index_0, index_1 (closed = fist)
DEX3_CLOSED = {
    "left": [0.0, 0.5, 1.2, -1.0, -1.2, -1.0, -1.2],
    "right": [0.0, -0.5, -1.2, 1.0, 1.2, 1.0, 1.2],
}


class G1KeyboardTeleop:
    def __init__(self, hand: str, domain: int, interface: str | None):
        if interface:
            ChannelFactoryInitialize(domain, interface)
        else:
            ChannelFactoryInitialize(domain)
        self.hand = hand
        self.crc = CRC()
        self.lowcmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_pub.Init()
        self.lowstate = None
        ChannelSubscriber("rt/lowstate", LowState_).Init(lambda m: setattr(self, "lowstate", m), 1)

        self.dex1_state = {"left": None, "right": None}
        if hand == "dex1":
            self.hand_pub = {s: ChannelPublisher(f"rt/dex1/{s}/cmd", MotorCmds_) for s in ("left", "right")}
            for s in ("left", "right"):
                ChannelSubscriber(f"rt/dex1/{s}/state", MotorStates_).Init(lambda m, s=s: self.dex1_state.__setitem__(s, m), 1)
        elif hand == "dex3":
            self.hand_pub = {s: ChannelPublisher(f"rt/dex3/{s}/cmd", HandCmd_) for s in ("left", "right")}
        else:
            self.hand_pub = {"both": ChannelPublisher("rt/inspire/cmd", MotorCmds_)}
        for p in self.hand_pub.values():
            p.Init()

        self.q = [0.0] * 29
        self.grasp = {"left": 0.0, "right": 0.0}  # 0 = open, 1 = closed
        self.arms = None

    def wait_for_sim(self, timeout=10.0):
        print("waiting for the simulator (rt/lowstate) ...")
        t0 = time.time()
        while self.lowstate is None and time.time() - t0 < timeout:
            time.sleep(0.05)
        if self.lowstate is None:
            print("no rt/lowstate received - is sim_main.py running (--robot_type g129)? Starting from zero targets.")
            self.arms = CartesianArms(self.q[15:29], TCP_OFFSET[self.hand])
            return
        # start from the current pose so nothing jumps (non-arm motors keep their state)
        for i in range(29):
            self.q[i] = float(self.lowstate.motor_state[i].q)
        for s in ("left", "right"):
            st = self.dex1_state[s]
            if st is not None and len(st.states):
                self.grasp[s] = clamp(1.0 - float(st.states[0].q) / DEX1_OPEN_Q, 0.0, 1.0)
        self.arms = CartesianArms(self.q[15:29], TCP_OFFSET[self.hand])
        print("connected.")

    # ------------------------------------------------------------------ publishing
    def publish(self):
        msg = unitree_hg_msg_dds__LowCmd_()
        msg.mode_pr = 0
        msg.mode_machine = int(self.lowstate.mode_machine) if self.lowstate is not None else 0
        for i in range(29):
            m = msg.motor_cmd[i]
            m.mode, m.q, m.dq, m.tau, m.kp, m.kd = 1, float(self.q[i]), 0.0, 0.0, 60.0, 1.5
        msg.crc = self.crc.Crc(msg)
        self.lowcmd_pub.Write(msg)

        if self.hand == "dex1":
            for s, pub in self.hand_pub.items():
                g = MotorCmds_()
                g.cmds = [unitree_go_msg_dds__MotorCmd_()]
                g.cmds[0].q = (1.0 - self.grasp[s]) * DEX1_OPEN_Q
                g.cmds[0].kp, g.cmds[0].kd = 5.0, 0.05
                pub.Write(g)
        elif self.hand == "dex3":
            for s, pub in self.hand_pub.items():
                h = unitree_hg_msg_dds__HandCmd_()
                for i, closed in enumerate(DEX3_CLOSED[s]):
                    c = h.motor_cmd[i]
                    c.q, c.dq, c.tau, c.kp, c.kd = self.grasp[s] * closed, 0.0, 0.0, 1.5, 0.2
                pub.Write(h)
        else:  # inspire: 6 right values then 6 left values, 1 = open, 0 = closed
            g = MotorCmds_()
            g.cmds = [unitree_go_msg_dds__MotorCmd_() for _ in range(12)]
            for i in range(12):
                g.cmds[i].q = 1.0 - self.grasp["right" if i < 6 else "left"]
            self.hand_pub["both"].Write(g)

    # ------------------------------------------------------------------ control
    def step(self, held, once, dt):
        for k in once:
            if k == "tab":
                self.arms.cycle_side()
            elif k == "p":
                self.arms.reset()
        dpos, drot = cartesian_keys(held)
        self.arms.step(dpos, drot, dt)
        self.q[15:29] = self.arms.q_arms()

        for s, close_k, open_k in (("left", "z", "x"), ("right", "c", "v")):
            dg = ((1 if close_k in held else 0) - (1 if open_k in held else 0)) * GRIP_RATE * dt
            self.grasp[s] = clamp(self.grasp[s] + dg, 0.0, 1.0)

    def status(self):
        pl, pr = self.arms.tcp_position("left"), self.arms.tcp_position("right")
        return (
            f"\r hand: {self.arms.side:5s} | L tcp x={pl[0]:+.3f} y={pl[1]:+.3f} z={pl[2]:+.3f} | "
            f"R tcp x={pr[0]:+.3f} y={pr[1]:+.3f} z={pr[2]:+.3f} | {self.hand} close L={self.grasp['left'] * 100:3.0f}% "
            f"R={self.grasp['right'] * 100:3.0f}%   "
        )


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hand", choices=["dex1", "dex3", "inspire"], default="dex1", help="end effector of the simulated G1")
    ap.add_argument("--domain", type=int, default=1, help="DDS domain id (the simulator uses 1)")
    ap.add_argument("--interface", type=str, default=None, help="network interface for DDS (default: auto)")
    args = ap.parse_args()

    print("Keys (hold to move)" + __doc__.partition("Keys (hold to move)")[2])
    teleop = G1KeyboardTeleop(args.hand, args.domain, args.interface)
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
