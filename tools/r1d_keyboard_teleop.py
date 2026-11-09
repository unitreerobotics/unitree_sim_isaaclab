#!/usr/bin/env python3
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
Keyboard teleoperation test for the simulated Unitree R1-D (base, lift column, arms, Dex1 grippers, waist, head).

Same DDS topics as the real robot / xr_teleoperate (DDS domain 1):

    arms      rt/lowcmd   motors 15..28 (unitree_hg/LowCmd)
    waist     rt/lowcmd   motor 13 (waist yaw)
    head      rt/lowcmd   motors 29 / 30 (head pitch / yaw)
    grippers  rt/dex1/{left,right}/cmd (unitree_go/MotorCmds: q in [0 closed, 5.4 open])
    base+lift rt/cmd_vel (geometry_msgs/Twist, NORMALIZED [-1, 1]: linear.x, angular.z, linear.z = column)
    state     rt/state_vel (normalized), rt/height_state (z = column height [mm]), rt/height_alarm

Start the simulator first:

    python sim_main.py --device cpu --task Isaac-PickPlace-Cylinder-R1D-Dex1-Joint --enable_dex1_dds --robot_type r1d

then, in another terminal (same conda env, it needs unitree_sdk2py and pynput):

    python tools/r1d_keyboard_teleop.py

Keys (hold to move). The hand (tool point between the fingers) is moved in the waist frame,
the arm joints are solved by inverse kinematics (tools/arm_ik.py):
    I / K        hand forward / backward          J / L   hand left / right
    U / O        hand up / down
    [ / ]        roll  (about forward axis)       ; / '   pitch (about left axis)
    , / .        yaw   (about up axis)
    TAB          switch hand: left / right / both (mirrored)     P   arms / waist / head back to the start pose
    W / S        base forward / backward          A / D   base turn left / right
    R / F        lift column up / down            SPACE   stop the base and the column
    Q / E        waist yaw left / right
    T / G        head down / up                   Y / H   head yaw left / right
    Z / X        left gripper close / open        C / V   right gripper close / open
    ESC          quit
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arm_ik import CartesianArms  # noqa: E402
from g1d_keyboard_teleop import GRIPPER_RANGE, GRIPPER_RATE, RATE_HZ, Keyboard, cartesian_keys, clamp  # noqa: E402
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber  # noqa: E402
from unitree_sdk2py.idl.default import (  # noqa: E402
    geometry_msgs_msg_dds__Twist_,
    unitree_go_msg_dds__MotorCmd_,
    unitree_hg_msg_dds__LowCmd_,
)
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Point32_, Twist_  # noqa: E402
from unitree_sdk2py.idl.unitree_go.msg.dds_ import Error_  # noqa: E402
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_  # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_  # noqa: E402
from unitree_sdk2py.utils.crc import CRC  # noqa: E402

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
R1D_URDF = os.path.join(_REPO, "assets", "urdf", "r1_d_description", "r1_d_with_dex1_1.urdf")
TCP_OFFSET = (0.15, 0.0, 0.02)  # tool point between the Dex1 fingers, in <side>_wrist_yaw_link
NUM_MOTORS = 31  # 0..30 used (xr_teleoperate R1_A7_JointIndex)
WAIST_YAW, HEAD_PITCH, HEAD_YAW = 13, 29, 30
LIMITS = {WAIST_YAW: (-2.618, 2.618), HEAD_PITCH: (-0.628, 0.628), HEAD_YAW: (-2.007, 2.007)}
JOINT_RATE = 0.6  # rad/s for waist / head
BASE_X, BASE_WZ = 0.6, 0.6  # normalized rt/cmd_vel while W/S, A/D are held (x gear vmax / 1.2 rad/s)


class R1DKeyboardTeleop:
    def __init__(self, domain: int, interface: str | None):
        if interface:
            ChannelFactoryInitialize(domain, interface)
        else:
            ChannelFactoryInitialize(domain)
        self.crc = CRC()
        self.lowcmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_pub.Init()
        self.grip_pub = {s: ChannelPublisher(f"rt/dex1/{s}/cmd", MotorCmds_) for s in ("left", "right")}
        for p in self.grip_pub.values():
            p.Init()
        self.vel_pub = ChannelPublisher("rt/cmd_vel", Twist_)
        self.vel_pub.Init()
        self.state_vel = self.height = self.alarm = None
        ChannelSubscriber("rt/state_vel", Twist_).Init(lambda m: setattr(self, "state_vel", m), 1)
        ChannelSubscriber("rt/height_state", Point32_).Init(lambda m: setattr(self, "height", m), 1)
        ChannelSubscriber("rt/height_alarm", Error_).Init(lambda m: setattr(self, "alarm", m), 1)
        self.cmd_vel = (0.0, 0.0, 0.0)
        self.lowstate = None
        self.grip_state = {"left": None, "right": None}
        ChannelSubscriber("rt/lowstate", LowState_).Init(lambda m: setattr(self, "lowstate", m), 1)
        for s in ("left", "right"):
            ChannelSubscriber(f"rt/dex1/{s}/state", MotorStates_).Init(lambda m, s=s: self.grip_state.__setitem__(s, m), 1)
        self.q = [0.0] * NUM_MOTORS
        self.q_start = list(self.q)
        self.grip = {"left": GRIPPER_RANGE[1], "right": GRIPPER_RANGE[1]}
        self.arms = None

    def wait_for_sim(self, timeout=10.0):
        print("waiting for the simulator (rt/lowstate) ...")
        t0 = time.time()
        while self.lowstate is None and time.time() - t0 < timeout:
            time.sleep(0.05)
        if self.lowstate is None:
            print("no rt/lowstate received - is sim_main.py running with --robot_type r1d? Starting from zero targets.")
        else:
            for i in range(NUM_MOTORS):  # start from the current pose so nothing jumps
                self.q[i] = float(self.lowstate.motor_state[i].q)
            for s in ("left", "right"):
                st = self.grip_state[s]
                if st is not None and len(st.states):
                    self.grip[s] = clamp(float(st.states[0].q), *GRIPPER_RANGE)
            print("connected.")
        self.q_start = list(self.q)
        self.arms = CartesianArms(self.q[15:29], TCP_OFFSET, R1D_URDF, base_link="waist_yaw_link")

    def publish(self):
        t = geometry_msgs_msg_dds__Twist_()
        t.linear.x, t.angular.z, t.linear.z = self.cmd_vel
        self.vel_pub.Write(t)
        msg = unitree_hg_msg_dds__LowCmd_()
        msg.mode_pr = 0
        msg.mode_machine = int(self.lowstate.mode_machine) if self.lowstate is not None else 0
        for i in range(NUM_MOTORS):
            m = msg.motor_cmd[i]
            m.mode, m.q, m.dq, m.tau, m.kp, m.kd = 1, float(self.q[i]), 0.0, 0.0, 60.0, 1.5
        msg.crc = self.crc.Crc(msg)
        self.lowcmd_pub.Write(msg)
        for s, pub in self.grip_pub.items():
            g = MotorCmds_()
            g.cmds = [unitree_go_msg_dds__MotorCmd_()]
            g.cmds[0].q = float(self.grip[s])
            g.cmds[0].kp, g.cmds[0].kd = 5.0, 0.05
            pub.Write(g)

    def step(self, held, once, dt):
        for k in once:
            if k == "tab":
                self.arms.cycle_side()
            elif k == "p":
                self.arms.reset()
                for i in (WAIST_YAW, HEAD_PITCH, HEAD_YAW):
                    self.q[i] = self.q_start[i]

        def axis(pos, neg):
            return (1 if pos in held else 0) - (1 if neg in held else 0)

        if "space" in held:
            self.cmd_vel = (0.0, 0.0, 0.0)
        else:
            self.cmd_vel = (axis("w", "s") * BASE_X, axis("a", "d") * BASE_WZ, float(axis("r", "f")))

        for i, d in ((WAIST_YAW, axis("q", "e")), (HEAD_PITCH, axis("t", "g")), (HEAD_YAW, axis("y", "h"))):
            self.q[i] = clamp(self.q[i] + d * JOINT_RATE * dt, *LIMITS[i])

        dpos, drot = cartesian_keys(held)
        self.arms.step(dpos, drot, dt)
        self.q[15:29] = self.arms.q_arms()

        for s, close_k, open_k in (("left", "z", "x"), ("right", "c", "v")):
            dg = ((1 if open_k in held else 0) - (1 if close_k in held else 0)) * GRIPPER_RATE * dt
            self.grip[s] = clamp(self.grip[s] + dg, *GRIPPER_RANGE)

    def status(self):
        pl, pr = self.arms.tcp_position("left"), self.arms.tcp_position("right")
        sv = self.state_vel
        vel = f"x={sv.linear.x:+.2f} wz={sv.angular.z:+.2f} z={sv.linear.z:+.2f}" if sv is not None else "-"
        h = f"{self.height.z:5.0f}mm" if self.height is not None else "-"
        al = f"{self.alarm.state:04b}" if self.alarm is not None else "-"
        return (
            f"\r state_vel {vel} | column {h} alarm {al} | hand {self.arms.side:5s} L=({pl[0]:+.2f},{pl[1]:+.2f},{pl[2]:+.2f}) R=({pr[0]:+.2f},{pr[1]:+.2f},{pr[2]:+.2f}) | "
            f"waist {self.q[WAIST_YAW]:+.2f} head p={self.q[HEAD_PITCH]:+.2f} y={self.q[HEAD_YAW]:+.2f} | "
            f"grip L={self.grip['left']:.1f} R={self.grip['right']:.1f}   "
        )


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--domain", type=int, default=1, help="DDS domain id (the simulator uses 1)")
    ap.add_argument("--interface", type=str, default=None, help="network interface for DDS (default: auto)")
    args = ap.parse_args()

    print("Keys (hold to move)" + __doc__.partition("Keys (hold to move)")[2])
    teleop = R1DKeyboardTeleop(args.domain, args.interface)
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
