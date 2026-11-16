#!/usr/bin/env python3
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
Keyboard teleoperation test for the simulated Unitree R1-A7-D (base, lift column, arms, waist, head and its end
effectors: Dex1 grippers or Dex2-5 tendon hands). The end effector is detected from the running task (rt/sim_state),
``--hand dex1`` / ``--hand dex2_5`` selects it by hand.

Same DDS topics as the real robot / xr_teleoperate (DDS domain 1):

    arms      rt/lowcmd   motors 15..28 (unitree_hg/LowCmd)
    waist     rt/lowcmd   motor 13 (waist yaw)
    head      rt/lowcmd   motors 29 / 30 (head pitch / yaw)
    grippers  rt/dex1/{left,right}/cmd (unitree_go/MotorCmds: q in [0 closed, 5.4 open])
    Dex2-5    rt/lowcmd   motors 31 / 32 (left fingers, thumb), 33 / 34 (right fingers, thumb): torque only
              (mode 1, q = dq = kp = kd = 0, tau +-0.3 Nm ramped with 0.6 Nm/s; like unirobot r1_a7_d/dex2_5)
    base+lift rt/cmd_vel (geometry_msgs/Twist, NORMALIZED [-1, 1]: linear.x, angular.z, linear.z = column)
    state     rt/state_vel (normalized), rt/height_state (z = column height [mm]), rt/height_alarm,
              rt/lowstate (Dex2-5: motors 31..34), rt/sim_state (task name -> end effector)

Start the simulator first:

    python sim_main.py --device cpu --task Isaac-PickPlace-Cylinder-R1A7D-Dex1-Joint --enable_dex1_dds --robot_type r1a7d

then, in another terminal (same conda env, it needs unitree_sdk2py and pynput):

    python tools/r1a7d_keyboard_teleop.py

The same command works for the Dex2-5 tasks (their hands need no hand DDS flag):

    python sim_main.py --device cpu --task Isaac-PickPlace-Cylinder-R1A7D-Dex2_5-Joint --robot_type r1a7d

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
                 Dex1:   the gripper position follows while a key is held
                 Dex2-5: closing / opening torque while a key is held (fingers and thumb together);
                         released = zero torque: the hand keeps its pose and holds what it has grasped,
                         open it with X / V
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
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber  # noqa: E402
from unitree_sdk2py.idl.default import (  # noqa: E402
    geometry_msgs_msg_dds__Twist_,
    unitree_go_msg_dds__MotorCmd_,
    unitree_hg_msg_dds__LowCmd_,
)
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Point32_, Twist_  # noqa: E402
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_  # noqa: E402
from unitree_sdk2py.idl.unitree_go.msg.dds_ import Error_  # noqa: E402
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_  # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_  # noqa: E402
from unitree_sdk2py.utils.crc import CRC  # noqa: E402

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
R1A7D_URDF = {hand: os.path.join(_REPO, "assets", "urdf", "r1_d_description", f"r1_d_with_{name}.urdf")
              for hand, name in (("dex1", "dex1_1"), ("dex2_5", "dex2_5"))}
# tool point in <side>_wrist_yaw_link: between the Dex1 fingers / in the Dex2-5 palm, at the finger bases
TCP_OFFSET = {"dex1": (0.15, 0.0, 0.02), "dex2_5": (0.12, 0.0, 0.02)}
DEX2_5_TORQUE_RATE = 0.6  # Nm/s (unirobot qtau_rate_limit)
HANDS = ("dex1", "dex2_5")
_TASK_NAME = re.compile(r'"task_name"\s*:\s*"([^"]*)"')  # in the rt/sim_state JSON (its scene state is not parsed)


def hand_of_task(task_name: str | None) -> str | None:
    """End effector named in a task name (``Isaac-...-R1A7D-Dex2_5-Joint`` -> "dex2_5"), None if it names none."""
    name = (task_name or "").lower().replace("-", "_")
    return "dex2_5" if "dex2_5" in name else "dex1" if "dex1" in name else None


def detect_hand(task_name: str | None, dex1_state_seen: bool, lowstate_q=None) -> tuple[str, str]:
    """(hand, reason) of the simulated robot from what the simulator publishes.

    1. the task name of rt/sim_state; 2. rt/dex1/*/state is only published by the Dex1 tasks; 3. rt/lowstate
    motors 32 / 34 are the Dex2-5 thumbs and always 0 with Dex1 grippers (0 as well for open Dex2-5 hands, so
    this only tells closed Dex2-5 hands); otherwise Dex1.
    """
    hand = hand_of_task(task_name)
    if hand:
        return hand, f"task {task_name}"
    where = f"task {task_name} names no end effector" if task_name else "no rt/sim_state"
    if dex1_state_seen:
        return "dex1", f"{where}; rt/dex1 state received"
    if lowstate_q is not None and len(lowstate_q) > 34 and max(abs(lowstate_q[32]), abs(lowstate_q[34])) > 1.0e-3:
        return "dex2_5", f"{where}; rt/lowstate motors 32 / 34 are in use"
    return "dex1", f"{where}; assuming Dex1 - use --hand dex2_5 for the Dex2-5 hands"

NUM_MOTORS = 31  # 0..30 used (xr_teleoperate R1_A7_JointIndex)
WAIST_YAW, HEAD_PITCH, HEAD_YAW = 13, 29, 30
LIMITS = {WAIST_YAW: (-2.618, 2.618), HEAD_PITCH: (-0.628, 0.628), HEAD_YAW: (-2.007, 2.007)}
JOINT_RATE = 0.6  # rad/s for waist / head
BASE_X, BASE_WZ = 0.6, 0.6  # normalized rt/cmd_vel while W/S, A/D are held (x gear vmax / 1.2 rad/s)


class R1A7DKeyboardTeleop:
    def __init__(self, domain: int, interface: str | None, hand: str = "auto"):
        self.hand = hand  # "auto" until wait_for_sim() has detected it
        self.task_name = None
        self.hand_tau = {"left": 0.0, "right": 0.0}  # Dex2-5 motor torque (fingers and thumb of a hand alike)
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
        self._sim_state_sub = ChannelSubscriber("rt/sim_state", String_)
        self._sim_state_sub.Init(self._on_sim_state, 1)
        self.q = [0.0] * NUM_MOTORS
        self.q_start = list(self.q)
        self.grip = {"left": GRIPPER_RANGE[1], "right": GRIPPER_RANGE[1]}
        self.arms = None

    def _on_sim_state(self, msg):
        if self.task_name is None:  # only the task name is needed, once
            m = _TASK_NAME.search(msg.data)
            if m:
                self.task_name = m.group(1)

    def _select_hand(self):
        """Fix ``self.hand``: the --hand choice, or detected from the simulator's topics."""
        if self.hand in HANDS:
            reason = "--hand"
            named = hand_of_task(self.task_name)
            if named and named != self.hand:
                print(f"WARNING: --hand {self.hand}, but the simulator runs {self.task_name} ({named}): the hands will not react.")
        else:
            q = [float(m.q) for m in self.lowstate.motor_state] if self.lowstate is not None else None
            self.hand, reason = detect_hand(self.task_name, any(st is not None for st in self.grip_state.values()), q)
        print(f"end effector: {'Dex2-5 hands (torque, rt/lowcmd motors 31..34)' if self.hand == 'dex2_5' else 'Dex1 grippers (rt/dex1/*/cmd)'} [{reason}]")
        if self.task_name and "r1a7d" not in self.task_name.lower():
            print(f"WARNING: the simulator runs {self.task_name}, which is not an R1-A7-D task.")

    def wait_for_sim(self, timeout=10.0):
        print("waiting for the simulator (rt/lowstate) ...")
        t0 = time.time()
        while self.lowstate is None and time.time() - t0 < timeout:
            time.sleep(0.05)
        if self.lowstate is not None and self.hand not in HANDS:
            t0 = time.time()
            while self.task_name is None and time.time() - t0 < 3.0:  # rt/sim_state follows the lowstate
                time.sleep(0.05)
        self._select_hand()
        if self.lowstate is None:
            print("no rt/lowstate received - is sim_main.py running with --robot_type r1a7d? Starting from zero targets.")
        else:
            for i in range(NUM_MOTORS):  # start from the current pose so nothing jumps
                self.q[i] = float(self.lowstate.motor_state[i].q)
            for s in ("left", "right"):
                st = self.grip_state[s]
                if st is not None and len(st.states):
                    self.grip[s] = clamp(float(st.states[0].q), *GRIPPER_RANGE)
            print("connected.")
        self.q_start = list(self.q)
        self.arms = CartesianArms(self.q[15:29], TCP_OFFSET[self.hand], R1A7D_URDF[self.hand], base_link="waist_yaw_link")

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
        if self.hand == "dex2_5":  # torque-only motors of the body lowcmd
            for s, slots in DEX2_5_LOWCMD_SLOTS.items():
                for slot in slots:
                    m = msg.motor_cmd[slot]
                    m.mode, m.q, m.dq, m.tau, m.kp, m.kd = 1, 0.0, 0.0, float(self.hand_tau[s]), 0.0, 0.0
        msg.crc = self.crc.Crc(msg)
        self.lowcmd_pub.Write(msg)
        if self.hand == "dex2_5":
            return
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
            if self.hand == "dex2_5":
                # like the unirobot runtime: ramp to +-0.3 Nm while a key is held, zero at once when released
                goal = ((1 if close_k in held else 0) - (1 if open_k in held else 0)) * DEX2_5_MAX_TORQUE
                step = DEX2_5_TORQUE_RATE * dt
                self.hand_tau[s] = 0.0 if goal == 0.0 else clamp(goal, self.hand_tau[s] - step, self.hand_tau[s] + step)
                continue
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
            + (self._dex2_5_status() if self.hand == "dex2_5" else f"grip L={self.grip['left']:.1f} R={self.grip['right']:.1f}   ")
        )

    def _dex2_5_status(self):
        """Commanded torque and the lowstate flexion (mean joint angle [rad], 0 = open) of fingers / thumb."""
        text = f"hand tau L={self.hand_tau['left']:+.2f} R={self.hand_tau['right']:+.2f} Nm"
        if self.lowstate is not None:
            flex = {s: "/".join(f"{float(self.lowstate.motor_state[i].q):.1f}" for i in slots) for s, slots in DEX2_5_LOWCMD_SLOTS.items()}
            text += f" flex L={flex['left']} R={flex['right']}"
        return text + "   "


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--domain", type=int, default=1, help="DDS domain id (the simulator uses 1)")
    ap.add_argument("--interface", type=str, default=None, help="network interface for DDS (default: auto)")
    ap.add_argument("--hand", choices=["auto", *HANDS], default="auto",
                    help="end effector of the simulated robot; auto (default): detected from the running task (rt/sim_state)")
    args = ap.parse_args()

    print("Keys (hold to move)" + __doc__.partition("Keys (hold to move)")[2])
    teleop = R1A7DKeyboardTeleop(args.domain, args.interface, args.hand)
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
