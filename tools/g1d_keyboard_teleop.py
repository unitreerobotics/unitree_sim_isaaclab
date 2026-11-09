#!/usr/bin/env python3
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
Keyboard teleoperation test for the simulated Unitree G1-D (base, lift, waist, arms, Dex1 grippers).

It talks to the simulator over the same DDS topics as the real robot / xr_teleoperate (DDS domain 1):

    base      rt/cmd_vel_no_limit   (geometry_msgs/Twist: linear.x, angular.z)
    lift      rt/cmd_hispeed        (geometry_msgs/Point32: z = speed command in [-1, 1])
    waist     rt/lowcmd             (unitree_hg/LowCmd: motor 12 = waist yaw, 13 = waist pitch)
    arms      rt/lowcmd             (motors 15..28)
    grippers  rt/dex1/{left,right}/cmd (unitree_go/MotorCmds: q in [0 closed, 5.4 open])

Start the simulator first:

    python sim_main.py --device cpu --task Isaac-PickPlace-Cylinder-G1D-Dex1-Joint --enable_dex1_dds --robot_type g1d

then, in another terminal (same conda env, it needs unitree_sdk2py and pynput):

    python tools/g1d_keyboard_teleop.py

Keys (hold to move):
    W / S        base forward / backward          A / D   base turn left / right
    R / F        lift up / down                   SPACE   stop the base and the lift
    T / G        waist pitch forward / back       Y / H   waist yaw left / right
    Arms: the hand (tool point between the fingers) is moved in the torso frame, the arm joints
    are solved by inverse kinematics (tools/arm_ik.py):
    I / K        hand forward / backward          J / L   hand left / right
    U / O        hand up / down
    [ / ]        roll  (about forward axis)       ; / '   pitch (about left axis)
    , / .        yaw   (about up axis)
    TAB          switch hand: left / right / both (mirrored)     P   arms + waist back to the start pose
    Z / X        left gripper close / open        C / V   right gripper close / open
    ESC          quit
"""

import argparse
import math
import os
import sys
import termios
import threading
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import (
    geometry_msgs_msg_dds__Point32_,
    geometry_msgs_msg_dds__Twist_,
    unitree_go_msg_dds__MotorCmd_,
    unitree_hg_msg_dds__LowCmd_,
)
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Point32_, Twist_
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arm_ik import CartesianArms  # noqa: E402

# ----------------------------------------------------------------------------- robot constants
WAIST_YAW, WAIST_PITCH = 12, 13
ARM_OFFSET = {"left": 15, "right": 22}
ARM_JOINT_NAMES = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw"]
# joint limits (G1 arm, left side; the right side mirrors roll / yaw)
ARM_LIMITS_LEFT = [(-3.089, 2.670), (-1.588, 2.251), (-2.618, 2.618), (-1.047, 2.094), (-1.972, 1.972), (-1.614, 1.614), (-1.614, 1.614)]
MIRROR = [1, -1, -1, 1, -1, 1, -1]  # sign to mirror a left joint angle onto the right arm
WAIST_PITCH_LIMITS = (-0.02, 2.3)
WAIST_YAW_LIMITS = (-2.5, 2.5)
GRIPPER_RANGE = (0.0, 5.4)  # motor q: 0 = closed, 5.4 = open (xr_teleoperate Dex1)
TCP_OFFSET = (0.15, 0.0, 0.0)  # tool point between the Dex1 fingers, in <side>_wrist_yaw_link

# ----------------------------------------------------------------------------- speeds
BASE_V = 0.2  # m/s
BASE_W = 0.5  # rad/s
WAIST_RATE = 0.5  # rad/s
ARM_RATE = 0.8  # rad/s
GRIPPER_RATE = 5.4  # motor rad/s (full stroke in 1 s)
RATE_HZ = 50.0


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def cartesian_keys(held):
    """Held keys -> (translation, rotation) directions in the torso frame (x forward, y left, z up)."""
    def axis(pos, neg):
        return (1 if pos in held else 0) - (1 if neg in held else 0)

    dpos = (axis("i", "k"), axis("j", "l"), axis("u", "o"))
    drot = (axis("[", "]"), axis(";", "'"), axis(",", "."))
    return dpos, drot


def arm_limits(side, j):
    lo, hi = ARM_LIMITS_LEFT[j]
    if side == "right" and MIRROR[j] < 0:
        lo, hi = -hi, -lo
    return lo, hi


# ----------------------------------------------------------------------------- keyboard
class Keyboard:
    """Set of currently held keys (pynput); falls back to the terminal with key auto-repeat."""

    def __init__(self):
        self.held = set()
        self.pressed_once = []  # edge-triggered keys
        self.lock = threading.Lock()
        self.quit = False
        self.backend = None
        self._fd = sys.stdin.fileno() if sys.stdin.isatty() else None
        self._old_term = termios.tcgetattr(self._fd) if self._fd is not None else None
        if self._fd is not None:  # no echo / line buffering while we run
            new = termios.tcgetattr(self._fd)
            new[3] &= ~(termios.ECHO | termios.ICANON)
            termios.tcsetattr(self._fd, termios.TCSADRAIN, new)
        try:
            if not os.environ.get("DISPLAY"):
                raise RuntimeError("no DISPLAY")
            from pynput import keyboard

            self._kb = keyboard
            self._listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
            self._listener.start()
            self.backend = "pynput"
        except Exception as e:
            print(f"[keyboard] pynput not usable ({e}); using the terminal (keys auto-repeat, keep this terminal focused)")
            self._last_seen = {}
            threading.Thread(target=self._tty_loop, daemon=True).start()
            self.backend = "tty"

    def _name(self, key):
        k = self._kb.Key
        special = {k.space: "space", k.tab: "tab", k.esc: "esc"}
        if key in special:
            return special[key]
        ch = getattr(key, "char", None)
        return ch.lower() if ch else None

    def _on_press(self, key):
        name = self._name(key)
        if name is None:
            return
        with self.lock:
            if name not in self.held:
                self.pressed_once.append(name)
            self.held.add(name)
        if name == "esc":
            self.quit = True

    def _on_release(self, key):
        name = self._name(key)
        with self.lock:
            self.held.discard(name)

    def _tty_loop(self):
        while not self.quit:
            ch = os.read(self._fd, 1).decode(errors="ignore")
            name = {" ": "space", "\t": "tab", "\x1b": "esc"}.get(ch, ch.lower())
            with self.lock:
                if time.time() - self._last_seen.get(name, 0) > 0.6:
                    self.pressed_once.append(name)
                self._last_seen[name] = time.time()
            if name == "esc":
                self.quit = True

    def snapshot(self):
        with self.lock:
            if self.backend == "tty":
                now = time.time()
                held = {k for k, t in self._last_seen.items() if now - t < 0.6}  # covers the auto-repeat delay
            else:
                held = set(self.held)
            once, self.pressed_once = self.pressed_once, []
        return held, once

    def close(self):
        if self.backend == "pynput":
            self._listener.stop()
        if self._old_term is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_term)


# ----------------------------------------------------------------------------- teleop
class G1DKeyboardTeleop:
    def __init__(self, domain: int, interface: str | None):
        if interface:
            ChannelFactoryInitialize(domain, interface)
        else:
            ChannelFactoryInitialize(domain)
        self.crc = CRC()
        self.lowcmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_pub.Init()
        self.vel_pub = ChannelPublisher("rt/cmd_vel_no_limit", Twist_)
        self.vel_pub.Init()
        self.lift_pub = ChannelPublisher("rt/cmd_hispeed", Point32_)
        self.lift_pub.Init()
        self.grip_pub = {s: ChannelPublisher(f"rt/dex1/{s}/cmd", MotorCmds_) for s in ("left", "right")}
        for p in self.grip_pub.values():
            p.Init()

        self.lowstate = None
        self.height = None
        self.odom = None
        self.grip_state = {"left": None, "right": None}
        ChannelSubscriber("rt/lowstate", LowState_).Init(self._on_lowstate, 1)
        ChannelSubscriber("rt/hispeed_state", Point32_).Init(lambda m: setattr(self, "height", m), 1)
        ChannelSubscriber("rt/slamware_ros_sdk_server_node/odom", Odometry_).Init(lambda m: setattr(self, "odom", m), 1)
        for s in ("left", "right"):
            ChannelSubscriber(f"rt/dex1/{s}/state", MotorStates_).Init(lambda m, s=s: self.grip_state.__setitem__(s, m), 1)

        self.q = [0.0] * 29  # lowcmd targets
        self.grip = {"left": GRIPPER_RANGE[1], "right": GRIPPER_RANGE[1]}
        self.arms = None
        self.waist_start = (0.0, 0.0)
        self.mode_machine = 0

    def _on_lowstate(self, msg):
        self.lowstate = msg
        self.mode_machine = msg.mode_machine

    def wait_for_sim(self, timeout=10.0):
        print("waiting for the simulator (rt/lowstate) ...")
        t0 = time.time()
        while self.lowstate is None and time.time() - t0 < timeout:
            time.sleep(0.05)
        if self.lowstate is None:
            print("no rt/lowstate received - is sim_main.py running with --robot_type g1d? Starting from zero targets.")
            self.arms = CartesianArms(self.q[15:29], TCP_OFFSET)
            return
        # start from the current pose so nothing jumps
        for i in [WAIST_YAW, WAIST_PITCH] + list(range(15, 29)):
            self.q[i] = float(self.lowstate.motor_state[i].q)
        for s in ("left", "right"):
            st = self.grip_state[s]
            if st is not None and len(st.states):
                self.grip[s] = clamp(float(st.states[0].q), *GRIPPER_RANGE)
        self.arms = CartesianArms(self.q[15:29], TCP_OFFSET)
        self.waist_start = (self.q[WAIST_YAW], self.q[WAIST_PITCH])
        print("connected.")

    # ------------------------------------------------------------------ publishing
    def publish(self, v, w, lift):
        msg = unitree_hg_msg_dds__LowCmd_()
        msg.mode_pr = 0
        msg.mode_machine = self.mode_machine
        for i in range(29):
            m = msg.motor_cmd[i]
            m.mode = 1
            m.q = float(self.q[i])
            m.dq = 0.0
            m.tau = 0.0
            m.kp = 60.0
            m.kd = 1.5
        msg.crc = self.crc.Crc(msg)
        self.lowcmd_pub.Write(msg)

        t = geometry_msgs_msg_dds__Twist_()
        t.linear.x, t.angular.z = float(v), float(w)
        self.vel_pub.Write(t)
        p = geometry_msgs_msg_dds__Point32_()
        p.z = float(lift)
        self.lift_pub.Write(p)

        for s, pub in self.grip_pub.items():
            g = MotorCmds_()
            g.cmds = [unitree_go_msg_dds__MotorCmd_()]
            g.cmds[0].q = float(self.grip[s])
            g.cmds[0].kp, g.cmds[0].kd = 5.0, 0.05
            pub.Write(g)

    # ------------------------------------------------------------------ control
    def step(self, held, once, dt):
        for k in once:
            if k == "tab":
                self.arms.cycle_side()
            elif k == "p":
                self.arms.reset()
                self.q[WAIST_YAW], self.q[WAIST_PITCH] = self.waist_start

        v = (BASE_V if "w" in held else 0.0) - (BASE_V if "s" in held else 0.0)
        w = (BASE_W if "a" in held else 0.0) - (BASE_W if "d" in held else 0.0)
        lift = (1.0 if "r" in held else 0.0) - (1.0 if "f" in held else 0.0)
        if "space" in held:
            v = w = lift = 0.0

        dp = ((1 if "t" in held else 0) - (1 if "g" in held else 0)) * WAIST_RATE * dt
        dy = ((1 if "y" in held else 0) - (1 if "h" in held else 0)) * WAIST_RATE * dt
        self.q[WAIST_PITCH] = clamp(self.q[WAIST_PITCH] + dp, *WAIST_PITCH_LIMITS)
        self.q[WAIST_YAW] = clamp(self.q[WAIST_YAW] + dy, *WAIST_YAW_LIMITS)

        dpos, drot = cartesian_keys(held)
        self.arms.step(dpos, drot, dt)
        self.q[15:29] = self.arms.q_arms()

        for s, close_k, open_k in (("left", "z", "x"), ("right", "c", "v")):
            dg = ((1 if open_k in held else 0) - (1 if close_k in held else 0)) * GRIPPER_RATE * dt
            self.grip[s] = clamp(self.grip[s] + dg, *GRIPPER_RANGE)
        return v, w, lift

    def status(self, v, w, lift):
        h = f"{self.height.y:.3f}m" if self.height is not None else "  -  "
        if self.odom is not None:
            p, q = self.odom.pose.pose.position, self.odom.pose.pose.orientation
            od = f"x={p.x:+.2f} y={p.y:+.2f} yaw={math.degrees(2 * math.atan2(q.z, q.w)):+6.1f}"
        else:
            od = "-"
        tcp = self.arms.tcp_position("right" if self.arms.side == "right" else "left")
        return (
            f"\r base v={v:+.2f} w={w:+.2f} | odom {od} | lift cmd={lift:+.0f} h={h} | "
            f"waist p={self.q[WAIST_PITCH]:+.2f} y={self.q[WAIST_YAW]:+.2f} | "
            f"hand {self.arms.side} tcp=({tcp[0]:+.2f},{tcp[1]:+.2f},{tcp[2]:+.2f}) | "
            f"grip L={self.grip['left']:.1f} R={self.grip['right']:.1f}   "
        )


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--domain", type=int, default=1, help="DDS domain id (the simulator uses 1)")
    ap.add_argument("--interface", type=str, default=None, help="network interface for DDS (default: auto)")
    args = ap.parse_args()

    print("Keys (hold to move)" + __doc__.partition("Keys (hold to move)")[2])
    teleop = G1DKeyboardTeleop(args.domain, args.interface)
    teleop.wait_for_sim()
    kb = Keyboard()
    print(f"keyboard backend: {kb.backend}. Press ESC to quit.\n")
    dt = 1.0 / RATE_HZ
    try:
        last = time.time()
        while not kb.quit:
            now = time.time()
            held, once = kb.snapshot()
            v, w, lift = teleop.step(held, once, now - last)
            last = now
            teleop.publish(v, w, lift)
            sys.stdout.write(teleop.status(v, w, lift))
            sys.stdout.flush()
            time.sleep(max(0.0, dt - (time.time() - now)))
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):  # leave the base and the lift stopped
            teleop.publish(0.0, 0.0, 0.0)
            time.sleep(0.02)
        kb.close()
        print("\nbye")


if __name__ == "__main__":
    main()
