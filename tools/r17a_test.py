# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Headless test of the simulated Unitree R1-7A arm robots (one left arm with Dex1, Dex2-5 or LinkerHand O6):

    python tools/r17a_test.py --device cpu --viz none                 # all three robots
    python tools/r17a_test.py --device cpu --viz none --hand o6       # one robot

Runs a task of each robot without DDS: the action provider gets the commands a client would send (rt/lowcmd and
the hand topics) from stubs, and the test checks the simulated robot and what it would publish. ``TEST`` lines;
exit code 0 = all passed. Each robot runs in its own simulator process.

1. arm: lowcmd motors 0..6 move the arm joints; rt/lowstate reports them on the same slots (7..13 stay 0)
2. Dex1: lowcmd motor 31 (mode 1, kp > 0) and rt/dex1/left/cmd (motor q 0 closed .. 5.4 open), lowcmd first;
   rt/lowstate motor 31 and rt/dex1/left/state report the gripper
3. Dex2-5: torque motors 31 / 32 close / open the hand (the R1-A7-D behaviour, tools/dex2_5_test.py has the details)
4. O6: rt/linkerhand/left/cmd raw angles 0..255 (255 = open) move the 6 joints, the distal joints follow (mimic);
   rt/linkerhand/left/state reports the raw angles
5. the robot base stays fixed, no NaN
"""

import argparse
import os
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# Dex2-5 on a column: with the arm base on the table top, the test pose of step 1 pushes the hand onto the table
TASKS = {"dex1": "Isaac-PickPlace-RedBlock-R17A-Dex1-Joint", "dex2_5": "Isaac-PickPlace-RedBlock-R17A-Dex2_5-Column-Joint",
         "o6": "Isaac-Stack-RgyBlock-R17A-O6-Joint"}

p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
p.add_argument("--hand", choices=["all", *TASKS], default="all")
p.add_argument("--task_name", default=None, help="task to use (default: one per robot, see TASKS)")
known, rest = p.parse_known_args()

if known.hand == "all":  # one process per robot (one simulator per process)
    failed = []
    for hand in TASKS:
        print(f"TEST ===== {hand}: {TASKS[hand]}", flush=True)
        r = subprocess.run([sys.executable, os.path.abspath(__file__), "--hand", hand, *rest], cwd=REPO,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        for line in r.stdout.splitlines():
            if line.startswith("TEST") or "Traceback" in line or line.startswith(("Error", "  File")):
                print(line, flush=True)
        if r.returncode != 0:
            failed.append(hand)
    print(f"TEST {'PASSED (all robots)' if not failed else 'FAILED: ' + ', '.join(failed)}", flush=True)
    sys.exit(1 if failed else 0)

sys.path.insert(0, REPO)
os.chdir(REPO)
os.environ.setdefault("PROJECT_ROOT", REPO)

from isaaclab.app import add_launcher_args, launch_simulation  # noqa: E402

lp = argparse.ArgumentParser()
add_launcher_args(lp)
a = lp.parse_args(rest)
HAND = known.hand
TASK = known.task_name or TASKS[HAND]

import tasks  # noqa: E402,F401
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg  # noqa: E402

cfg = parse_env_cfg(TASK, device=a.device, num_envs=1)
cfg.scene.world_camera = None
cfg.episode_length_s = 1.0e6


class LowCmdStub:
    """G1RobotDDS.get_robot_command() of a client: arm positions on 0..6, hand slots 31..34."""

    def __init__(self):
        self.q = {}  # slot -> (mode, q, kp, tau)

    def get_robot_command(self):
        if not self.q:
            return None  # no client yet
        mode, pos, kp, tau = [0] * 35, [0.0] * 35, [0.0] * 35, [0.0] * 35
        for s, (m, q, k, t) in self.q.items():
            mode[s], pos[s], kp[s], tau[s] = m, q, k, t
        return {"motor_cmd": {"modes": mode, "positions": pos, "velocities": [0.0] * 35, "torques": tau, "kp": kp, "kd": [0.0] * 35}}

    def write_robot_state(self, p, v, t, imu):
        self.state = (list(p), list(v), list(t))

    def stop_communication(self):
        pass


class GripperStub:
    """GripperDDS: rt/dex1/left/cmd (already converted to the G1 Dex1 joint range) and the published state."""

    def __init__(self):
        self.cmd, self.state = None, None

    def get_gripper_command(self):
        return self.cmd

    def write_gripper_state(self, lp, lv, lt, rp, rv, rt):
        self.state = (list(lp), list(lv), list(lt))

    def stop_communication(self):
        pass


class LinkerHandStub:
    def __init__(self):
        self.cmd, self.state = None, None

    def get_hand_commands(self):
        return self.cmd

    def write_hand_states(self, states):
        self.state = states

    def stop_communication(self):
        pass


with launch_simulation(cfg, a):
    import gymnasium as gym
    import torch

    import tasks.common_observations.camera_state as cs
    import tasks.common_observations.g1_29dof_state as g1s
    import tasks.common_observations.gripper_state as grs
    import tasks.common_observations.r17a_state as r17s
    from robots.linkerhand_o6 import O6_MIMIC, o6_joint, o6_motor_joints
    from robots.r17a_urdf import R17A_ARMS, r17a_arm_joints
    from tools.data_convert import convert_to_joint_range

    cs._camera_cache["write_interval_steps"] = 10 ** 9
    env = gym.make(TASK, cfg=cfg).unwrapped
    env.reset()

    def T(x):
        return x.torch if hasattr(x, "torch") else x

    robot = env.scene["robot"]
    names = list(robot.data.joint_names)
    J = {n: i for i, n in enumerate(names)}
    failures = []

    def check(cond, msg):
        if not cond:
            failures.append(msg)
        print(f"TEST {'ok  ' if cond else 'FAIL'} {msg}", flush=True)

    # stubs instead of the DDS objects (provider and the state observations)
    lowcmd, grip, o6 = LowCmdStub(), GripperStub(), LinkerHandStub()
    g1s._get_g1_robot_dds_instance = lambda: lowcmd
    r17s._get_g1_robot_dds_instance = lambda: lowcmd
    r17s._get_gripper_dds_instance = lambda: grip
    grs._get_gripper_dds_instance = lambda: grip
    r17s._linkerhand_dds[:] = [o6]
    r17s._dds_due = lambda st: True

    from action_provider.create_action_provider import create_action_provider

    a.action_source, a.robot_type, a.task = "dds", "r17a", TASK
    a.enable_dex1_dds = a.enable_dex3_dds = a.enable_inspire_dds = False
    a.enable_linkerhand_dds = False
    prov = create_action_provider(env, a)
    prov.robot_dds, prov.gripper_dds, prov.o6_dds = lowcmd, (grip if HAND == "dex1" else None), (o6 if HAND == "o6" else None)
    root0 = T(robot.data.root_pos_w)[0].clone()

    def run(seconds):
        for _ in range(max(1, round(seconds / env.step_dt))):
            env.step(prov.get_action(env))
        env.observation_manager.compute()  # publishes the states to the stubs

    def q(joints):
        return T(robot.data.joint_pos)[0][[J[j] for j in joints]]

    # ------------------------------------------------------------------ 1. arms
    sides = tuple(R17A_ARMS[HAND])
    home = {s: q(r17a_arm_joints(s)).clone() for s in sides}
    run(0.5)
    check(all(float((q(r17a_arm_joints(s)) - home[s]).abs().max()) < 0.03 for s in sides),
          f"{HAND}: arms hold the default pose without a lowcmd ({len(sides)} arm(s))")
    delta = torch.tensor([0.25, -0.20, 0.20, 0.25, 0.30, 0.20, -0.30])
    target = {}
    for i, s in enumerate(sides):
        target[s] = home[s].cpu() + delta * (1 if i == 0 else -1)
        for k in range(7):
            lowcmd.q[(0 if s == "left" else 7) + k] = (1, float(target[s][k]), 60.0, 0.0)
    run(2.0)
    for s in sides:
        err = float((q(r17a_arm_joints(s)).cpu() - target[s]).abs().max())
        check(err < 0.03, f"{s} arm follows lowcmd motors {0 if s == 'left' else 7}..{6 if s == 'left' else 13}: max joint error {err:.3f} rad")
        base = 0 if s == "left" else 7
        ls = torch.tensor(lowcmd.state[0][base:base + 7])
        check(float((ls - q(r17a_arm_joints(s)).cpu()).abs().max()) < 1e-4, f"{s} arm reported on rt/lowstate motors {base}..{base + 6}")
    if len(sides) == 1:
        check(max(abs(v) for v in lowcmd.state[0][7:14]) < 1e-6, "one-arm robot: rt/lowstate motors 7..13 are 0")
    for s in sides:  # back to the default pose for the hand tests (hands free above the table)
        for k in range(7):
            lowcmd.q[(0 if s == "left" else 7) + k] = (1, float(home[s][k]), 60.0, 0.0)
    run(2.0)

    # ------------------------------------------------------------------ hands
    if HAND == "dex1":
        fj = ["left_dex1_finger_joint_1", "left_dex1_finger_joint_2"]
        lowcmd.q[31] = (1, 0.0, 5.0, 0.0)  # internal: motor q 0 = closed
        run(1.0)
        check(float(q(fj).max()) < 0.003, f"Dex1 closes on lowcmd motor 31 q=0 (fingers {q(fj).tolist()})")
        check(abs(lowcmd.state[0][31]) < 0.05 and grip.state is not None and grip.state[0][0] > 0.02,
              f"Dex1 reported on rt/lowstate motor 31 (q {lowcmd.state[0][31]:.2f}) and rt/dex1/left/state (G1 range {grip.state[0][0]:.3f})")
        grip.cmd = {"left_gripper_cmd": {"positions": [convert_to_joint_range(5.4)]}}  # rt/dex1/left/cmd: open
        run(1.0)
        check(float(q(fj).max()) < 0.003, "Dex1: lowcmd motor 31 (mode 1, kp > 0) takes precedence over rt/dex1/left/cmd")
        lowcmd.q[31] = (0, 0.0, 0.0, 0.0)  # slot disabled: rt/dex1/left/cmd is used
        run(1.0)
        check(float(q(fj).min()) > 0.038, f"Dex1 opens on rt/dex1/left/cmd q=5.4 once motor 31 is disabled (fingers {q(fj).tolist()})")
        check(abs(lowcmd.state[0][31] - 5.4) < 0.15, f"rt/lowstate motor 31 = 5.4 when open ({lowcmd.state[0][31]:.2f})")
    elif HAND == "dex2_5":
        from tasks.common_observations.dex2_5_state import dex2_5_layout

        lay = dex2_5_layout(robot)

        def closure():
            qq = T(robot.data.joint_pos)[0][lay.ids]
            return lay.motor_mean((qq - lay.open) / (torch.where(lay.dir > 0, lay.upper, lay.lower) - lay.open))

        check(lay.slots == [31, 32], f"Dex2-5: one left hand on lowcmd motors 31 / 32 (slots {lay.slots})")
        for slot in (31, 32):
            lowcmd.q[slot] = (1, 0.0, 0.0, 0.3)
        run(1.5)
        c = closure()
        check(float(c.min()) > 0.97, f"Dex2-5: +0.3 Nm on lowcmd motors 31 / 32 closes the hand (closure {[round(float(v), 2) for v in c]})")
        check(all(abs(lowcmd.state[2][s] - 0.3) < 0.02 for s in (31, 32)), "Dex2-5: rt/lowstate tau_est 31 / 32 = commanded torque")
        check(all(abs(lowcmd.state[k][s]) < 1e-6 for k in range(3) for s in (33, 34)), "Dex2-5: rt/lowstate motors 33 / 34 (no right hand) are 0")
        for slot in (31, 32):
            lowcmd.q[slot] = (1, 0.0, 0.0, 0.0)
        run(1.0)
        check(float((closure() - c).abs().max()) < 0.03, "Dex2-5: zero torque keeps the hand closed")
        for slot in (31, 32):
            lowcmd.q[slot] = (1, 0.0, 0.0, -0.3)
        run(1.5)
        check(float(closure().abs().max()) < 0.03, "Dex2-5: -0.3 Nm opens the hand")
        lowcmd.q[31] = (1, 0.0, 0.0, 0.3)
        lowcmd.q[32] = (1, 0.0, 0.0, 0.0)
        lowcmd.q[33] = (1, 0.0, 0.0, 0.3)  # right hand slots: ignored (no right hand)
        run(1.5)
        c = closure()
        check(float(c[0]) > 0.97 and abs(float(c[1])) < 0.03, f"Dex2-5: motor 31 moves the fingers alone, motor 33 is ignored (closure {[round(float(v), 2) for v in c]})")
    else:
        mj = o6_motor_joints("left")
        lim = T(robot.data.joint_pos_limits)[0][[J[j] for j in mj]]
        open_ = torch.where(lim[:, 0].abs() <= lim[:, 1].abs(), lim[:, 0], lim[:, 1])
        closed = lim[:, 0] + lim[:, 1] - open_
        run(0.5)
        check(float((q(mj) - open_).abs().max()) < 0.03, "O6: hand open without a command")
        raw = [255.0, 255.0, 0.0, 128.0, 255.0, 0.0]  # thumb flex open, thumb abd open, index closed, middle half, ring open, pinky closed
        o6.cmd = {"left_hand_cmd": {"positions": raw, "stamp": time.time()}}
        run(1.5)
        want = open_ + torch.tensor([(255 - r) / 255 for r in raw], device=open_.device) * (closed - open_)
        err = float((q(mj) - want).abs().max())
        check(err < 0.05, f"O6: rt/linkerhand/left/cmd raw {raw} -> joints (max error {err:.3f} rad)")
        mim = max(abs(float(q([o6_joint('left', d)])[0] - m * q([o6_joint('left', p)])[0])) for d, (p, m) in O6_MIMIC.items())
        check(mim < 0.05, f"O6: distal joints follow their proximal joint (mimic, max error {mim:.3f} rad)")
        st = o6.state["left"]["positions"] if o6.state else []
        check(len(st) == 6 and max(abs(s - r) for s, r in zip(st, raw)) < 8.0, f"O6: rt/linkerhand/left/state raw angles {[round(v) for v in st]}")
        o6.cmd = {"left_hand_cmd": {"positions": [0.0] * 6, "stamp": time.time() - 3600}}  # left over from an earlier run
        run(0.5)
        check(float((q(mj) - want).abs().max()) < 0.05, "O6: commands older than the simulator start are ignored")

    # ------------------------------------------------------------------ 5. base / NaN
    drift = float((T(robot.data.root_pos_w)[0] - root0).norm())
    check(drift < 1e-3 and bool(torch.isfinite(T(robot.data.joint_pos)).all()), f"fixed base (moved {drift * 1000:.2f} mm), no NaN")
    obs = env.observation_manager.compute()["policy"]
    check(tuple(obs["robot_joint_state"].shape) == (1, 42), f"body observation: 14 slots x (q, dq, tau) {tuple(obs['robot_joint_state'].shape)}")

    print(f"TEST {'PASSED' if not failures else f'FAILED ({len(failures)})'} {HAND}", flush=True)
    for f in failures:
        print(f"TEST   - {f}", flush=True)
    os._exit(0 if not failures else 1)
