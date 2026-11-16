# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Headless test of the simulated Unitree Dex2-5 tendon hands (R1-A7-D):

    python tools/dex2_5_test.py --device cpu --viz none

Runs the Dex2-5 task without DDS: the action provider gets the lowcmd a real client would send (torque-only
motors 31 / 32 = left fingers, thumb and 33 / 34 = right fingers, thumb; unirobot r1_a7_d/dex2_5) from a stub and
the test checks the simulated hands and the rt/lowstate values. Prints ``TEST`` lines; exit code 0 = all passed.

1. torque control: close / open, zero torque keeps the pose, the four motors are independent, a disabled slot
   (mode 0) and position commands (q, kp) are ignored, torques are limited to +-0.3 Nm
2. rt/lowstate: motors 31..34 report the hand (q = mean flexion, tau_est = commanded torque while it is applied)
3. grasp (``--no_grasp`` skips it): the task's cylinder is put into each hand, the hand is closed on it and has
   to hold it against gravity - while the torque is applied and after it is released (zero torque), also while
   the column moves - and to drop it when it is opened
4. environment reset: the hands are open again
"""

import argparse
import os
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
os.chdir(REPO)
os.environ.setdefault("PROJECT_ROOT", REPO)

from isaaclab.app import add_launcher_args, launch_simulation  # noqa: E402

p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
p.add_argument("--task_name", default="Isaac-PickPlace-Cylinder-R1A7D-Dex2_5-Joint", help="an R1-A7-D Dex2-5 task")
p.add_argument("--no_grasp", action="store_true", help="skip the grasp test (it needs the task's rigid object 'object')")
add_launcher_args(p)
a = p.parse_args()

import tasks  # noqa: E402,F401
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg  # noqa: E402

cfg = parse_env_cfg(a.task_name, device=a.device, num_envs=1)
cfg.scene.world_camera = None  # no rendering needed
cfg.episode_length_s = 1.0e6  # no time-out reset during the test

TAU = 0.3  # [Nm] motor torque limit of the real hand
TAU_RATE = 0.6  # [Nm/s] torque ramp of the unirobot runtime
# cylinder axis in <side>_0_link (x along the fingers, palm at y = -+0.015, z = finger stacking axis): in front of
# the palm, between the thumb (x < 0.025) and the finger bases (x > 0.07); 4 mm clearance for the 18 mm radius
GRASP_POINT = {"left": (0.05, -0.037, 0.0), "right": (0.05, 0.037, 0.0)}
GRASP_LIFT = 0.20  # [m] the column is raised by this much first, so the cylinder hangs free above the table


class LowCmdStub:
    """What dds.g1_robot_dds.G1RobotDDS.get_robot_command() returns for the lowcmd of a Dex2-5 client."""

    def __init__(self, hold: dict):
        self.hold = hold  # body motor slot -> position target (arms, waist, head)
        self.tau = [0.0] * 4  # motors 31, 32, 33, 34
        self.mode = [1] * 4
        self.q = [0.0] * 4  # a correct client sends q = kp = 0; used to test that they are ignored
        self.kp = [0.0] * 4

    def get_robot_command(self):
        pos, tau, kp, mode = [0.0] * 35, [0.0] * 35, [0.0] * 35, [0] * 35
        for slot, q in self.hold.items():
            pos[slot], kp[slot], mode[slot] = q, 60.0, 1
        for i, slot in enumerate((31, 32, 33, 34)):
            pos[slot], tau[slot], kp[slot], mode[slot] = self.q[i], self.tau[i], self.kp[i], self.mode[i]
        return {"motor_cmd": {"modes": mode, "positions": pos, "velocities": [0.0] * 35, "torques": tau, "kp": kp, "kd": [0.0] * 35}}

    def stop_communication(self):
        pass


class MobileStub:
    """rt/cmd_vel stand-in: only the column speed (normalized) is used, to raise the hands for the grasp test."""

    def __init__(self):
        self.column = 0.0

    def get_mobile_command(self):
        return {"height_speed": self.column, "height_stamp": time.time(), "lin_x": 0.0, "ang_z": 0.0, "move_stamp": time.time()}

    def write_mobile_state(self, *args):
        pass

    def stop_communication(self):
        pass


with launch_simulation(cfg, a):
    import gymnasium as gym
    import torch
    from isaaclab.utils.math import quat_apply, quat_apply_inverse

    import tasks.common_observations.camera_state as cs

    cs._camera_cache["write_interval_steps"] = 10 ** 9  # no image shared memory
    env = gym.make(a.task_name, cfg=cfg).unwrapped
    env.reset()

    def T(x):
        return x.torch if hasattr(x, "torch") else x

    from action_provider.create_action_provider import create_action_provider
    from robots.unitree_r1a7d import R1A7D_HEAD_PITCH_DEFAULT, R1A7D_LIFT_JOINTS
    from tasks.common_observations.dex2_5_state import dex2_5_layout, has_dex2_5
    from tasks.common_observations.g1d_state import ARM_JOINTS, _gather_states, dds_lowstate_arrays
    from tasks.common_observations.r1a7d_state import R1A7D_LOWSTATE_JOINTS

    robot = env.scene["robot"]
    names = list(robot.data.joint_names)
    failures = []

    def check(cond, msg):
        if not cond:
            failures.append(msg)
        print(f"TEST {'ok  ' if cond else 'FAIL'} {msg}", flush=True)

    def info(msg):
        print(f"TEST info {msg}", flush=True)

    if not has_dex2_5(names):
        check(False, f"{a.task_name}: the robot has no Dex2-5 hands")
        os._exit(1)

    a.action_source, a.robot_type, a.task = "dds", "r1a7d", a.task_name
    a.enable_dex1_dds = a.enable_dex3_dds = a.enable_inspire_dds = False
    prov = create_action_provider(env, a)
    cmd = prov.robot_dds = LowCmdStub({29: R1A7D_HEAD_PITCH_DEFAULT})  # arms / waist at 0, head at its default
    mobile = prov.mobile_dds = MobileStub()
    lay = dex2_5_layout(robot)
    closed_limit = torch.where(lay.dir > 0, lay.upper, lay.lower)
    dt = float(env.step_dt)
    arm_ids = [names.index(n) for n in ARM_JOINTS]
    lift_ids = [names.index(n) for n in R1A7D_LIFT_JOINTS]
    M = ("left fingers", "left thumb", "right fingers", "right thumb")

    def closure():
        """Flexion of the four motors' joints, 0 = open .. 1 = closed (mean over the joints of a motor)."""
        q = T(robot.data.joint_pos)[0][lay.ids]
        return lay.motor_mean((q - lay.open) / (closed_limit - lay.open))

    def lowstate():
        """rt/lowstate arrays (q, dq, tau_est; 35 slots) as written to DDS by the body state observation."""
        _, pos, vel, torque = _gather_states(env, R1A7D_LOWSTATE_JOINTS, "dex2_5_test")
        return dds_lowstate_arrays(env, pos, vel, torque)

    def run(seconds, tau=None, ramp=True, each_step=None):
        """Step the task; ``tau``: target torque of the four motors, ramped up from zero like the unirobot runtime
        (a zero target, a smaller torque or the other direction is applied at once). Returns the time after
        which the closure stopped changing."""
        if tau is not None:
            tau = [float(t) for t in tau]
            for m in range(4):  # never ramp down: start from zero / the target
                if cmd.tau[m] * tau[m] <= 0.0:
                    cmd.tau[m] = 0.0
                elif abs(cmd.tau[m]) > abs(tau[m]):
                    cmd.tau[m] = tau[m]
        settled, last = 0.0, closure()
        for i in range(max(1, round(seconds / dt))):
            if tau is not None:
                for m in range(4):
                    step = TAU_RATE * dt if (ramp and tau[m] != 0.0) else 1.0e9
                    cmd.tau[m] += max(-step, min(step, tau[m] - cmd.tau[m]))
            if each_step is not None:
                each_step()
            env.step(prov.get_action(env))
            c = closure()
            if float((c - last).abs().max()) > 1.0e-4:
                settled = (i + 1) * dt
            last = c
        return settled

    def fmt(v):
        return "[" + ", ".join(f"{float(x):+.2f}" for x in v) + "]"

    # ------------------------------------------------------------------ 1. torque control
    root0 = T(robot.data.root_pos_w)[0].clone()
    run(1.0, [0, 0, 0, 0])
    check(float(closure().abs().max()) < 0.02, f"default pose: hands open at zero torque (closure {fmt(closure())})")

    t = run(2.5, [TAU] * 4)
    c = closure()
    check(float(c.min()) > 0.97, f"+{TAU} Nm on all motors closes both hands (closure {fmt(c)}, after {t:.2f} s)")
    _, _, tl = lowstate()
    check(float((tl[31:35] - TAU).abs().max()) < 0.03, f"lowstate tau_est 31..34 = commanded torque while it is applied {fmt(tl[31:35])}")

    run(3.0, [0, 0, 0, 0])
    c2 = closure()
    check(float((c2 - c).abs().max()) < 0.03, f"zero torque for 3 s: hands stay closed (closure {fmt(c2)})")
    _, _, tl = lowstate()
    check(float(tl[31:35].abs().max()) < 0.03, f"lowstate tau_est 31..34 = 0 at zero torque {fmt(tl[31:35])}")

    t = run(2.5, [-TAU] * 4)
    c = closure()
    check(float(c.abs().max()) < 0.03, f"-{TAU} Nm opens both hands (closure {fmt(c)}, after {t:.2f} s)")

    for m in range(4):
        tau = [0.0] * 4
        tau[m] = TAU
        run(2.0, tau)
        c = closure()
        others = [float(c[i]) for i in range(4) if i != m]
        check(float(c[m]) > 0.97 and max(abs(v) for v in others) < 0.03, f"motor {lay.slots[m]} ({M[m]}) moves alone (closure {fmt(c)})")
        run(2.0, [-TAU] * 4)

    run(0.4, [TAU] * 4, ramp=False)
    c = closure()
    run(2.0, [0, 0, 0, 0])
    c2 = closure()
    half = float(c2[0]), float(c2[2])  # the fingers (the thumbs are faster and may already be closed)
    check(float((c2 - c).abs().max()) < 0.05 and all(0.1 < v < 0.9 for v in half),
          f"torque released half-way: pose kept (closure {fmt(c)} -> {fmt(c2)} after 2 s)")

    cmd.mode = [0] * 4
    run(1.0, [TAU] * 4, ramp=False)
    check(float((closure() - c2).abs().max()) < 0.03, f"mode 0: torque ignored (closure {fmt(closure())})")
    cmd.mode = [1] * 4
    run(2.0, [-TAU] * 4)

    cmd.q, cmd.kp = [1.5] * 4, [60.0] * 4  # a position command as for the Dex1 motors on these slots
    run(1.5, [0, 0, 0, 0])
    check(float(closure().abs().max()) < 0.03, f"q / kp on motors 31..34 ignored: no position control (closure {fmt(closure())})")
    cmd.q, cmd.kp = [0.0] * 4, [0.0] * 4

    t_limit = run(2.0, [TAU] * 4, ramp=False)
    run(2.0, [-TAU] * 4)
    t_over = run(2.0, [1.0] * 4, ramp=False)
    _, _, tl = lowstate()
    check(abs(t_over - t_limit) < 0.05 and float((tl[31:35] - TAU).abs().max()) < 0.03,
          f"1.0 Nm is limited to {TAU} Nm (closes in {t_over:.2f} s like {TAU} Nm in {t_limit:.2f} s, tau_est {fmt(tl[31:35])})")

    # ------------------------------------------------------------------ 2. lowstate
    pl, vl, tl = lowstate()
    q = T(robot.data.joint_pos)[0][lay.ids]
    flex = lay.motor_mean((q - lay.open) * lay.dir)
    check(len(pl) == 35 and float((pl[31:35] - flex).abs().max()) < 1.0e-3 and float(flex.min()) > 1.0,
          f"lowstate q 31..34 = mean flexion of the motor's joints [rad] {fmt(pl[31:35])}")
    run(2.5, [-TAU] * 4)
    run(0.5, [0, 0, 0, 0])
    pl, vl, tl = lowstate()
    check(float(pl[31:35].abs().max()) < 0.03 and float(vl[31:35].abs().max()) < 0.05, f"lowstate q / dq 31..34 = 0 with open, resting hands {fmt(pl[31:35])}")
    obs = env.observation_manager.compute()["policy"]["robot_gipper_state"]
    check(tuple(obs.shape) == (1, 20) and float(obs.abs().max()) < 0.05, f"hand observation: 20 joint positions {tuple(obs.shape)}")

    arm = float(T(robot.data.joint_pos)[0][arm_ids].abs().max())
    drift = float((T(robot.data.root_pos_w)[0] - root0)[:2].norm())
    check(arm < 0.05 and drift < 0.03 and bool(torch.isfinite(T(robot.data.joint_pos)).all()),
          f"arms and base not disturbed by the hands (max arm joint {arm:.3f} rad, base drift {drift * 100:.1f} cm)")

    # ------------------------------------------------------------------ 3. grasp
    obj = env.scene["object"] if (not a.no_grasp and "object" in env.scene.keys()) else None
    if obj is None:
        info("grasp test skipped" + ("" if a.no_grasp else f": {a.task_name} has no rigid object 'object'"))
    else:
        bodies = list(robot.data.body_names)
        env_ids = torch.tensor([0], device=env.device)
        obj_start = T(obj.data.root_pos_w)[0].clone()

        def lift_height():
            return float(T(robot.data.joint_pos)[0][lift_ids].sum())

        h0 = lift_height()
        mobile.column = 1.0
        for _ in range(round(8.0 / dt)):
            if lift_height() - h0 >= GRASP_LIFT:
                break
            env.step(prov.get_action(env))
        mobile.column = 0.0
        run(1.0)
        check(lift_height() - h0 > GRASP_LIFT - 0.02, f"column raised by {lift_height() - h0:.2f} m for the grasp test")

        for side, (fm, tm) in (("left", (0, 1)), ("right", (2, 3))):
            hand = bodies.index(f"{side}_0_link")
            point = torch.tensor(GRASP_POINT[side], device=env.device)

            def hand_pose():
                return T(robot.data.body_pos_w)[0][hand], T(robot.data.body_quat_w)[0][hand]

            def in_hand():
                """Cylinder center in the hand frame (z = along the cylinder / finger stacking axis)."""
                pos, quat = hand_pose()
                return quat_apply_inverse(quat, T(obj.data.root_pos_w)[0] - pos)

            def place():
                """Put the resting cylinder at the grasp point, its axis along the hand's z axis."""
                pos, quat = hand_pose()
                pose = torch.cat([pos + quat_apply(quat, point), quat]).unsqueeze(0)
                obj.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
                obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros(1, 6, device=env.device), env_ids=env_ids)

            tau = [0.0] * 4
            tau[fm] = tau[tm] = TAU
            run(1.5, tau, each_step=place)  # the cylinder is held in place while the hand closes on it
            c = closure()
            check(0.2 < float(c[fm]) < 0.97, f"{side} hand closes onto the cylinder and is stopped by it (closure fingers {float(c[fm]):.2f}, thumb {float(c[tm]):.2f})")
            before = in_hand()
            run(1.0, tau)  # cylinder released: held by the hand only; it settles against the palm
            settled = in_hand()
            run(3.0, tau)
            after = in_hand()
            z_obj = float(T(obj.data.root_pos_w)[0][2])
            down = float((before - after)[2])  # along the cylinder axis = the direction gravity pulls it out
            creep = float((after - settled).norm())
            check(abs(down) < 0.01 and creep < 0.002,
                  f"{side} hand holds the cylinder for 4 s at {TAU} Nm: slid {down * 1000:+.1f} mm along its axis, "
                  f"moved {creep * 1000:.1f} mm in the last 3 s (settled {float((settled - before)[:2].norm()) * 1000:.1f} mm into the palm)")
            _, _, tl = lowstate()
            check(float((tl[[lay.slots[fm], lay.slots[tm]]] - TAU).abs().max()) < 0.03, f"{side} hand presses with the commanded torque (tau_est {fmt(tl[[lay.slots[fm], lay.slots[tm]]])})")

            run(3.0, [0, 0, 0, 0])  # torque released: the hand has to keep its grip (DEX2_5_HOLD_GRIP)
            released = in_hand()
            check(float((released - after).norm()) < 0.002,
                  f"{side} hand keeps the cylinder at zero torque for 3 s: moved {float((released - after).norm()) * 1000:.1f} mm in the hand")
            _, _, tl = lowstate()
            check(float(tl[[lay.slots[fm], lay.slots[tm]]].abs().max()) < 0.01, f"{side} hand: lowstate tau_est = 0 while it holds the cylinder at zero torque")
            h_top = lift_height()
            mobile.column = -1.0  # carry it: column down and up again
            run(1.5)
            h_low = lift_height()
            mobile.column = 1.0
            run(1.5)
            mobile.column = 0.0
            run(1.0)
            carried = in_hand()
            check(h_top - h_low > 0.05 and float((carried - released).norm()) < 0.005,
                  f"{side} hand carries the cylinder at zero torque while the column moves {(h_top - h_low) * 100:.0f} cm down and up: "
                  f"moved {float((carried - released).norm()) * 1000:.1f} mm in the hand")
            z_obj = float(T(obj.data.root_pos_w)[0][2])

            tau[fm] = tau[tm] = -TAU
            run(2.0, tau)
            drop = z_obj - float(T(obj.data.root_pos_w)[0][2])
            check(drop > 0.03 and float(closure()[[fm, tm]].abs().max()) < 0.05, f"{side} hand opened with -{TAU} Nm: cylinder released (fell {drop * 100:.0f} cm)")
            run(0.5, [0, 0, 0, 0])

    # ------------------------------------------------------------------ 4. reset
    run(2.0, [TAU] * 4)
    env.reset()
    run(1.0, [0, 0, 0, 0])
    c = closure()
    check(float(c.abs().max()) < 0.03, f"after an environment reset the hands are open again (closure {fmt(c)})")
    if obj is not None:
        d = T(obj.data.root_pos_w)[0] - obj_start  # the reset event places it within +-5 cm in x / y
        check(abs(float(d[2])) < 0.01 and float(d[:2].norm()) < 0.15,
              f"after the reset the cylinder stands on the table again (height {float(d[2]) * 1000:+.0f} mm, {float(d[:2].norm()) * 100:.0f} cm from its first place)")

    print(f"TEST {'PASSED' if not failures else f'FAILED ({len(failures)})'}", flush=True)
    for f in failures:
        print(f"TEST   - {f}", flush=True)
    os._exit(0 if not failures else 1)
