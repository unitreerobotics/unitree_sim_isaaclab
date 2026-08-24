#!/usr/bin/env python3
import argparse
import traceback

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Visual debug runner for the randomized G1 pick/place task.")
parser.add_argument("--task", default="Isaac-PickPlace-Cylinder-G129-Dex1-Joint")
parser.add_argument("--step_hz", type=float, default=30.0)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import time
import torch

import tasks  # noqa: F401
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg


def main():
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=1)
    env_cfg.terminations = None
    env_cfg.curriculum = None
    env_cfg.viewer.origin_type = "world"
    env_cfg.viewer.eye = (-7.5, -3.2, 4.2)
    env_cfg.viewer.lookat = (-7.5, -7.6, 0.8)

    env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
    env.sim.reset()
    env.reset()

    print("[VISUAL_DEBUG] Scene randomized once. No DDS reset listener is running.", flush=True)
    print("[VISUAL_DEBUG] Click the viewport once. Press Ctrl+C in this terminal to stop.", flush=True)

    dt = 1.0 / max(args_cli.step_hz, 1.0)
    last = time.time()
    action = torch.zeros((env.num_envs, env.action_manager.total_action_dim), device=env.device)
    env.action_manager.process_action(action)
    is_rendering = env.sim.has_gui() or env.sim.has_rtx_sensors()

    with torch.inference_mode():
        while simulation_app.is_running():
            for _ in range(env.cfg.decimation):
                env._sim_step_counter += 1
                env.action_manager.apply_action()
                robot = env.scene["robot"]
                robot.set_joint_position_target(robot.data.default_joint_pos)
                robot.set_joint_velocity_target(robot.data.default_joint_vel)
                env.scene.write_data_to_sim()
                env.sim.step(render=False)
                if env._sim_step_counter % env.cfg.sim.render_interval == 0 and is_rendering:
                    env.sim.render()
                env.scene.update(dt=env.physics_dt)
            now = time.time()
            sleep_s = dt - (now - last)
            if sleep_s > 0:
                time.sleep(sleep_s)
            last = time.time()

    env.close()


try:
    main()
except KeyboardInterrupt:
    print("\n[VISUAL_DEBUG] Stopped by user.", flush=True)
except BaseException:
    traceback.print_exc()
    raise
finally:
    simulation_app.close()
