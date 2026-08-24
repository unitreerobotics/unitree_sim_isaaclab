#!/usr/bin/env python3
# run_randomizer.py
# Example script to launch the Isaac Lab simulation, instantiate the 
# room environment, and step through episodes to see the randomization.

import argparse
import sys
import traceback
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

# 1. Setup Isaac Lab app launcher (must happen before importing torch or isaaclab)
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Run the Hospital Room Randomizer.")
parser.add_argument("--num_envs", type=int, default=4, help="Number of environments to spawn.")
parser.add_argument("--max_steps", type=int, default=0, help="Stop after this many simulation steps. Use 0 to run until closed.")
parser.add_argument("--save_camera", action="store_true", help="Save top-down camera renders to camera_output/ on each reset.")
parser.add_argument("--reset_interval", type=int, default=150, help="Simulation steps between layout randomizations.")
parser.add_argument("--visualize_obbs", action="store_true", help="Draw accepted 2D OBB footprints in the viewport.")
parser.add_argument(
    "--visualize_obb_margins",
    action="store_true",
    help="Also draw the collision-clearance margin around each floor-level OBB.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
if args_cli.reset_interval <= 0:
    parser.error("--reset_interval must be greater than zero")

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# 2. Imports after app launcher
import torch
import numpy as np
import isaaclab.sim as sim_utils
from isaaclab.envs import ManagerBasedEnv
from isaaclab.managers import ActionTermCfg, ObservationGroupCfg, ObservationTermCfg

from room_randomizer_lab.room_env_cfg import RoomEnvCfg

# --- Minimal Dummy Managers to satisfy ManagerBasedEnv ---
# Since you haven't defined what the RL agent controls or sees yet,
# we need dummy managers just so the environment initializes properly.

from isaaclab.utils import configclass


def dummy_observation(env: ManagerBasedEnv) -> torch.Tensor:
    """Single zero observation so ObservationManager has a valid policy group."""
    return torch.zeros((env.num_envs, 1), device=env.device)

@configclass
class DummyActionsCfg:
    """Empty action space."""
    pass

@configclass
class DummyObservationsCfg:
    @configclass
    class PolicyCfg(ObservationGroupCfg):
        dummy = ObservationTermCfg(func=dummy_observation)
    policy: PolicyCfg = PolicyCfg()

# ---------------------------------------------------------

def _save_camera_images(env, reset_num, output_dir):
    """Save top-down camera RGB images for every environment."""
    try:
        camera = env.scene["top_down_camera"]
        rgb_data = camera.data.output["rgb"]  # (num_envs, H, W, 3 or 4)
        for i in range(rgb_data.shape[0]):
            img = rgb_data[i].cpu().numpy()
            if img.shape[-1] == 4:  # RGBA → RGB
                img = img[:, :, :3]
            if img.max() <= 1.0:
                img = (img * 255).clip(0, 255).astype(np.uint8)
            else:
                img = img.clip(0, 255).astype(np.uint8)
            try:
                from PIL import Image
                Image.fromarray(img).save(output_dir / f"reset_{reset_num:04d}_env_{i}.png")
            except ImportError:
                np.save(output_dir / f"reset_{reset_num:04d}_env_{i}.npy", img)
        print(f"[INFO] Saved {rgb_data.shape[0]} top-down images (reset {reset_num})", flush=True)
    except Exception as e:
        print(f"[WARN] Camera image save failed: {e}", flush=True)

def main():
    # 3. Load the configuration
    env_cfg = RoomEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    
    # Inject dummy actions/observations so the environment can run
    env_cfg.actions = DummyActionsCfg()
    env_cfg.observations = DummyObservationsCfg()

    # 4. Initialize the environment
    print("[INFO] Initializing ManagerBasedEnv...", flush=True)
    env = ManagerBasedEnv(cfg=env_cfg)
    print("[INFO] ManagerBasedEnv initialized.", flush=True)

    obb_visualizer = None
    if args_cli.visualize_obbs:
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.util.debug_draw")
        simulation_app.update()
        from room_randomizer_lab.obb_visualizer import ObbVisualizer

        obb_visualizer = ObbVisualizer(env, show_margins=args_cli.visualize_obb_margins)

    # 5. Simulation Loop
    print("[INFO] Starting simulation loop. Press Ctrl+C to stop.", flush=True)
    step_count = 0
    reset_count = 0
    reset_interval = args_cli.reset_interval

    # Camera output setup
    camera_output_dir = None
    if args_cli.save_camera:
        camera_output_dir = Path(__file__).parent / "camera_output"
        camera_output_dir.mkdir(exist_ok=True)
        print(f"[INFO] Camera images will be saved to {camera_output_dir}", flush=True)

    while simulation_app.is_running():
        with torch.inference_mode():
            # Reset environments periodically to see the randomizer in action
            if step_count % reset_interval == 0:
                print(f"[INFO] Triggering environment reset (step {step_count})...", flush=True)
                env.reset()
                reset_count += 1
                if obb_visualizer is not None:
                    obb_visualizer.draw()
            
            # Step the simulation forward (with empty actions)
            env.step(action=torch.empty(env.num_envs, 0, device=env.device))
            
            # Save top-down camera images on the step following each reset
            if args_cli.save_camera and step_count % reset_interval == 0:
                _save_camera_images(env, reset_count, camera_output_dir)
            
            step_count += 1
            if args_cli.max_steps > 0 and step_count >= args_cli.max_steps:
                print(f"[INFO] Reached --max_steps={args_cli.max_steps}; exiting.", flush=True)
                break

    if obb_visualizer is not None:
        obb_visualizer.clear()
    env.close()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] Simulation stopped by user.", flush=True)
    except SystemExit as exc:
        print(f"[ERROR] SystemExit during run: code={exc.code!r}", flush=True)
        raise
    except BaseException:
        traceback.print_exc()
        raise
    finally:
        simulation_app.close()
