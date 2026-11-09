
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
#!/usr/bin/env python3
# main.py
import os

project_root = os.path.dirname(os.path.abspath(__file__))
os.environ["PROJECT_ROOT"] = project_root

import argparse
import contextlib
import time
import sys
import signal
import torch
import gymnasium as gym
from pathlib import Path

# Isaac Lab 3.0 simulation launcher
from isaaclab.app import add_launcher_args, launch_simulation

from teleimager.image_server import run_isaacsim_server
from dds.dds_create import create_dds_objects,create_dds_objects_replay
# add command line arguments
parser = argparse.ArgumentParser(description="Unitree Simulation")
parser.add_argument("--task", type=str, default="Isaac-PickPlace-G129-Head-Waist-Fix", help="task name")
parser.add_argument("--action_source", type=str, default="dds",
                   choices=["dds", "file", "trajectory", "policy", "replay","dds_wholebody"],
                   help="Action source")


parser.add_argument("--robot_type", type=str, default="g129", choices=["g129", "h1_2", "g1d", "r1d"], help="robot type (selects the body DDS / action mapping): g129 | h1_2 | g1d | r1d")
parser.add_argument("--enable_dex1_dds", action="store_true", help="enable gripper DDS")
parser.add_argument("--enable_dex3_dds", action="store_true", help="enable dexterous hand DDS")
parser.add_argument("--enable_inspire_dds", action="store_true", help="enable inspire hand DDS")
parser.add_argument("--stats_interval", type=float, default=10.0, help="statistics print interval (seconds)")

parser.add_argument("--file_path", type=str, default="/home/unitree/Code/xr_teleoperate/teleop/utils/data", help="file path (when action_source=file)")
parser.add_argument("--generate_data_dir", type=str, default="./data", help="save data dir")
parser.add_argument("--generate_data", action="store_true", default=False, help="generate data")
parser.add_argument("--rerun_log", action="store_true", default=False, help="rerun log")
parser.add_argument("--replay_data",  action="store_true", default=False, help="replay data")

parser.add_argument("--modify_light",  action="store_true", default=False, help="modify light")
parser.add_argument("--modify_camera",  action="store_true", default=False,    help="modify camera")

# performance analysis parameters
parser.add_argument("--step_hz", type=int, default=100, help="control frequency")
parser.add_argument("--enable_profiling", action="store_true", default=True, help="enable performance analysis")
parser.add_argument("--profile_interval", type=int, default=500, help="performance analysis report interval (steps)")

parser.add_argument("--model_path", type=str, default="assets/model/policy.onnx", help="model path")
parser.add_argument("--reward_interval", type=int, default=10, help="step interval for reward calculation")
parser.add_argument("--enable_wholebody_dds", action="store_true", default=False, help="enable wh dds")

parser.add_argument("--physics_dt", type=float, default=None, help="physics time step, e.g., 0.005")
parser.add_argument("--render_interval", type=int, default=None, help="render interval in physics steps (>=1); overrides --render_fps")
parser.add_argument("--render_fps", type=float, default=30.0,
                    help="rendering rate [Hz] of the cameras and the viewport (each render is a full RTX frame of all cameras); "
                         "the control loop keeps running at --step_hz. <=0: render every control step (old behaviour)")
parser.add_argument("--image_server_in_process", action="store_true", default=False,
                    help="run the image server in the simulator process (old behaviour). Default: own process, so its "
                         "image encoding does not compete with the simulation for the Python GIL")
parser.add_argument("--view_camera", type=str, default="auto", choices=["auto", "world", "follow", "none"],
                    help="viewport camera at start-up (GUI): world = fixed overview camera (/World/PerspectiveCamera), "
                         "follow = chase camera behind the robot (/World/FollowCamera), auto = follow for mobile robots "
                         "(G1-D, R1-D, Wholebody tasks) else world. Both can also be selected in the viewport camera menu")
parser.add_argument("--render_sync", action="store_true", default=False,
                    help="wait for the GPU at every render (Isaac Lab default for synthetic data). Off: camera images may be "
                         "one frame (<= 1/render_fps) old, but rendering is much faster")
parser.add_argument("--camera_write_interval", type=int, default=None, help="camera write interval steps (>=1)")


parser.add_argument("--no_render",action="store_true",default=False,help="disable rendering updates entirely (overrides render interval)",)
parser.add_argument("--public_ip",type=str,default="127.0.0.1",help="public ip")
parser.add_argument("--livestream_type", type=int, default=2, help="livestream type (0: no livestream, 1: WebRTC public network, 2:  WebRTC private network)")

parser.add_argument("--solver_iterations", type=int, default=None, help="physx solver iteration count (e.g., 4)")
parser.add_argument("--gravity_z", type=float, default=None, help="override gravity z (e.g., -9.8)")
parser.add_argument("--skip_cvtcolor", action="store_true", default=False, help="skip cv2.cvtColor if upstream already BGR")

parser.add_argument("--camera_jpeg", action="store_true", default=True, help="enable JPEG compression for camera frames")
parser.add_argument("--camera_jpeg_quality", type=int, default=85, help="JPEG quality (1-100)")

parser.add_argument("--physx_substeps", type=int, default=None, help="physx substeps per step")
parser.add_argument("--camera_include", type=str, default="front_camera,front_right_camera,left_wrist_camera,right_wrist_camera", help="comma-separated camera names to enable")
parser.add_argument("--camera_exclude", type=str, default="world_camera", help="comma-separated camera names to disable")

parser.add_argument("--env_reward_interval", type=int, default=5, help="environment reward compute interval (steps)")
parser.add_argument("--seed", type=int, default=42, help="environment seed")
parser.add_argument("--camera_calib", type=str, default=None,
                    help="camera calibration file (uniclib_calibration json) used to create the cameras; "
                         "default: $UNITREE_CAMERA_CALIB or robots/uniclib_calibration.json")
# add Isaac Lab launcher parameters
add_launcher_args(parser)
args_cli = parser.parse_args()
if args_cli.no_render:
    os.environ["LIVESTREAM"] = str(args_cli.livestream_type)
    os.environ["PUBLIC_IP"] = args_cli.public_ip
else:
    os.environ["LIVESTREAM"] = "0"

if args_cli.enable_dex3_dds and args_cli.enable_dex1_dds and args_cli.enable_inspire_dds:
    print("Error: enable_dex3_dds and enable_dex1_dds and enable_inspire_dds cannot be enabled at the same time")
    print("Please select one of the options")
    sys.exit(1)


# The camera presets read the calibration while `tasks` is imported, so select the file first.
if args_cli.camera_calib:
    _calib = os.path.abspath(os.path.expanduser(args_cli.camera_calib))
    if not os.path.isfile(_calib):
        print(f"Error: camera calibration file not found: {_calib}")
        sys.exit(1)
    os.environ["UNITREE_CAMERA_CALIB"] = _calib

import tasks
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg


def _create_env_cfg():
    """Create the task configuration and apply CLI overrides before launch."""
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=1)
    env_cfg.env_name = args_cli.task
    env_cfg.seed = args_cli.seed

    if args_cli.physics_dt is not None:
        env_cfg.sim.dt = float(args_cli.physics_dt)
    if args_cli.render_interval is not None:
        env_cfg.sim.render_interval = max(1, int(args_cli.render_interval))
    if args_cli.no_render:
        env_cfg.sim.render_interval = 1_000_000
    if args_cli.gravity_z is not None:
        env_cfg.sim.gravity = (0.0, 0.0, float(args_cli.gravity_z))
    if args_cli.solver_iterations is not None:
        env_cfg.sim.physics.min_position_iteration_count = max(1, int(args_cli.solver_iterations))

    # PhysX 6 no longer exposes a scene-level substeps field. Preserve the
    # control period by reducing the physics dt and increasing decimation.
    if args_cli.physx_substeps is not None:
        substeps = max(1, int(args_cli.physx_substeps))
        env_cfg.sim.dt /= substeps
        env_cfg.decimation *= substeps
        env_cfg.sim.render_interval *= substeps

    _configure_cameras(env_cfg)
    return env_cfg


def _view_camera_cfg(cam_cfg):
    """Plain USD camera at the pose of a camera sensor config, for the viewport only.

    A camera *sensor* adds a render product that is rendered every frame (~10 ms per step for one 640x480 camera)
    even if its image is never read. A plain camera prim is only rendered while the viewport looks through it
    (e.g. the overview camera ``world_camera`` = /World/PerspectiveCamera).
    """
    import torch
    from isaaclab.assets import AssetBaseCfg
    from isaaclab.utils.math import convert_camera_frame_orientation_convention

    rot = tuple(cam_cfg.offset.rot)
    if cam_cfg.offset.convention != "opengl":  # USD cameras use the OpenGL convention
        rot = tuple(convert_camera_frame_orientation_convention(
            torch.tensor([rot], dtype=torch.float32), origin=cam_cfg.offset.convention, target="opengl")[0].tolist())
    return AssetBaseCfg(
        prim_path=cam_cfg.prim_path,
        spawn=cam_cfg.spawn,
        init_state=AssetBaseCfg.InitialStateCfg(pos=tuple(cam_cfg.offset.pos), rot=rot),
    )


def _set_viewport_camera(prim_path: str = "/World/PerspectiveCamera") -> None:
    """Look through the overview camera in the GUI viewport (if both exist)."""
    try:
        import omni.usd
        from omni.kit.viewport.utility import get_active_viewport

        viewport = get_active_viewport()
        if viewport is not None and omni.usd.get_context().get_stage().GetPrimAtPath(prim_path).IsValid():
            viewport.camera_path = prim_path
            print(f"[camera] viewport camera: {prim_path}")
    except Exception as e:
        print(f"[camera] could not set the viewport camera: {e}")


def _configure_cameras(env_cfg):
    """Camera / render rate settings (rendering is by far the most expensive part of a step).

    * Cameras that are not used as sensors (not in --camera_include, or in --camera_exclude, e.g. the overview
      camera world_camera) become plain USD cameras for the viewport: every camera *sensor* is rendered at every
      RTX frame, whether its image is read or not.
    * Cameras render at --render_fps (Isaac Lab 3: a camera whose update_period has elapsed triggers a full
      RTX frame); the viewport (render_interval) follows the same rate. The control loop is not affected.
    """
    from isaaclab.sensors import CameraCfg

    include = {n.strip() for n in (args_cli.camera_include or "").split(",") if n.strip()}
    exclude = {n.strip() for n in (args_cli.camera_exclude or "").split(",") if n.strip()}
    view_only = []
    for name, cam in list(vars(env_cfg.scene).items()):
        if isinstance(cam, CameraCfg) and (name in exclude or (include and name not in include)):
            setattr(env_cfg.scene, name, _view_camera_cfg(cam))
            view_only.append(name)
    if view_only:
        print(f"[camera] viewport-only cameras (no sensor, not in --camera_include / in --camera_exclude): {', '.join(view_only)}")

    if args_cli.no_render or args_cli.render_interval is not None or args_cli.render_fps <= 0:
        return
    period = 1.0 / float(args_cli.render_fps)
    for cam in vars(env_cfg.scene).values():
        if isinstance(cam, CameraCfg):
            cam.update_period = max(cam.update_period, period)
    steps = max(1, round(period / env_cfg.sim.dt))
    env_cfg.sim.render_interval = max(env_cfg.sim.render_interval, steps)
    print(f"[camera] render rate {1.0 / (env_cfg.sim.render_interval * env_cfg.sim.dt):.1f} Hz "
          f"(render_interval {env_cfg.sim.render_interval} physics steps, camera update period {period * 1000:.0f} ms)")


env_cfg = _create_env_cfg()

# Isaac Lab 3.0 opens a viewport only when the Kit visualizer is selected.
if args_cli.visualizer is None and not args_cli.no_render and args_cli.livestream <= 0:
    args_cli.visualizer = ["kit"]

_runtime_stack = contextlib.ExitStack()
_runtime_stack.enter_context(launch_simulation(env_cfg, args_cli))

import omni.kit.app
import pinocchio

if not args_cli.render_sync:
    # Isaac Lab waits for the GPU at every render so that annotator data belongs to exactly this frame. For
    # teleoperation an image that is at most one render old is fine, and not waiting cuts the render time ~40 %.
    import carb

    carb.settings.get_settings().set("/app/renderer/waitIdle", False)
    carb.settings.get_settings().set("/app/hydraEngine/waitIdle", False)

simulation_app = omni.kit.app.get_app()

from layeredcontrol.robot_control_system import (
    RobotController,
    ControlConfig,
)

from dds.reset_pose_dds import *
from tools.augmentation_utils import (
    update_light,
    batch_augment_cameras_by_name,
)

from tools.data_json_load import sim_state_to_json
from dds.sim_state_dds import *
from action_provider.create_action_provider import create_action_provider
from tools.get_stiffness import get_robot_stiffness_from_env
from tools.get_reward import get_step_reward_value,get_current_rewards

def setup_signal_handlers(controller,dds_manager=None,image_server=None):
    """set signal handlers"""
    def signal_handler(signum, frame):
        print(f"\nreceived signal {signum}, stopping controller...")
        try:
            controller.stop()
        except Exception as e:
            print(f"Failed to stop controller: {e}")
        try:
            if dds_manager is not None:
                dds_manager.stop_all_communication()
        except Exception as e:
            print(f"Failed to stop DDS: {e}")
        try:
            if image_server is not None:
                image_server.stop()
        except Exception as e:
            print(f"Failed to stop image server: {e}")
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)



def main(env_cfg):
    """main function"""
    # import cProfile
    # import pstats
    # import io
    # profiler = cProfile.Profile()
    # profiler.enable()
    import os
    import atexit
    try:
        os.setpgrp()
        current_pgid = os.getpgrp()
        print(f"Setting process group: {current_pgid}")

        def cleanup_process_group():
            try:
                print(f"Cleaning up process group: {current_pgid}")
                import signal
                os.killpg(current_pgid, signal.SIGTERM)
            except Exception as e:
                print(f"Failed to clean up process group: {e}")

        atexit.register(cleanup_process_group)

    except Exception as e:
        print(f"Failed to set process group: {e}")
    print("=" * 60)
    print("robot control system started")
    print(f"Task: {args_cli.task}")
    print(f"Action source: {args_cli.action_source}")
    print("=" * 60)

    # create environment
    print("\ncreate environment...")
    try:
        env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
        env.seed(args_cli.seed)
        try:
            sensors_dict = getattr(env.scene, "sensors", {})
            if sensors_dict:
                print("Sensors in the environment:")
                for name, sensor in sensors_dict.items():
                    print(name, sensor)
                print("="*60)
        except Exception as e:
            print(f"[sim] failed to list sensors: {e}")
        print(f"\ncreate environment success ...")
        follow_camera = None
        if not args_cli.no_render:
            try:
                from tools.follow_camera import FollowCamera

                follow_camera = FollowCamera(env)
            except Exception as e:
                print(f"[camera] follow camera not available: {e}")
            view = args_cli.view_camera
            if view == "auto":
                mobile = args_cli.robot_type in ("g1d", "r1d") or "Wholebody" in args_cli.task
                view = "follow" if (mobile and follow_camera is not None) else "world"
            if view == "follow" and follow_camera is not None:
                _set_viewport_camera(follow_camera.path)
            elif view == "world":
                _set_viewport_camera()
        try:
            env._reward_interval = max(1, int(args_cli.env_reward_interval))
            env._reward_counter = 0
            env._reward_last = None
            print(f"[env] reward compute interval set to {env._reward_interval} steps")
        except Exception as e:
            print(f"[env] failed to set reward interval: {e}")
        headless_mode = bool(getattr(args_cli, "headless", False))
        if args_cli.no_render:
            print("[sim] rendering disabled via --no_render")
        elif headless_mode:
            print(f"[sim] headless offscreen rendering every {env_cfg.sim.render_interval} steps")
        else:
            print(f"[sim] render_interval set to {env_cfg.sim.render_interval}")
        if args_cli.camera_write_interval is not None:
            try:
                import tasks.common_observations.camera_state as cam_state
                cam_state._camera_cache['write_interval_steps'] = max(1, int(args_cli.camera_write_interval))
                print(f"[camera] write interval steps set to {cam_state._camera_cache['write_interval_steps']}")
            except Exception as e:
                print(f"[camera] failed to set write interval: {e}")

        if args_cli.skip_cvtcolor:
            os.environ["CAMERA_SKIP_CVTCOLOR"] = "1"
        try:
            import tasks.common_observations.camera_state as cam_state
            enable_jpeg = bool(args_cli.camera_jpeg) or (os.getenv("CAMERA_JPEG") == "1")
            jpeg_quality = int(args_cli.camera_jpeg_quality if args_cli.camera_jpeg else os.getenv("CAMERA_JPEG_QUALITY", args_cli.camera_jpeg_quality))
            cam_state.set_writer_options(enable_jpeg=enable_jpeg, jpeg_quality=jpeg_quality, skip_cvtcolor=args_cli.skip_cvtcolor)
            include = [n.strip() for n in (args_cli.camera_include or "").split(',') if n.strip()]
            exclude = [n.strip() for n in (args_cli.camera_exclude or "").split(',') if n.strip()]
            try:
                cam_state.set_camera_allowlist(include)
            except Exception:
                pass
            try:
                sensors_dict = getattr(env.scene, "sensors", {})
                for name, sensor in sensors_dict.items():
                    lname = name.lower()
                    if "camera" not in lname:
                        continue
                    if exclude and name in exclude:
                        for attr_name, value in [("enabled", False), ("is_enabled", False)]:
                            if hasattr(sensor, attr_name):
                                try:
                                    setattr(sensor, attr_name, value)
                                except Exception:
                                    pass
                        for meth in ("set_active", "disable", "pause"):
                            if hasattr(sensor, meth):
                                try:
                                    getattr(sensor, meth)(False)
                                except Exception:
                                    pass
                        for attr_name in ("update_period", "_update_period"):
                            if hasattr(sensor, attr_name):
                                try:
                                    setattr(sensor, attr_name, 1e6)
                                except Exception:
                                    pass
                    elif include and name not in include:
                        for attr_name in ("update_period", "_update_period"):
                            if hasattr(sensor, attr_name):
                                try:
                                    setattr(sensor, attr_name, 1e6)
                                except Exception:
                                    pass
            except Exception as e:
                print(f"[camera] failed to tune sensors: {e}")
        except Exception as e:
            print(f"[camera] failed to apply writer options: {e}")
    except Exception as e:
        print(f"\nFailed to create environment: {e}")
        return

    # get robot stiffness and damping parameters from runtime environment
    print("\n" + "="*60)
    print("🔍 Getting robot stiffness and damping parameters from runtime environment")
    print("="*60)

    try:
        stiffness_data = get_robot_stiffness_from_env(env)
        if stiffness_data:
            print("✅ Successfully got robot parameters!")
        else:
            print("⚠️ Failed to get robot parameters, will try again after environment reset")
    except Exception as e:
        print(f"⚠️ Error getting robot parameters: {e}")

    print("="*60)

    if not getattr(args_cli, "headless", False) and not args_cli.no_render:
        print("\n")
        print("***  Please left-click on the Sim window to activate rendering. ***")
        print("\n")
    else:
        print("\n")
        print("***  Running without GUI; rendering handled offscreen. ***")
        print("\n")
    # reset environment
    if args_cli.modify_light:
        update_light(
            prim_path="/World/light",
            color=(0.75, 0.75, 0.75),
            intensity=500.0,
            # position=(1.0, 2.0, 3.0),
            radius=0.1,
            enabled=True,
            cast_shadows=True
        )
    if args_cli.modify_camera:
        batch_augment_cameras_by_name(
            names=["front_cam"],
            focal_length=3.0,
            horizontal_aperture=22.0,
            vertical_aperture=16.0,
            exposure=0.8,
            focus_distance=1.2
        )
    env.sim.reset()
    env.reset()

    # create simplified control configuration
    try:
        control_config = ControlConfig(
            step_hz=args_cli.step_hz,
            replay_mode=args_cli.replay_data
        )
    except Exception as e:
        print(f"Failed to create control configuration: {e}")
        return

    # create controller

    if not args_cli.replay_data:
        print("========= create image server =========")
        try:
            if args_cli.image_server_in_process:
                image_server = run_isaacsim_server()
            else:
                from tools.image_server_process import ImageServerProcess

                image_server = ImageServerProcess().start()
        except Exception as e:
            print(f"Failed to create image server: {e}")
            return
        print("========= create image server success =========")
        print("========= create dds =========")
        try:
            reset_pose_dds,sim_state_dds,dds_manager = create_dds_objects(args_cli,env)
        except Exception as e:
            print(f"Failed to create dds: {e}")
            return
        print("========= create dds success =========")
    else:
        print("========= create dds =========")
        try:
            create_dds_objects_replay(args_cli,env)
        except Exception as e:
            print(f"Failed to create dds: {e}")
            return
        print("========= create dds success =========")
        from tools.data_json_load import get_data_json_list
        print("========= get data json list =========")
        data_idx=0
        data_json_list = get_data_json_list(args_cli.file_path)
        if args_cli.action_source != "replay":
            args_cli.action_source = "replay"
        print("========= get data json list success =========")
    # create action provider

    print(f"\ncreate action provider: {args_cli.action_source}...")
    try:
        print(f"args_cli.task: {args_cli.task}")
        if not args_cli.replay_data and ("Wholebody" in args_cli.task or args_cli.enable_wholebody_dds):
            args_cli.action_source = "dds_wholebody"
            args_cli.enable_wholebody_dds = True
            control_config.use_rl_action_mode = True
        action_provider = create_action_provider(env,args_cli)
        if action_provider is None:
            print("action provider creation failed, exiting")
            return
    except Exception as e:
        print(f"Failed to create action provider: {e}")
        return

    # set action provider
    print("========= create controller =========")
    controller = RobotController(env, control_config)
    controller.set_action_provider(action_provider)
    print("========= create controller success =========")

    # configure performance analysis
    if args_cli.enable_profiling:
        controller.set_profiling(True, args_cli.profile_interval)
        print(f"performance analysis enabled, report every {args_cli.profile_interval} steps")
    else:
        controller.set_profiling(False)
        print("performance analysis disabled")


    # set signal handlers
    if not args_cli.replay_data:
        setup_signal_handlers(controller,dds_manager,image_server)
    else:
        setup_signal_handlers(controller)

    print("Note: The DDS in Sim transmits messages on channel 1. Please ensure that other DDS instances use the same channel for message exchange by setting: ChannelFactoryInitialize(1).")
    try:
        # start controller - start asynchronous components
        print("========= start controller =========")
        controller.start()
        print("========= start controller success =========")

        # main loop - execute in main thread to support rendering
        last_stats_time = time.time()
        loop_start_time = time.time()
        loop_count = 0
        last_loop_time = time.time()
        recent_loop_times = []  # for calculating moving average frequency


        reward_interval = max(1, args_cli.reward_interval)

        # use torch.inference_mode() and exception suppression
        with contextlib.suppress(KeyboardInterrupt), torch.inference_mode():
            while simulation_app.is_running() and controller.is_running:
                current_time = time.time()
                loop_count += 1
                if not args_cli.replay_data:
                    try:
                        env_state = env.scene.get_state()
                        env_state_json =  sim_state_to_json(env_state)
                        sim_state = {"init_state":env_state_json,"task_name":args_cli.task}
                    except Exception as e:
                        print(f"Failed to get env state: {e}")
                        raise e
                    try:
                    # sim_state = json.dumps(sim_state)
                        sim_state_dds.write_sim_state_data(sim_state)
                    except Exception as e:
                        print(f"Failed to write sim state: {e}")
                        raise e
                    try:
                        reset_pose_cmd = reset_pose_dds.get_reset_pose_command()
                    except Exception as e:
                        print(f"Failed to get reset pose command: {e}")
                        raise e
                    # Compute current reward values manually if needed for debugging
                    try:
                        if (loop_count % reward_interval) == 0:
                            pass
                            # current_reward = get_step_reward_value(env)
                    except Exception as e:
                        print(f"奖励计算失败: {e}")
                        pass

                    if reset_pose_cmd is not None:
                        try:
                            reset_category = reset_pose_cmd.get("reset_category")
                            if (args_cli.enable_wholebody_dds and (reset_category == '1' or reset_category == '2')) or (not args_cli.enable_wholebody_dds and reset_category == '1'):
                                print("reset object")
                                env_cfg.event_manager.trigger("reset_object_self", env)
                                reset_pose_dds.write_reset_pose_command(-1)
                            elif reset_category == '2' and not args_cli.enable_wholebody_dds:
                                print("reset all")
                                env_cfg.event_manager.trigger("reset_all_self", env)
                                reset_pose_dds.write_reset_pose_command(-1)
                        except Exception as e:
                            print(f"Failed to write reset pose command: {e}")
                            raise e
                else:
                    if action_provider.get_start_loop() and data_idx<len(data_json_list):
                        print(f"data_idx: {data_idx}")
                        try:
                            sim_state,task_name = action_provider.load_data(data_json_list[data_idx])
                            if task_name!=args_cli.task:
                                raise ValueError(f" The {task_name} in the dataset is different from the {args_cli.task} being executed .")
                        except Exception as e:
                            print(f"Failed to load data: {e}")
                            raise e
                        try:
                            env.reset_to(sim_state, torch.tensor([0], device=str(env.device)), is_relative=True)
                            env.sim.reset()
                            time.sleep(1)
                            action_provider.start_replay()
                            data_idx+=1
                        except Exception as e:
                            print(f"Failed to start replay: {e}")
                            raise e
                # print(f"env_state: {env_state}")
                # calculate instantaneous loop time
                loop_dt = current_time - last_loop_time
                last_loop_time = current_time
                recent_loop_times.append(loop_dt)

                # keep recent 100 loop times
                if len(recent_loop_times) > 100:
                    recent_loop_times.pop(0)

                # execute control step (in main thread, support rendering)
                controller.step()
                if follow_camera is not None:
                    follow_camera.update()

                # print statistics and loop frequency periodically
                if current_time - last_stats_time >= args_cli.stats_interval:
                    # calculate while loop execution frequency
                    elapsed_time = current_time - loop_start_time
                    loop_frequency = loop_count / elapsed_time if elapsed_time > 0 else 0

                    # calculate moving average frequency (based on recent loop times)
                    if recent_loop_times:
                        avg_loop_time = sum(recent_loop_times) / len(recent_loop_times)
                        moving_avg_frequency = 1.0 / avg_loop_time if avg_loop_time > 0 else 0
                        min_loop_time = min(recent_loop_times)
                        max_loop_time = max(recent_loop_times)
                        max_freq = 1.0 / min_loop_time if min_loop_time > 0 else 0
                        min_freq = 1.0 / max_loop_time if max_loop_time > 0 else 0
                    else:
                        moving_avg_frequency = 0
                        min_freq = max_freq = 0

                    print(f"\n=== While loop execution frequency statistics ===")
                    print(f"loop execution count: {loop_count}")
                    print(f"running time: {elapsed_time:.2f} seconds")
                    print(f"overall average frequency: {loop_frequency:.2f} Hz")
                    print(f"moving average frequency: {moving_avg_frequency:.2f} Hz (last {len(recent_loop_times)} times)")
                    print(f"frequency range: {min_freq:.2f} - {max_freq:.2f} Hz")
                    print(f"average loop time: {(elapsed_time/loop_count*1000):.2f} ms")
                    if recent_loop_times:
                        print(f"recent loop time: {(avg_loop_time*1000):.2f} ms")
                    print(f"=============================")

                    # print_stats(controller)
                    last_stats_time = current_time

                # check environment state
                if env.sim.is_stopped():
                    print("\nenvironment stopped")
                    break
                # rate_limiter.sleep(env)
    except KeyboardInterrupt:
        print("\nuser interrupted program")

    except Exception as e:
        print(f"\nprogram exception: {e}")

    finally:
        # clean up resources
        print("\nclean up resources...")
        controller.cleanup()
        image_server.stop()
        env.close()
        print("cleanup completed")
    # profiler.disable()
    # s = io.StringIO()
    # ps = pstats.Stats(profiler, stream=s).strip_dirs().sort_stats("time")
    # ps.print_stats(30)

    # print(s.getvalue())

if __name__ == "__main__":
    try:
        main(env_cfg)
    finally:
        print("Performing final cleanup...")

        # Get current process information
        import os
        import subprocess
        import signal
        import time

        current_pid = os.getpid()
        print(f"Current main process PID: {current_pid}")

        # Terminate this process's own child processes (image server, DDS helpers, ...). Only descendants of
        # this process are touched: matching processes by name ("sim_main.py") also killed other simulators,
        # editors and shells of the user.
        try:
            import psutil

            children = psutil.Process(current_pid).children(recursive=True)
            for child in children:
                try:
                    print(f"Terminating child process: {child.pid}")
                    child.terminate()
                except psutil.NoSuchProcess:
                    pass
            _, alive = psutil.wait_procs(children, timeout=2)
            for child in alive:
                try:
                    print(f"Force killing process: {child.pid}")
                    child.kill()
                except psutil.NoSuchProcess:
                    pass
        except Exception as e:
            print(f"Error during process cleanup: {e}")

        try:
            _runtime_stack.close()
        except Exception as e:
            print(f"Failed to close simulation application: {e}")

        print("Program exit completed")

        # Force exit
        os._exit(0)

# python sim_main.py --device cpu  --task  Isaac-PickPlace-Cylinder-G129-Dex1-Joint   --enable_dex1_dds --robot_type g129
# python sim_main.py --device cpu  --task Isaac-PickPlace-Cylinder-G129-Dex3-Joint    --enable_dex3_dds --robot_type g129
# python sim_main.py --device cpu  --task Isaac-PickPlace-Cylinder-G129-Inspire-Joint    --enable_inspire_dds --robot_type g129

# python sim_main.py --device cpu  --task Isaac-PickPlace-RedBlock-G129-Dex1-Joint     --enable_dex1_dds --robot_type g129
# python sim_main.py --device cpu  --task Isaac-PickPlace-RedBlock-G129-Dex3-Joint    --enable_dex3_dds --robot_type g129
# python sim_main.py --device cpu  --task  Isaac-PickPlace-RedBlock-G129-Inspire-Joint    --enable_inspire_dds --robot_type g129


# python sim_main.py --device cpu  --task Isaac-Stack-RgyBlock-G129-Dex1-Joint     --enable_dex1_dds --robot_type g129
# python sim_main.py --device cpu  --task Isaac-Stack-RgyBlock-G129-Dex3-Joint     --enable_dex3_dds --robot_type g129
# python sim_main.py --device cpu  --task Isaac-Stack-RgyBlock-G129-Inspire-Joint     --enable_inspire_dds --robot_type g129




# python sim_main.py --device cpu  --task Isaac-Move-Cylinder-G129-Dex1-Wholebody  --robot_type g129 --enable_dex1_dds
# python sim_main.py --device cpu  --task Isaac-Move-Cylinder-G129-Dex3-Wholebody  --robot_type g129 --enable_dex3_dds
# python sim_main.py --device cpu  --task Isaac-Move-Cylinder-G129-Inspire-Wholebody  --robot_type g129 --enable_inspire_dds


# python sim_main.py --device cpu  --task Isaac-PickPlace-Cylinder-H12-27dof-Inspire-Joint  --enable_inspire_dds --robot_type h1_2
# python sim_main.py --device cpu  --task Isaac-PickPlace-RedBlock-H12-27dof-Inspire-Joint  --enable_inspire_dds --robot_type h1_2
# python sim_main.py --device cpu  --task Isaac-Stack-RgyBlock-H12-27dof-Inspire-Joint --enable_inspire_dds --robot_type h1_2

# Unitree G1-D (wheeled base + lift, Dex1 grippers; robot from URDF assets/urdf/g1_d_description)
# python sim_main.py --device cpu  --task Isaac-PickPlace-Cylinder-G1D-Dex1-Joint  --enable_dex1_dds --robot_type g1d

# Unitree R1-D (wheeled base + lift, Dex1 grippers; robot from URDF assets/urdf/r1_d_description)
# python sim_main.py --device cpu  --task Isaac-PickPlace-Cylinder-R1D-Dex1-Joint  --enable_dex1_dds --robot_type r1d
