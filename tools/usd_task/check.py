# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Headless check of a generated USD task (tools/create_usd_task.py --check, or directly):

    python tools/usd_task/check.py --task_name Isaac-PickPlace-Mug-R1D-Dex1-Joint --device cpu --viz none

Runs the task without DDS for ~2 s and prints ``CHECK`` lines: no NaN, robot stable, the scene's articulations
(doors, drawers) not pushed by the robot; renders a preview from the task's overview camera (img/tasks/<task>.jpg).
"""

import argparse
import math
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)
os.chdir(REPO)
os.environ.setdefault("PROJECT_ROOT", REPO)

from isaaclab.app import add_launcher_args, launch_simulation  # noqa: E402

p = argparse.ArgumentParser()
p.add_argument("--task_name", required=True)
p.add_argument("--steps", type=int, default=200)
p.add_argument("--out", default=None, help="preview image (default img/tasks/<task>.jpg)")
add_launcher_args(p)
a = p.parse_args()

import tasks  # noqa: E402,F401
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg  # noqa: E402

cfg = parse_env_cfg(a.task_name, device=a.device, num_envs=1)
from isaaclab.sensors import CameraCfg  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402

wc = getattr(cfg.scene, "world_camera", None)
if wc is not None:  # preview camera at the overview pose
    cfg.scene.preview_camera = CameraCfg(
        prim_path="/World/envs/env_.*/preview_camera", height=450, width=800, data_types=["rgb"], update_period=0,
        spawn=sim_utils.PinholeCameraCfg(focal_length=14.0, horizontal_aperture=20.955, clipping_range=(0.05, 100.0)),
        offset=CameraCfg.OffsetCfg(pos=wc.offset.pos, rot=wc.offset.rot, convention=wc.offset.convention))
    cfg.scene.world_camera = None

with launch_simulation(cfg, a):
    import cv2
    import gymnasium as gym
    import numpy as np
    import torch

    import tasks.common_observations.camera_state as cs

    cs._camera_cache["write_interval_steps"] = 10 ** 9
    env = gym.make(a.task_name, cfg=cfg).unwrapped
    env.reset()
    T = lambda x: (x.torch if hasattr(x, "torch") else x)  # noqa: E731
    robot = env.scene["robot"]
    rt = "g1d" if "-G1D-" in a.task_name else "r1d" if "-R1D-" in a.task_name else "g129"
    prov = None
    if rt in ("g1d", "r1d"):  # floating base is held by the action provider
        from action_provider.create_action_provider import create_action_provider

        a.action_source, a.robot_type, a.task = "dds", rt, a.task_name
        a.enable_dex1_dds = a.enable_dex3_dds = a.enable_inspire_dds = False
        prov = create_action_provider(env, a)
    zero = torch.zeros(env.action_space.shape, device=env.device)

    # scene articulations (doors, drawers, ...): their joints must not move while the robot stands still
    import omni.usd
    from pxr import Usd, UsdPhysics

    stage = omni.usd.get_context().get_stage()
    from isaaclab_physx.physics.physx_manager import PhysxManager

    sim_view = PhysxManager.get_physics_sim_view()  # Isaac Lab's (warp) view
    scene_arts = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/envs/env_0/Scene")):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            try:
                v = sim_view.create_articulation_view(str(prim.GetPath()))
                if v.count > 0 and v.max_dofs > 0:
                    scene_arts.append((str(prim.GetPath()).split("/Scene/")[-1], v))
            except Exception as e:
                print(f"CHECK info cannot watch {prim.GetPath()}: {e}", flush=True)
    art_start = {n: torch.as_tensor(np.array(v.get_dof_positions().numpy())).clone() for n, v in scene_arts}

    start_root = T(robot.data.root_pos_w)[0].clone()
    for _ in range(a.steps):
        env.step(prov.get_action(env) if prov is not None else zero)
    end_root = T(robot.data.root_pos_w)[0]
    ok = True

    def check(cond, msg):
        global ok
        ok &= bool(cond)
        print(f"CHECK {'ok  ' if cond else 'FAIL'} {msg}", flush=True)

    check(bool(torch.isfinite(T(robot.data.joint_pos)).all()), "no NaN in the robot state")
    drift = float((end_root - start_root)[:2].norm())
    check(drift < 0.03, f"robot base stable: drift {drift * 100:.1f} cm")
    for n, v in scene_arts:
        d = (torch.as_tensor(np.array(v.get_dof_positions().numpy())) - art_start[n]).abs().reshape(-1)
        j = int(d.argmax())
        names = getattr(v.shared_metatype, "dof_names", None) or [str(i) for i in range(v.max_dofs)]
        check(float(d[j]) < 0.05, f"scene {n} not moved by the robot: largest joint change {float(d[j]):.3f} ({names[j]})")
    if wc is not None:
        im = T(env.scene["preview_camera"].data.output["rgb"])[0].cpu().numpy()
        out = a.out or os.path.join(REPO, "img", "tasks", f"{a.task_name}.jpg")
        os.makedirs(os.path.dirname(out), exist_ok=True)
        cv2.imwrite(out, cv2.cvtColor(im, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 85])
        print(f"CHECK info preview: {os.path.relpath(out, REPO)}", flush=True)
    print(f"CHECK {'PASSED' if ok else 'PROBLEMS FOUND'}", flush=True)
    os._exit(0 if ok else 1)
