# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Headless check of a generated USD task (tools/create_usd_task.py --check, or directly):

    python tools/usd_task/check.py --task_name Isaac-PickPlace-Mug-R1A7D-Dex1-Joint --device cpu --viz none

Runs the task without DDS for ~2 s and prints ``CHECK`` lines: no NaN, robot stable, the scene's articulations
(doors, drawers) not pushed by the robot; renders a preview (img/tasks/<task>.jpg, the README task images):

* ``--preview robot`` (default): seen from the front (across the table, 20 deg to the side, 25 deg from above);
  distance and aim are fitted so that the robot from 15 cm below its hands up (legs / base are behind the table)
  and the task objects fill ~94 % of the (square) image. ``Wholebody`` tasks: the whole robot
* ``--preview overview``: the task's overview camera (``world_camera``)
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
p.add_argument("--preview", choices=["robot", "overview", "none"], default="robot", help="preview camera (see above)")
add_launcher_args(p)
a = p.parse_args()

import tasks  # noqa: E402,F401
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg  # noqa: E402

cfg = parse_env_cfg(a.task_name, device=a.device, num_envs=1)
from isaaclab.sensors import CameraCfg  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402

wc = getattr(cfg.scene, "world_camera", None)
PREVIEW_W, PREVIEW_H, PREVIEW_FOCAL, PREVIEW_APERTURE = 640, 640, 14.0, 20.955
preview = a.preview if (a.preview != "overview" or wc is not None) else "none"
if preview != "none":  # preview camera; "robot": moved in front of the robot once the scene has settled
    offset = (CameraCfg.OffsetCfg(pos=wc.offset.pos, rot=wc.offset.rot, convention=wc.offset.convention) if preview == "overview"
              else CameraCfg.OffsetCfg(pos=(0.0, 0.0, 3.0), rot=(0.0, 0.0, 0.0, 1.0), convention="ros"))
    cfg.scene.preview_camera = CameraCfg(
        prim_path="/World/envs/env_.*/preview_camera", height=PREVIEW_H, width=PREVIEW_W, data_types=["rgb"], update_period=0,
        spawn=sim_utils.PinholeCameraCfg(focal_length=PREVIEW_FOCAL, horizontal_aperture=PREVIEW_APERTURE, clipping_range=(0.05, 100.0)),
        offset=offset)
if wc is not None:
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
    rt = "g1d" if "-G1D-" in a.task_name else "r1a7d" if "-R1A7D-" in a.task_name else "g129"
    prov = None
    if rt in ("g1d", "r1a7d"):  # floating base is held by the action provider
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
    if preview == "robot":
        # frame the robot: its link origins seen from ``az`` around its heading and ``el`` above; distance and aim
        # are fitted so that the projected robot fills FILL of the image (link origins + MARGIN for the geometry)
        FILL, MARGIN = 0.94, 0.10
        bp = T(robot.data.body_pos_w)[0]
        hand_ids = [i for i, n in enumerate(robot.data.body_names)
                    if any(k in n.lower() for k in ("wrist", "hand", "dex", "link7")) or n.startswith(("lh_", "rh_"))]
        if "Wholebody" in a.task_name:  # walking tasks: no table in front, the whole robot
            az, el = math.radians(20.0), math.radians(15.0)
        else:
            # robot at a table: the part above its hands (legs / base / pedestal are behind the table) and the
            # task objects on the table, seen from the front (across the table) and from above
            if hand_ids:
                bp = bp[bp[:, 2] >= float(bp[hand_ids, 2].min()) - 0.15]
            objs = []
            for k in env.scene.keys():  # rigid objects (blocks, cylinder); not articulations (e.g. the cabinet)
                data = getattr(env.scene[k], "data", None) if k != "robot" else None
                pos = getattr(data, "root_pos_w", None)
                if pos is not None and not hasattr(data, "joint_pos"):
                    objs.append(T(pos)[0])
            print(f"CHECK info preview frames the robot and {len(objs)} task object(s)", flush=True)
            if objs:
                bp = torch.cat([bp, torch.stack(objs).to(bp.device)])
            # drawer scene: the cabinet stands between camera and robot, look over it
            az, el = math.radians(20.0), math.radians(40.0 if "Drawer" in a.task_name else 25.0)
        if "-Cylinder-" in a.task_name and "-R17A-" not in a.task_name:
            az = -az  # cylinder scene: the tall cylinder stands at the robot's left hand, look from its right side
        q = T(robot.data.root_quat_w)[0]
        x, y, z, w = (float(v) for v in q)
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        dev = bp.device
        fwd = torch.tensor([math.cos(yaw), math.sin(yaw), 0.0], device=dev)
        left = torch.tensor([-math.sin(yaw), math.cos(yaw), 0.0], device=dev)
        up = torch.tensor([0.0, 0.0, 1.0], device=dev)
        view = fwd * math.cos(az) * math.cos(el) + left * math.sin(az) * math.cos(el) + up * math.sin(el)
        right = torch.linalg.cross(-view, up)
        right = right / right.norm()
        cam_up = torch.linalg.cross(right, -view)
        tx = math.tan(math.atan(PREVIEW_APERTURE / 2 / PREVIEW_FOCAL))  # half image width / focal length
        ty = tx * PREVIEW_H / PREVIEW_W
        target = (bp.min(0).values + bp.max(0).values) / 2
        span = float((bp.max(0).values - bp.min(0).values).norm())
        dist = 2.0 * span + 0.5
        for _ in range(12):  # fit: projected extent -> FILL of the image, centred
            eye = target + dist * view
            d = bp - eye
            zc = (d @ -view).clamp(min=0.05)
            xs = torch.cat([(d @ right - MARGIN) / zc, (d @ right + MARGIN) / zc]) / tx
            ys = torch.cat([(d @ cam_up - MARGIN) / zc, (d @ cam_up + MARGIN) / zc]) / ty
            cx, cy = float(xs.max() + xs.min()) / 2, float(ys.max() + ys.min()) / 2
            extent = max(float(xs.max() - xs.min()) / 2, float(ys.max() - ys.min()) / 2)
            target = target + right * cx * tx * dist + cam_up * cy * ty * dist
            dist = dist * extent / FILL
        cam = env.scene["preview_camera"]
        cam.set_world_poses_from_view((target + dist * view).unsqueeze(0), target.unsqueeze(0))
        for _ in range(3):
            env.step(prov.get_action(env) if prov is not None else zero)
        print(f"CHECK info preview camera {dist:.2f} m from the robot ({'whole robot' if 'Wholebody' in a.task_name else 'robot above the hands + task objects'})", flush=True)
    if preview != "none":
        im = T(env.scene["preview_camera"].data.output["rgb"])[0].cpu().numpy()
        out = a.out or os.path.join(REPO, "img", "tasks", f"{a.task_name}.jpg")
        os.makedirs(os.path.dirname(out), exist_ok=True)
        cv2.imwrite(out, cv2.cvtColor(im, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 85])
        print(f"CHECK info preview: {os.path.relpath(out, REPO)}", flush=True)
    print(f"CHECK {'PASSED' if ok else 'PROBLEMS FOUND'}", flush=True)
    os._exit(0 if ok else 1)
