#!/usr/bin/env python3
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Assemble an arbitrary USD scene and the Unitree robots into simulation tasks (one task per robot).

    python tools/create_usd_task.py --usd ~/scenes/kitchen/scene.usda [--name Kitchen] \\
        [--robots r1d-dex1,g1d-dex1] [--robot-pose 6.3,-2.0,0] [--check]

then run e.g.

    python sim_main.py --device cpu --task Isaac-Kitchen-R1D-Dex1-Joint --enable_dex1_dds --robot_type r1d

What it does:
  1. analyzes the USD (units / up axis, floor, lights, physics scene, rigid bodies, articulations, colliders,
     a ``RobotBase`` marker) and prints what it found;
  2. copies the USD and every file it depends on to assets/scenes/<name>/src/ and writes a wrapper layer
     (assets/scenes/<name>/scene_isaaclab.usda) that deactivates the scene's own physics scene (Isaac Lab owns it);
  3. robot standing pose: --robot-pose, else a prim named ``RobotBase`` in the USD (position + heading), else a free
     spot on the floor near the middle of the scene (no collider within the robot's footprint);
  4. writes tasks/common_scene/usd_scene_<name>.py (scene spec; the robot pose is edited THERE) and, per robot, a
     task package in the current layout (tasks/<robot>_tasks/<name>_<robot>/) derived from the robot's reference
     task (robot, cameras, observations, actions and DDS unchanged);
  5. --check: runs every task headless (robot stable, no NaN, scene doors / drawers untouched by the robot) and
     renders a preview to img/tasks/<task>.jpg.

The scene is simulated as it is (its rigid bodies, doors, lights); the tasks have no task object or success
condition. Re-running the tool keeps generated files you edited (e.g. the robot pose); --force overwrites them.
"""

from __future__ import annotations

import argparse
import hashlib
import math
import os
import re
import shutil
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

from tools.usd_task.analyze import Box, analyze, report  # noqa: E402

PROFILES = {
    "g129-dex1": dict(pkg="g1_tasks", suffix="G129-Dex1", robot_type="g129", hand="--enable_dex1_dds", kind="g1",
                      ref="tasks.g1_tasks.pick_place_cylinder_g1_29dof_dex1.pickplace_cylinder_g1_29dof_dex1_joint_env_cfg:PickPlaceG129DEX1BaseFixEnvCfg"),
    "g129-dex3": dict(pkg="g1_tasks", suffix="G129-Dex3", robot_type="g129", hand="--enable_dex3_dds", kind="g1",
                      ref="tasks.g1_tasks.pick_place_cylinder_g1_29dof_dex3.pickplace_cylinder_g1_29dof_dex3_joint_env_cfg:PickPlaceG129DEX3JointEnvCfg"),
    "g129-inspire": dict(pkg="g1_tasks", suffix="G129-Inspire", robot_type="g129", hand="--enable_inspire_dds", kind="g1",
                         ref="tasks.g1_tasks.pick_place_cylinder_g1_29dof_inspire.pickplace_cylinder_g1_29dof_inspire_env_cfg:PickPlaceG129InspireBaseFixEnvCfg"),
    "g1d-dex1": dict(pkg="g1d_tasks", suffix="G1D-Dex1", robot_type="g1d", hand="--enable_dex1_dds", kind="g1d",
                     ref="tasks.g1d_tasks.pick_place_cylinder_g1d_dex1.pickplace_cylinder_g1d_dex1_joint_env_cfg:PickPlaceG1DDEX1JointEnvCfg"),
    "r1d-dex1": dict(pkg="r1d_tasks", suffix="R1D-Dex1", robot_type="r1d", hand="--enable_dex1_dds", kind="r1d",
                     ref="tasks.r1d_tasks.pick_place_cylinder_r1d_dex1.pickplace_cylinder_r1d_dex1_joint_env_cfg:PickPlaceR1DDEX1JointEnvCfg"),
}
KIND = {
    # root height that puts the robot on the floor; footprint (robot frame, x forward) for the free-spot search;
    # editable joints written to the spec (lift column)
    "g1": dict(root_z=0.76, base=((-0.25, 0.25), (-0.25, 0.25)), joints={},
               note="fixed base (pelvis held at this height); z 0.76 puts the feet on the floor"),
    "g1d": dict(root_z=0.1114, base=((-0.35, 0.35), (-0.30, 0.30)), joints={"LZ_mt_Joint": 0.0, "LZ_it_Joint": 0.0},
                note="z 0.1114 puts the wheels on the floor; lift: LZ_mt_Joint / LZ_it_Joint 0 .. 0.21 m each"),
    "r1d": dict(root_z=0.155, base=((-0.35, 0.35), (-0.30, 0.30)), joints={"lift_stage1_joint": 0.27, "lift_stage2_joint": 0.27},
                note="z 0.155 puts the wheels on the floor; lift: lift_stage1/2_joint 0 .. 0.425 m each"),
}
MARGIN = 0.10  # [m] free space around the robot footprint for the automatic spot


# ---------------------------------------------------------------------- helpers
def snake(name: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")


def camel(name: str) -> str:
    return "".join(w[:1].upper() + w[1:] for w in re.split(r"[^A-Za-z0-9]+", name) if w)


def floats(s, n):
    v = [float(x) for x in s.split(",")]
    if len(v) != n:
        raise argparse.ArgumentTypeError(f"expected {n} comma-separated numbers: {s}")
    return v


def footprint(x, y, yaw, base, margin=MARGIN, z0=0.03, z1=1.8):
    (bx0, bx1), (by0, by1) = base
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(x + c * px - s * py, y + s * px + c * py) for px in (bx0 - margin, bx1 + margin) for py in (by0 - margin, by1 + margin)]
    return Box((min(p[0] for p in pts), min(p[1] for p in pts), z0), (max(p[0] for p in pts), max(p[1] for p in pts), z1))


def on_floor(info, x, y):
    """A floor collider (top near z 0) exists at (x, y) - or the scene has none (a ground plane is added)."""
    if not info.has_floor:
        return True
    for c in info.colliders:
        if c.box.lo[2] > 0.1 or not c.box.contains_xy(x, y):
            continue
        z = c.top_at(x, y, 0.06)
        if z is not None and -0.1 <= z <= 0.06:
            return True
    return False


def free_spot(info, base):
    """Free floor spot closest to the middle of the scene (yaw 0), searched on a 0.25 m grid."""
    b = info.bounds
    cx, cy = (b.lo[0] + b.hi[0]) / 2, (b.lo[1] + b.hi[1]) / 2
    step = 0.25
    nx, ny = int((b.hi[0] - b.lo[0]) / step / 2) + 1, int((b.hi[1] - b.lo[1]) / step / 2) + 1
    cands = sorted(((cx + i * step, cy + j * step) for i in range(-nx, nx + 1) for j in range(-ny, ny + 1)),
                   key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2)
    for x, y in cands:
        if on_floor(info, x, y) and not info.obstacles(footprint(x, y, 0.0, base)):
            return x, y
    return None


# ---------------------------------------------------------------------- USD packaging
def copy_scene(usd_path, dest_dir):
    """Copy the USD and its dependencies (sublayers, references, textures, ...) keeping relative paths."""
    from pxr import UsdUtils

    src_root = os.path.dirname(os.path.abspath(usd_path))
    layers, assets, unresolved = UsdUtils.ComputeAllDependencies(usd_path)
    files = {os.path.abspath(l.realPath) for l in layers if l.realPath} | {os.path.abspath(a) for a in assets}
    external = []
    for f in sorted(files):
        if not f.startswith(src_root + os.sep):
            external.append(f)
            continue
        dst = os.path.join(dest_dir, "src", os.path.relpath(f, src_root))
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        if not os.path.exists(dst) or os.path.getmtime(dst) < os.path.getmtime(f):
            shutil.copy2(f, dst)
    return os.path.join(dest_dir, "src", os.path.relpath(os.path.abspath(usd_path), src_root)), len(files), external, list(unresolved)


def write_wrapper(path, scene_usd_rel, info):
    """scene_isaaclab.usda: references the copied scene; deactivates its physics scene (Isaac Lab owns one)."""
    from pxr import Usd, UsdGeom

    if os.path.exists(path):
        os.remove(path)
    st = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    scene = UsdGeom.Xform.Define(st, "/Scene").GetPrim()
    scene.GetReferences().AddReference(scene_usd_rel)
    st.SetDefaultPrim(scene)
    for p in info.physics_scenes:
        st.OverridePrim("/Scene/" + info.rel(p)).SetActive(False)
    st.GetRootLayer().Save()


# ---------------------------------------------------------------------- code generation
HEADER = "# Generated by tools/create_usd_task.py - edit freely; edited files are not overwritten (unless --force).\n"


def write_generated(path, body, force):
    """Write ``body`` with a content hash; keep files edited since they were generated (unless force)."""
    digest = hashlib.sha1(body.encode()).hexdigest()[:12]
    if os.path.exists(path) and not force:
        old = open(path).read()
        m = re.search(r"^# content-hash: (\w+)\n", old, re.M)
        old_body = old.split("\n", 2)[2] if old.startswith(HEADER) else None
        if m is None or old_body is None or hashlib.sha1(old_body.encode()).hexdigest()[:12] != m.group(1):
            print(f"  kept (edited by hand): {os.path.relpath(path, REPO)}")
            return False
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(HEADER + f"# content-hash: {digest}\n" + body)
    print(f"  wrote {os.path.relpath(path, REPO)}")
    return True


def spec_source(args, spec, robots, poses):
    lines = [
        f'"""Scene spec of the tasks Isaac-{args.name}-* (USD: {os.path.basename(args.usd)}), see tasks/common_scene/usd_scene.py."""',
        "",
        "SPEC = {",
        f'    "name": {args.name!r},',
        f'    "scene_usd": {spec["scene_usd"]!r},',
        f'    "scene_scale": {spec["scene_scale"]!r},  # USD units -> metres',
        f'    "scene_rot": {spec["scene_rot"]!r},  # Y-up -> Z-up',
        f'    "ground": {spec["ground"]!r},  # add a ground plane (the scene has no floor collider)',
        f'    "dome_light": {spec["dome_light"]!r},  # 0: only the scene\'s own lights',
        "    # ======================================================================================",
        "    # ROBOT STANDING POSE - edit here (metres / degrees, scene coordinates: z up, yaw 0 = facing +x).",
        "    # pos: robot root [x, y, z]; keep z (it puts the robot on the floor). yaw_deg: heading.",
        "    # joint_pos: initial joint positions, e.g. the lift column of G1-D / R1-D (table height).",
        "    # Restart sim_main.py after editing. In Isaac Sim, read a spot's coordinates by selecting a prim",
        "    # and looking at its Transform (Property panel), or place a prim named RobotBase in the USD and",
        "    # re-run tools/create_usd_task.py --force.",
        "    # ======================================================================================",
        '    "robots": {',
    ]
    for key in robots:
        kind = KIND[PROFILES[key]["kind"]]
        x, y, yaw = poses[key]
        lines.append(f"        # {kind['note']}")
        lines.append(f'        {key!r}: {{"pos": [{x:.3f}, {y:.3f}, {kind["root_z"]}], "yaw_deg": {math.degrees(yaw):.1f}, '
                     f'"joint_pos": {kind["joints"]!r}}},')
    lines += ["    },", "}", ""]
    return "\n".join(lines)


def register_in_package(pkg_dir, module):
    p = os.path.join(pkg_dir, "__init__.py")
    s = open(p).read()
    if f"from . import {module}" not in s:
        with open(p, "w") as f:
            f.write(s.rstrip("\n") + f"\nfrom . import {module}  # generated by tools/create_usd_task.py\n")
        print(f"  registered in {os.path.relpath(p, REPO)}")


def generate(args, spec, robots, poses):
    name_s = snake(args.name)
    spec_mod = f"usd_scene_{name_s}"
    spec_path = os.path.join(REPO, "tasks", "common_scene", f"{spec_mod}.py")
    write_generated(spec_path, spec_source(args, spec, robots, poses), args.force)
    commands = []
    for key in robots:
        prof = PROFILES[key]
        task_id = f"Isaac-{args.name}-{prof['suffix']}-Joint"
        module = f"{name_s}_{key.replace('-', '_')}"
        cls = camel(args.name) + camel(prof["suffix"]) + "EnvCfg"
        ref_mod, ref_cls = prof["ref"].split(":")
        pkg_dir = os.path.join(REPO, "tasks", prof["pkg"])
        env_body = (
            f'"""{task_id}: USD scene {args.name} + the robot setup of {ref_cls}.\n\n'
            f'The robot standing pose is set in tasks/common_scene/{spec_mod}.py (SPEC["robots"]["{key}"])."""\n\n'
            "from isaaclab.utils import configclass\n\n"
            "from tasks.common_scene.usd_scene import apply_usd_task\n"
            f"from tasks.common_scene.{spec_mod} import SPEC\n"
            f"from {ref_mod} import {ref_cls} as _ReferenceEnvCfg\n\n\n"
            "@configclass\n"
            f"class {cls}(_ReferenceEnvCfg):\n"
            "    def __post_init__(self):\n"
            "        super().__post_init__()\n"
            f'        apply_usd_task(self, SPEC, "{key}")\n')
        init_body = (
            "import gymnasium as gym\n\n"
            "from . import env_cfg\n\n"
            "gym.register(\n"
            f'    id="{task_id}",\n'
            '    entry_point="isaaclab.envs:ManagerBasedRLEnv",\n'
            f'    kwargs={{"env_cfg_entry_point": env_cfg.{cls}}},\n'
            "    disable_env_checker=True,\n"
            ")\n")
        write_generated(os.path.join(pkg_dir, module, "env_cfg.py"), env_body, args.force)
        write_generated(os.path.join(pkg_dir, module, "__init__.py"), init_body, args.force)
        register_in_package(pkg_dir, module)
        commands.append((key, task_id, f"python sim_main.py --device cpu --task {task_id} {prof['hand']} --robot_type {prof['robot_type']}"))
    return spec_path, commands


# ---------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--usd", required=True, help="scene USD (.usd/.usda/.usdc); files it references are copied too")
    ap.add_argument("--name", default=None, help="task name part (default: Scene-<usd file name>) -> Isaac-<name>-R1D-Dex1-Joint")
    ap.add_argument("--robots", default=",".join(PROFILES), help=f"comma-separated, from: {', '.join(PROFILES)}")
    ap.add_argument("--robot-pose", type=lambda s: floats(s, 3), default=None,
                    help="x,y,yaw_deg robot standing pose (scene coordinates); default: RobotBase marker, else a free spot")
    ap.add_argument("--dome-light", type=float, default=None, help="dome light intensity (default: 0 if the scene has lights, else 1000)")
    ap.add_argument("--force", action="store_true", help="overwrite generated files edited by hand")
    ap.add_argument("--dry-run", action="store_true", help="analyze only, write nothing")
    ap.add_argument("--check", action="store_true", help="run each task headless, check it and render a preview")
    args = ap.parse_args()

    usd = os.path.abspath(os.path.expanduser(args.usd))
    args.usd = usd
    if args.name is None:
        args.name = "Scene-" + camel(os.path.splitext(os.path.basename(usd))[0])
    robots = [r.strip() for r in args.robots.split(",") if r.strip()]
    for r in robots:
        if r not in PROFILES:
            raise SystemExit(f"unknown robot {r!r}; choose from {', '.join(PROFILES)}")

    info = analyze(usd)
    print(report(info))
    scale, yup = info.meters_per_unit, info.up_axis == "Y"
    if abs(scale - 1.0) > 1e-6 or yup:
        print(f"  the scene is scaled / rotated to metres, Z up (metersPerUnit {scale}, upAxis {info.up_axis}); "
              "robot poses are in metres, Z up")

    # --- robot pose
    marker = info.markers.get("RobotBase")
    poses = {}
    for key in robots:
        base = KIND[PROFILES[key]["kind"]]["base"]
        if args.robot_pose:
            pose, src = (args.robot_pose[0], args.robot_pose[1], math.radians(args.robot_pose[2])), "--robot-pose"
        elif marker:
            pose, src = (marker[0].center[0], marker[0].center[1], marker[1]), "RobotBase marker"
        else:
            spot = free_spot(info, base)
            if spot is None:
                raise SystemExit("[create_usd_task] no free floor spot found - give one with --robot-pose x,y,yaw_deg")
            pose, src = (spot[0], spot[1], 0.0), "free floor spot near the middle of the scene"
        hits = info.obstacles(footprint(pose[0], pose[1], pose[2], base, margin=0.02))
        warn = f"  ! overlaps {info.rel(hits[0].path)}" if hits else ""
        print(f"  {key:13s} at ({pose[0]:.2f}, {pose[1]:.2f}), yaw {math.degrees(pose[2]):.0f} deg ({src}){warn}")
        poses[key] = pose

    spec = {
        "scene_usd": f"assets/scenes/{snake(args.name)}/scene_isaaclab.usda",
        "scene_scale": [scale] * 3 if abs(scale - 1.0) > 1e-6 else None,
        "scene_rot": [0.7071068, 0.0, 0.0, 0.7071068] if yup else [0.0, 0.0, 0.0, 1.0],
        "ground": not info.has_floor,
        "dome_light": args.dome_light if args.dome_light is not None else (0.0 if info.lights else 1000.0),
    }
    if args.dry_run:
        return

    dest = os.path.join(REPO, "assets", "scenes", snake(args.name))
    os.makedirs(dest, exist_ok=True)
    copied_usd, n, external, unresolved = copy_scene(usd, dest)
    print(f"  copied {n} files to {os.path.relpath(dest, REPO)}/src")
    for e in external:
        print(f"  ! dependency outside the USD folder (not copied, its absolute path is kept): {e}")
    for u in unresolved:
        print(f"  ! unresolved reference in the USD: {u}")
    write_wrapper(os.path.join(dest, "scene_isaaclab.usda"), "./" + os.path.relpath(copied_usd, dest), info)
    print(f"  wrote {os.path.relpath(os.path.join(dest, 'scene_isaaclab.usda'), REPO)}")

    spec_path, commands = generate(args, spec, robots, poses)
    rel = os.path.relpath(spec_path, REPO)
    line = next((i + 1 for i, l in enumerate(open(spec_path)) if '"robots": {' in l), 1)
    print("\nRun:")
    for _, _, c in commands:
        print("  " + c)
    print(f"\nRobot standing pose: edit {rel}:{line}  (SPEC[\"robots\"][<robot>]: pos [x, y, z], yaw_deg, joint_pos)")
    print("  or re-run with --robot-pose x,y,yaw_deg --force, or put a prim named RobotBase into the USD.")
    if args.check:
        print("\nChecking the tasks (headless) ...")
        for key, task_id, _ in commands:
            r = subprocess.run([sys.executable, os.path.join(REPO, "tools", "usd_task", "check.py"), "--task_name", task_id,
                                "--device", "cpu", "--viz", "none"], capture_output=True, text=True,
                               env={**os.environ, "OMNI_KIT_ACCEPT_EULA": "YES"})
            lines = [l for l in r.stdout.splitlines() if l.startswith("CHECK")]
            print(f"  {task_id}:")
            for l in lines or [f"CHECK failed to run (exit {r.returncode}): " + (r.stderr.strip().splitlines() or ['?'])[-1]]:
                print("    " + l[6:])


if __name__ == "__main__":
    main()
