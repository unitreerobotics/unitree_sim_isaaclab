#!/usr/bin/env python3
"""Verify bilateral Dex1 contact on both scored hospital pill bottles."""

from __future__ import annotations

import argparse
import os
import traceback
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("PROJECT_ROOT", str(PROJECT_ROOT))
os.environ.setdefault("LIVESTREAM", "0")

from isaaclab.app import AppLauncher


BOTTLES = ("pill_bottle_t", "pill_bottle_v")
TABLETOP_PROPS = (
    "pill_bottle_t",
    "pill_bottle_v",
    "medical_bottle_a",
    "medical_bottle_f",
    "marker_blue",
    "marker_yellow",
)
SIDES = ("left", "right")

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--contact_threshold", type=float, default=0.05)
parser.add_argument("--max_center_error", type=float, default=0.003)
parser.add_argument("--bottle", choices=BOTTLES)
parser.add_argument("--side", choices=SIDES)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


import gymnasium as gym
import torch
from pxr import Usd, UsdGeom, UsdPhysics

import tasks  # noqa: F401,E402
from isaaclab.sensors import ContactSensorCfg  # noqa: E402
from isaaclab.sim.utils import get_current_stage  # noqa: E402
from isaaclab.utils.math import project_points, quat_apply, quat_apply_inverse  # noqa: E402
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg  # noqa: E402
from tasks.utils.room_randomizer.placement_utils import make_obb, obb_overlap  # noqa: E402


TASK_ID = "Isaac-PickPlace-MedicineBottle-Hospital-G129-Dex1-Joint"
OPEN_JOINT_POS = -0.020
CLOSED_JOINT_POS = 0.0245
SETTLE_STEPS = 70
CONTACT_STEPS = 70

# Centers of the authored distal collision meshes in their rigid-link frames.
DISTAL_COLLIDER_LOCAL_CENTERS = {
    "Link1_3": (0.001997, 0.022747, -0.014200),
    "Link2_3": (-0.001997, 0.022747, -0.014200),
}


def _audit_bottle_physics(bottle_name: str) -> None:
    """Verify the live USD prim has the requested rigid/collider presets."""
    root_name = "".join(part.title() for part in bottle_name.split("_"))
    root_path = f"/World/envs/env_0/{root_name}"
    stage = get_current_stage()
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        raise RuntimeError(f"Missing live bottle prim: {root_path}")

    meshes = [prim for prim in Usd.PrimRange(root_prim) if prim.IsA(UsdGeom.Mesh)]
    if len(meshes) != 1:
        raise RuntimeError(f"Expected one collision mesh below {root_path}, found {len(meshes)}")
    collider = meshes[0]
    collision_enabled = UsdPhysics.CollisionAPI(collider).GetCollisionEnabledAttr().Get()
    approximation = UsdPhysics.MeshCollisionAPI(collider).GetApproximationAttr().Get()
    mass = UsdPhysics.MassAPI(root_prim).GetMassAttr().Get()
    if (
        not root_prim.HasAPI(UsdPhysics.RigidBodyAPI)
        or mass is None
        or float(mass) <= 0.0
        or collision_enabled is not True
        or approximation != UsdPhysics.Tokens.convexDecomposition
    ):
        raise RuntimeError(
            f"Invalid physics preset for {bottle_name}: rigid="
            f"{root_prim.HasAPI(UsdPhysics.RigidBodyAPI)} mass={mass} "
            f"collision={collision_enabled} approximation={approximation}"
        )
    print(
        f"[physics-audit] bottle={bottle_name} scale=1.0 rigid=True "
        f"mass={float(mass):.3f}kg collision=True approximation={approximation}",
        flush=True,
    )


def _audit_tabletop_visibility_and_separation(env) -> None:
    """Check the reset layout against the live front camera and stored OBBs."""
    layout = env._room_layout_state[0]
    placements = [layout.tabletop_placements[name] for name in TABLETOP_PROPS]
    boxes = [
        make_obb(
            placement.local_pose[0],
            placement.local_pose[1],
            placement.bbox,
            placement.local_pose[3],
        )
        for placement in placements
    ]
    for index, box in enumerate(boxes):
        for other_index in range(index):
            if obb_overlap(box, boxes[other_index], margin=0.015):
                raise RuntimeError(
                    f"Overlapping tabletop props: {TABLETOP_PROPS[index]} and "
                    f"{TABLETOP_PROPS[other_index]}"
                )

    camera = env.scene["front_camera"]
    points_w = torch.stack(
        [env.scene[name].data.root_pos_w[0] for name in TABLETOP_PROPS]
    )
    # Source pivots sit on the table, so audit a point inside each visible body.
    points_w[:, 2] += 0.03
    camera_quat = camera.data.quat_w_ros[0].expand(len(TABLETOP_PROPS), -1)
    points_camera = quat_apply_inverse(
        camera_quat, points_w - camera.data.pos_w[0]
    )
    pixels = project_points(points_camera, camera.data.intrinsic_matrices[0])
    if pixels.ndim == 3:
        pixels = pixels.squeeze(0)
    width, height = camera.cfg.width, camera.cfg.height
    failures = []
    for name, pixel in zip(TABLETOP_PROPS, pixels, strict=True):
        u, v, depth = (float(value) for value in pixel)
        if depth <= 0.0 or not (0.05 * width <= u <= 0.95 * width) or not (
            0.05 * height <= v <= 0.95 * height
        ):
            failures.append((name, u, v, depth))
        print(
            f"[camera-audit] object={name} pixel=({u:.1f}, {v:.1f}) depth={depth:.3f}m",
            flush=True,
        )
    if failures:
        raise RuntimeError(f"Tabletop props outside the front-camera safe frame: {failures}")
    print(
        "[placement-audit] PASS: all six props are separated and inside the front-camera safe frame",
        flush=True,
    )


def _sensor_name(side: str, jaw: int, bottle_name: str) -> str:
    return f"audit_{side}_jaw_{jaw}_{bottle_name}"


def _step(env, action: torch.Tensor, count: int) -> None:
    for _ in range(count):
        env.step(action)


def _set_absolute_joint_target(
    action: torch.Tensor,
    joint_ids: list[int],
    absolute_target: float,
    default_joint_pos: torch.Tensor,
) -> None:
    """Account for JointPositionActionCfg's task-default action offset."""
    action[:, joint_ids] = absolute_target - default_joint_pos[:, joint_ids]


def _audit_observation(env) -> torch.Tensor:
    """Keep the validation environment independent of teleoperation DDS I/O."""
    return torch.zeros((env.num_envs, 1), device=env.device)


def _distal_centers(robot, side: str) -> tuple[torch.Tensor, torch.Tensor]:
    centers = []
    for jaw, link_name in enumerate(("Link1_3", "Link2_3"), start=1):
        body_name = f"{side}_hand_{link_name}"
        body_ids, _ = robot.find_bodies(body_name)
        if len(body_ids) != 1:
            raise RuntimeError(f"Expected one body named {body_name!r}, found {body_ids}")
        body_id = body_ids[0]
        local_center = torch.tensor(
            DISTAL_COLLIDER_LOCAL_CENTERS[link_name],
            device=robot.device,
            dtype=robot.data.body_pos_w.dtype,
        ).unsqueeze(0)
        centers.append(
            robot.data.body_pos_w[:, body_id]
            + quat_apply(robot.data.body_quat_w[:, body_id], local_center)
        )
    return centers[0], centers[1]


def _distal_midpoint(robot, side: str) -> torch.Tensor:
    center_1, center_2 = _distal_centers(robot, side)
    return 0.5 * (center_1 + center_2)


def _place_bottle_between_jaws(env, bottle_name: str, side: str) -> None:
    bottle = env.scene[bottle_name]
    midpoint = _distal_midpoint(env.scene["robot"], side)
    root_state = bottle.data.root_state_w.clone()
    root_state[:, 0:2] = midpoint[:, 0:2]
    # Both target source meshes have a base-origin pivot. Keep the base just
    # above the table while centering it laterally between the jaw pads.
    root_state[:, 2] = 0.797 + env.scene.env_origins[:, 2]
    root_state[:, 3:7] = torch.tensor(
        (1.0, 0.0, 0.0, 0.0), device=env.device, dtype=root_state.dtype
    )
    root_state[:, 7:] = 0.0
    bottle.write_root_state_to_sim(root_state)


def _remove_bottle(env, bottle_name: str) -> None:
    bottle = env.scene[bottle_name]
    root_state = bottle.data.root_state_w.clone()
    root_state[:, 0:3] = env.scene.env_origins + torch.tensor(
        (0.0, 0.0, -5.0), device=env.device, dtype=root_state.dtype
    )
    root_state[:, 7:] = 0.0
    bottle.write_root_state_to_sim(root_state)


def _contact_forces(env, side: str, jaw: int, bottle_name: str) -> tuple[float, float]:
    sensor = env.scene.sensors[_sensor_name(side, jaw, bottle_name)]
    forces = sensor.data.force_matrix_w
    if forces is None:
        raise RuntimeError(f"Contact sensor {sensor.cfg.prim_path!r} has no filtered force matrix")
    filtered_force = float(torch.linalg.vector_norm(forces, dim=-1).max().item())
    net_force = float(torch.linalg.vector_norm(sensor.data.net_forces_w, dim=-1).max().item())
    return filtered_force, net_force


def main() -> None:
    env_cfg = parse_env_cfg(TASK_ID, device=args_cli.device, num_envs=1)
    env_cfg.terminations = None
    env_cfg.rewards = None
    env_cfg.observations.policy.robot_joint_state.func = _audit_observation
    env_cfg.observations.policy.robot_joint_state.params = {}
    env_cfg.observations.policy.robot_gipper_state = None
    env_cfg.observations.policy.camera_image = None
    selected_bottles = (args_cli.bottle,) if args_cli.bottle else BOTTLES
    selected_sides = (args_cli.side,) if args_cli.side else SIDES
    for bottle_name in selected_bottles:
        filter_path = f"{{ENV_REGEX_NS}}/{''.join(part.title() for part in bottle_name.split('_'))}"
        for side in selected_sides:
            for jaw in (1, 2):
                setattr(
                    env_cfg.scene,
                    _sensor_name(side, jaw, bottle_name),
                    ContactSensorCfg(
                        prim_path=f"{{ENV_REGEX_NS}}/Robot/{side}_hand_Link{jaw}_3",
                        filter_prim_paths_expr=[filter_path],
                        update_period=0.0,
                        history_length=1,
                        debug_vis=False,
                    ),
                )

    env = gym.make(TASK_ID, cfg=env_cfg).unwrapped
    try:
        env.reset()
        _audit_tabletop_visibility_and_separation(env)
        for bottle_name in selected_bottles:
            _audit_bottle_physics(bottle_name)
        robot = env.scene["robot"]
        reference_joint_pos = robot.data.default_joint_pos.clone()
        reference_joint_vel = torch.zeros_like(robot.data.default_joint_vel)
        action = torch.zeros((env.num_envs, env.action_manager.total_action_dim), device=env.device)
        finger_joint_ids, _ = robot.find_joints(".*_hand_Joint[12]_1")
        if len(finger_joint_ids) != 4:
            raise RuntimeError(f"Expected four Dex1 drive joints, found {finger_joint_ids}")

        failures = []
        with torch.inference_mode():
            for bottle_name in selected_bottles:
                for side in selected_sides:
                    action.zero_()
                    robot.write_joint_state_to_sim(
                        reference_joint_pos,
                        reference_joint_vel,
                    )
                    robot.set_joint_position_target(reference_joint_pos)
                    robot.write_data_to_sim()
                    robot.reset()
                    for scene_bottle_name in BOTTLES:
                        _remove_bottle(env, scene_bottle_name)
                    side_joint_ids, _ = robot.find_joints(f"{side}_hand_Joint[12]_1")
                    _set_absolute_joint_target(
                        action, finger_joint_ids, OPEN_JOINT_POS, reference_joint_pos
                    )
                    _step(env, action, SETTLE_STEPS)

                    _set_absolute_joint_target(
                        action, side_joint_ids, CLOSED_JOINT_POS, reference_joint_pos
                    )
                    _step(env, action, CONTACT_STEPS)
                    baseline_centers = _distal_centers(robot, side)
                    baseline_joint_positions = robot.data.joint_pos[
                        0, side_joint_ids
                    ].clone()
                    max_empty_close_error = float(
                        torch.max(
                            torch.abs(baseline_joint_positions - CLOSED_JOINT_POS)
                        ).item()
                    )
                    if max_empty_close_error > 0.002:
                        raise RuntimeError(
                            f"{side} Dex1 failed to reach its visible full-close limit: "
                            f"positions={baseline_joint_positions.tolist()} "
                            f"target={CLOSED_JOINT_POS:.4f}"
                        )
                    baseline_gap = float(
                        torch.linalg.vector_norm(
                            baseline_centers[1] - baseline_centers[0], dim=-1
                        ).item()
                    )

                    _set_absolute_joint_target(
                        action, side_joint_ids, OPEN_JOINT_POS, reference_joint_pos
                    )
                    _step(env, action, SETTLE_STEPS)
                    open_centers = _distal_centers(robot, side)
                    open_gap = float(torch.linalg.vector_norm(open_centers[1] - open_centers[0], dim=-1).item())
                    _place_bottle_between_jaws(env, bottle_name, side)
                    _step(env, action, 5)

                    _set_absolute_joint_target(
                        action, side_joint_ids, CLOSED_JOINT_POS, reference_joint_pos
                    )
                    max_forces = {1: 0.0, 2: 0.0}
                    max_net_forces = {1: 0.0, 2: 0.0}
                    for _ in range(CONTACT_STEPS):
                        env.step(action)
                        for jaw in (1, 2):
                            filtered_force, net_force = _contact_forces(
                                env, side, jaw, bottle_name
                            )
                            max_forces[jaw] = max(max_forces[jaw], filtered_force)
                            max_net_forces[jaw] = max(max_net_forces[jaw], net_force)

                    closed_centers = _distal_centers(robot, side)
                    closed_gap = float(
                        torch.linalg.vector_norm(closed_centers[1] - closed_centers[0], dim=-1).item()
                    )
                    jaw_midpoint = (0.5 * (closed_centers[0] + closed_centers[1]))[0].tolist()
                    jaw_1_position = closed_centers[0][0].tolist()
                    jaw_2_position = closed_centers[1][0].tolist()
                    bottle_position = env.scene[bottle_name].data.root_pos_w[0].tolist()
                    gap_block = closed_gap - baseline_gap
                    center_error = float(
                        torch.linalg.vector_norm(
                            env.scene[bottle_name].data.root_pos_w[0, 0:2]
                            - torch.tensor(
                                jaw_midpoint[0:2],
                                device=env.device,
                                dtype=env.scene[bottle_name].data.root_pos_w.dtype,
                            )
                        ).item()
                    )
                    print(
                        f"[grasp-contact] bottle={bottle_name} hand={side} "
                        f"filtered=({max_forces[1]:.4f}, {max_forces[2]:.4f})N "
                        f"net=({max_net_forces[1]:.4f}, {max_net_forces[2]:.4f})N "
                        f"jaw_gap={open_gap:.4f}->{closed_gap:.4f}m "
                        f"empty_gap={baseline_gap:.4f}m block={gap_block:.4f}m "
                        f"empty_joint_q={baseline_joint_positions.tolist()} "
                        f"center_error={center_error:.4f}m "
                        f"jaw1_xyz=({jaw_1_position[0]:.4f}, {jaw_1_position[1]:.4f}, "
                        f"{jaw_1_position[2]:.4f}) "
                        f"jaw2_xyz=({jaw_2_position[0]:.4f}, {jaw_2_position[1]:.4f}, "
                        f"{jaw_2_position[2]:.4f}) "
                        f"jaw_mid_xyz=({jaw_midpoint[0]:.4f}, {jaw_midpoint[1]:.4f}, "
                        f"{jaw_midpoint[2]:.4f}) "
                        f"bottle_xyz=({bottle_position[0]:.4f}, {bottle_position[1]:.4f}, "
                        f"{bottle_position[2]:.4f})",
                        flush=True,
                    )
                    force_failed = min(max_forces.values()) < args_cli.contact_threshold
                    if center_error > args_cli.max_center_error or force_failed:
                        failures.append(
                            {
                                "bottle": bottle_name,
                                "hand": side,
                                "filtered_forces": max_forces,
                                "gap_block": gap_block,
                                "center_error": center_error,
                            }
                        )

                    # Remove the tested body before opening the next hand.
                    _remove_bottle(env, bottle_name)

        if failures:
            raise RuntimeError(f"Dex1 bilateral contact validation failed: {failures}")
        print(
            "PASS: every selected pill-bottle/hand pair registered bilateral collision "
            "contact and remained centered between the Dex1 jaws",
            flush=True,
        )
    finally:
        env.close()


if __name__ == "__main__":
    try:
        main()
    except BaseException:
        # Isaac Sim's shutdown can consume the pending traceback, so emit it
        # before closing the application and preserve a non-zero exit status.
        traceback.print_exc()
        raise
    finally:
        try:
            simulation_app.close()
        except SystemExit:
            # Do not let Isaac's successful shutdown replace a validation
            # exception with exit code zero.
            pass
