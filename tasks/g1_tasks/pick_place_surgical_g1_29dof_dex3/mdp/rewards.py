from __future__ import annotations


__all__ = [
"lift_trocars_reward",
"trocar_insertion_reward",
"trocar_placement_reward",
"task_success_termination",
"reset_task_stage"
]


import torch
from typing import TYPE_CHECKING

from isaaclab.managers import SceneEntityCfg
from isaaclab.assets import RigidObject
from isaaclab.utils.math import quat_apply

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def get_task_stage(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Get or initialize task stage tracker for each environment.
    
    Stage 0: Initial state (need to lift)
    Stage 1: Lifted (need to insert)
    Stage 2: Inserted (need to place)
    Stage 3: Placed (task complete)
    
    Returns:
        torch.Tensor: Current stage for each environment (num_envs,)
    """
    if not hasattr(env, '_task_stage'):
        env._task_stage = torch.zeros(env.num_envs, dtype=torch.long, device=env.device)
    return env._task_stage


def update_task_stage(
    env: ManagerBasedRLEnv,
    asset_cfg1: SceneEntityCfg,
    asset_cfg2: SceneEntityCfg,
    table_height: float = 0.85483,
    lift_threshold: float = 0.05,
    insertion_dist_threshold: float = 0.03,
    insertion_angle_threshold: float = 0.15,
    placement_x_min: float = -1.8,
    placement_x_max: float = -1.4,
    placement_y_min: float = 1.5,
    placement_y_max: float = 1.8,
    placement_z_min: float = 0.9,
) -> torch.Tensor:
    """Update task stage based on current state.
    
    This function checks conditions and advances stages automatically.
    Once a stage is completed, it never goes back.
    """
    stage = get_task_stage(env)
    
    obj1: RigidObject = env.scene[asset_cfg1.name]
    obj2: RigidObject = env.scene[asset_cfg2.name]
    
    pos1 = obj1.data.root_pos_w
    pos2 = obj2.data.root_pos_w
    quat1 = obj1.data.root_quat_w
    quat2 = obj2.data.root_quat_w
    
    # Stage 0 -> 1: Check if lifted
    target_z = table_height + lift_threshold
    is_lifted_1 = pos1[:, 2] > target_z
    is_lifted_2 = pos2[:, 2] > target_z
    both_lifted = is_lifted_1 & is_lifted_2
    stage = torch.where((stage == 0) & both_lifted, torch.ones_like(stage), stage)
    
    # Stage 1 -> 2: Check if inserted (close distance + aligned)
    dist = torch.norm(pos1 - pos2, dim=-1)
    
    target_axis1 = torch.tensor([0.0, 0.0, -1.0], device=env.device).repeat(env.num_envs, 1)
    target_axis2 = torch.tensor([0.0, 0.0, -1.0], device=env.device).repeat(env.num_envs, 1)
    axis1 = quat_apply(quat1, target_axis1)
    axis2 = quat_apply(quat2, target_axis2)
    dot_prod = torch.sum(axis1 * axis2, dim=-1)
    abs_dot = torch.clamp(torch.abs(dot_prod), max=1.0)
    angle = torch.acos(abs_dot)
    
    is_inserted = (dist < insertion_dist_threshold) & (angle < insertion_angle_threshold)
    print(f' Inserted: {is_inserted[0].item()} | dist: {dist[0].item():.4f} | angle: {angle[0].item():.3f}')
    stage = torch.where((stage == 1) & is_inserted, torch.full_like(stage, 2), stage)
    
    
    # Stage 2 -> 3: Check if placed in target zone
    curr_x_min, curr_x_max = min(placement_x_min, placement_x_max), max(placement_x_min, placement_x_max)
    curr_y_min, curr_y_max = min(placement_y_min, placement_y_max), max(placement_y_min, placement_y_max)
    
    in_zone_1 = (pos1[:, 0] >= curr_x_min) & (pos1[:, 0] <= curr_x_max) & \
                (pos1[:, 1] >= curr_y_min) & (pos1[:, 1] <= curr_y_max) & \
                (pos1[:, 2] < placement_z_min)
    in_zone_2 = (pos2[:, 0] >= curr_x_min) & (pos2[:, 0] <= curr_x_max) & \
                (pos2[:, 1] >= curr_y_min) & (pos2[:, 1] <= curr_y_max) & \
                (pos2[:, 2] < placement_z_min)
    both_in_zone = in_zone_1 & in_zone_2
    stage = torch.where((stage == 2) & both_in_zone, torch.full_like(stage, 3), stage)
    
    env._task_stage = stage
    return stage

def lift_trocars_reward(
    env: ManagerBasedRLEnv, 
    table_height: float = 0.85483,
    lift_threshold: float = 0.05,
    asset_cfg1: SceneEntityCfg = SceneEntityCfg("trocar_1"),
    asset_cfg2: SceneEntityCfg = SceneEntityCfg("trocar_2"),
    insertion_dist_threshold: float = 0.03,
    insertion_angle_threshold: float = 0.15,
    placement_x_min: float = -1.8,
    placement_x_max: float = -1.4,
    placement_y_min: float = 1.5,
    placement_y_max: float = 1.8,
    placement_z_min: float = 0.9,
) -> torch.Tensor:
    """Reward for lifting both trocars above the table.
    
    Only active in Stage 0. Once completed, this reward is locked at 1.0.
    """
    # Update task stage first - check ALL stage transitions once per step
    stage = update_task_stage(env, asset_cfg1, asset_cfg2, 
                             table_height, lift_threshold,
                             insertion_dist_threshold, insertion_angle_threshold,
                             placement_x_min, placement_x_max,
                             placement_y_min, placement_y_max,
                             placement_z_min)
    
    # Get the rigid objects from the scene
    obj1: RigidObject = env.scene[asset_cfg1.name]
    obj2: RigidObject = env.scene[asset_cfg2.name]
    
    # Get positions (num_envs, 3)
    pos1 = obj1.data.root_pos_w
    pos2 = obj2.data.root_pos_w
    
    target_z = table_height + lift_threshold
    
    # Check if lifted
    is_lifted_1 = pos1[:, 2] > target_z
    is_lifted_2 = pos2[:, 2] > target_z
    both_lifted = is_lifted_1 & is_lifted_2
    
    # Stage 0: give reward based on current state
    # Stage >= 1: always return 1.0 (stage completed)
    reward = torch.where(stage == 0, both_lifted.float(), torch.ones(env.num_envs, device=env.device))
    
    print(f' Stage: {stage[0].item()} | Lift reward: {reward[0].item():.2f} | pos1_z: {pos1[0,2]:.3f} | pos2_z: {pos2[0,2]:.3f}')
    return reward


def trocar_insertion_reward(
    env: ManagerBasedRLEnv,
    dist_std: float = 0.1,
    angle_std: float = 0.2,
    angle_threshold: float = 0.15, # Tolerance for parallelism (radians)
    asset_cfg1: SceneEntityCfg = SceneEntityCfg("trocar_1"),
    asset_cfg2: SceneEntityCfg = SceneEntityCfg("trocar_2"),
) -> torch.Tensor:
    """Reward for inserting trocar_2 into trocar_1.
    
    Only active in Stage 1. Once completed, this reward is locked at 1.0.
    """
    # Read current stage (already updated by lift_trocars_reward)
    stage = get_task_stage(env)
    
    obj1: RigidObject = env.scene[asset_cfg1.name]
    obj2: RigidObject = env.scene[asset_cfg2.name]

    # Positions and Rotations
    pos1 = obj1.data.root_pos_w
    quat1 = obj1.data.root_quat_w
    pos2 = obj2.data.root_pos_w
    quat2 = obj2.data.root_quat_w

    # 1. Distance Reward with threshold
    dist = torch.norm(pos1 - pos2, dim=-1)
    
    threshold = 0.05
    dist_clamped = torch.clamp(dist - threshold, max=0.0)
    dist_reward = torch.exp(-torch.square(dist_clamped) / (2 * dist_std**2))
    dist_reward = torch.where(dist < threshold, dist_reward, torch.zeros_like(dist_reward))
    
    # 2. Alignment Reward
    target_axis1 = torch.tensor([0.0, 0.0, -1.0], device=env.device).repeat(env.num_envs, 1)
    target_axis2 = torch.tensor([0.0, 0.0, -1.0], device=env.device).repeat(env.num_envs, 1)
    
    axis1 = quat_apply(quat1, target_axis1)
    axis2 = quat_apply(quat2, target_axis2)
    
    dot_prod = torch.sum(axis1 * axis2, dim=-1)
    abs_dot = torch.clamp(torch.abs(dot_prod), max=1.0)
    angle = torch.acos(abs_dot)
    
    excess_angle = torch.clamp(angle - angle_threshold, min=0.0)
    align_reward = torch.exp(-torch.square(excess_angle) / (2 * angle_std**2))
    
    # Combine rewards
    total_reward = dist_reward * align_reward
    
    # Stage 0: no reward (not lifted yet)
    # Stage 1: give reward based on current state
    # Stage >= 2: always return 1.0 (stage completed)
    reward = torch.where(stage < 1, torch.zeros(env.num_envs, device=env.device),
                        torch.where(stage == 1, total_reward,
                                   torch.ones(env.num_envs, device=env.device)))
    
    print(f' Insert reward: {reward[0].item():.3f} | dist: {dist[0].item():.4f} | angle: {angle[0].item():.3f}')
    return reward


def trocar_placement_reward(
    env: ManagerBasedRLEnv,
    x_min: float = -1.8,
    x_max: float = -1.4,
    y_min: float = 1.5,
    y_max: float = 1.8,
    z_min: float = 0.9,
    asset_cfg1: SceneEntityCfg = SceneEntityCfg("trocar_1"),
    asset_cfg2: SceneEntityCfg = SceneEntityCfg("trocar_2"),
) -> torch.Tensor:
    """Reward for placing both trocars in the target tray region.
    
    Only active in Stage 2. Once completed, this reward is locked at 1.0.
    """
    # Read current stage (already updated by lift_trocars_reward)
    stage = get_task_stage(env)
    
    # Get rigid objects
    obj1: RigidObject = env.scene[asset_cfg1.name]
    obj2: RigidObject = env.scene[asset_cfg2.name]
    
    # Get root positions (num_envs, 3)
    pos1 = obj1.data.root_pos_w
    pos2 = obj2.data.root_pos_w
    
    # Handle potential min/max swap in config
    curr_x_min, curr_x_max = min(x_min, x_max), max(x_min, x_max)
    curr_y_min, curr_y_max = min(y_min, y_max), max(y_min, y_max)
    
    # Check bounds for object 1
    in_x_1 = (pos1[:, 0] >= curr_x_min) & (pos1[:, 0] <= curr_x_max)
    in_y_1 = (pos1[:, 1] >= curr_y_min) & (pos1[:, 1] <= curr_y_max)
    in_z_1 = pos1[:, 2] < z_min
    in_zone_1 = in_x_1 & in_y_1 & in_z_1
    
    # Check bounds for object 2
    in_x_2 = (pos2[:, 0] >= curr_x_min) & (pos2[:, 0] <= curr_x_max)
    in_y_2 = (pos2[:, 1] >= curr_y_min) & (pos2[:, 1] <= curr_y_max)
    in_z_2 = pos2[:, 2] < z_min
    in_zone_2 = in_x_2 & in_y_2 & in_z_2
    
    both_in_zone = in_zone_1 & in_zone_2
    
    # Stage < 2: no reward (not inserted yet)
    # Stage 2: give reward based on current state
    # Stage 3: always return 1.0 (task complete)
    reward = torch.where(stage < 2, torch.zeros(env.num_envs, device=env.device),
                        torch.where(stage == 2, both_in_zone.float(),
                                   torch.ones(env.num_envs, device=env.device)))
    
    print(f' Place reward: {reward[0].item():.2f} | in_zone: {both_in_zone[0].item()} | z1: {pos1[0,2]:.3f} | z2: {pos2[0,2]:.3f}')
    return reward


def task_success_termination(
    env: ManagerBasedRLEnv,
    asset_cfg1: SceneEntityCfg = SceneEntityCfg("trocar_1"),
    asset_cfg2: SceneEntityCfg = SceneEntityCfg("trocar_2"),
) -> torch.Tensor:
    """Termination condition: task is complete when stage reaches 3.
    
    Returns:
        torch.Tensor: Boolean tensor indicating which environments should terminate (num_envs,)
    """
    stage = get_task_stage(env)
    task_complete = stage >= 3
    
    if task_complete.any():
        print(f"🎉 Task completed in {task_complete.sum().item()} environment(s)!")
    
    return task_complete


def reset_task_stage(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor,
) -> None:
    """Reset task stage to 0 for specified environments.
    
    This should be called during environment reset events.
    
    Args:
        env: The environment instance
        env_ids: Indices of environments to reset
    """
    if hasattr(env, '_task_stage'):
        env._task_stage[env_ids] = 0
        print(f"Reset task stage for {len(env_ids)} environment(s)")
