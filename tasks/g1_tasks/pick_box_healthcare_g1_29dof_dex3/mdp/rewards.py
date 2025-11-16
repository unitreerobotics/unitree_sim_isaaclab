# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  

"""
Reward functions for box picking task in healthcare environment
"""

from __future__ import annotations
import torch
import sys
import os
from typing import TYPE_CHECKING

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

# Global variable for DDS communication
_rewards_dds = None
_dds_initialized = False


def _get_rewards_dds_instance():
    """Get the DDS instance for reward communication (lazy initialization)"""
    global _rewards_dds, _dds_initialized
    
    if not _dds_initialized or _rewards_dds is None:
        try:
            # Dynamically import the DDS module
            sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'dds'))
            from dds.dds_master import dds_manager
            
            _rewards_dds = dds_manager.get_object("rewards")
            print("[Box Picking Rewards] DDS communication instance obtained")
            
            # Register cleanup function
            import atexit
            def cleanup_dds():
                try:
                    if _rewards_dds:
                        dds_manager.unregister_object("rewards")
                        print("[Box Picking Rewards] DDS communication closed")
                except Exception as e:
                    print(f"[Box Picking Rewards] Error closing DDS: {e}")
            atexit.register(cleanup_dds)
            
        except Exception as e:
            print(f"[Box Picking Rewards] Failed to get DDS instance: {e}")
            _rewards_dds = None
        
        _dds_initialized = True
    
    return _rewards_dds


def compute_box_picking_reward(
    env: ManagerBasedRLEnv,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    target_height: float = 1.0,  # target height for picking (1 meter)
    pick_height_threshold: float = 0.9,  # minimum height for successful pick
    workspace_bounds: dict = None,  # workspace boundaries
) -> torch.Tensor:
    """Compute reward for box picking task.
    
    Reward structure:
    - Positive reward for lifting the box above pick_height_threshold
    - Distance-based reward for moving towards target height
    - Bonus for maintaining stable grasp
    
    Args:
        env: The RL environment instance
        object_cfg: Configuration for the object entity
        target_height: Target height for successful pick (meters)
        pick_height_threshold: Minimum height to consider pick successful
        workspace_bounds: Optional workspace boundaries dict with 'x', 'y', 'z' ranges
        
    Returns:
        torch.Tensor: Reward values for each environment [batch]
    """
    # Rate limiting for DDS publishing
    interval = getattr(env, "_reward_interval", 1) or 1
    counter = getattr(env, "_reward_counter", 0)
    last_reward = getattr(env, "_reward_last", None)
    
    if interval > 1 and last_reward is not None and counter % interval != 0:
        env._reward_counter = counter + 1
        return last_reward
    
    # Get the box object from the scene
    object: RigidObject = env.scene[object_cfg.name]
    
    # Extract box position and velocity
    box_pos = object.data.root_pos_w  # [batch, 3]
    box_vel = object.data.root_lin_vel_w  # [batch, 3]
    
    box_height = box_pos[:, 2]  # z-coordinate (height)
    
    # Initialize reward tensor
    reward = torch.zeros(env.num_envs, device=env.device, dtype=torch.float)
    
    # 1. Height-based reward: reward for lifting the box
    height_reward = torch.clamp(box_height - 0.5, min=0.0, max=target_height - 0.5) / (target_height - 0.5)
    reward += height_reward
    
    # 2. Success bonus: large bonus for reaching pick height threshold
    success_mask = box_height >= pick_height_threshold
    reward[success_mask] += 2.0
    
    # 3. Stability bonus: reward for keeping the box stable (low velocity)
    box_speed = torch.norm(box_vel, dim=-1)
    stability_reward = torch.exp(-box_speed * 2.0) * 0.5  # exponential decay with speed
    reward += stability_reward
    
    # 4. Workspace penalty: negative reward if box is outside workspace
    if workspace_bounds is not None:
        out_of_bounds = torch.zeros(env.num_envs, device=env.device, dtype=torch.bool)
        
        if 'x' in workspace_bounds:
            x_min, x_max = workspace_bounds['x']
            out_of_bounds |= (box_pos[:, 0] < x_min) | (box_pos[:, 0] > x_max)
        
        if 'y' in workspace_bounds:
            y_min, y_max = workspace_bounds['y']
            out_of_bounds |= (box_pos[:, 1] < y_min) | (box_pos[:, 1] > y_max)
        
        if 'z' in workspace_bounds:
            z_min, z_max = workspace_bounds['z']
            out_of_bounds |= (box_pos[:, 2] < z_min) | (box_pos[:, 2] > z_max)
        
        reward[out_of_bounds] = -1.0
    
    # Publish rewards via DDS for monitoring
    rewards_dds = _get_rewards_dds_instance()
    if rewards_dds:
        try:
            rewards_dds.write_rewards_data(reward)
        except Exception as e:
            print(f"[Box Picking Rewards] Error writing to DDS: {e}")
    
    # Cache for rate limiting
    env._reward_last = reward
    env._reward_counter = counter + 1
    
    return reward


def box_drop_penalty(
    env: ManagerBasedRLEnv,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    drop_height_threshold: float = 0.5,  # height below which box is considered dropped
    initial_height: float = 0.85,  # initial spawn height
) -> torch.Tensor:
    """Compute penalty for dropping the box.
    
    Args:
        env: The RL environment instance
        object_cfg: Configuration for the object entity
        drop_height_threshold: Height below which box is considered dropped
        initial_height: Initial height of the box
        
    Returns:
        torch.Tensor: Penalty values (negative) for each environment [batch]
    """
    # Get the box object
    object: RigidObject = env.scene[object_cfg.name]
    box_height = object.data.root_pos_w[:, 2]
    
    # Penalty if box drops below threshold
    penalty = torch.zeros(env.num_envs, device=env.device, dtype=torch.float)
    dropped_mask = box_height < drop_height_threshold
    penalty[dropped_mask] = -1.0
    
    return penalty


# Export functions
__all__ = [
    "compute_box_picking_reward",
    "box_drop_penalty",
]

