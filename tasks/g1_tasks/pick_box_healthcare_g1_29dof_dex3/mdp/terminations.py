# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  

"""
Termination conditions for box picking task in healthcare environment
"""

from __future__ import annotations
import torch
from typing import TYPE_CHECKING

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def object_out_of_bounds(
    env: ManagerBasedRLEnv,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    min_height: float = 0.3,
    max_height: float = 2.0,
    x_range: tuple = (-2.0, 2.0),
    y_range: tuple = (-2.0, 2.0),
) -> torch.Tensor:
    """Check if the box object has fallen out of the valid workspace.
    
    Args:
        env: The RL environment instance
        object_cfg: Configuration for the object entity
        min_height: Minimum valid height (below this, episode terminates)
        max_height: Maximum valid height (above this, episode terminates)
        x_range: Valid x-coordinate range (min, max)
        y_range: Valid y-coordinate range (min, max)
        
    Returns:
        torch.Tensor: Boolean tensor [batch] indicating termination condition
    """
    # Get the box object
    object: RigidObject = env.scene[object_cfg.name]
    box_pos = object.data.root_pos_w  # [batch, 3]
    
    # Check bounds
    out_of_bounds = torch.zeros(env.num_envs, device=env.device, dtype=torch.bool)
    
    # Height bounds
    out_of_bounds |= (box_pos[:, 2] < min_height)
    out_of_bounds |= (box_pos[:, 2] > max_height)
    
    # X bounds
    out_of_bounds |= (box_pos[:, 0] < x_range[0])
    out_of_bounds |= (box_pos[:, 0] > x_range[1])
    
    # Y bounds
    out_of_bounds |= (box_pos[:, 1] < y_range[0])
    out_of_bounds |= (box_pos[:, 1] > y_range[1])
    
    return out_of_bounds


def box_pick_success(
    env: ManagerBasedRLEnv,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    success_height: float = 1.0,
    hold_time_steps: int = 50,
) -> torch.Tensor:
    """Check if the box has been successfully picked and held.
    
    Args:
        env: The RL environment instance
        object_cfg: Configuration for the object entity
        success_height: Height above which pick is considered successful
        hold_time_steps: Number of consecutive steps box must be held at height
        
    Returns:
        torch.Tensor: Boolean tensor [batch] indicating success condition
    """
    # Get the box object
    object: RigidObject = env.scene[object_cfg.name]
    box_height = object.data.root_pos_w[:, 2]
    
    # Initialize success counter if not exists
    if not hasattr(env, "_pick_success_counter"):
        env._pick_success_counter = torch.zeros(env.num_envs, device=env.device, dtype=torch.int32)
    
    # Check if box is at success height
    at_success_height = box_height >= success_height
    
    # Increment counter where condition is met, reset where not
    env._pick_success_counter[at_success_height] += 1
    env._pick_success_counter[~at_success_height] = 0
    
    # Success if held for required time
    success = env._pick_success_counter >= hold_time_steps
    
    return success


# Export functions
__all__ = [
    "object_out_of_bounds",
    "box_pick_success",
]

