
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
from tasks.common_termination.base_termination_pick_place_cylinder import reset_object_estimate
import torch
from typing import TYPE_CHECKING

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg

__all__ = [
"reset_object_estimate",
"object_moved"
]

def object_moved(
    env,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    distance_threshold: float = 0.07,
) -> torch.Tensor:
    """Terminate if the object has moved by a certain distance from its initial position.

    This function caches the object's initial position at the start of the episode
    and checks if the object has moved beyond the specified distance threshold.
    It uses `env.extras` to store the initial position across steps for each environment.

    Args:
        env: The RL environment instance.
        object_cfg: Configuration for the object entity.
        distance_threshold: The distance threshold (in meters) to trigger termination.
                           Defaults to 0.05 (5 cm).

    Returns:
        Boolean tensor indicating which environments have met the termination condition.
    """
    # Get object entity from the scene
    obj: RigidObject = env.scene[object_cfg.name]

    # Define a unique key for storing the initial position in extras
    initial_pos_key = f"initial_pos_{object_cfg.name}"

    # At the very first step of the simulation, create the buffer in extras
    if initial_pos_key not in env.extras:
        env.extras[initial_pos_key] = obj.data.root_pos_w.clone()

    # For any environment that was reset, update its initial position.
    # The first step of an episode is indicated by episode_length_buf being 0.
    is_first_step = env.episode_length_buf == 0
    if torch.any(is_first_step):
        env.extras[initial_pos_key][is_first_step] = obj.data.root_pos_w[is_first_step].clone()

    # Retrieve initial and current positions
    initial_pos = env.extras[initial_pos_key]
    current_pos = obj.data.root_pos_w

    # Compute the Euclidean distance moved from the initial position
    distance_moved = torch.norm(current_pos[:, :3] - initial_pos[:, :3], p=2, dim=1)

    # Check if the distance moved exceeds the threshold
    return distance_moved > distance_threshold
