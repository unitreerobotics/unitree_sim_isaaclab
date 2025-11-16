# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  

"""
Observation functions for box picking task in healthcare environment
"""

from __future__ import annotations
import torch
from typing import TYPE_CHECKING

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg

# Import common observation functions
from tasks.common_observations.g1_29dof_state import get_robot_boy_joint_states
from tasks.common_observations.dex3_state import get_robot_dex3_joint_states
from tasks.common_observations.camera_state import get_camera_image

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def get_object_state(
    env: ManagerBasedRLEnv,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:
    """Get the box object state including position, orientation, and velocities.
    
    Args:
        env: The RL environment instance
        object_cfg: Configuration for the object entity
        
    Returns:
        torch.Tensor: Object state tensor [batch, 13] containing:
            - position (3): xyz world coordinates
            - orientation (4): quaternion (w, x, y, z)
            - linear velocity (3): xyz velocities
            - angular velocity (3): xyz angular velocities
    """
    # Get the object from the scene
    object: RigidObject = env.scene[object_cfg.name]
    
    # Extract state components
    position = object.data.root_pos_w  # [batch, 3]
    orientation = object.data.root_quat_w  # [batch, 4]
    linear_velocity = object.data.root_lin_vel_w  # [batch, 3]
    angular_velocity = object.data.root_ang_vel_w  # [batch, 3]
    
    # Concatenate all state information
    object_state = torch.cat([
        position,
        orientation,
        linear_velocity,
        angular_velocity
    ], dim=-1)
    
    return object_state


# Export functions for external use
__all__ = [
    "get_robot_boy_joint_states",
    "get_robot_dex3_joint_states",
    "get_camera_image",
    "get_object_state",
]

