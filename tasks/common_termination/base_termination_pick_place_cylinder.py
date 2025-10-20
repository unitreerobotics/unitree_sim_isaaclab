# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0      
from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def reset_object_estimate(
    env: ManagerBasedRLEnv,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    min_x: float = -0.42,                # minimum x position threshold
    max_x: float = 1.0,                # maximum x position threshold
    min_y: float = 0.2,                # minimum y position threshold
    max_y: float = 0.7,                # maximum y position threshold
    min_height: float = 0.5,
) -> torch.Tensor:
    # Return a boolean tensor per environment indicating reset condition
    object: RigidObject = env.scene[object_cfg.name]
    positions_w = object.data.root_pos_w  # shape: (num_envs, 3)
    wheel_x = positions_w[:, 0]
    wheel_y = positions_w[:, 1]
    wheel_height = positions_w[:, 2]

    in_x = torch.logical_and(wheel_x > min_x, wheel_x < max_x)
    in_y = torch.logical_and(wheel_y > min_y, wheel_y < max_y)
    in_h = wheel_height > min_height
    inside_region = torch.logical_and(torch.logical_and(in_x, in_y), in_h)
    # Done means object is inside region; reset signal is the negation
    return ~inside_region
