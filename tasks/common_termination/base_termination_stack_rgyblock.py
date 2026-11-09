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
    red_block_cfg: SceneEntityCfg = SceneEntityCfg("red_block"),
    yellow_block_cfg: SceneEntityCfg = SceneEntityCfg("yellow_block"),
    green_block_cfg: SceneEntityCfg = SceneEntityCfg("green_block"),
    min_x: float = -5.4,                # minimum x position threshold
    max_x: float = -2.9,                # maximum x position threshold
    min_y: float = -5.05,                # minimum y position threshold
    max_y: float = -2.8,                # maximum y position threshold
    min_height: float = 0.5,
) -> torch.Tensor:
   # when the object is not in the set return, reset
    # Get object entity from the scene
    # 1. get object entity from the scene
    red_block: RigidObject = env.scene[red_block_cfg.name]
    yellow_block: RigidObject = env.scene[yellow_block_cfg.name]
    green_block: RigidObject = env.scene[green_block_cfg.name]
    
    # Extract wheel position relative to environment origin
    # 2. get object position
    red_block_x = red_block.data.root_pos_w.torch[:, 0]         # x position
    red_block_y = red_block.data.root_pos_w.torch[:, 1]        # y position
    red_block_height = red_block.data.root_pos_w.torch[:, 2]   # z position (height)

    yellow_block_x = yellow_block.data.root_pos_w.torch[:, 0]         # x position
    yellow_block_y = yellow_block.data.root_pos_w.torch[:, 1]        # y position
    yellow_block_height = yellow_block.data.root_pos_w.torch[:, 2]   # z position (height)

    green_block_x = green_block.data.root_pos_w.torch[:, 0]         # x position
    green_block_y = green_block.data.root_pos_w.torch[:, 1]        # y position
    green_block_height = green_block.data.root_pos_w.torch[:, 2]   # z position (height)
    
    red_done_x = (red_block_x < max_x) and  (red_block_x > min_x)
    red_done_y = (red_block_y < max_y) and (red_block_y > min_y)
    red_done_height = (red_block_height > min_height)
    red_done = red_done_x and red_done_y and red_done_height

    yellow_done_x = (yellow_block_x < max_x) and  (yellow_block_x > min_x)
    yellow_done_y = (yellow_block_y < max_y) and (yellow_block_y > min_y)
    yellow_done_height = (yellow_block_height > min_height)
    yellow_done = yellow_done_x and yellow_done_y and yellow_done_height

    green_done_x = (green_block_x < max_x) and  (green_block_x > min_x)
    green_done_y = (green_block_y < max_y) and (green_block_y > min_y)
    green_done_height = (green_block_height > min_height)
    green_done = green_done_x and green_done_y and green_done_height

    done = not (red_done and yellow_done and green_done)

    return done
