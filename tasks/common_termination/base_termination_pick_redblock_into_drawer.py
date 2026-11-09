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
    min_x: float = -2.7,                # minimum x position threshold
    max_x: float = -2.2,                # maximum x position threshold
    min_y: float = -4.15,                # minimum y position threshold
    max_y: float = -3.55,                # maximum y position threshold
    min_height: float = 0.2,
) -> torch.Tensor:
    # when the object is not in the set return, reset
    # Get object entity from the scene
    # 1. get object entity from the scene
    object: RigidObject = env.scene[object_cfg.name]
    
    # Extract wheel position relative to environment origin
    # 2. get object position
    wheel_x = object.data.root_pos_w.torch[:, 0]         # x position
    wheel_y = object.data.root_pos_w.torch[:, 1]        # y position
    wheel_height = object.data.root_pos_w.torch[:, 2]   # z position (height)
    done_x = (wheel_x < max_x) and  (wheel_x > min_x)
    done_y = (wheel_y < max_y) and (wheel_y > min_y)
    done_height = (wheel_height > min_height)
    done = done_x and done_y and done_height
    # print(f"done_x: {done_x}, done_y: {done_y}, done_height: {done_height}, done: {done}")
    # print(f"wheel_x: {wheel_x}, wheel_y: {wheel_y}, wheel_height: {wheel_height}")
    return  not done
