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
    min_x: float = -5.4,                # minimum x position threshold
    max_x: float = -2.9,                # maximum x position threshold
    min_y: float = -5.05,                # minimum y position threshold
    max_y: float = -2.8,                # maximum y position threshold
    min_height: float = 0.5,
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
    return not done
