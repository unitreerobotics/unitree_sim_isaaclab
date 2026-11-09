# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0      
from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

# global variable to cache the DDS instance
_rewards_dds = None
_dds_initialized = False
import sys
import os
def _get_rewards_dds_instance():
    """get the DDS instance, delay initialization"""
    global _rewards_dds, _dds_initialized
    
    if not _dds_initialized or _rewards_dds is None:
        try:
            # dynamically import the DDS module
            sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), 'dds'))
            from dds.dds_master import dds_manager
            
            _rewards_dds = dds_manager.get_object("rewards")
            print("[Observations Rewards] DDS communication instance obtained")
            
            # register the cleanup function
            import atexit
            def cleanup_dds():
                try:
                    if _rewards_dds:
                        dds_manager.unregister_object("rewards")
                        print("[rewards_dds] DDS communication closed correctly")
                except Exception as e:
                    print(f"[rewards_dds] Error closing DDS: {e}")
            atexit.register(cleanup_dds)
            
        except Exception as e:
            print(f"[Observations Rewards] Failed to get DDS instances: {e}")
            _rewards_dds = None
        
        _dds_initialized = True
    
    return _rewards_dds
def compute_reward(
    env: ManagerBasedRLEnv,
    red_block_cfg: SceneEntityCfg = SceneEntityCfg("red_block"),
    yellow_block_cfg: SceneEntityCfg = SceneEntityCfg("yellow_block"),
    green_block_cfg: SceneEntityCfg = SceneEntityCfg("green_block"),
    min_x: float = -5.4,                # minimum x position threshold
    max_x: float = -2.9,                #  maximum x position threshold
    min_y: float = -5.05,               # minimum y position threshold
    max_y: float = -2.8,                # maximum y  position threshold
    min_height: float = 0.5,            # minimum height threshold
    post_min_x: float = -4.22,        # minimum x position threshold
    post_max_x: float = -4.16,        # maximum x position threshold
    post_min_y: float = -3.98,        # minimum y position threshold
    post_max_y: float = -3.91,        # maximum y position threshold
    post_min_height: float = 0.815,      # minimum height threshold
    post_max_height: float = 0.83,      # maximum height threshold
    stack_type: str = "red_yellow_green", # stack order type
    block_height: float = 0.05,         # height of a single block
) -> torch.Tensor:

   # when the object is not in the set return, reset
    interval = getattr(env, "_reward_interval", 1) or 1
    counter = getattr(env, "_reward_counter", 0)
    last = getattr(env, "_reward_last", None)
    if interval > 1 and last is not None and counter % interval != 0:
        env._reward_counter = counter + 1
        return last

    # 1. get object entity from the scene
    red_block: RigidObject = env.scene[red_block_cfg.name]
    yellow_block: RigidObject = env.scene[yellow_block_cfg.name]
    green_block: RigidObject = env.scene[green_block_cfg.name]
    
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
    
    # check if all blocks are in the working area
    red_done_x = (red_block_x < max_x) & (red_block_x > min_x)
    red_done_y = (red_block_y < max_y) & (red_block_y > min_y)
    red_done_height = (red_block_height > min_height)
    red_in_area = red_done_x & red_done_y & red_done_height

    yellow_done_x = (yellow_block_x < max_x) & (yellow_block_x > min_x)
    yellow_done_y = (yellow_block_y < max_y) & (yellow_block_y > min_y)
    yellow_done_height = (yellow_block_height > min_height)
    yellow_in_area = yellow_done_x & yellow_done_y & yellow_done_height

    green_done_x = (green_block_x < max_x) & (green_block_x > min_x)
    green_done_y = (green_block_y < max_y) & (green_block_y > min_y)
    green_done_height = (green_block_height > min_height)
    green_in_area = green_done_x & green_done_y & green_done_height

    # all blocks are in the working area
    all_in_area = red_in_area & yellow_in_area & green_in_area

    # create reward tensor
    reward = torch.zeros(env.num_envs, device=str(env.device), dtype=torch.float)
    
    # basic penalty: if any block is not in the working area
    reward[~all_in_area] = -1.0
    
    # parse stack order
    stack_order = stack_type.split("_")
    first_block = stack_order[0]
    second_block = stack_order[1] 
    third_block = stack_order[2]
    
    # create a dictionary of block positions and heights for dynamic access
    block_positions = {
        "red": (red_block_x, red_block_y, red_block_height),
        "yellow": (yellow_block_x, yellow_block_y, yellow_block_height),
        "green": (green_block_x, green_block_y, green_block_height)
    }
    
    # get the position information of each block
    first_x, first_y, first_height = block_positions[first_block]
    second_x, second_y, second_height = block_positions[second_block]
    third_x, third_y, third_height = block_positions[third_block]
    
    # check if each block is in the correct stack position and height
    # first block (bottom)
    first_block_in_area = (first_x < post_max_x) & (first_x > post_min_x) & \
                         (first_y < post_max_y) & (first_y > post_min_y) & \
                         (first_height >= post_min_height) & (first_height < post_max_height)
    
    # second block (middle)
    second_block_in_area = (second_x < post_max_x) & (second_x > post_min_x) & \
                          (second_y < post_max_y) & (second_y > post_min_y) & \
                          (second_height >= post_min_height + block_height) & (second_height < post_max_height +  block_height)
    
    # third block (top)
    third_block_in_area = (third_x < post_max_x) & (third_x > post_min_x) & \
                         (third_y < post_max_y) & (third_y > post_min_y) & \
                         (third_height >= post_min_height + 2 * block_height) & (third_height < post_max_height + 2 * block_height)
    
    # hierarchical reward system
    reward[all_in_area & first_block_in_area] = 0.3  # bottom block correct
    reward[all_in_area & first_block_in_area & second_block_in_area] = 0.6  # bottom and middle block correct
    reward[all_in_area & first_block_in_area & second_block_in_area & third_block_in_area] = 1.0  # perfect stack

    rewards_dds = _get_rewards_dds_instance()
    if rewards_dds:
        rewards_dds.write_rewards_data(reward)  
    env._reward_last = reward
    env._reward_counter = counter + 1
    return reward
