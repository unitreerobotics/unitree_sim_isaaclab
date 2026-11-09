# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0      
from __future__ import annotations

import torch
from typing import TYPE_CHECKING
import sys
import os
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
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    min_x: float = -5.4,                # minimum x position threshold
    max_x: float = -2.9,                # maximum x position threshold
    min_y: float = -5.05,                # minimum y position threshold
    max_y: float = -2.8,                # maximum y position threshold
    post_min_x: float = -4.2125,
    post_max_x: float = -4.1755,
    post_min_y: float = -3.9606,
    post_max_y: float = -3.9121,
    min_height: float = 0.5,
    post_min_height: float = 0.82,
    post_max_height: float = 0.825,
) -> torch.Tensor:
   # when the object is not in the set return, reset
    interval = getattr(env, "_reward_interval", 1) or 1
    counter = getattr(env, "_reward_counter", 0)
    last = getattr(env, "_reward_last", None)
    if interval > 1 and last is not None and counter % interval != 0:
        env._reward_counter = counter + 1
        return last

    # 1. get object entity from the scene
    object: RigidObject = env.scene[object_cfg.name]
    
    # 2. get object position
    wheel_x = object.data.root_pos_w.torch[:, 0]         # x position
    wheel_y = object.data.root_pos_w.torch[:, 1]        # y position
    wheel_height = object.data.root_pos_w.torch[:, 2]   # z position (height)

    # element-wise operations
    done_x = (wheel_x < max_x) & (wheel_x > min_x)
    done_y = (wheel_y < max_y) & (wheel_y > min_y)
    done_height = (wheel_height > min_height)
    done = done_x & done_y & done_height

    # 3. get post position conditions
    done_post_x = (wheel_x < post_max_x) & (wheel_x > post_min_x)
    done_post_y = (wheel_y < post_max_y) & (wheel_y > post_min_y)
    done_post_height = (wheel_height > post_min_height) & (wheel_height < post_max_height)
    done_post = done_post_x & done_post_y & done_post_height

    # Create reward tensor for all environments
    reward = torch.zeros(env.num_envs, device=str(env.device), dtype=torch.float)
    
    # Set rewards based on conditions
    reward[~done] = -1.0  # Not in valid area
    reward[done_post] = 1.0  # In target post area
    reward[done & ~done_post] = 0.0  # In valid area but not target

    env._reward_last = reward
    env._reward_counter = counter + 1
    rewards_dds = _get_rewards_dds_instance()
    if rewards_dds:
        rewards_dds.write_rewards_data(reward)
    return reward
