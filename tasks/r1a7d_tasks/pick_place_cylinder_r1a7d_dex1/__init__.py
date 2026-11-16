
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  

import gymnasium as gym
import os

from . import pickplace_cylinder_r1a7d_dex1_joint_env_cfg


gym.register(
    id="Isaac-PickPlace-Cylinder-R1A7D-Dex1-Joint",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": pickplace_cylinder_r1a7d_dex1_joint_env_cfg.PickPlaceR1A7DDEX1JointEnvCfg,
    },
    disable_env_checker=True,
)

