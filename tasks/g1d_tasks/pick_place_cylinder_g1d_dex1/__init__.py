
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  

import gymnasium as gym
import os

from . import pickplace_cylinder_g1d_dex1_joint_env_cfg


gym.register(
    id="Isaac-PickPlace-Cylinder-G1D-Dex1-Joint",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": pickplace_cylinder_g1d_dex1_joint_env_cfg.PickPlaceG1DDEX1JointEnvCfg,
    },
    disable_env_checker=True,
)

