
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  

import gymnasium as gym
import os

from . import pickplace_cylinder_r1d_dex1_joint_env_cfg


gym.register(
    id="Isaac-PickPlace-Cylinder-R1D-Dex1-Joint",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": pickplace_cylinder_r1d_dex1_joint_env_cfg.PickPlaceR1DDEX1JointEnvCfg,
    },
    disable_env_checker=True,
)

