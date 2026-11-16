# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0

import gymnasium as gym

from . import pickplace_redblock_r1a7d_dex2_5_joint_env_cfg


gym.register(
    id="Isaac-PickPlace-RedBlock-R1A7D-Dex2_5-Joint",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": pickplace_redblock_r1a7d_dex2_5_joint_env_cfg.PickPlaceRedBlockR1A7DDEX25JointEnvCfg,
    },
    disable_env_checker=True,
)
