# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  

"""
Healthcare Box Picking Task with G1 Robot (29 DOF + Dex3 Hands)

This module defines a box picking task in a healthcare environment using the 
Unitree G1 humanoid robot with wholebody control and Dex3 dexterous hands.

Environment Details:
- Scene: Healthcare environment from Omniverse
- Object: Cubic box (6cm x 6cm x 6cm, 200g)
- Robot: G1 29 DOF + Dex3 hands (wholebody control)
- Task: Pick up the box and lift it to target height
"""

import gymnasium as gym

from . import pick_box_healthcare_g1_29dof_dex3_env_cfg


# Register the environment with Gymnasium
gym.register(
    id="Isaac-Pick-Box-Healthcare-G129-Dex3",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": pick_box_healthcare_g1_29dof_dex3_env_cfg.PickBoxHealthcareG129Dex3EnvCfg,
    },
    disable_env_checker=True,
)

