# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
import gymnasium as gym

from . import pickplace_medicine_bottle_hospital_g1_29dof_dex1_joint_env_cfg


gym.register(
    id="Isaac-PickPlace-MedicineBottle-Hospital-G129-Dex1-Joint",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": pickplace_medicine_bottle_hospital_g1_29dof_dex1_joint_env_cfg.PickPlaceMedicineBottleHospitalG129DEX1EnvCfg,
    },
    disable_env_checker=True,
)
