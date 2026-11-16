# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Unitree R1-7A arm tasks (fixed-base 7-DOF arm on the table top or on a column; every task has a ``-Table-`` and a
``-Column-`` variant, the plain id is the task's default), see r17a_env_cfg.py."""

import gymnasium as gym

from .r17a_env_cfg import ENV_CFGS

for _task_id, _cfg in ENV_CFGS.items():
    gym.register(
        id=_task_id,
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        kwargs={"env_cfg_entry_point": _cfg},
        disable_env_checker=True,
    )

__all__ = ["ENV_CFGS"]
