# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Reward for placing GUI-important medical objects in the rear crate."""

from __future__ import annotations

import torch

from tasks.common_rewards.base_reward_pickplace_redblock import (
    _get_rewards_dds_instance,
)

from .container_goal import important_objects_contained


def compute_important_object_reward(env) -> torch.Tensor:
    """Return the fraction of important objects currently inside the crate."""
    reward = important_objects_contained(env).to(dtype=torch.float32).mean(dim=-1)
    rewards_dds = _get_rewards_dds_instance()
    if rewards_dds:
        rewards_dds.write_rewards_data(reward)
    return reward


compute_pill_bottle_reward = compute_important_object_reward


__all__ = ["compute_important_object_reward", "compute_pill_bottle_reward"]
