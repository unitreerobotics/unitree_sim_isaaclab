# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Shared reset wiring for randomized pick-and-place room tasks."""

from __future__ import annotations

import torch

import isaaclab.envs.mdp as base_mdp
from tasks.common_event.event_manager import SimpleEvent, SimpleEventManager

from .room_events import (
    randomize_pickplace_room_layout,
    randomize_wall_props_layout,
    reset_target_on_current_table,
)

WALL_PROP_NAMES = [
    "medical_cabinet",
    "shelf_set",
    "supply_cabinet",
    "supply_cart_a",
    "supply_cart_b",
    "trash_can",
    "plant_a",
    "plant_b",
]

TABLE_PROP_NAMES = [
    "desk_lamp",
    "blue_cube",
    "yellow_cube",
]

HOSPITAL_TABLE_PROP_NAMES = [
    "hand_sanitizer",
    "gauze_box",
    "specimen_cup",
]


def randomize_room_for_all_envs(env) -> None:
    """Randomize the pick-and-place room layout for every environment instance."""
    randomize_pickplace_room_layout(
        env,
        torch.arange(env.num_envs, device=env.device),
        wall_prop_names=WALL_PROP_NAMES,
        table_prop_names=TABLE_PROP_NAMES,
        min_table_objects=len(TABLE_PROP_NAMES),
    )


def reset_target_for_all_envs(env) -> None:
    """Reset only the target, preserving the current randomized room layout."""
    reset_target_on_current_table(
        env,
        torch.arange(env.num_envs, device=env.device),
    )


def reset_all_then_randomize_room(env) -> None:
    """Reset all scene entities to defaults, then generate a fresh room layout."""
    env_ids = torch.arange(env.num_envs, device=env.device)
    base_mdp.reset_scene_to_default(env, env_ids)
    randomize_pickplace_room_layout(
        env,
        env_ids,
        wall_prop_names=WALL_PROP_NAMES,
        table_prop_names=TABLE_PROP_NAMES,
        min_table_objects=len(TABLE_PROP_NAMES),
    )


def reset_all_then_randomize_wall_props(env) -> None:
    """Reset a fixed task scene, then reapply wall placement from shared constants."""
    env_ids = torch.arange(env.num_envs, device=env.device)
    base_mdp.reset_scene_to_default(env, env_ids)
    randomize_wall_props_layout(env, env_ids, wall_prop_names=WALL_PROP_NAMES)


def register_randomized_room_reset_events(cfg) -> None:
    """Install DDS/manual reset callbacks that mirror the native reset randomizer."""
    cfg.event_manager = SimpleEventManager()
    cfg.event_manager.register("reset_object_self", SimpleEvent(func=reset_target_for_all_envs))
    cfg.event_manager.register("reset_all_self", SimpleEvent(func=reset_all_then_randomize_room))
