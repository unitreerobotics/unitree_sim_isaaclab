# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""R1-7A arm tasks: the R1-A7-D tasks' scenes, events, rewards and terminations with the arm robots.

Each scene can be used with both mountings (the robot faces the table like the R1-A7-D of these scenes):

* ``Isaac-<task>-R17A-<hand>-Table-Joint``:  arm base flush on the table top at the robot-side edge
* ``Isaac-<task>-R17A-<hand>-Column-Joint``: arm base on a column on the floor right in front of the table edge
* ``Isaac-<task>-R17A-<hand>-Joint``:        the task's default mounting (cylinder / red block: table,
  stacking: column)

``<task>``: ``PickPlace-Cylinder``, ``PickPlace-RedBlock``, ``Stack-RgyBlock``; ``<hand>``: ``Dex1``, ``Dex2_5``,
``O6`` (LinkerHand O6), each on one left arm.
"""

from __future__ import annotations

import math

from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.utils import configclass

from tasks.common_config.r17a_robot_configs import (
    R17A_MOUNTS,
    r17a_anchor,
    r17a_column,
    r17a_dex1_wrist_camera,
    r17a_front_camera,
    r17a_mount,
    r17a_robot,
)
from tasks.common_observations import r17a_state
from tasks.r1a7d_tasks.pick_place_cylinder_r1a7d_dex1 import pickplace_cylinder_r1a7d_dex1_joint_env_cfg as cylinder
from tasks.r1a7d_tasks.pick_place_redblock_r1a7d_dex1 import pickplace_redblock_r1a7d_dex1_joint_env_cfg as redblock
from tasks.r1a7d_tasks.stack_rgyblock_r1a7d_dex1 import stack_rgyblock_r1a7d_dex1_joint_env_cfg as stack

# Both scenes use the PackingTable at z = -0.2: its top (M_HeavyDutyPackingTable_C01_TableTop, collider
# boundingCube) is at 0.794 m.
TABLE_TOP = 0.794  # [m]
HANDS = {"dex1": "Dex1", "dex2_5": "Dex2_5", "o6": "O6"}
HAND_OBS = {
    "dex1": r17a_state.get_robot_r17a_dex1_joint_states,
    "dex2_5": r17a_state.get_robot_r17a_dex2_5_joint_states,
    "o6": r17a_state.get_robot_r17a_o6_joint_states,
}
# task -> (R1-A7-D Dex1 module, env class, default mount, robot yaw (0 = arms reach towards +x),
#          (x, y) on the robot-side table edge where the arm goes)
# Cylinder scene: table top x -1.22..1.22, y 0.169..0.931 (robot-side edge y = 0.169); cylinder (r 18 mm, 0.35 m
# high) at x -0.40..-0.30, y 0.35..0.45; container on the table at x 0.26..0.99, y 0.21..0.70 (rim 0.883 m). The arm
# goes between cylinder and container (reaches both).
# Red block / stacking scene: table top x -5.52..-3.08, y -4.58..-3.819 (robot-side edge y = -3.819); target square
# x -4.25..-4.14, y -4.00..-3.90; blocks around x -4.40..-3.95, y -4.17..-3.97. The arm goes to the right of the
# target square and the blocks (red block) or in front of them (stacking).
TASKS = {
    "PickPlace-Cylinder": (cylinder, "PickPlaceR1A7DDEX1JointEnvCfg", "table", math.pi / 2, (-0.05, 0.169)),
    "PickPlace-RedBlock": (redblock, "PickPlaceRedBlockR1A7DDEX1JointEnvCfg", "table", -math.pi / 2, (-3.92, -3.819)),
    "Stack-RgyBlock": (stack, "StackRgyBlockR1A7DDEX1JointEnvCfg", "column", -math.pi / 2, (-4.20, -3.819)),
}


def _scene_cfg(base, hand: str, mount: str, yaw: float, table_top: float, anchor):
    pos, rot, columns = r17a_mount(hand, mount, anchor, yaw, table_top)
    fx, fy = math.cos(yaw), math.sin(yaw)  # forward
    eye = (anchor[0] - 0.45 * fx, anchor[1] - 0.45 * fy, pos[2] + 0.70)  # behind and above the arm base(s)
    attrs = {
        "robot": r17a_robot(hand, pos, rot),
        "front_camera": r17a_front_camera(eye, (anchor[0] + 0.30 * fx, anchor[1] + 0.30 * fy, table_top)),
        "front_right_camera": None,
        "left_wrist_camera": r17a_dex1_wrist_camera() if hand == "dex1" else None,
        "right_wrist_camera": None,
    }
    for i, (x, y, top) in enumerate(columns):
        attrs[f"arm_column_{i}"] = r17a_column(x, y, top, f"ArmColumn_{i}")
    return configclass(type(f"R17A{HANDS[hand]}{mount.title()}SceneCfg", (base,), attrs))


def _obs_cfg(base, hand: str):
    policy = configclass(type("PolicyCfg", (base.PolicyCfg,), {
        "robot_joint_state": ObsTerm(func=r17a_state.get_robot_r17a_body_joint_states),
        "robot_gipper_state": ObsTerm(func=HAND_OBS[hand]),
    }))
    return configclass(type("ObservationsCfg", (base,), {"PolicyCfg": policy, "policy": policy()}))


ENV_CFGS = {}
for task, (module, env_name, default_mount, yaw, edge_point) in TASKS.items():
    for hand, tag in HANDS.items():
        obs = _obs_cfg(module.ObservationsCfg, hand)
        for mount in R17A_MOUNTS:
            scene = _scene_cfg(module.ObjectTableSceneCfg, hand, mount, yaw, TABLE_TOP, r17a_anchor(mount, edge_point, yaw))
            name = f"{task.replace('-', '')}R17A{tag.replace('_', '')}{mount.title()}JointEnvCfg"
            cls = configclass(type(name, (getattr(module, env_name),), {
                "__doc__": f"Unitree R1-7A ({tag}, {mount} mount) {task} environment",
                "scene": scene(num_envs=1, env_spacing=2.5, replicate_physics=True),
                "observations": obs(),
            }))
            globals()[name] = cls
            ENV_CFGS[f"Isaac-{task}-R17A-{tag}-{mount.title()}-Joint"] = cls
            if mount == default_mount:  # plain task id / class name: the default mounting
                globals()[name.replace(mount.title(), "", 1)] = cls
                ENV_CFGS[f"Isaac-{task}-R17A-{tag}-Joint"] = cls
