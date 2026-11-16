# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Unitree R1-A7-D with Dex2-5 tendon hands: stack (red / green / yellow blocks).

The task (scene, events, rewards, terminations, simulation settings) is the Dex1 task
``stack_rgyblock_r1a7d_dex1``; only the robot, its cameras and the hand observation differ.
The hands are torque controlled through rt/lowcmd motors 31..34 (see action_provider/dex2_5_torque.py).
"""

from isaaclab.assets import ArticulationCfg
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.utils import configclass

from robots.unitree_r1a7d import R1A7D_BASE_HEIGHT
from tasks.common_config import R1A7DRobotPresets  # isort: skip
from tasks.common_config.r1a7d_camera_configs import R1A7DCameraPresets
from tasks.common_observations.r1a7d_state import get_robot_r1a7d_dex2_5_joint_states
from tasks.r1a7d_tasks.stack_rgyblock_r1a7d_dex1.stack_rgyblock_r1a7d_dex1_joint_env_cfg import (
    ObjectTableSceneCfg as _Dex1SceneCfg,
    ObservationsCfg as _Dex1ObservationsCfg,
    StackRgyBlockR1A7DDEX1JointEnvCfg,
)


@configclass
class ObjectTableSceneCfg(_Dex1SceneCfg):
    """Scene of the Dex1 task with the Dex2-5 robot (same base pose)."""

    robot: ArticulationCfg = R1A7DRobotPresets.r1_d_dex2_5(init_pos=(-4.2, -3.43, R1A7D_BASE_HEIGHT), init_rot=(0, 0, -0.7071, 0.7071))
    # head stereo (left -> image "head", right -> "head_right"); wrist cameras only with a hand-eye calibration
    # of this robot (the Dex2-5 hand has no camera mount), otherwise None = no camera
    front_camera = R1A7DCameraPresets.head_left_camera("dex2_5")
    front_right_camera = R1A7DCameraPresets.head_right_camera("dex2_5")
    left_wrist_camera = R1A7DCameraPresets.left_wrist_camera("dex2_5")
    right_wrist_camera = R1A7DCameraPresets.right_wrist_camera("dex2_5")


@configclass
class ObservationsCfg(_Dex1ObservationsCfg):
    @configclass
    class PolicyCfg(_Dex1ObservationsCfg.PolicyCfg):
        # 20 finger joint positions (left 10, right 10); the hand motors are part of rt/lowstate (31..34)
        robot_gipper_state = ObsTerm(func=get_robot_r1a7d_dex2_5_joint_states)

    policy: PolicyCfg = PolicyCfg()


@configclass
class StackRgyBlockR1A7DDEX25JointEnvCfg(StackRgyBlockR1A7DDEX1JointEnvCfg):
    """Unitree R1-A7-D (wheeled base, Dex2-5 hands) stack (red / green / yellow blocks) environment."""

    scene: ObjectTableSceneCfg = ObjectTableSceneCfg(num_envs=1, env_spacing=2.5, replicate_physics=True)
    observations: ObservationsCfg = ObservationsCfg()
