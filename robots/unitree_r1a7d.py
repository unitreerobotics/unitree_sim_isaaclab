# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Configuration for the Unitree R1-A7-D (wheeled base + two-stage lift + R1 upper body) with Dex1 grippers or
Dex2-5 tendon hands.

R1-A7-D is an independent robot (assets: assets/urdf/r1_d_description/r1_d_with_dex1_1.urdf and
r1_d_with_dex2_5.urdf, loaded through ``robots/robot_assets.py``; a converted .usd works as well).

Joints:
    Left/Right_Wheel_joint, *_caster_{Yaw,Roll}_joint : drive wheels and passive casters (free; the held base hovers)
    lift_stage1_joint, lift_stage2_joint               : two-stage lift, prismatic 0 .. 0.425 m each (0.04 m/s each)
    waist_yaw_joint                                    : waist yaw          (lowcmd motor 13)
    head_pitch_joint, head_yaw_joint                   : head pitch / yaw   (lowcmd motors 29 / 30)
    <side>_{shoulder_*,elbow,wrist_*}_joint            : 7-dof arms         (lowcmd motors 15..28)
    <side>_dex1_finger_joint_1/2                       : Dex1 fingers, prismatic 0 = closed .. 0.041 m = open
    <side>_{11,12,21,..,52}_joint                      : Dex2-5 hand (Dex2-5 asset), 10 revolute joints per hand,
                                                         0 = open; two torque motors per hand, lowcmd motors
                                                         31 / 32 (left) and 33 / 34 (right), see robots/dex2_5_hand.py

The motor indices follow xr_teleoperate (R1_A7_JointIndex). The base is a floating base held / driven
by the action provider (see action_provider/action_provider_r1a7d_dds.py), like the G1-D.
"""

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

from robots.dex2_5_hand import DEX2_5_ARMATURE, DEX2_5_DAMPING, DEX2_5_JOINT_EXPR, DEX2_5_STIFFNESS
from robots.robot_assets import robot_spawn_cfg

project_root = os.environ.get("PROJECT_ROOT") or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

R1A7D_ASSET_DIR = f"{project_root}/assets/urdf/r1_d_description"
R1A7D_DEX1_ASSET = f"{R1A7D_ASSET_DIR}/r1_d_with_dex1_1.urdf"
R1A7D_DEX2_5_ASSET = f"{R1A7D_ASSET_DIR}/r1_d_with_dex2_5.urdf"

R1A7D_DEX1_FINGER_OPEN = 0.041
R1A7D_DEX1_FINGER_CLOSED = 0.0

# lift / waist / head (driven from DDS)
R1A7D_LIFT_JOINTS = ("lift_stage1_joint", "lift_stage2_joint")
R1A7D_LIFT_STAGE_MAX = 0.425
R1A7D_LIFT_SOFT_MAX = 0.84  # [m] column soft limit (850 mm travel, [0, ~840] mm)
R1A7D_COLUMN_RADIUS = 0.003820  # [m/rad] column motor: height = q * kColumnRadius_m (lowstate motor 12)
# default lift: 0.27 m per stage puts the shoulders at ~1.05 m, the height of the fixed-base G1 shoulders
R1A7D_LIFT_DEFAULT = 0.27
R1A7D_WAIST_YAW_JOINT = "waist_yaw_joint"  # lowcmd/lowstate motor 13
R1A7D_HEAD_PITCH_JOINT = "head_pitch_joint"  # lowcmd/lowstate motor 29
R1A7D_HEAD_YAW_JOINT = "head_yaw_joint"  # lowcmd/lowstate motor 30
R1A7D_HEAD_PITCH_DEFAULT = 0.5  # [rad] look ~29 deg down at the table (limit +-0.628)
R1A7D_WHEEL_RADIUS = 0.0625
R1A7D_WHEEL_SEPARATION = 2 * 0.1968
R1A7D_BASE_HEIGHT = 0.155  # base_link above the floor with the wheels / casters on the ground

# R1-A5/A7D twist module (rt/cmd_vel / rt/state_vel are normalized to [-1, 1]):
R1A7D_GEAR_VMAX = {"low": 0.5, "mid": 1.0, "high": 1.5}  # kGearLinearVel_mps [m/s]
R1A7D_GEAR = os.environ.get("UNITREE_R1A7D_GEAR", "low")  # simulated gear (env UNITREE_R1A7D_GEAR=low|mid|high)
R1A7D_MAX_ANGULAR_VEL = 1.2  # kMaxAngularVel_rad [rad/s]
# base / column response (slew limits, column speed, command timeout, state rate): robots/sysid/r1a7d.py

# Appearance: link colors are the ones in the URDF; the finish (metallic / roughness) is applied at
# start-up by robots.robot_assets.apply_material_finish.

R1A7D_ARM_JOINT_POS = {
    f"{side}_{j}_joint": 0.0
    for side in ("left", "right")
    for j in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")
}


def _r1a7d_cfg(asset: str, hand_joint_pos: dict, hand_actuators: dict) -> ArticulationCfg:
    """R1-A7-D articulation with the given asset; ``hand_*`` = default pose / actuators of its end effectors."""
    return ArticulationCfg(
        spawn=robot_spawn_cfg(
            asset,
            fix_base=False,  # floating base, held / moved by the action provider
            activate_contact_sensors=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                retain_accelerations=True,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=1000.0,
                max_angular_velocity=1000.0,
                max_depenetration_velocity=1.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=4,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, R1A7D_BASE_HEIGHT),
            joint_pos={
                "(Left|Right)_Wheel_joint": 0.0,
                ".*_caster_.*": 0.0,
                "lift_stage1_joint": R1A7D_LIFT_DEFAULT,
                "lift_stage2_joint": R1A7D_LIFT_DEFAULT,
                R1A7D_WAIST_YAW_JOINT: 0.0,
                R1A7D_HEAD_PITCH_JOINT: R1A7D_HEAD_PITCH_DEFAULT,
                R1A7D_HEAD_YAW_JOINT: 0.0,
                **R1A7D_ARM_JOINT_POS,
                **hand_joint_pos,
            },
            joint_vel={".*": 0.0},
        ),
        # 1.0: several defaults (lift, fingers) lie at a joint limit
        soft_joint_pos_limit_factor=1.0,
        actuators={
            "wheels": ImplicitActuatorCfg(joint_names_expr=["(Left|Right)_Wheel_joint", ".*_caster_.*"], stiffness=0.0, damping=1.0),
            "lift": ImplicitActuatorCfg(
                # 600 N per stage: ~3x the load above the column (~21 kg); bounds the force when pressing onto obstacles
                # no solver velocity limit: the URDF 0.04 m/s limit together with the stiff drive makes PhysX diverge
                # while lowering; the column speed is limited by the action provider (0.08 m/s + slew limits)
                joint_names_expr=list(R1A7D_LIFT_JOINTS), joint_effort_limit=600.0, joint_velocity_limit=1.0, stiffness=1.0e5, damping=1.0e3
            ),
            "waist": ImplicitActuatorCfg(joint_names_expr=[R1A7D_WAIST_YAW_JOINT], stiffness=1.0e4, damping=1.0e3),
            "head": ImplicitActuatorCfg(joint_names_expr=[R1A7D_HEAD_PITCH_JOINT, R1A7D_HEAD_YAW_JOINT], stiffness=100.0, damping=5.0),
            "arms": ImplicitActuatorCfg(
                joint_names_expr=[".*_shoulder_.*_joint", ".*_elbow_joint", ".*_wrist_.*_joint"],
                stiffness={".*_shoulder_.*_joint": 300.0, ".*_elbow_joint": 400.0, ".*_wrist_.*_joint": 400.0},
                damping={".*_shoulder_.*_joint": 3.0, ".*_elbow_joint": 2.5, ".*_wrist_.*_joint": 2.5},
            ),
            **hand_actuators,
        },
    )


R1A7D_CFG_WITH_DEX1 = _r1a7d_cfg(
    R1A7D_DEX1_ASSET,
    {".*_dex1_finger_joint_.*": R1A7D_DEX1_FINGER_OPEN},
    # armature: the finger links are very light (8 g, ~1e-7 kg m^2); keeps contacts pressed by the arm / lift
    # (e.g. hands on the table) from blowing up the solver
    {"grippers": ImplicitActuatorCfg(joint_names_expr=[".*_dex1_finger_joint_.*"], stiffness=800.0, damping=10.0, armature=0.05)},
)

# Dex2-5 tendon hands: open (0 rad) by default. The joint drive is the simulated torque transmission of
# action_provider/dex2_5_torque.py (the action provider turns the two motor torques of a hand into joint targets).
R1A7D_CFG_WITH_DEX2_5 = _r1a7d_cfg(
    R1A7D_DEX2_5_ASSET,
    {DEX2_5_JOINT_EXPR: 0.0},
    {"hands": ImplicitActuatorCfg(joint_names_expr=[DEX2_5_JOINT_EXPR], stiffness=DEX2_5_STIFFNESS, damping=DEX2_5_DAMPING, armature=DEX2_5_ARMATURE)},
)
