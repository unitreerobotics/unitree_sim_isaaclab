# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Configuration for the Unitree R1-D (wheeled base + two-stage lift + R1 upper body) with Dex1 grippers.

R1-D is an independent robot (asset: assets/urdf/r1_d_description/r1_d_with_dex1_1.urdf, loaded through
``robots/robot_assets.py``; a converted .usd works as well).

Joints:
    Left/Right_Wheel_joint, *_caster_{Yaw,Roll}_joint : drive wheels and passive casters (free; the held base hovers)
    lift_stage1_joint, lift_stage2_joint               : two-stage lift, prismatic 0 .. 0.425 m each (0.04 m/s each)
    waist_yaw_joint                                    : waist yaw          (lowcmd motor 13)
    head_pitch_joint, head_yaw_joint                   : head pitch / yaw   (lowcmd motors 29 / 30)
    <side>_{shoulder_*,elbow,wrist_*}_joint            : 7-dof arms         (lowcmd motors 15..28)
    <side>_dex1_finger_joint_1/2                       : Dex1 fingers, prismatic 0 = closed .. 0.041 m = open

The motor indices follow xr_teleoperate (R1_A7_JointIndex). The base is a floating base held / driven
by the action provider (see action_provider/action_provider_r1d_dds.py), like the G1-D.
"""

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

from robots.robot_assets import robot_spawn_cfg

project_root = os.environ.get("PROJECT_ROOT") or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

R1D_ASSET_DIR = f"{project_root}/assets/urdf/r1_d_description"
R1D_DEX1_ASSET = f"{R1D_ASSET_DIR}/r1_d_with_dex1_1.urdf"

R1D_DEX1_FINGER_OPEN = 0.041
R1D_DEX1_FINGER_CLOSED = 0.0

# lift / waist / head (driven from DDS)
R1D_LIFT_JOINTS = ("lift_stage1_joint", "lift_stage2_joint")
R1D_LIFT_STAGE_MAX = 0.425
R1D_LIFT_MAX_SPEED = 0.08  # [m/s] column speed at |rt/cmd_vel.linear.z| = 1 (kMaxColumnVel_mps; URDF: 0.04 m/s per stage)
R1D_LIFT_SOFT_MAX = 0.84  # [m] column soft limit (850 mm travel, [0, ~840] mm)
R1D_COLUMN_RADIUS = 0.003820  # [m/rad] column motor: height = q * kColumnRadius_m (lowstate motor 12)
# default lift: 0.27 m per stage puts the shoulders at ~1.05 m, the height of the fixed-base G1 shoulders
R1D_LIFT_DEFAULT = 0.27
R1D_WAIST_YAW_JOINT = "waist_yaw_joint"  # lowcmd/lowstate motor 13
R1D_HEAD_PITCH_JOINT = "head_pitch_joint"  # lowcmd/lowstate motor 29
R1D_HEAD_YAW_JOINT = "head_yaw_joint"  # lowcmd/lowstate motor 30
R1D_HEAD_PITCH_DEFAULT = 0.5  # [rad] look ~29 deg down at the table (limit +-0.628)
R1D_WHEEL_RADIUS = 0.0625
R1D_WHEEL_SEPARATION = 2 * 0.1968
R1D_BASE_HEIGHT = 0.155  # base_link above the floor with the wheels / casters on the ground
R1D_CMD_VEL_TIMEOUT = 0.6  # rt/cmd_vel considered lost after 600 ms

# R1-A5/A7D twist module (rt/cmd_vel / rt/state_vel are normalized to [-1, 1]):
R1D_GEAR_VMAX = {"low": 0.5, "mid": 1.0, "high": 1.5}  # kGearLinearVel_mps [m/s]
R1D_GEAR = os.environ.get("UNITREE_R1D_GEAR", "low")  # simulated gear (env UNITREE_R1D_GEAR=low|mid|high)
R1D_MAX_ANGULAR_VEL = 1.2  # kMaxAngularVel_rad [rad/s]
# slew limits (accel, decel) per axis
R1D_SLEW_LINEAR = (0.6, 2.5)  # m/s^2
R1D_SLEW_ANGULAR = (3.5, 4.0)  # rad/s^2
R1D_SLEW_COLUMN = (0.15, 0.25)  # m/s^2

# Appearance (sRGB; the finish - metallic / roughness - is applied at start-up by
# robots.robot_assets.apply_material_finish): silver body, aluminium column, light silver-grey base;
# only the Dex1 grippers are black.
_SILVER = (0.80, 0.81, 0.83, 1.0)
_ALUMINIUM = (0.72, 0.73, 0.75, 1.0)
_BASE = (0.84, 0.85, 0.86, 1.0)
_BLACK = (0.03, 0.03, 0.03, 1.0)
R1D_LINK_COLORS = {
    ".*_dex1_.*": _BLACK,
    "lift_.*": _ALUMINIUM,
    "base_link": _BASE,
    ".*_caster_.*": _BASE,
    "(Left|Right)_Wheel_link": _BASE,
    ".*": _SILVER,  # everything else (head, torso, arms, pelvis)
}

R1D_ARM_JOINT_POS = {
    f"{side}_{j}_joint": 0.0
    for side in ("left", "right")
    for j in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")
}

R1D_CFG_WITH_DEX1 = ArticulationCfg(
    spawn=robot_spawn_cfg(
        R1D_DEX1_ASSET,
        fix_base=False,  # floating base, held / moved by the action provider
        link_colors=R1D_LINK_COLORS,
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
        pos=(0.0, 0.0, R1D_BASE_HEIGHT),
        joint_pos={
            "(Left|Right)_Wheel_joint": 0.0,
            ".*_caster_.*": 0.0,
            "lift_stage1_joint": R1D_LIFT_DEFAULT,
            "lift_stage2_joint": R1D_LIFT_DEFAULT,
            R1D_WAIST_YAW_JOINT: 0.0,
            R1D_HEAD_PITCH_JOINT: R1D_HEAD_PITCH_DEFAULT,
            R1D_HEAD_YAW_JOINT: 0.0,
            **R1D_ARM_JOINT_POS,
            ".*_dex1_finger_joint_.*": R1D_DEX1_FINGER_OPEN,
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
            joint_names_expr=list(R1D_LIFT_JOINTS), joint_effort_limit=600.0, joint_velocity_limit=1.0, stiffness=1.0e5, damping=1.0e3
        ),
        "waist": ImplicitActuatorCfg(joint_names_expr=[R1D_WAIST_YAW_JOINT], stiffness=1.0e4, damping=1.0e3),
        "head": ImplicitActuatorCfg(joint_names_expr=[R1D_HEAD_PITCH_JOINT, R1D_HEAD_YAW_JOINT], stiffness=100.0, damping=5.0),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[".*_shoulder_.*_joint", ".*_elbow_joint", ".*_wrist_.*_joint"],
            stiffness={".*_shoulder_.*_joint": 300.0, ".*_elbow_joint": 400.0, ".*_wrist_.*_joint": 400.0},
            damping={".*_shoulder_.*_joint": 3.0, ".*_elbow_joint": 2.5, ".*_wrist_.*_joint": 2.5},
        ),
        # armature: the finger links are very light (8 g, ~1e-7 kg m^2); keeps contacts pressed by the arm / lift
        # (e.g. hands on the table) from blowing up the solver
        "grippers": ImplicitActuatorCfg(joint_names_expr=[".*_dex1_finger_joint_.*"], stiffness=800.0, damping=10.0, armature=0.05),
    },
)
