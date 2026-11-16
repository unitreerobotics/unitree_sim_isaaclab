# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Configuration for the Unitree G1-D robot (wheeled base + lift + G1 upper body).

G1-D is an independent robot: its asset is loaded from URDF (or USD) through
``robots/robot_assets.py``; nothing here is shared with the G1 29dof configs in ``robots/unitree.py``.
"""

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

from robots.robot_assets import robot_spawn_cfg

project_root = os.environ.get("PROJECT_ROOT") or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

##
# Unitree G1-D (wheeled base + lift + G1 upper body) with Dex1 grippers.
# Asset: assets/urdf/g1_d_description/g1_d_with_dex1_1.urdf (a .usd works as well, see robots/robot_assets.py)
#
# The base is a floating base driven by the G1-D action provider: it hovers 3 mm above the floor and its
# root velocity is set from rt/cmd_vel_no_limit every control step (height / tilt are held), so it follows
# the command and is stopped by obstacles. Lift and waist are position-controlled joints.
# G1-D specific joints:
#   Left/Right_Wheel_Joint, Caster_*_Joint : drive wheels and passive casters (free, they do not touch the floor)
#   LZ_mt_Joint, LZ_it_Joint               : two-stage lift (prismatic, 0 .. 0.21 m each)
#   Yaw_Joint                              : waist PITCH (axis y, despite its name), -0.044 .. 2.356 rad
#   torso_Joint                            : waist YAW (axis z)
#   <side>_dex1_finger_joint_1/2           : gripper fingers (prismatic, 0 = closed .. 0.041 m = open;
#                                            at 0 the finger tips meet in the middle, offset in z)
##
G1D_ASSET_DIR = f"{project_root}/assets/urdf/g1_d_description"
G1D_DEX1_ASSET = f"{G1D_ASSET_DIR}/g1_d_with_dex1_1.urdf"

# Upper body finish like the G1 29dof USD (OmniPBR: light grey metallic 1.0 / roughness 0.6, dark parts metallic 1.0):
# the URDF only carries the colors, so the converted torso / arm (rgba 0.82 0.83 0.84) and head (0.09 0.10 0.11)
# materials get this metallic / roughness at start-up (robots.robot_assets.apply_material_finish); base, lift and
# grippers keep the URDF finish.
G1D_UPPER_BODY_FINISH = {"materials": r".*_(0820_0830_0840|0090_0100_0110)_1000", "metal": (1.0, 0.6), "dark": (1.0, 0.5)}
G1D_DEX3_ASSET = f"{G1D_ASSET_DIR}/g1_d_dex3_1.urdf"

G1D_DEX1_FINGER_OPEN = 0.041
G1D_DEX1_FINGER_CLOSED = 0.0

# mobile base / lift / waist (driven from DDS, see action_provider/action_provider_g1d_dds.py)
G1D_LIFT_JOINTS = ("LZ_mt_Joint", "LZ_it_Joint")  # two equal stages, 0 .. 0.21 m each
G1D_LIFT_STAGE_MAX = 0.21
G1D_WAIST_YAW_JOINT = "torso_Joint"  # lowcmd/lowstate motor 12
G1D_WAIST_PITCH_JOINT = "Yaw_Joint"  # lowcmd/lowstate motor 13
G1D_WHEEL_RADIUS = 0.0849
G1D_WHEEL_SEPARATION = 2 * 0.2031
# base / lift response identified on the real robot (delays, accel / decel, lift speed, timeout): robots/sysid/g1d.py

G1D_BASE_JOINT_POS = {
    "(Left|Right)_Wheel_Joint": 0.0,
    "Caster_.*": 0.0,
    "LZ_mt_Joint": 0.0,
    "LZ_it_Joint": 0.0,
    "Yaw_Joint": 0.0,
    "torso_Joint": 0.0,
}
G1D_ARM_JOINT_POS = {
    f"{side}_{j}_joint": 0.0
    for side in ("left", "right")
    for j in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")
}
G1D_DEX1_JOINT_POS = {".*_dex1_finger_joint_.*": G1D_DEX1_FINGER_OPEN}
# Dex3 hands (7 joints each, same names / limits as the Dex3 of the G1): hands open at q = 0
G1D_DEX3_JOINT_POS = {f"{side}_hand_{j}_joint": 0.0 for side in ("left", "right") for j in (
    "thumb_0", "thumb_1", "thumb_2", "middle_0", "middle_1", "index_0", "index_1")}

G1D_BASE_ACTUATORS = {
    # wheels / casters roll freely (the base pose is set kinematically)
    "wheels": ImplicitActuatorCfg(
        joint_names_expr=["(Left|Right)_Wheel_Joint", "Caster_.*"],
        stiffness=0.0,
        damping=1.0,
    ),
    "lift": ImplicitActuatorCfg(
        joint_names_expr=["LZ_mt_Joint", "LZ_it_Joint"],
        joint_effort_limit=5000.0,
        # no solver velocity limit (URDF 0.05 m/s + stiff drive can make PhysX diverge);
        # the lift speed is limited by the action provider (robots/sysid/g1d.py)
        joint_velocity_limit=1.0,
        stiffness=1.0e5,
        damping=1.0e3,
    ),
    "waist": ImplicitActuatorCfg(
        joint_names_expr=["Yaw_Joint", "torso_Joint"],
        joint_effort_limit=2000.0,
        stiffness=1.0e5,
        damping=1.0e3,
    ),
}
G1D_ARM_ACTUATOR = ImplicitActuatorCfg(
    joint_names_expr=[".*_shoulder_.*_joint", ".*_elbow_joint", ".*_wrist_.*_joint"],
    stiffness={".*_shoulder_.*_joint": 300.0, ".*_elbow_joint": 400.0, ".*_wrist_.*_joint": 400.0},
    damping={".*_shoulder_.*_joint": 3.0, ".*_elbow_joint": 2.5, ".*_wrist_.*_joint": 2.5},
)

def _g1d_cfg(asset: str, hand_joint_pos: dict, hand_actuators: dict) -> ArticulationCfg:
    """G1-D articulation for ``asset`` (the URDF variants only differ in the hands)."""
    return ArticulationCfg(
        spawn=robot_spawn_cfg(
            asset,
            fix_base=False,  # floating base, moved kinematically by the action provider
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
            # AGV_link origin: drive wheels (r = 0.0849 m, axle at z = -0.0265) touch the floor
            pos=(0.0, 0.0, 0.1114),
            joint_pos={**G1D_BASE_JOINT_POS, **G1D_ARM_JOINT_POS, **hand_joint_pos},
            joint_vel={".*": 0.0},
        ),
        # 1.0: the defaults of several G1-D joints (lift = 0, waist pitch = 0) lie at a joint limit
        soft_joint_pos_limit_factor=1.0,
        actuators={**G1D_BASE_ACTUATORS, "arms": G1D_ARM_ACTUATOR, **hand_actuators},
    )


G1D_CFG_WITH_DEX1 = _g1d_cfg(
    G1D_DEX1_ASSET,
    G1D_DEX1_JOINT_POS,
    {
        "grippers": ImplicitActuatorCfg(
            joint_names_expr=[".*_dex1_finger_joint_.*"],
            stiffness=800.0,
            damping=10.0,
            # the finger links are very light (8 g, ~1e-7 kg m^2): armature keeps contacts pressed by the
            # arm / lift (e.g. hands on the table) from blowing up the solver
            armature=0.05,
        ),
    },
)

# G1-D with Dex3 hands (asset: assets/urdf/g1_d_description/g1_d_dex3_1.urdf); hand gains as on the G1 Dex3
G1D_CFG_WITH_DEX3 = _g1d_cfg(
    G1D_DEX3_ASSET,
    G1D_DEX3_JOINT_POS,
    {
        "hands": ImplicitActuatorCfg(
            joint_names_expr=[".*_hand_index_.*_joint", ".*_hand_middle_.*_joint", ".*_hand_thumb_.*_joint"],
            effort_limit=300,
            velocity_limit=100.0,
            stiffness={".*": 100.0},
            damping={".*": 10.0},
            armature={".*": 0.1},
        ),
    },
)
