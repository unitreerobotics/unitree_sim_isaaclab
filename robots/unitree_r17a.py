# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Configuration for the Unitree R1-7A arm (7-DOF, fixed base) with its end effectors.

Robots (built from assets/urdf/r1_7a_description by robots/r17a_urdf.py; root link ``mount_link``,
x = forward, y = left, z = up):

* Dex1:    one arm (``left``) with a Dex1 gripper and wrist camera
* Dex2-5:  one arm (``left``) with a Dex2-5 tendon hand (left)
* O6:      one arm (``left``) with a LinkerHand O6 left hand (source URDF R1_7a_linkerhandO6_left.urdf)

Joints: ``<side>_J1..J7`` arm, ``<side>_dex1_finger_joint_1/2`` (0 closed .. 0.041 m open),
``<side>_[1-5][12]_joint`` Dex2-5, ``lh_*`` LinkerHand. The arm base sits on ``mount_link`` rotated by -90 deg
about z, so that J2 swings the arm forward. Mounting (table top or column) is set by the task
(tasks/common_config/r17a_robot_configs.py).

DDS (simulated, no real R1-7A interface exists yet): rt/lowcmd / rt/lowstate (unitree_hg) motors 0..6 = left arm
J1..J7 (7..13 would be a right arm; all robots are one left arm); end effectors as on the R1-A7-D left hand (Dex1
lowcmd 31 or rt/dex1/left, Dex2-5 torque motors 31 / 32) and rt/linkerhand/left/* for the O6, see action_provider/action_provider_r17a_dds.py.
"""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

from robots.dex2_5_hand import DEX2_5_ARMATURE, DEX2_5_DAMPING, DEX2_5_JOINT_EXPR, DEX2_5_STIFFNESS
from robots.linkerhand_o6 import O6_ARMATURE, O6_DAMPING, O6_OPEN_POSE, O6_STIFFNESS, o6_all_joints, o6_joint_expr
from robots.r17a_urdf import R17A_ARMS, R17A_HOME, R17A_TCP, r17a_asset, r17a_arm_joints  # noqa: F401
from robots.robot_assets import robot_spawn_cfg

R17A_ASSET = {hand: r17a_asset(hand) for hand in R17A_ARMS}
R17A_DEX1_FINGER_OPEN = 0.041

R17A_COLUMN_RADIUS = 0.06  # [m] column under an arm base (column mounting)


def _arm_actuators(sides) -> dict:
    """Arm joint drives (J1/J2: 60 Nm, J3-J5: 33 Nm, J6/J7: 10 Nm motors)."""
    expr = [f"({'|'.join(sides)})_J[1-7]"]
    return {
        "arms": ImplicitActuatorCfg(
            joint_names_expr=expr,
            stiffness={".*_J[12]": 300.0, ".*_J[345]": 200.0, ".*_J[67]": 80.0},
            damping={".*_J[12]": 6.0, ".*_J[345]": 4.0, ".*_J[67]": 1.5},
            armature=0.01,
        )
    }


def _r17a_cfg(hand: str, hand_joint_pos: dict, hand_actuators: dict, convert_mimic: bool = False) -> ArticulationCfg:
    sides = tuple(R17A_ARMS[hand])
    arm_pos = {j: q for side in sides for j, q in zip(r17a_arm_joints(side), R17A_HOME[hand][side])}
    return ArticulationCfg(
        spawn=robot_spawn_cfg(
            R17A_ASSET[hand],
            fix_base=True,  # mount_link is fixed to the world (table / column)
            convert_mimic_joints=convert_mimic,
            activate_contact_sensors=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False, retain_accelerations=True, linear_damping=0.0, angular_damping=0.0,
                max_linear_velocity=1000.0, max_angular_velocity=1000.0, max_depenetration_velocity=1.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False, solver_position_iteration_count=8, solver_velocity_iteration_count=4,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0), joint_pos={**arm_pos, **hand_joint_pos}, joint_vel={".*": 0.0}),
        soft_joint_pos_limit_factor=1.0,  # the open hand poses lie at a joint limit
        actuators={**_arm_actuators(sides), **hand_actuators},
    )


R17A_CFG_DEX1 = _r17a_cfg(
    "dex1",
    {"left_dex1_finger_joint_.*": R17A_DEX1_FINGER_OPEN},
    {"grippers": ImplicitActuatorCfg(joint_names_expr=["left_dex1_finger_joint_.*"], stiffness=800.0, damping=10.0, armature=0.05)},
)
R17A_CFG_DEX2_5 = _r17a_cfg(
    "dex2_5",
    {DEX2_5_JOINT_EXPR: 0.0},
    {"hands": ImplicitActuatorCfg(joint_names_expr=[DEX2_5_JOINT_EXPR], stiffness=DEX2_5_STIFFNESS, damping=DEX2_5_DAMPING, armature=DEX2_5_ARMATURE)},
)
R17A_CFG_O6 = _r17a_cfg(
    "o6",
    {j: O6_OPEN_POSE.get(j[len("lh_"):], 0.0) for j in o6_all_joints("left")},  # open hand (raw 255)
    {"hands": ImplicitActuatorCfg(joint_names_expr=[o6_joint_expr("left")], stiffness=O6_STIFFNESS, damping=O6_DAMPING, armature=O6_ARMATURE)},
    convert_mimic=True,  # the distal finger joints are driven by the action provider (robots/linkerhand_o6.py)
)
R17A_CFG = {"dex1": R17A_CFG_DEX1, "dex2_5": R17A_CFG_DEX2_5, "o6": R17A_CFG_O6}

