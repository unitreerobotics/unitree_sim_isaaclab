# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""LinkerHand O6 (6 actuated joints, left hand ``lh_*`` / right hand ``rh_*``): joints, couplings, command mapping.

Real hand (LinkerHand SDK, ``linkerbot-hand-cpp-sdk`` model capabilities): 6 joints in the order
``thumb_flex, thumb_abd, index, middle, ring, pinky``, Classic CAN (left 0x28, right 0x27, 1 Mbps), angle
commands raw ``0..255`` per joint, 255 = fully open (the polarity of the LinkerHand O6 driver): raw 255 -> joint at
its open limit, 0 -> closed limit.

URDF (R1_7a_linkerhandO6_left.urdf): the actuated joints below plus one distal joint per finger coupled by ``<mimic>``
(``dip = multiplier * proximal``). The importer turns the mimic joints into normal joints
(robot_spawn_cfg(convert_mimic_joints=True)); the controller writes their targets from the coupling.

Simulated DDS (no Unitree topic exists for this hand): rt/linkerhand/{left,right}/cmd (unitree_go MotorCmds_,
6 motors, ``q`` = raw angle 0..255 in the SDK joint order) and .../state (MotorStates_, same order, ``q`` raw,
``dq`` raw per second, ``tau_est`` joint torque [Nm]).

This module has no simulator imports (it is also used by the keyboard tools).
"""

O6_RAW_MAX = 255.0
O6_RAW_OPEN = 255.0  # polarity (see above)
O6_NUM_MOTORS = 6

# SDK joint order -> URDF joint (without the lh_ / rh_ prefix)
O6_MOTOR_JOINTS = ("thumb_cmc_pitch", "thumb_cmc_roll", "index_mcp_pitch", "middle_mcp_pitch", "ring_mcp_pitch", "pinky_mcp_pitch")
# coupled distal joint -> (driving joint, multiplier), from the URDF <mimic> tags
O6_MIMIC = {
    "thumb_dip": ("thumb_cmc_pitch", 1.226495),
    "index_dip": ("index_mcp_pitch", 1.125676),
    "middle_dip": ("middle_mcp_pitch", 1.125676),
    "ring_dip": ("ring_mcp_pitch", 1.125676),
    "pinky_dip": ("pinky_mcp_pitch", 1.125676),
}
# the open end of each actuated joint is its limit closer to 0 (all joints flex towards the other limit)
O6_PREFIX = {"left": "lh_", "right": "rh_"}
# open hand (raw 255 on every joint): the joint values at the open limits (URDF); all others 0
O6_OPEN_POSE = {"thumb_cmc_roll": -0.087266}


def o6_joint(side: str, name: str) -> str:
    return f"{O6_PREFIX[side]}{name}"


def o6_motor_joints(side: str) -> list[str]:
    return [o6_joint(side, j) for j in O6_MOTOR_JOINTS]


def o6_all_joints(side: str) -> list[str]:
    return o6_motor_joints(side) + [o6_joint(side, j) for j in O6_MIMIC]


def o6_joint_expr(side: str) -> str:
    return f"{O6_PREFIX[side]}(thumb_cmc_(pitch|roll)|thumb_dip|(index|middle|ring|pinky)_(mcp_pitch|dip))"


def raw_to_fraction_closed(raw: float) -> float:
    """raw command 0..255 -> 0 (open) .. 1 (closed), clamped."""
    raw = min(max(float(raw), 0.0), O6_RAW_MAX)
    return (O6_RAW_OPEN - raw) / O6_RAW_MAX if O6_RAW_OPEN > 0 else raw / O6_RAW_MAX


def fraction_closed_to_raw(c: float) -> float:
    c = min(max(float(c), 0.0), 1.0)
    return O6_RAW_OPEN - c * O6_RAW_MAX if O6_RAW_OPEN > 0 else c * O6_RAW_MAX


# joint drive (assumed): URDF effort limits 0.57 .. 1.0 Nm, finger links of a few grams
O6_STIFFNESS = 3.0  # [Nm/rad]
O6_DAMPING = 0.1  # [Nm s/rad]
O6_ARMATURE = 0.001  # [kg m^2]
