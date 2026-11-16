# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Unitree Dex2-5 tendon hand: joints, motor layout and the simulated transmission.

Real hand (unirobot ``end_effector.unitree.dex2_5``, "internal" wiring): two motors per hand, ``[four fingers, thumb]``,
on the body rt/lowcmd / rt/lowstate slots 31 / 32 (left) and 33 / 34 (right). The motors are torque controlled only
(``mode = 1``, ``q = dq = kp = kd = 0``, ``tau`` in +-0.3 Nm): a positive torque closes, a negative one opens, zero
torque does not open the hand (simulated: it keeps holding what it has grasped, see ``DEX2_5_HOLD_GRIP``). The motor ``q`` / ``dq`` feedback is a diagnostic value, not a finger angle.

URDF model: 10 revolute joints per hand, ``<side>_<finger><segment>_joint`` with finger 1 = thumb, 2..5 = the four
fingers and segment 1 = proximal, 2 = distal; 0 rad = open, the closed pose is the non-zero joint limit.

The tendon transmission (motor -> joints) is not published, so the simulation uses a simple model (see
``action_provider/dex2_5_torque.py``): every joint driven by a motor gets ``DEX2_5_JOINT_TORQUE_GAIN`` x motor torque
in its closing direction. The gains and the joint drive below are ASSUMED values, tune them against the real hand.

This module has no simulator imports (it is also used by the keyboard tools).
"""

DEX2_5_SIDES = ("left", "right")
DEX2_5_MOTORS = ("fingers", "thumb")  # motor order per hand
DEX2_5_LOWCMD_SLOTS = {"left": (31, 32), "right": (33, 34)}  # (fingers, thumb) in rt/lowcmd / rt/lowstate
DEX2_5_MAX_TORQUE = 0.3  # [Nm] motor torque limit (unirobot qtau_limits)

_MOTOR_JOINT_IDS = {"fingers": ("21", "22", "31", "32", "41", "42", "51", "52"), "thumb": ("11", "12")}


def dex2_5_motor_joints(side: str, motor: str) -> list[str]:
    """URDF joints driven by one motor (``motor`` = "fingers" / "thumb")."""
    return [f"{side}_{j}_joint" for j in _MOTOR_JOINT_IDS[motor]]


# all hand joints: left 10 (thumb 11 12, fingers 21 22 .. 51 52), then right 10
DEX2_5_JOINTS = [j for side in DEX2_5_SIDES for motor in ("thumb", "fingers") for j in dex2_5_motor_joints(side, motor)]
DEX2_5_JOINT_EXPR = "(left|right)_[1-5][12]_joint"

# ---- simulated transmission / joint drive (assumed, not from the real hand)
# joint torque [Nm] per motor torque [Nm]: 0.3 Nm at the motor gives 0.3 Nm at every finger joint and 0.6 Nm at
# the thumb joints (URDF effort limits: 0.5 Nm fingers, 2.5 Nm thumb)
DEX2_5_JOINT_TORQUE_GAIN = {"fingers": 1.0, "thumb": 2.0}
DEX2_5_STIFFNESS = 5.0  # [Nm/rad] joint drive; holds the pose at zero motor torque
DEX2_5_DAMPING = 0.1  # [Nm s/rad] sets the free closing speed (~2.5 rad/s at 0.3 Nm)
DEX2_5_ARMATURE = 0.001  # [kg m^2] the finger links are very light (7.5 g): keeps contacts stable
# Zero motor torque: True = the transmission is treated as not back-drivable - fingers that are stopped by an object
# (or their limit) when the torque is released keep pressing with the last torque, so a grasped object stays in the
# hand until an opening torque is commanded; fingers that are still moving stop where they are.
# False = only the finger pose is kept, the grip force is gone (a held object slips).
DEX2_5_HOLD_GRIP = True
DEX2_5_STALL_SPEED = 0.1  # [rad/s] a joint slower than this counts as stopped (free speed at 0.3 Nm: ~3 rad/s)
