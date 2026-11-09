# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""DDS action provider for the Unitree R1-D (same structure as the G1-D provider, R1-D joints / motors).

* rt/lowcmd: motors 15..28 -> arms, 13 -> waist yaw, 29 / 30 -> head pitch / yaw (xr_teleoperate R1_A7_JointIndex)
* rt/dex1/*/cmd: Dex1 grippers (same mapping as the G1-D)
* rt/cmd_vel (``r1d_mobile``, normalized): linear.x / angular.z drive the base, linear.z the column, with
  the twist-module slew limits and the 600 ms command timeout; the column is limited to its 840 mm soft limit.
"""

from action_provider.action_provider_g1d_dds import G1DDDSActionProvider
from robots.unitree_r1d import (
    R1D_CMD_VEL_TIMEOUT,
    R1D_LIFT_SOFT_MAX,
    R1D_SLEW_ANGULAR,
    R1D_SLEW_COLUMN,
    R1D_SLEW_LINEAR,
    R1D_HEAD_PITCH_JOINT,
    R1D_HEAD_YAW_JOINT,
    R1D_LIFT_JOINTS,
    R1D_LIFT_MAX_SPEED,
    R1D_LIFT_STAGE_MAX,
    R1D_WAIST_YAW_JOINT,
)


class R1DDDSActionProvider(G1DDDSActionProvider):
    ROBOT = "R1-D"
    LOWCMD_EXTRA = {R1D_WAIST_YAW_JOINT: 13, R1D_HEAD_PITCH_JOINT: 29, R1D_HEAD_YAW_JOINT: 30}
    LIFT_JOINTS = R1D_LIFT_JOINTS
    LIFT_STAGE_MAX = R1D_LIFT_STAGE_MAX
    LIFT_MAX_SPEED = R1D_LIFT_MAX_SPEED
    CMD_TIMEOUT = R1D_CMD_VEL_TIMEOUT
    MOBILE_DDS = "r1d_mobile"  # rt/cmd_vel -> base + column, rt/state_vel / rt/height_state / rt/height_alarm
    LIFT_MAX_TOTAL = R1D_LIFT_SOFT_MAX
    SLEW_LINEAR = R1D_SLEW_LINEAR
    SLEW_ANGULAR = R1D_SLEW_ANGULAR
    SLEW_COLUMN = R1D_SLEW_COLUMN
