# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""DDS action provider for the Unitree R1-A7-D (same structure as the G1-D provider, R1-A7-D joints / motors).

* rt/lowcmd: motors 15..28 -> arms, 13 -> waist yaw, 29 / 30 -> head pitch / yaw (xr_teleoperate R1_A7_JointIndex)
* Dex1 grippers: lowcmd motors 31 / 33 (real "internal" wiring, unirobot r1_a7_d) or rt/dex1/*/cmd (same mapping
  as the G1-D)
* Dex2-5 tendon hands (r1_d_with_dex2_5.urdf): torque-only lowcmd motors 31 / 32 (left fingers, thumb) and
  33 / 34 (right fingers, thumb), like unirobot ``r1_a7_d/dex2_5`` (action_provider/dex2_5_torque.py)
* rt/cmd_vel (``r1a7d_mobile``, normalized): linear.x / angular.z drive the base, linear.z the column, with
  the response in robots/sysid/r1a7d.py (twist-module slew limits, 600 ms command timeout; not identified on the
  real robot yet); the column is limited to its 840 mm soft limit.
"""

from action_provider.action_provider_g1d_dds import G1DDDSActionProvider
from robots.sysid.r1a7d import R1A7D_SYSID
from robots.unitree_r1a7d import (
    R1A7D_LIFT_SOFT_MAX,
    R1A7D_HEAD_PITCH_JOINT,
    R1A7D_HEAD_YAW_JOINT,
    R1A7D_LIFT_JOINTS,
    R1A7D_LIFT_STAGE_MAX,
    R1A7D_WAIST_YAW_JOINT,
)


class R1A7DDDSActionProvider(G1DDDSActionProvider):
    ROBOT = "R1-A7-D"
    LOWCMD_EXTRA = {R1A7D_WAIST_YAW_JOINT: 13, R1A7D_HEAD_PITCH_JOINT: 29, R1A7D_HEAD_YAW_JOINT: 30}
    LOWCMD_GATED = {}
    WAIST_PITCH_DDS = None  # R1-A7-D has no waist pitch
    WAIST_PITCH_JOINT = None
    LIFT_JOINTS = R1A7D_LIFT_JOINTS
    LIFT_STAGE_MAX = R1A7D_LIFT_STAGE_MAX
    MOBILE_DDS = "r1a7d_mobile"  # rt/cmd_vel -> base + column, rt/state_vel / rt/height_state / rt/height_alarm
    LIFT_MAX_TOTAL = R1A7D_LIFT_SOFT_MAX
    SYSID = R1A7D_SYSID
