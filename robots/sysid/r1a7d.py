# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""R1-A7-D base / column response: NOT identified yet, nominal values of the real twist module (unirobot r1_a7_d:
slew limits, kMaxColumnVel_mps, 600 ms command timeout, 50 Hz state). Replace with measured values (robots/sysid)."""

from robots.sysid import AxisResponse, MobileBaseSysID

R1A7D_SYSID = MobileBaseSysID(
    robot="R1-A7-D",
    identified=False,
    linear=AxisResponse(accel=0.6, decel=2.5),  # [m/s^2]
    angular=AxisResponse(accel=3.5, decel=4.0),  # [rad/s^2]
    lift=AxisResponse(accel=0.15, decel=0.25),  # [m/s^2]
    lift_max_speed=0.08,  # [m/s] at |rt/cmd_vel.linear.z| = 1 (kMaxColumnVel_mps; URDF: 0.04 m/s per stage)
    cmd_timeout=0.6,
    odom_latency=0.0,
    state_rate=50.0,  # rt/state_vel, rt/height_state, rt/height_alarm
    source="nominal (real controller constants), not identified",
)
