# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""G1-D base / lift response, identified on the real robot.

Data: 2026-11-13, remote-controlled drives (all four speed gears, 0 .. 1.0 m/s, turns, stops, reversing) on two floors,
lift up / down. Inputs rt/cmd_vel_no_limit / rt/cmd_hispeed; true velocity from the body IMU (yaw rate) and the
odometry fused with the IMU forward acceleration (forward velocity); odometry rt/slamware_ros_sdk_server_node/odom.

Fit quality (held-out recordings, vs true velocity): forward median |e| 0.009 m/s, yaw rate median |e| 0.002 rad/s.
Floor dependent: forward decel 1.05 (floor B) .. 1.33 (floor A) m/s^2 - 1.2 used. Not modelled: rarely (~1 in 20
accelerations, when the command dropped to 0 for a moment) the real base accelerated at 1 .. 1.6 m/s^2.
Forward gain: 1.05 .. 1.07 relative to the slamware odometry scale (no external reference) - 1.0 used.
"""

from robots.sysid import AxisResponse, MobileBaseSysID

G1D_SYSID = MobileBaseSysID(
    robot="G1-D",
    identified=True,
    linear=AxisResponse(delay=0.14, gain=1.0, accel=0.35, decel=1.2, tau=0.02),
    angular=AxisResponse(delay=0.10, gain=0.97, accel=1.8, decel=4.0, tau=0.02),
    lift=AxisResponse(),  # constant speed, starts / stops within the 20 Hz state rate
    lift_max_speed=0.077,  # [m/s] at |rt/cmd_hispeed.z| = 1 (0.076 up / 0.077 down), travel 0 .. 0.4265 m
    cmd_timeout=0.3,
    odom_latency=0.105,  # odometry behind the IMU
    state_rate=20.0,  # rt/slamware_ros_sdk_server_node/odom, rt/hispeed_state
    source="real G1-D, 2026-11-13, two floors",
)
