# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Real-robot response parameters of the simulated mobile robots, one module per robot.

The simulated base is velocity-controlled; to make it respond like the real one, the commanded twist goes through
the response of each axis (action_provider/base_response.py):

    command (0 after ``cmd_timeout`` without a message) -> dead time ``delay`` -> ``gain`` -> rate limit
    (``accel`` while speeding up, ``decel`` while slowing down / reversing) -> first-order lag ``tau`` -> base velocity

and the published base state (odometry, lift) lags the simulated motion by ``odom_latency``.

Adding / updating a robot: identify it on the real robot (record its DDS topics while driving it, fit the model above;
the recording / fitting tools are kept outside this repository), write ``robots/sysid/<robot>.py`` with a
``MobileBaseSysID`` (``identified=True``, where / how it was measured in ``source``) and set ``SYSID`` in the robot's
action provider. Robots without an identification use nominal values (``identified=False``).
"""

from dataclasses import dataclass, field
from typing import Optional


@dataclass(frozen=True)
class AxisResponse:
    """Response of one velocity axis (units of the axis: m/s or rad/s)."""

    delay: float = 0.0  # [s] dead time
    gain: float = 1.0
    accel: Optional[float] = None  # [unit/s^2] while speeding up; None: no limit
    decel: Optional[float] = None  # [unit/s^2] while slowing down or reversing; None: no limit
    tau: float = 0.0  # [s] first-order lag; 0: none


@dataclass(frozen=True)
class MobileBaseSysID:
    robot: str
    identified: bool  # False: nominal values, not measured on the real robot
    linear: AxisResponse  # forward velocity
    angular: AxisResponse  # yaw rate
    lift: AxisResponse = field(default_factory=AxisResponse)  # lift / column speed
    lift_max_speed: float = 0.1  # [m/s] lift speed at a full (normalised) command
    cmd_timeout: float = 0.5  # [s] the base / lift stop when no command arrives for this long
    odom_latency: float = 0.0  # [s] published base state lags the motion by this
    state_rate: Optional[float] = None  # [Hz] publish rate of the base state topics; None: DDS default
    source: str = ""  # how / when the values were obtained
