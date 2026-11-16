# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Simulated response of a mobile base axis / delayed state, driven by the parameters in robots/sysid."""

import collections
from typing import Optional

from robots.sysid import AxisResponse


def slew(cur: float, target: float, accel: Optional[float], decel: Optional[float], dt: float) -> float:
    """Move ``cur`` towards ``target``: ``accel`` while speeding up, ``decel`` while slowing down or reversing."""
    speeding_up = abs(target) > abs(cur) and (cur == 0.0 or (target > 0) == (cur > 0))
    limit = accel if speeding_up else decel
    if limit is None:
        return target
    step = limit * dt
    return cur + max(-step, min(step, target - cur))


class AxisResponseSim:
    """command -> dead time -> gain -> accel / decel limit -> first-order lag, one call of ``step`` per control step."""

    def __init__(self, params: AxisResponse, dt: float):
        self.p, self.dt = params, dt
        self._delay_steps = max(0, int(round(params.delay / dt)))
        self._k = 1.0 if params.tau <= 0.0 else min(1.0, dt / params.tau)
        self.reset()

    def reset(self):
        self._queue = collections.deque([0.0] * self._delay_steps)
        self.r = self.y = 0.0  # rate-limited value, lag output

    def step(self, cmd: float) -> float:
        self._queue.append(cmd)
        u = self.p.gain * self._queue.popleft()
        self.r = slew(self.r, u, self.p.accel, self.p.decel, self.dt)
        self.y += (self.r - self.y) * self._k
        return self.y


class DelayLine:
    """Returns the value pushed ``delay`` ago (the oldest available until filled)."""

    def __init__(self, delay: float, dt: float):
        self._queue = collections.deque(maxlen=max(0, int(round(delay / dt))) + 1)

    def reset(self):
        self._queue.clear()

    def __call__(self, value):
        self._queue.append(value)
        return self._queue[0]
