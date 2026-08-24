"""Lightweight camera optics shared by Isaac Sim and XR launch code."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class PinholeCameraOptics:
    focal_length: float
    focus_distance: float
    horizontal_aperture: float
    width: int
    height: int

    @property
    def horizontal_fov_degrees(self) -> float:
        return math.degrees(
            2.0 * math.atan(self.horizontal_aperture / (2.0 * self.focal_length))
        )

    @property
    def vertical_fov_degrees(self) -> float:
        vertical_aperture = self.horizontal_aperture * self.height / self.width
        return math.degrees(
            2.0 * math.atan(vertical_aperture / (2.0 * self.focal_length))
        )


HOSPITAL_FRONT_CAMERA_OPTICS = PinholeCameraOptics(
    focal_length=4.5,
    focus_distance=400.0,
    horizontal_aperture=10.0,
    width=640,
    height=480,
)
