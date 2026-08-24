"""Compatibility launcher for the teleimager version pinned by this repository."""

from __future__ import annotations

import os
from collections.abc import Mapping, MutableMapping
from typing import Any


_TRUE_VALUES = frozenset({"1", "true", "yes", "on"})
_ARM_WORK_PANEL = {
    "preset": "arm_work_panel",
    "horizontal_fov_degrees": 82.0,
    "distance": 2.0,
    "vertical_offset": -0.10,
}
_SIM_TORSO_CONTROL = {
    "enabled": True,
    "deadzone": 0.15,
    # The mirrored crates are behind-left and behind-right of G1 in the
    # calibrated layout.  2.2 rad reaches both while retaining margin from the
    # G1 waist-yaw mechanical limit of 2.618 rad.
    "max_yaw_speed_rad_s": 0.80,
    "max_yaw_angle_rad": 2.20,
    # Only forward lean is useful for lowering a held object into a crate.
    # Stay inside the waist-pitch mechanical range [-0.52, 0.52] rad.
    "max_pitch_speed_rad_s": 0.35,
    "min_pitch_angle_rad": 0.0,
    "max_pitch_angle_rad": 0.45,
}


def configure_camera_transports(
    camera_config: MutableMapping[str, MutableMapping[str, Any]],
    environ: Mapping[str, str] | None = None,
) -> MutableMapping[str, MutableMapping[str, Any]]:
    """Apply simulator transport overrides without requiring a submodule update.

    The teleimager commit currently pinned by the superproject predates its
    ``TELEIMAGER_DISABLE_WEBRTC`` environment override. Quest mode needs that
    override because xr_teleoperate consumes the simulator feeds over ZMQ and
    hosts the browser-facing Vuer server itself.
    """

    environ = os.environ if environ is None else environ
    disable_webrtc = environ.get("TELEIMAGER_DISABLE_WEBRTC", "").strip().lower()
    if disable_webrtc in _TRUE_VALUES:
        for config in camera_config.values():
            config["enable_webrtc"] = False

    head_camera = camera_config.get("head_camera")
    if head_camera is None:
        raise ValueError("Camera configuration is missing head_camera")

    quest_view_preset = environ.get("TELEIMAGER_HEAD_QUEST_VIEW_PRESET", "").strip()
    if quest_view_preset:
        if quest_view_preset != _ARM_WORK_PANEL["preset"]:
            raise ValueError(f"Unsupported Quest view preset: {quest_view_preset!r}")
        head_camera["quest_view"] = dict(_ARM_WORK_PANEL)

    enable_sim_torso_control = (
        environ.get("TELEIMAGER_ENABLE_SIM_TORSO_CONTROL", "").strip().lower()
    )
    if enable_sim_torso_control in _TRUE_VALUES:
        head_camera["sim_torso_control"] = dict(_SIM_TORSO_CONTROL)

    return camera_config


def run_isaacsim_server():
    """Start teleimager after applying superproject transport overrides."""

    import yaml
    from teleimager import image_server

    try:
        with open(image_server.CONFIG_PATH, encoding="utf-8") as config_file:
            camera_config = yaml.safe_load(config_file)
    except Exception as exc:
        image_server.logger_mp.error(
            f"Failed to load configuration file at {image_server.CONFIG_PATH}: {exc}"
        )
        raise

    configure_camera_transports(camera_config)
    server = image_server.ImageServer(
        camera_config,
        realsense_enable=False,
        camera_finder_verbose=False,
        isaacsim_enable=True,
    )
    server.start()
    return server
