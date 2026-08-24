"""Meta Quest launch profiles for camera-enabled manipulation tasks."""

from __future__ import annotations

from dataclasses import dataclass
from typing import MutableMapping


CAMERA_SENSOR_NAMES = "front_camera,left_wrist_camera,right_wrist_camera"


@dataclass(frozen=True)
class MetaQuestTaskProfile:
    """Robot and hand DDS settings required by one teleoperated task."""

    robot_type: str
    hand_flag: str
    quest_view_preset: str | None = None
    sim_torso_control: bool = False


# Keep this list explicit.  A task is only added after verifying that its scene
# contains the head and both wrist cameras consumed by xr_teleoperate.
META_QUEST_REDBLOCK_PROFILES: dict[str, MetaQuestTaskProfile] = {
    "Isaac-PickPlace-RedBlock-G129-Dex1-Joint": MetaQuestTaskProfile("g129", "enable_dex1_dds"),
    "Isaac-PickPlace-MedicineBottle-Hospital-G129-Dex1-Joint": MetaQuestTaskProfile(
        "g129",
        "enable_dex1_dds",
        quest_view_preset="arm_work_panel",
        sim_torso_control=True,
    ),
    "Isaac-PickPlace-RedBlock-G129-Dex3-Joint": MetaQuestTaskProfile("g129", "enable_dex3_dds"),
    "Isaac-PickPlace-RedBlock-G129-Inspire-Joint": MetaQuestTaskProfile(
        "g129", "enable_inspire_dds"
    ),
    "Isaac-PickPlace-RedBlock-H12-27dof-Inspire-Joint": MetaQuestTaskProfile(
        "h1_2", "enable_inspire_dds"
    ),
    "Isaac-Pick-Redblock-Into-Drawer-G129-Dex1-Joint": MetaQuestTaskProfile(
        "g129", "enable_dex1_dds"
    ),
    "Isaac-Pick-Redblock-Into-Drawer-G129-Dex3-Joint": MetaQuestTaskProfile(
        "g129", "enable_dex3_dds"
    ),
}


class MetaQuestConfigurationError(ValueError):
    """Raised when CLI options cannot produce a working Quest session."""


def configure_meta_quest(args, environ: MutableMapping[str, str]) -> MetaQuestTaskProfile | None:
    """Apply and validate the simulator side of an xr_teleoperate Quest launch.

    Quest video travels from Isaac Sim to xr_teleoperate over teleimager's ZMQ
    endpoints.  RTX sensors must therefore continue rendering even when the
    simulator has no GUI.  Direct teleimager WebRTC is disabled because the
    Quest browser connects to xr_teleoperate, not to these camera endpoints.
    """

    if not getattr(args, "meta_quest", False):
        return None

    profile = META_QUEST_REDBLOCK_PROFILES.get(args.task)
    if profile is None:
        supported = ", ".join(sorted(META_QUEST_REDBLOCK_PROFILES))
        raise MetaQuestConfigurationError(
            f"--meta_quest does not have a verified profile for task {args.task!r}. "
            f"Supported red-block tasks: {supported}"
        )
    if getattr(args, "no_render", False):
        raise MetaQuestConfigurationError(
            "--meta_quest cannot be combined with --no_render because Quest camera frames "
            "require RTX render updates. Use --headless for offscreen execution."
        )
    if getattr(args, "replay_data", False):
        raise MetaQuestConfigurationError(
            "--meta_quest is a live DDS mode and cannot be used with --replay_data"
        )

    hand_flags = ("enable_dex1_dds", "enable_dex3_dds", "enable_inspire_dds")
    conflicting = [
        flag
        for flag in hand_flags
        if flag != profile.hand_flag and bool(getattr(args, flag, False))
    ]
    if conflicting:
        raise MetaQuestConfigurationError(
            f"Task {args.task} requires --{profile.hand_flag}; conflicting DDS option(s): "
            + ", ".join(f"--{flag}" for flag in conflicting)
        )

    for flag in hand_flags:
        setattr(args, flag, flag == profile.hand_flag)
    args.robot_type = profile.robot_type
    args.action_source = "dds"
    args.enable_cameras = True
    args.camera_include = CAMERA_SENSOR_NAMES
    if getattr(args, "camera_write_interval", None) is None:
        args.camera_write_interval = 1

    # xr_teleoperate receives these feeds through ZMQ (55555-55557).  Avoid
    # launching the optional certificate-backed WebRTC servers as well.
    environ["TELEIMAGER_DISABLE_WEBRTC"] = "1"
    if profile.quest_view_preset is not None:
        environ["TELEIMAGER_HEAD_QUEST_VIEW_PRESET"] = profile.quest_view_preset
    else:
        environ.pop("TELEIMAGER_HEAD_QUEST_VIEW_PRESET", None)
    if profile.sim_torso_control:
        environ["TELEIMAGER_ENABLE_SIM_TORSO_CONTROL"] = "1"
    else:
        environ.pop("TELEIMAGER_ENABLE_SIM_TORSO_CONTROL", None)
    return profile
