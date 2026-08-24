"""Room randomizer package.

Keep package import lightweight so pure-Python utilities and tests do not
pull in the full Isaac Lab / Isaac Sim stack during collection.
"""

__all__ = ["RoomEnvCfg", "RoomSceneCfg"]


def __getattr__(name: str):
    if name == "RoomEnvCfg":
        from .room_env_cfg import RoomEnvCfg

        return RoomEnvCfg
    if name == "RoomSceneCfg":
        from .room_scene_cfg import RoomSceneCfg

        return RoomSceneCfg
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
