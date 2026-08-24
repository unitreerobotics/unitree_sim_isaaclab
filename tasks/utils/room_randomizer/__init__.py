"""Room randomizer package for pick and place tasks."""

__all__ = [
    "StaticClusterMember",
    "TabletopSpawnRegion",
    "randomize_pickplace_room_layout",
    "randomize_wall_props_layout",
]


def __getattr__(name: str):
    if name in {"StaticClusterMember", "TabletopSpawnRegion"}:
        from .room_events import StaticClusterMember, TabletopSpawnRegion

        return {
            "StaticClusterMember": StaticClusterMember,
            "TabletopSpawnRegion": TabletopSpawnRegion,
        }[name]
    if name == "randomize_pickplace_room_layout":
        from .room_events import randomize_pickplace_room_layout

        return randomize_pickplace_room_layout
    if name == "randomize_wall_props_layout":
        from .room_events import randomize_wall_props_layout

        return randomize_wall_props_layout
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
