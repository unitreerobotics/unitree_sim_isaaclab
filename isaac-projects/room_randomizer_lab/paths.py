"""Location-independent paths for the room randomizer project."""

from __future__ import annotations

import os
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent
PROJECTS_ROOT = PACKAGE_ROOT.parent


def _path_from_env(env_var: str, default_path: Path) -> Path:
    """Return an override path from the environment or a repo-relative default."""
    override = os.environ.get(env_var)
    if override:
        return Path(override).expanduser()
    return default_path


ROOM_SHELL_USD = _path_from_env("ROOM_RANDOMIZER_ROOM_SHELL_USD", PROJECTS_ROOT / "new_base_room.usda")

