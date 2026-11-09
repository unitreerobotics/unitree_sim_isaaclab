# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Table height adaptation of the shared table scenes for the (taller) H1-2.

The shared scenes are laid out for the fixed-base G1: pelvis at 0.76 m (its standing height) and the work table
top at 0.795 m. The H1-2 needs its pelvis at ~1.03 m to stand on the floor (soles 1.028 m below the pelvis), so for
the H1-2 tasks the work table, the task objects and the height thresholds of the termination / reward terms are
raised by ``H12_TABLE_LIFT``; the hands are then at the same height above the table as for the G1.

Usage, in the ``__post_init__`` of an H1-2 environment config::

    lift_table_scene_for_h12(self, objects=["object"])
"""

from __future__ import annotations

import inspect

G1_PELVIS_HEIGHT = 0.76  # [m] pelvis height of the fixed-base robots in the shared scenes (table top 0.795 m)
H12_PELVIS_HEIGHT = 1.03  # [m] H1-2 standing on the floor (straight legs, soles 1.028 m below the pelvis)
H12_TABLE_LIFT = H12_PELVIS_HEIGHT - G1_PELVIS_HEIGHT  # 0.27 m

# Both table models (PackingTable.usd, table_with_yellowbox.usd) have their top at z = 0.995 m in the model
# frame and are placed at z = -0.2 m in the scenes (legs partly below the floor).
TABLE_TOP_LOCAL = 0.995

# termination / reward parameters that are absolute heights [m]
_HEIGHT_PARAMS = ("min_height", "post_min_height", "post_max_height")


def lift_table_cfg(table_cfg, lift: float) -> None:
    """Raise the top of a table (``AssetBaseCfg`` with a USD spawn) by ``lift`` without lifting its legs off the
    floor: the table is moved up so that its feet stand on the floor and stretched along z."""
    x, y, z = table_cfg.init_state.pos
    top = z + TABLE_TOP_LOCAL + lift
    base_z = max(z, 0.0)
    sz = (top - base_z) / TABLE_TOP_LOCAL
    table_cfg.init_state.pos = (x, y, base_z)
    sx, sy, _ = table_cfg.spawn.scale or (1.0, 1.0, 1.0)
    table_cfg.spawn.scale = (sx, sy, sz)


def _lift_term_heights(term_cfg, lift: float) -> None:
    """Add ``lift`` to the absolute height parameters of a termination / reward term (explicit params or the
    defaults of its function)."""
    if term_cfg is None:
        return
    defaults = {k: p.default for k, p in inspect.signature(term_cfg.func).parameters.items()}
    params = dict(term_cfg.params or {})
    for name in _HEIGHT_PARAMS:
        if name in params:
            params[name] = params[name] + lift
        elif isinstance(defaults.get(name), (int, float)):
            params[name] = defaults[name] + lift
    term_cfg.params = params


def lift_table_scene_for_h12(env_cfg, objects: list[str], table: str = "packing_table", lift: float = H12_TABLE_LIFT):
    """Raise the work table, the given task objects and the height thresholds of all termination and reward
    terms of ``env_cfg`` by ``lift``."""
    lift_table_cfg(getattr(env_cfg.scene, table), lift)
    for name in objects:
        obj = getattr(env_cfg.scene, name)
        x, y, z = obj.init_state.pos
        obj.init_state.pos = (x, y, z + lift)
    for group in (env_cfg.terminations, env_cfg.rewards):
        for name, term in vars(group).items():
            if hasattr(term, "func") and name != "time_out":
                _lift_term_heights(term, lift)
