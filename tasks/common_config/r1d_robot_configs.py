# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Robot presets for the Unitree R1-D (independent of the G1 / G1-D presets)."""

from typing import Dict, Optional, Tuple

from isaaclab.assets import ArticulationCfg
from isaaclab.utils import configclass

from robots.unitree_r1d import R1D_BASE_HEIGHT, R1D_CFG_WITH_DEX1


@configclass
class R1DRobotPresets:
    """R1-D (wheeled base + lift + R1 upper body) presets.

    The default pose faces +y, ~14 cm clearance between the base front and the table (10 cm further back than
    the fixed-base G1), shoulders at the G1 shoulder height (lift 0.27 m per stage). Move it with ``init_pos``
    (the robot faces +y: a smaller y is further away from the table).
    """

    @classmethod
    def r1_d_dex1(
        cls,
        init_pos: Tuple[float, float, float] = (-0.15, -0.16, R1D_BASE_HEIGHT),
        init_rot: Tuple[float, float, float, float] = (0, 0, 0.7071, 0.7071),
        custom_joint_pos: Optional[Dict[str, float]] = None,
        prim_path: str = "/World/envs/env_.*/Robot",
    ) -> ArticulationCfg:
        """R1-D with Dex1 grippers. ``custom_joint_pos`` overrides defaults, e.g. {"lift_stage1_joint": 0.1}."""
        base = R1D_CFG_WITH_DEX1
        joint_pos = {**base.init_state.joint_pos, **(custom_joint_pos or {})}
        return base.replace(
            prim_path=prim_path,
            init_state=ArticulationCfg.InitialStateCfg(pos=init_pos, rot=init_rot, joint_pos=joint_pos, joint_vel={".*": 0.0}),
        )
