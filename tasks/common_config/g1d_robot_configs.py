# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Robot presets for the Unitree G1-D (independent of the G1 29dof presets)."""

from typing import Dict, Optional, Tuple

from isaaclab.assets import ArticulationCfg
from isaaclab.utils import configclass

from robots.unitree_g1d import G1D_CFG_WITH_DEX1


@configclass
class G1DRobotPresets:
    """G1-D (wheeled base + lift + G1 upper body) preset configurations.

    The default pose faces +y with the torso ~2 cm behind the fixed-base G1 torso of the existing scenes
    (the base front then has ~2 cm clearance to the table) and ~4 cm higher at lift = 0. The base,
    lift and waist are driven from DDS by action_provider/action_provider_g1d_dds.py.
    """

    @classmethod
    def g1_d_dex1(
        cls,
        init_pos: Tuple[float, float, float] = (-0.15, -0.08, 0.1114),
        init_rot: Tuple[float, float, float, float] = (0, 0, 0.7071, 0.7071),
        custom_joint_pos: Optional[Dict[str, float]] = None,
        prim_path: str = "/World/envs/env_.*/Robot",
    ) -> ArticulationCfg:
        """G1-D with Dex1 grippers.

        Args:
            init_pos: position of AGV_link (z = 0.1114 puts the drive wheels on the floor)
            init_rot: orientation quaternion (x, y, z, w)
            custom_joint_pos: overrides, e.g. {"LZ_mt_Joint": 0.1} to raise the lift
        """
        base = G1D_CFG_WITH_DEX1
        joint_pos = {**base.init_state.joint_pos, **(custom_joint_pos or {})}
        return base.replace(
            prim_path=prim_path,
            init_state=ArticulationCfg.InitialStateCfg(pos=init_pos, rot=init_rot, joint_pos=joint_pos, joint_vel={".*": 0.0}),
        )

