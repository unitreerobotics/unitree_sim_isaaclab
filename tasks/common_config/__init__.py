"""
公共配置模块
提供可复用的机器人和相机配置
"""

from .robot_configs import RobotBaseCfg, H12RobotPresets, RobotJointTemplates,G1RobotPresets
from .g1d_robot_configs import G1DRobotPresets
from .r1d_robot_configs import R1DRobotPresets
from .camera_configs import CameraBaseCfg, CameraPresets
from .env_configs import UnitreeManagerBasedRLEnvCfg

__all__ = [
    "RobotBaseCfg",
    "G1RobotPresets",
    "G1DRobotPresets",
    "R1DRobotPresets",
    "H12RobotPresets",
    "RobotJointTemplates",
    "CameraBaseCfg",
    "CameraPresets",
    "UnitreeManagerBasedRLEnvCfg",
]
