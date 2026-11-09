# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
URDF转USD格式的工具脚本。

URDF (Unified Robot Description Format) 是ROS中用于描述机器人所有元素的XML文件格式。
更多信息请参考: http://wiki.ros.org/urdf

本脚本使用Isaac Sim的URDF导入器扩展(``isaacsim.asset.importer.urdf``)将URDF资源转换为USD格式。
这是一个命令行使用的便捷脚本。关于URDF导入器的更多信息，请参考扩展文档：
https://docs.isaacsim.omniverse.nvidia.com/latest/robot_setup/ext_isaacsim_asset_importer_urdf.html

参数说明:
  input               输入URDF文件的路径
  output              输出USD文件的存储路径

可选参数:
  -h, --help          显示帮助信息
  --merge-joints      合并由固定关节连接的链接 (默认: False)
  --fix-base          将基座固定在导入位置 (默认: False)
  --joint-stiffness   关节驱动的刚度 (默认: 100.0)
  --joint-damping     关节驱动的阻尼 (默认: 1.0)
  --joint-target-type 关节驱动的控制类型 (默认: "position")
"""

"""首先启动Isaac Sim模拟器"""

import argparse

from isaaclab.app import add_launcher_args, launch_simulation

# 添加命令行参数解析
parser = argparse.ArgumentParser(description="URDF转USD格式的工具")
# parser.add_argument("input", type=str, default="/home/unitree/newDisk/URDF/urdf-to-usd/g1withdex1/g1_29dof_with_dex1_rev_1_0.urdf", help="输入URDF文件的路径")
# parser.add_argument("output", type=str, default="/home/unitree/Code/isaaclab_demo/usd/g1_body29_hand14.usd", help="输出USD文件的路径")
parser.add_argument(
    "--merge-joints",
    action="store_true",
    default=False,
    help="合并由固定关节连接的链接",
)
parser.add_argument("--fix-base", action="store_true", default=True, help="将基座固定在导入位置")
parser.add_argument(
    "--joint-stiffness",
    type=float,
    default=100.0,
    help="关节驱动的刚度",
)
parser.add_argument(
    "--joint-damping",
    type=float,
    default=1.0,
    help="关节驱动的阻尼",
)
parser.add_argument(
    "--joint-target-type",
    type=str,
    default="position",
    choices=["position", "velocity", "none"],
    help="关节驱动的控制类型",
)

# 添加 Isaac Lab 3.0 启动参数
add_launcher_args(parser)
# 解析命令行参数
args_cli = parser.parse_args()

"""以下是主要功能实现"""

import contextlib
import os

import carb
import isaacsim.core.utils.stage as stage_utils
import omni.kit.app

from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg
from isaaclab.physics import PhysicsCfg
from isaaclab.utils.assets import check_file_path
from isaaclab.utils.dict import print_dict


def main():
    # 检查输入文件路径是否有效
    urdf_path = "/home/unitree/newDisk/URDF/urdf-to-usd/h1_2_inspire/h1_2.urdf" #args_cli.input
    print(urdf_path)
    if not os.path.isabs(urdf_path):
        urdf_path = os.path.abspath(urdf_path)
    if not check_file_path(urdf_path):
        raise ValueError(f"无效的文件路径: {urdf_path}")

    # 创建输出文件路径
    dest_path = "/home/unitree/newDisk/URDF/urdf-to-usd/h1_2_inspire/h1_2.urdf.usd"
    if not os.path.isabs(dest_path):
        dest_path = os.path.abspath(dest_path)

    # 创建URDF转换器配置
    urdf_converter_cfg = UrdfConverterCfg(
        asset_path=urdf_path,
        usd_dir=os.path.dirname(dest_path),
        usd_file_name=os.path.basename(dest_path),
        fix_base=args_cli.fix_base,
        merge_fixed_joints=args_cli.merge_joints,
        force_usd_conversion=True,
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                stiffness=args_cli.joint_stiffness,
                damping=args_cli.joint_damping,
            ),
            target_type=args_cli.joint_target_type,
        ),
    )

    # 打印配置信息
    print("-" * 80)
    print("-" * 80)
    print(f"输入URDF文件: {urdf_path}")
    print("URDF导入器配置:")
    print_dict(urdf_converter_cfg.to_dict(), nesting=0)
    print("-" * 80)
    print("-" * 80)

    # Isaac Lab 3.0 uses a context-managed launcher instead of AppLauncher.
    with launch_simulation(cfg=PhysicsCfg(), launcher_args=args_cli):
        # 创建URDF转换器并导入文件
        urdf_converter = UrdfConverter(urdf_converter_cfg)
        # 打印输出信息
        print("URDF导入器输出:")
        print(f"生成的USD文件: {urdf_converter.usd_path}")
        print("-" * 80)
        print("-" * 80)

        # 检查是否有GUI需要更新:
        carb_settings_iface = carb.settings.get_settings()
        local_gui = carb_settings_iface.get("/app/window/enabled")
        livestream_gui = carb_settings_iface.get("/app/livestream/enabled")

        # 如果启用了GUI，则运行模拟场景
        if local_gui or livestream_gui:
            stage_utils.open_stage(urdf_converter.usd_path)
            app = omni.kit.app.get_app_interface()
            with contextlib.suppress(KeyboardInterrupt):
                while app.is_running():
                    app.update()


if __name__ == "__main__":
    main()
