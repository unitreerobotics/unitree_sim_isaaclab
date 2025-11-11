# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0      
"""
public base scene configuration module
provides reusable scene element configurations, such as tables, objects, ground, lights, etc.
"""
import isaaclab.sim as sim_utils
from isaaclab.assets import  AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaacsim.core.utils.torch.rotations import euler_angles_to_quats
from tasks.common_config import   CameraBaseCfg  # isort: skip
import os
import torch
project_root = os.environ.get("PROJECT_ROOT")
# usd_root = "/mnt/hdd/Data"
usd_root = "/home/nvidia/workspace/yunl/assets"
@configclass
class SurgicalSceneCfg(InteractiveSceneCfg): # inherit from the interactive scene configuration class
    """object table scene configuration class
    defines a complete scene containing robot, object, table, etc.
    """
      # 1. room wall configuration - simplified configuration to avoid rigid body property conflicts
    scene = AssetBaseCfg(
        prim_path="/World/envs/env_.*/Scene",
        spawn=UsdFileCfg(
            usd_path=f"{usd_root}/lw_v2/scene.usd",  # use simple room model
        ),
    )
    # Trocar (rigid object inside the loaded scene.usd)
    # trocar_1 = RigidObjectCfg(
    #     prim_path="/World/envs/env_.*/Scene/Trocar002",
    #     init_state=RigidObjectCfg.InitialStateCfg(
    #         pos=[-1.61873, 1.9629, 0.82559],
    #         rot=[0.60545, 0.00148, -0.72054, -0.33799]
    #     ),
    # )
    # trocar_2 = RigidObjectCfg(
    #     prim_path="/World/envs/env_.*/Scene/DisposableLaparoscopicPunctureDevice001",
    #     init_state=RigidObjectCfg.InitialStateCfg(
    #         pos=[-1.52635, 2.09436, 0.85483],
    #         rot=[0.63046, -0.59294, -0.33848, 0.36928]
    #     ),
    # )

    # Cart (example)
    # cart = RigidObjectCfg(
    #     prim_path="/World/envs/env_.*/Cart001"
    # )
    # Plate (example)
    # plate = RigidObjectCfg(
    #     prim_path="/World/envs/env_.*/Scene/plate001",
    #     init_state=RigidObjectCfg.InitialStateCfg(
    #         pos=[-1.46817, 2.07344, 0.80932],
    #         rot=[1.0, 0.0, 0.0, 0.0]
    #     ),
    # )

    
    # 
    # Lights
    # 4. light configuration
    light = AssetBaseCfg(
        prim_path="/World/light",   # light in the scene
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), # light color (white)
                                     intensity=1000.0),    # light intensity
    )
    # world_camera = CameraBaseCfg.get_camera_config(prim_path="/World/PerspectiveCamera",
    #                                                 pos_offset=(-1.85014, 1.9196, 1.20101),
    #                                                 rot_offset=euler_angles_to_quats(torch.tensor([46.0, 180.0, 92.0]), degrees=True),
    #                                                 focal_length = 13.0) 
