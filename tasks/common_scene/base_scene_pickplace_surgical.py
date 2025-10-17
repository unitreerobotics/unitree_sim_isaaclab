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
from tasks.common_config import   CameraBaseCfg  # isort: skip
import os
project_root = os.environ.get("PROJECT_ROOT")
@configclass
class SurgicalSceneCfg(InteractiveSceneCfg): # inherit from the interactive scene configuration class
    """object table scene configuration class
    defines a complete scene containing robot, object, table, etc.
    """
      # 1. room wall configuration - simplified configuration to avoid rigid body property conflicts
    # room_walls = AssetBaseCfg(
    #     prim_path="/World/envs/env_.*/Room",
    #     init_state=AssetBaseCfg.InitialStateCfg(
    #         pos=[0.0, 0.0, 0],  # 房间中心点
    #         rot=[1.0, 0.0, 0.0, 0.0]
    #     ),
    #     spawn=UsdFileCfg(
    #         usd_path=f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/warehouse.usd",  # use simple room model
    #     ),
    # )
    
    # cart
    cart = AssetBaseCfg(
        prim_path="/World/envs/env_.*/Cart",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[0.38297, 0.36, 0.0],
            rot=[1.0, 0.0, 0.0, 0.0]
        ),
        spawn=UsdFileCfg(
            usd_path=f"/home/nvidia/workspace/yunl/assets/MedicalAssets20251013/Cart001/Cart001.usd",  # use simple room model
        ),
    )
    
    # mayo stand
    mayo = AssetBaseCfg(
        prim_path="/World/envs/env_.*/MayoStand",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[0.36603, -0.17142, 0.0],
            rot=[1.0, 0.0, 0.0, 0.0]
        ),
        spawn=UsdFileCfg(
            usd_path=f"/home/nvidia/workspace/yunl/assets/InstrumentTrolley001/InstrumentTrolley001.usd",
        ),
    )
    
    # plate
    plate = AssetBaseCfg(
        prim_path="/World/envs/env_.*/Plate",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[0.37286, 0.20589, 0.81333],
            # pos=[0.28, 0.17, 1.24],
            rot=[1.0, 0.0, 0.0, 0.0]
        ),
        spawn=UsdFileCfg(
            usd_path="/home/nvidia/workspace/yunl/assets/MedicalAssets20251013/Plate001/plate001.usd",  # use simple room model
        ),
    )
    
    # tube 
    tube = AssetBaseCfg(
        prim_path="/World/envs/env_.*/DrainageTube002",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[0.37669, 0.20845, 0.80146],
            rot=[0.70711, 0.0, 0.0, 0.70711]
        ),
        spawn=UsdFileCfg(
            # usd_path="/home/nvidia/workspace/yunl/assets/MedicalAssets2025093001/DrainageTube002/DrainageTube002.usd",
            usd_path="/home/nvidia/workspace/yunl/assets/DrainageTube002/DrainageTube003.usd",
            collision_props=sim_utils.CollisionPropertiesCfg(
                contact_offset=0.01,   # 提前生成接触
                rest_offset=0.001      # 静止间隙，小于 contact_offset
            ),
        ),
    )

    # trocar 
    trocar = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PunctureDevice001",
        init_state=AssetBaseCfg.InitialStateCfg(
            # pos=[0.235, 0.108, 0.74739],
            # pos=[0.235, 0.12664, 0.76121],
            # rot=[0.52133, 0.47772, 0.47772, 0.53134]
            # pos=[0.235, 0.14499, 0.760],
            # rot=[0.75849, 0.65037, 0.00244, 0.04115]
            pos=[0.26335, 0.19818, 0.81428],
            rot=[0.72537, 0.68835, 0.0, 0.0]
        ),
        spawn=UsdFileCfg(
            usd_path="/home/nvidia/workspace/yunl/assets/MedicalAssets2025093001/PunctureDevice001/PunctureDevice001.usd", 
        ),
    )
    trocar_object = RigidObjectCfg(
        prim_path="/World/envs/env_.*/PunctureDevice001/PunctureDevice001_Pipe",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=[0.311, 0.151, 0.765],
            rot=[0.0, 0.0, 0.68835, 0.72537]
        ),
    )
    # trocar_pipe = RigidObjectCfg(
    #     prim_path="/World/envs/env_.*/PunctureDevice001/PunctureDevice001_Pipe",
    #     init_state=RigidObjectCfg.InitialStateCfg(
    #         # pos=[0.235, 0.108, 0.74739],
    #         # pos=[0.235, 0.12664, 0.76121],
    #         # rot=[0.52133, 0.47772, 0.47772, 0.53134]
    #         # pos=[0.235, 0.14499, 0.760],
    #         # rot=[0.75849, 0.65037, 0.00244, 0.04115]
    #         pos=[0.311, 0.151, 0.765],
    #         rot=[0.0, 0.0, 0.68835, 0.72537]
    #     ),
    #     spawn=UsdFileCfg(
    #         usd_path="/home/nvidia/workspace/yunl/assets/MedicalAssets2025093001/PunctureDevice001/PunctureDevice001.usd", 
    #     ),
    # )
    
    # Ground plane
    # 3. ground configuration
    ground = AssetBaseCfg(
        prim_path="/World/GroundPlane",    # ground in the scene
        spawn=GroundPlaneCfg( ),    # ground configuration
    )

    # Lights
    # 4. light configuration
    light = AssetBaseCfg(
        prim_path="/World/light",   # light in the scene
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), # light color (white)
                                     intensity=3000.0),    # light intensity
    )

    world_camera = CameraBaseCfg.get_camera_config(prim_path="/World/PerspectiveCamera",
                                                    pos_offset=(-0.1, 3.6, 1.6),
                                                    rot_offset=( -0.00617,0.00617, 0.70708, -0.70708),
                                                    focal_length = 16.5)