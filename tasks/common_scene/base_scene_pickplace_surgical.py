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
usd_root = "/mnt/hdd/Data"
# usd_root = "/home/nvidia/workspace/yunl/assets"
@configclass
class SurgicalSceneCfg(InteractiveSceneCfg): # inherit from the interactive scene configuration class
    """object table scene configuration class
    defines a complete scene containing robot, object, table, etc.
    """
      # 1. room wall configuration - simplified configuration to avoid rigid body property conflicts
    scene = AssetBaseCfg(
        prim_path="/World/envs/env_.*/Scene",
        spawn=UsdFileCfg(
            usd_path=f"{usd_root}/lw_v1/surgery-room-dev-internal/assets/Assets/scene.usd",  # use simple room model
        ),
    )
    # Trocar (rigid object inside the loaded scene.usd)
    trocar_1 = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Scene/Trocar002",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=[-1.61873, 1.9629, 0.82559],
            rot=[0.60545, 0.00148, -0.72054, -0.33799]
        ),
    )
    trocar_2 = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Scene/DisposableLaparoscopicPunctureDevice001",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=[-1.52635, 2.09436, 0.85483],
            rot=[0.63046, -0.59294, -0.33848, 0.36928]
        ),
    )

    # Cart (example)
    # cart = RigidObjectCfg(
    #     prim_path="/World/envs/env_.*/Cart001"
    # )
    # Plate (example)
    plate = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Scene/plate001",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=[-1.46817, 2.07344, 0.80932],
            rot=[1.0, 0.0, 0.0, 0.0]
        ),
    )


    # Tube (example)
    tube = AssetBaseCfg(
        prim_path="/World/envs/env_.*/Scene/DrainageTube003",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[-1.49695, 2.087, 0.84494],
            rot=[0.98716, 0.1597, 0.0, 0.0]
        ),
    )
    
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
    # cart = AssetBaseCfg(
    #     prim_path="/World/envs/env_.*/Cart",
    #     init_state=AssetBaseCfg.InitialStateCfg(
    #         pos=[0.38297, 0.36, 0.0],
    #         rot=[1.0, 0.0, 0.0, 0.0]
    #     ),
    #     spawn=UsdFileCfg(
    #         usd_path=f"/home/nvidia/workspace/yunl/assets/MedicalAssets20251013/Cart001/Cart001.usd",  # use simple room model
    #     ),
    # )
    
    # mayo stand
    # mayo = AssetBaseCfg(
    #     prim_path="/World/envs/env_.*/MayoStand",
    #     init_state=AssetBaseCfg.InitialStateCfg(
    #         pos=[0.36603, -0.17142, 0.0],
    #         rot=[1.0, 0.0, 0.0, 0.0]
    #     ),
    #     spawn=UsdFileCfg(
    #         usd_path=f"/home/nvidia/workspace/yunl/assets/InstrumentTrolley001/InstrumentTrolley001.usd",
    #     ),
    # )
    
    # plate
    # plate = AssetBaseCfg(
    #     prim_path="/World/envs/env_.*/Plate",
    #     init_state=AssetBaseCfg.InitialStateCfg(
    #         pos=[0.37286, 0.20589, 0.81333],
    #         # pos=[0.28, 0.17, 1.24],
    #         rot=[1.0, 0.0, 0.0, 0.0]
    #     ),
    #     spawn=UsdFileCfg(
    #         usd_path="/home/nvidia/workspace/yunl/assets/MedicalAssets20251013/Plate001/plate001.usd",  # use simple room model
    #     ),
    # )
    
    # tube 
    # tube = AssetBaseCfg(
    #     prim_path="/World/envs/env_.*/DrainageTube002",
    #     init_state=AssetBaseCfg.InitialStateCfg(
    #         pos=[0.37669, 0.20845, 0.80146],
    #         rot=[0.70711, 0.0, 0.0, 0.70711]
    #     ),
    #     spawn=UsdFileCfg(
    #         # usd_path="/home/nvidia/workspace/yunl/assets/MedicalAssets2025093001/DrainageTube002/DrainageTube002.usd",
    #         usd_path="/home/nvidia/workspace/yunl/assets/DrainageTube.usd",
    #         collision_props=sim_utils.CollisionPropertiesCfg(
    #             contact_offset=0.01,
    #             rest_offset=0.001
    #         ),
    #     ),
    # )

    # trocar 
    # trocar = AssetBaseCfg(
    #     prim_path="/World/envs/env_.*/PunctureDevice001",
    #     init_state=AssetBaseCfg.InitialStateCfg(
    #         # pos=[0.235, 0.108, 0.74739],
    #         # pos=[0.235, 0.12664, 0.76121],
    #         # rot=[0.52133, 0.47772, 0.47772, 0.53134]
    #         # pos=[0.235, 0.14499, 0.760],
    #         # rot=[0.75849, 0.65037, 0.00244, 0.04115]
    #         pos=[0.26335, 0.19818, 0.81428],
    #         rot=[0.72537, 0.68835, 0.0, 0.0]
    #     ),
    #     spawn=UsdFileCfg(
    #         # usd_path="/home/nvidia/workspace/yunl/assets/MedicalAssets2025093001/PunctureDevice001/PunctureDevice001.usd", 
    #         usd_path="/home/nvidia/workspace/yunl/assets/Trocar02/Trocar002/Trocar002.usd", 
    #     ),
    # )
    # trocar_object = RigidObjectCfg(
    #     prim_path="/World/envs/env_.*/PunctureDevice001/PunctureDevice001_Pipe",
    #     init_state=RigidObjectCfg.InitialStateCfg(
    #         pos=[0.311, 0.151, 0.765],
    #         rot=[0.0, 0.0, 0.68835, 0.72537]
    #     ),
    # )
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
    # ground = AssetBaseCfg(
    #     prim_path="/World/GroundPlane",    # ground in the scene
    #     spawn=GroundPlaneCfg( ),    # ground configuration
    # )

    # Lights
    # 4. light configuration
    light = AssetBaseCfg(
        prim_path="/World/light",   # light in the scene
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), # light color (white)
                                     intensity=1000.0),    # light intensity
    )
    world_camera = CameraBaseCfg.get_camera_config(prim_path="/World/PerspectiveCamera",
                                                    pos_offset=(-1.85014, 1.9196, 1.20101),
                                                    rot_offset=euler_angles_to_quats(torch.tensor([46.0, 180.0, 92.0]), degrees=True),
                                                    focal_length = 13.0) 
