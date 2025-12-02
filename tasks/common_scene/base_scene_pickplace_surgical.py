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
    scene = AssetBaseCfg(
        prim_path="/World/envs/env_.*/Scene",
        spawn=UsdFileCfg(
            usd_path=f"{usd_root}/lw_v2/scene_v2.usd",  # use simple room model
        ),
    )

    trocar_1 = RigidObjectCfg(
        prim_path="/World/envs/env_.*/trocar_1",
        spawn=UsdFileCfg(
            usd_path=f"{usd_root}/lw_v2/Assets/Trocar002/Trocar002_wo.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=True,
                disable_gravity=False,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=1000.0,
                max_angular_velocity=1000.0,
                # low penetration recovery velocity, avoid sudden popping (critical!)
                max_depenetration_velocity=1.0,
            ),
            # collision properties: important for articulation interaction
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
                contact_offset=0.01,       # increase to 0.01, important for articulation interaction
                rest_offset=-0.001,        # negative value allows slight overlap, improve stability
            ),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            # pos=[-1.55953, 2.00288, 0.83483],
            pos=[-1.56134, 2.00047, 0.83483],
            # rot=[0.17602, -0.70516, 0.18787, 0.66066]
            rot=[0.09921, -0.72203, 0.1059, 0.67647]
        ),
    )
    
    trocar_2 = RigidObjectCfg(
        prim_path="/World/envs/env_.*/trocar_2",
        spawn=UsdFileCfg(
            usd_path=f"{usd_root}/lw_v2/Assets/DisposableLaparoscopicPunctureDevice001/DisposableLaparoscopicPunctureDevice003.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=True,
                disable_gravity=False,
            ),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=[-1.52635, 2.09436, 0.85483],
            rot=[0.63046, -0.59294, -0.33848, 0.36928]
        ),
    )
    
    # Lights
    light = AssetBaseCfg(
        prim_path="/World/light",   # light in the scene
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), # light color (white)
                                     intensity=1000.0),    # light intensity
    )

