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
from tasks.common_config import   CameraBaseCfg  
import os
project_root = os.environ.get("PROJECT_ROOT")
@configclass
class TableRedGreenYellowBlockSceneCfg(InteractiveSceneCfg): # inherit from the interactive scene configuration class
    """object table scene configuration class
    defines a complete scene containing robot, object, table, etc.
    """
      # 1. room wall configuration - simplified configuration to avoid rigid body property conflicts
    room_walls = AssetBaseCfg(
        prim_path="/World/envs/env_.*/Room",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[0.0, 0.0, 0],  # room center point
            rot=[0.0, 0.0, 0.0, 1.0]
        ),
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/small_warehouse_digital_twin/small_warehouse_digital_twin.usd",  # use simple room model
        ),
    )


    # 1. table configuration
    packing_table = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable",    # table in the scene
        init_state=AssetBaseCfg.InitialStateCfg(pos=[-4.3,-4.2,-0.2],   # initial position [x, y, z]
                                                rot=[0.0, 0.0, 0.0, 1.0]), # initial rotation [x, y, z, w]
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/table_with_yellowbox.usd",    # table model file
            # rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),    # set to kinematic object
        ),
    )
    # Object
    red_block = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Red_block",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos= [-4.1, -4.08, 0.84],
            rot=[0, 0, 0, 1]
        ),
        spawn=sim_utils.CuboidCfg(
            size=(0.05, 0.05, 0.05),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                retain_accelerations=False
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
                contact_offset=0.01,
                rest_offset=0.0
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(1.0, 0.0, 0.0), metallic=0
            ),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="max",
                restitution_combine_mode="min",
                static_friction=10,
                dynamic_friction=0.5,
                restitution=0.0,
            ),
        ),
    )

    yellow_block = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Yellow_block",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos= [-4.25, -4.05, 0.84],
            rot=[0, 0, 0, 1]
        ),
        spawn=sim_utils.CuboidCfg(
            size=(0.05, 0.05, 0.05),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                retain_accelerations=False
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
                contact_offset=0.01,
                rest_offset=0.0
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(1.0, 1.0, 0.0), metallic=0
            ),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="max",
                restitution_combine_mode="min",
                static_friction=10,
                dynamic_friction=0.5,
                restitution=0.0,
            ),
        ),
    )
    green_block = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Green_block",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=  [-4.18, -4.12, 0.84] ,
            rot=[0, 0, 0, 1]
        ),
        spawn=sim_utils.CuboidCfg(
            size=(0.05, 0.05, 0.05),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                retain_accelerations=False
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
                contact_offset=0.01,
                rest_offset=0.0
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.0, 1.0, 0.0), metallic=0
            ),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="max",
                restitution_combine_mode="min",
                static_friction=10,
                dynamic_friction=0.5,
                restitution=0.0,
            ),
        ),
    )
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
                                     intensity=500.0),    # light intensity
    )






    world_camera = CameraBaseCfg.get_camera_config(prim_path="/World/PerspectiveCamera",
                                                    pos_offset=(-4.1, -4.9, 1.8),
                                                    rot_offset=(0.94833, 0.0, 0.0, -0.3173))


