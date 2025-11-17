# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0      
"""
Healthcare scene configuration module for box picking task
Provides scene configuration with healthcare environment and box object
"""
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass
from tasks.common_config import CameraBaseCfg  # isort: skip
import os

project_root = os.environ.get("PROJECT_ROOT")

@configclass
class HealthcareBoxSceneCfg(InteractiveSceneCfg):
    """Healthcare scene configuration class for box picking task
    Defines a complete scene containing robot, box object, and healthcare environment
    """
    
    # 1. Healthcare scene configuration from Omniverse
    healthcare_scene = AssetBaseCfg(
        prim_path="/World/envs/env_.*/HealthcareScene",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[0.0, 0.0, 0.0],  # scene center point
            rot=[1.0, 0.0, 0.0, 0.0]
        ),
        spawn=UsdFileCfg(
            usd_path="omniverse://isaac-dev.ov.nvidia.com/Library/IsaacHealthcare/0.5.0/Props/OrcaScenes/Scene1DevMz/main.usd",
        ),
    )

    # 2. Cart002 - Medical cart with proper rigid body physics
    cart = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Cart",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=[-4.0, -1.5, 0.0],  # Position cart in workspace (adjust as needed)
            rot=[1.0, 0.0, 0.0, 0.0]  # Identity quaternion
        ),
        spawn=UsdFileCfg(
            usd_path="omniverse://isaac-dev.ov.nvidia.com/Library/IsaacHealthcare/0.5.0/Props/OrcaScenes/Scene1DevMz/SurgicalRoom/Assets/Cart002/Cart002.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=True,
                kinematic_enabled=False,  # Dynamic, can be moved
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.1),  # 1kg cart
        ),
    )
    
    # 3. Box object configuration for manipulation (optional, can be removed if only using cart)
    object = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Object",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=[-3.0, -0.6, 1.3],  # initial position on table/workspace (in front of robot)
            rot=[1.0, 0.0, 0.0, 0.0]  # initial rotation (identity quaternion)
        ),
        spawn=sim_utils.CuboidCfg(
            size=(0.2, 0.2, 0.2),  # box dimensions: 20cm x 20cm x 20cm
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.1),  # 100g box
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.8, 0.3, 0.1),  # orange/brown box color
                metallic=0.2,
                roughness=0.8
            ),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="max",
                restitution_combine_mode="max",
                static_friction=1.0,
                dynamic_friction=1.0,
                restitution=0.0,
            ),
        ),
    )
    

    # 4. Light configuration
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(
            color=(0.75, 0.75, 0.75),
            intensity=2500.0
        ),
    )
    
    # 5. World camera configuration
    world_camera = CameraBaseCfg.get_camera_config(
        prim_path="/World/PerspectiveCamera",
        pos_offset=(-5.0, 1.0, 3.0),
        rot_offset=(-0.2095, -0.3532, 0.7841, 0.4653)  # looking at workspace
        # rot_offset=(x, w, z, y)
    )

