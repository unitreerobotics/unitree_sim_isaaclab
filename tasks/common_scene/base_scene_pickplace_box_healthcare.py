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
            usd_path="omniverse://isaac-dev.ov.nvidia.com/Library/IsaacHealthcare/0.5.0/Props/OrcaScenes/Scene1/main.usd",
        ),
    )

    # 2. Box object configuration for manipulation
    object = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Object",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=[0.5, 0.0, 0.85],  # initial position on table/workspace
            rot=[1.0, 0.0, 0.0, 0.0]  # initial rotation (identity quaternion)
        ),
        spawn=sim_utils.CuboidCfg(
            size=(0.06, 0.06, 0.06),  # box dimensions: 6cm x 6cm x 6cm
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.2),  # 200g box
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.8, 0.3, 0.1),  # orange/brown box color
                metallic=0.2,
                roughness=0.8
            ),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="multiply",
                restitution_combine_mode="multiply",
                static_friction=1.0,
                dynamic_friction=1.0,
                restitution=0.0,
            ),
        ),
    )

    # 3. Light configuration
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(
            color=(0.75, 0.75, 0.75),
            intensity=2500.0
        ),
    )
    
    # 4. World camera configuration
    world_camera = CameraBaseCfg.get_camera_config(
        prim_path="/World/PerspectiveCamera",
        pos_offset=(1.5, -2.0, 1.5),
        rot_offset=(0.3827, 0.0, 0.0, 0.9239)  # looking at workspace
    )

