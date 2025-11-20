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
from tasks.common_scene.custom_spawners import spawn_usd_reference_direct  # isort: skip
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
                disable_gravity=False,
                linear_damping=0.005,  # Very low damping - easy to push (like wheels!)
                angular_damping=0.005,  # Very low damping
                max_linear_velocity=1000.0,
                max_angular_velocity=1000.0,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.1),  # 100g cart (matches ORCA)
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
            ),
        ),
    )
    
    # 3. ORCA Sterilization Container - realistic healthcare object for manipulation
    object = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Object",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=[-4.5, 0.6, 1.0],  # initial position on table/workspace (in front of robot)
            rot=[0.7071, 0.0, 0.0, 0.7071]  # initial rotation (identity quaternion)
        ),
        spawn=UsdFileCfg(
            func=spawn_usd_reference_direct,  # Use custom spawner for better compatibility
            usd_path="omniverse://isaac-dev.ov.nvidia.com/Library/IsaacHealthcare/0.5.0/Props/OrcaScenes/Scene1DevMz/SurgicalRoom/Assets/SterilizationContainer002/SterilizationContainer002.usd",
            scale=(1.0, 1.0, 1.0),  # use default scale
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=True,
                kinematic_enabled=False,
                disable_gravity=False,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.1),  # 100g sterilization container
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
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

