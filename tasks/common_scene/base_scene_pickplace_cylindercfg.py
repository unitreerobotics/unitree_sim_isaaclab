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


def hospital_medicine_bottle_cfg(
    prim_path: str = "/World/envs/env_.*/Object",
    init_pos: tuple[float, float, float] = (-0.68, 0.40, 0.86),
) -> RigidObjectCfg:
    """Build the shared prescription-bottle config without class-field lookup."""
    return RigidObjectCfg(
        prim_path=prim_path,
        # Measured PackingTable top is z~=0.794 after its -0.2 world offset.
        # Bottle bottom is -0.055, so z=0.86 leaves 11 mm spawn clearance.
        init_state=RigidObjectCfg.InitialStateCfg(
            # Leftmost item in the front 1x4 task row.  The complete row stays
            # away from the table's built-in container on the right.
            pos=init_pos,
            rot=[1, 0, 0, 0],
        ),
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/hospital_medicine_bottle.usda",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                linear_damping=1.5,
                angular_damping=3.0,
                max_linear_velocity=5.0,
                max_angular_velocity=10.0,
                max_depenetration_velocity=0.25,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.12),
        ),
    )


def hospital_hand_sanitizer_cfg(
    prim_path: str = "/World/envs/env_.*/HandSanitizer",
    init_pos: tuple[float, float, float] = (0.07, 0.40, 0.875),
) -> RigidObjectCfg:
    """Build the shared graspable sanitizer config."""
    return RigidObjectCfg(
        prim_path=prim_path,
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=init_pos,
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/hospital_hand_sanitizer.usda",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                linear_damping=1.5,
                angular_damping=3.0,
                max_linear_velocity=5.0,
                max_angular_velocity=10.0,
                max_depenetration_velocity=0.25,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.18),
        ),
    )


@configclass
class TableCylinderSceneCfg(InteractiveSceneCfg): # inherit from the interactive scene configuration class
    """object table scene configuration class
    defines a complete scene containing robot, object, table, etc.
    """
      # 1. room wall configuration - simplified configuration to avoid rigid body property conflicts
    room_walls = AssetBaseCfg(
        prim_path="/World/envs/env_.*/Room",
        init_state=AssetBaseCfg.InitialStateCfg(
            # The authored hospital floor spans roughly x=[-13, -1],
            # y=[-14, -4]. Translate its open interior around the existing
            # calibrated robot/table workspace at the origin.
            pos=[7.5, 7.5, 0.0],
            rot=[1.0, 0.0, 0.0, 0.0]
        ),
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/environments/hospital_room/new_base_room.usda",
        ),
    )
    # print(f"ISAAC_NUCLEUS_DIR: {ISAAC_NUCLEUS_DIR}")
    #ISAAC_NUCLEUS_DIR: http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac
        # 1. table configuration
    packing_table = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable",    # table in the scene
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.0, 0.55, -0.2],   # initial position [x, y, z]
                                                rot=[1.0, 0.0, 0.0, 0.0]), # initial rotation [x, y, z, w]
        spawn=UsdFileCfg(
            # usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/PackingTable/packing_table.usd",    # table model file
            usd_path=f"{project_root}/assets/objects/PackingTable/PackingTable.usd",    # table model file
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),    # set to kinematic object
        ),
    )

    packing_table_2 = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable_2",   
        init_state=AssetBaseCfg.InitialStateCfg(pos=[-3.5, 0.55, -0.2],  
                                                rot=[1.0, 0.0, 0.0, 0.0]), 
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/PackingTable/PackingTable.usd",    # table model file
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),   
        ),
    )
    packing_table_3 = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable_3",   
        init_state=AssetBaseCfg.InitialStateCfg(pos=[3.5, 0.55, -0.2],  
                                                rot=[1.0, 0.0, 0.0, 0.0]), 
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/PackingTable/PackingTable.usd",    # table model file
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),   
        ),
    )
    packing_table_4 = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable_4",   
        init_state=AssetBaseCfg.InitialStateCfg(pos=[3.5, -5, -0.2],  
                                                rot=[1.0, 0.0, 0.0, 0.0]), 
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/PackingTable/PackingTable.usd",    # table model file
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),   
        ),
    )
    packing_table_5 = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable_5",   
        init_state=AssetBaseCfg.InitialStateCfg(pos=[-3.5, -5, -0.2],  
                                                rot=[1.0, 0.0, 0.0, 0.0]), 
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/PackingTable/PackingTable.usd",    # table model file
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),   
        ),
    )
    packing_table_6 = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable_6",   
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.0, -5, -0.2],  
                                                rot=[1.0, 0.0, 0.0, 0.0]), 
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/PackingTable/PackingTable.usd",    # table model file
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),   
        ),
    )
    # Object: hospital prescription bottle
    object = hospital_medicine_bottle_cfg()
    # Collision-only support plane.  The hospital USD floor is visual geometry
    # and does not provide a reliable physics collider for a free-root G1.
    ground = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -0.01)),
        spawn=GroundPlaneCfg(visible=False, size=(30.0, 30.0)),
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
