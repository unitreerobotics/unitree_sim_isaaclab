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
_SUPPRESS_ROOM_WALLS = os.environ.get("UNITREE_SIM_SUPPRESS_ROOM_WALLS", "0") == "1"


@configclass
class TableCylinderSceneCfgWH(InteractiveSceneCfg): # inherit from the interactive scene configuration class
    """object table scene configuration class
    defines a complete scene containing robot, object, table, etc.

    Setting ``UNITREE_SIM_SUPPRESS_ROOM_WALLS=1`` in the environment swaps the
    warehouse mesh out for a plain ground plane. The walls don't carry any
    task semantics — they're decorative — but they block the default world
    camera view of the robot, which makes headed development awkward.
    """
    if _SUPPRESS_ROOM_WALLS:
        ground = AssetBaseCfg(
            prim_path="/World/Ground",
            spawn=GroundPlaneCfg(),
        )
    else:
        # 1. room wall configuration - simplified configuration to avoid rigid body property conflicts
        room_walls = AssetBaseCfg(
            prim_path="/World/envs/env_.*/Room",
            init_state=AssetBaseCfg.InitialStateCfg(
                pos=[0.0, 0.0, 0],  # room center point
                rot=[1.0, 0.0, 0.0, 0.0]
            ),
            spawn=UsdFileCfg(
                usd_path=f"{project_root}/assets/objects/small_warehouse/small_warehouse_digital_twin.usd",
            ),
        )


        # 1. table configuration
    packing_table1 = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable_1",    # table in the scene
        init_state=AssetBaseCfg.InitialStateCfg(pos=[-2.35644,-3.45572,-0.2],   # initial position [x, y, z]
                                                rot=[0.70091, 0.0, 0.0, 0.71325]), # initial rotation [x, y, z, w]
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/PackingTable_2/PackingTable.usd",    # table model file
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),    # set to kinematic object
        ),
    )

    packing_table2 = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable_2",    # table in the scene
        init_state=AssetBaseCfg.InitialStateCfg(pos=[-3.97225,-4.3424,-0.2],   # initial position [x, y, z]
                                                rot=[1.0, 0.0, 0.0, 0.0]), # initial rotation [x, y, z, w]
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/PackingTable/PackingTable.usd",    # table model file
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),    # set to kinematic object
        ),
    )
    # # Object
    # 2. object configuration (cylinder)     
    object = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Object",    # object in the scene
        init_state=RigidObjectCfg.InitialStateCfg(pos=[-2.58514,-2.78975,0.84], # initial position (pos) 
                                                  rot=[1, 0, 0, 0]), # initial rotation (rot)
        spawn=sim_utils.CylinderCfg(
            radius=0.018,    # cylinder radius (radius)
            height=0.35,     # cylinder height (height)
 
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
            ),    # rigid body properties configuration (rigid_props)
            mass_props=sim_utils.MassPropertiesCfg(mass=0.4),    # mass properties configuration (mass)
            collision_props=sim_utils.CollisionPropertiesCfg(),    # collision properties configuration (collision_props)
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.15, 0.15, 0.15), metallic=1.0),    # visual material configuration (visual_material)
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="max",    # friction combine mode
                restitution_combine_mode="min",    # restitution combine mode
                static_friction=1.5,    # static friction coefficient
                dynamic_friction=1.5,    # dynamic friction coefficient
                restitution=0.0,    # restitution coefficient (no restitution)
            ),
        ),
    )
    # Ground plane


    # Lights
    # 4. light configuration
    light = AssetBaseCfg(
        prim_path="/World/light",   # light in the scene
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), # light color (white)
                                     intensity=3000.0),    # light intensity
    )
    world_camera = CameraBaseCfg.get_camera_config(prim_path="/World/PerspectiveCamera",
                                                    pos_offset=(-1.9, -5.0, 1.8),
                                                    rot_offset=( -0.40614,0.78544, 0.4277, -0.16986))