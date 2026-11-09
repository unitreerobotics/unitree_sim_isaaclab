# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0     
"""
public base scene configuration module
provides reusable scene element configurations, such as tables, objects, ground, lights, etc.
"""
import isaaclab.sim as sim_utils
from isaaclab.assets import  AssetBaseCfg, RigidObjectCfg, ArticulationCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.actuators.actuator_cfg import ImplicitActuatorCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip
from isaaclab.sensors.frame_transformer import OffsetCfg
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR
FRAME_MARKER_SMALL_CFG = FRAME_MARKER_CFG.copy()
FRAME_MARKER_SMALL_CFG.markers["frame"].scale = (0.10, 0.10, 0.10)
from tasks.common_config import   CameraBaseCfg  
import os
project_root = os.environ.get("PROJECT_ROOT")


@configclass
class TablePickRedblockIntoDrawerSceneCfg(InteractiveSceneCfg): # inherit from the interactive scene configuration class
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
            usd_path=f"{project_root}/assets/objects/small_warehouse/small_warehouse_digital_twin.usd",  # use simple room model
        ),
    )


    # 1. table configuration
    # packing_table = AssetBaseCfg(
    #     prim_path="/World/envs/env_.*/PackingTable",    # table in the scene
    #     init_state=AssetBaseCfg.InitialStateCfg(pos=[-4.3,-4.2,-0.2],   # initial position [x, y, z]
    #                                             rot=[1.0, 0.0, 0.0, 0.0]), # initial rotation [x, y, z, w]
    #     spawn=UsdFileCfg(
    #         usd_path=f"{ASSET_DIR}/factory_peg_8mm.usd",    # table model file
    #         # rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),    # set to kinematic object
    #     ),
    # )
    # Object
    object = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Red_block",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=[-2.5, -4.15, 0.84],
            rot=[0, 0, 0, 1]
        ),
        spawn=sim_utils.CuboidCfg(
            size=(0.05, 0.05, 0.05),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                retain_accelerations=False
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=3.0),
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



    cabinet = ArticulationCfg(
        prim_path="/World/envs/env_.*/Cabinet",
        spawn=sim_utils.UsdFileCfg(
            ## update sektion_cabinet_instanceable.usd with the bottom_drawer's collider approximation as convex_decomposition.
            usd_path=f"{project_root}/assets/objects/drawers/cabinet_collider.usd",
            # usd_path=f"{ISAAC_NUCLEUS_DIR}/Isaac/Props/Furniture/Sektion_Cabinet/sektion_cabinet_instanceable.usd",
            activate_contact_sensors=False,
            # 设置刚体属性，为抽屉把手优化摩擦力
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                retain_accelerations=False,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(-2.5,-4.35,0.45),
            rot=(0, 0, 0.7071, 0.7071),
            joint_pos={
                "door_left_joint": 0.0,
                "door_right_joint": 0.0,
                "drawer_bottom_joint": 0.0,
                "drawer_top_joint": 0.15,
            },
        ),
        actuators={
            "drawers": ImplicitActuatorCfg(
                joint_names_expr=["drawer_top_joint", "drawer_bottom_joint"],
                effort_limit_sim=100.0,
                velocity_limit_sim=100.0,
                stiffness=100.0,
                damping=1.0,
            ),
            "doors": ImplicitActuatorCfg(
                joint_names_expr=["door_left_joint", "door_right_joint"],
                effort_limit_sim=100.0,
                velocity_limit_sim=100.0,
                stiffness=100.0,
                damping=1.0,
            ),
        },
    )
    
    
    cabinet_frame = FrameTransformerCfg(
        prim_path="/World/envs/env_.*/Cabinet/cabinet/sektion",
        debug_vis=True,
        visualizer_cfg=FRAME_MARKER_SMALL_CFG.replace(prim_path="/Visuals/CabinetFrameTransformer"),
        target_frames=[
            FrameTransformerCfg.FrameCfg(
                prim_path="/World/envs/env_.*/Cabinet/cabinet/drawer_handle_bottom",
                name="drawer_handle_bottom",
                offset=OffsetCfg(
                    pos=(0.305, 0.0, 0.01),
                    rot=(0.5, -0.5, -0.5, 0.5),  # align with end-effector frame
                ),
            ),
        ],
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
                                     intensity=3000.0),    # light intensity
    )



    world_camera = CameraBaseCfg.get_camera_config(prim_path="/World/PerspectiveCamera",
                                                    pos_offset=(-2.5, -4.8, 1.8),
                                                    rot_offset=(0.94833, 0.0, 0.0, -0.3173))


