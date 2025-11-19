# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
import torch
from dataclasses import MISSING

import isaaclab.envs.mdp as base_mdp
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.envs.common import ViewerCfg
from isaaclab.managers import EventTermCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.utils import configclass
from isaaclab.assets import ArticulationCfg
from isaaclab.sensors import ContactSensorCfg
from . import mdp

# use Isaac Lab native event system
from tasks.common_config import G1RobotPresets, CameraPresets  # isort: skip
from tasks.common_event.event_manager import SimpleEvent, SimpleEventManager

# import healthcare scene configuration
from tasks.common_scene.base_scene_pickplace_box_healthcare import HealthcareBoxSceneCfg

##
# Scene definition
##

@configclass
class HealthcareBoxPickSceneCfg(HealthcareBoxSceneCfg):
    """Healthcare box picking scene configuration class
    Inherits from HealthcareBoxSceneCfg, gets the complete healthcare scene configuration
    Can add task-specific scene elements or override default configurations here
    """
    
    # G1 robot with 29 DOF + Dex3 hands in wholebody control mode
    robot: ArticulationCfg = G1RobotPresets.g1_29dof_dex3_wholebody(
        init_pos=(-5.0, -0.05, 0.8),  # position robot in front of workspace
        init_rot=(1, 0, 0, 0),
    )
    # Contact force sensor for gripper feedback
    contact_forces = ContactSensorCfg(
        prim_path="/World/envs/env_.*/Robot/.*", 
        history_length=10, 
        track_air_time=True, 
        debug_vis=False
    )
    
    # Camera configuration for visual observations
    front_camera = CameraPresets.g1_front_camera()
    left_wrist_camera = CameraPresets.left_dex3_wrist_camera()
    right_wrist_camera = CameraPresets.right_dex3_wrist_camera()
    robot_camera = CameraPresets.g1_world_camera()

##
# MDP settings
##

@configclass
class ActionsCfg:
    """Action configuration for robot control
    Uses direct joint position control for all degrees of freedom
    """
    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot", 
        joint_names=[".*"], 
        scale=1.0, 
        use_default_offset=True
    )


@configclass
class ObservationsCfg:
    """Observation configuration
    Defines all available observation information for the policy
    """
    @configclass
    class PolicyCfg(ObsGroup):
        """Policy observation group configuration
        Defines all state observations for policy decision making
        """
        # Robot body joint states (29 DOF: legs, waist, arms)
        robot_joint_state = ObsTerm(func=mdp.get_robot_boy_joint_states)
        
        # Dex3 gripper states
        robot_gripper_state = ObsTerm(func=mdp.get_robot_dex3_joint_states)
        
        # Camera images from multiple viewpoints
        camera_image = ObsTerm(func=mdp.get_camera_image)
        
        # Box object state (position, orientation, velocity)
        object_state = ObsTerm(func=mdp.get_object_state)

        def __post_init__(self):
            """Post initialization settings"""
            self.enable_corruption = False  # disable observation noise
            self.concatenate_terms = False  # keep observations separate

    # Create policy observation group instance
    policy: PolicyCfg = PolicyCfg()


@configclass
class TerminationsCfg:
    """Termination conditions configuration"""
    # Episode terminates if box falls off workspace
    object_out_of_bounds = DoneTerm(
        func=mdp.object_out_of_bounds,
        params={"min_height": 0.3}  # terminate if box falls below 30cm
    )


@configclass
class RewardsCfg:
    """Reward function configuration"""
    # Main reward based on box picking success
    picking_reward = RewTerm(
        func=mdp.compute_box_picking_reward,
        weight=1.0
    )
    
    # Penalty for dropping the box
    drop_penalty = RewTerm(
        func=mdp.box_drop_penalty,
        weight=-2.0
    )


@configclass
class EventCfg:
    """Event configuration for scene resets"""
    pass


@configclass
class PickBoxHealthcareG129Dex3EnvCfg(ManagerBasedRLEnvCfg):
    """Complete environment configuration for healthcare box picking task
    Inherits from ManagerBasedRLEnvCfg and defines all parameters for the RL environment
    """

    # 1. Scene settings
    scene: HealthcareBoxPickSceneCfg = HealthcareBoxPickSceneCfg(
        num_envs=1,  # number of parallel environments
        env_spacing=2.5,  # spacing between environments
        replicate_physics=True  # enable physics replication for parallel envs
    )
    
    # 2. Basic MDP settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    
    # 3. Advanced MDP settings
    terminations: TerminationsCfg = TerminationsCfg()
    events = EventCfg()
    commands = None  # no command manager needed
    rewards: RewardsCfg = RewardsCfg()
    curriculum = None  # no curriculum learning
    
    # 4. Viewer settings - set default viewport camera perspective
    viewer: ViewerCfg = ViewerCfg(
        eye=(0.8, -1.0, 6.0),  # camera position matching world_camera config
        lookat=(-4.5, -1.2, 1.0),  # look at workspace center (between robot and box)
        origin_type="world"  # coordinates relative to world origin
    )
    
    def __post_init__(self):
        """Post initialization - configure simulation parameters"""
        # General settings
        self.decimation = 4  # control frequency = sim_freq / decimation
        self.episode_length_s = 30.0  # 30 second episodes
        
        # Simulation settings
        self.sim.dt = 0.005  # 200 Hz simulation
        self.scene.contact_forces.update_period = self.sim.dt
        self.sim.render_interval = self.decimation
        
        # PhysX settings for stable simulation
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625
        
        # Physics material properties
        self.sim.physics_material.static_friction = 1.0
        self.sim.physics_material.dynamic_friction = 1.0
        self.sim.physics_material.friction_combine_mode = "multiply"  # Use max for better stability
        self.sim.physics_material.restitution_combine_mode = "multiply"
        
        # Create event manager for dynamic scene control
        self.event_manager = SimpleEventManager()
        
        # Register box reset event - randomizes box position
        self.event_manager.register("reset_object_self", SimpleEvent(
            func=lambda env: base_mdp.reset_root_state_uniform(
                env,
                torch.arange(env.num_envs, device=env.device),
                pose_range={
                    "x": [-0.1, 0.1],  # ±10cm variation in x
                    "y": [-0.1, 0.1],  # ±10cm variation in y
                    "z": [0.0, 0.05]   # 0-5cm height variation
                },
                velocity_range={},
                asset_cfg=SceneEntityCfg("object"),
            )
        ))
        
        # Register full scene reset event
        self.event_manager.register("reset_all_self", SimpleEvent(
            func=lambda env: base_mdp.reset_scene_to_default(
                env,
                torch.arange(env.num_envs, device=env.device)
            )
        ))

    @staticmethod
    def apply_cart_low_friction(env):
        """Apply low friction to cart to simulate wheels (call this after scene creation)"""
        try:
            from pxr import UsdPhysics, PhysxSchema
            stage = env.sim.stage
            
            for env_idx in range(env.num_envs):
                cart_path = f"/World/envs/env_{env_idx}/Cart"
                cart_prim = stage.GetPrimAtPath(cart_path)
                
                if cart_prim and cart_prim.IsValid():
                    # Apply physics material with low friction for wheels
                    material_api = UsdPhysics.MaterialAPI.Apply(cart_prim)
                    material_api.CreateStaticFrictionAttr().Set(0.0001)  # Low friction
                    material_api.CreateDynamicFrictionAttr().Set(0.0001)  # Low friction
                    material_api.CreateRestitutionAttr().Set(0.0)  # No bouncing
                    
                    # Apply PhysX material schema
                    physx_material = PhysxSchema.PhysxMaterialAPI.Apply(cart_prim)
                    physx_material.CreateFrictionCombineModeAttr().Set("min")  # Use minimum friction
                    physx_material.CreateRestitutionCombineModeAttr().Set("min")
                    
                    print(f"✅ Applied low friction to Cart at {cart_path}")
        except Exception as e:
            print(f"⚠️  Failed to apply friction to cart: {e}")
