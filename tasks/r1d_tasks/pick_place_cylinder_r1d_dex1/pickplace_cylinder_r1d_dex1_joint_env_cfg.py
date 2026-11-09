# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  

import tempfile
import torch
from dataclasses import MISSING



import isaaclab.envs.mdp as base_mdp
from isaaclab.assets import ArticulationCfg
from tasks.common_config.env_configs import UnitreeManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.utils import configclass

from . import mdp
# use Isaac Lab native event system

from tasks.common_config import  R1DRobotPresets  # isort: skip
from tasks.common_config.r1d_camera_configs import R1DCameraPresets
from tasks.common_event.event_manager import SimpleEvent, SimpleEventManager
from robots.robot_assets import apply_material_finish

# import public scene configuration
from tasks.common_scene.base_scene_pickplace_cylindercfg import TableCylinderSceneCfg

##
# Scene definition
##

@configclass
class ObjectTableSceneCfg(TableCylinderSceneCfg):
    """object table scene configuration class
    
    inherits from G1SingleObjectSceneCfg, gets the complete G1 robot scene configuration
    can add task-specific scene elements or override default configurations here
    """
    
    # Humanoid robot w/ arms higher
    # 5. humanoid robot configuration 
    robot: ArticulationCfg = R1DRobotPresets.r1_d_dex1()
    # 6. add camera configuration 
    # head stereo (left -> image "head", right -> "head_right") + wrist cameras, from the calibration
    front_camera = R1DCameraPresets.head_left_camera()
    front_right_camera = R1DCameraPresets.head_right_camera()
    left_wrist_camera = R1DCameraPresets.left_wrist_camera()
    right_wrist_camera = R1DCameraPresets.right_wrist_camera()

##
# MDP settings
##
@configclass
class ActionsCfg:
    """defines the action configuration related to robot control, using direct joint angle control
    """
    joint_pos = mdp.JointPositionActionCfg(asset_name="robot", joint_names=[".*"], scale=1.0, use_default_offset=True)



@configclass
class ObservationsCfg:
    """defines all available observation information
    """
    @configclass
    class PolicyCfg(ObsGroup):
        """policy group observation configuration class
        defines all state observation values for policy decision
        inherit from ObsGroup base class 
        """

        # 1. robot joint state observation
        robot_joint_state = ObsTerm(func=mdp.get_robot_r1d_body_joint_states)
        # 2. gripper joint state observation 
        robot_gipper_state = ObsTerm(func=mdp.get_robot_r1d_dex1_joint_states)

        # 3. camera image observation
        camera_image = ObsTerm(func=mdp.get_camera_image)

        def __post_init__(self):
            """post initialization function
            set the basic attributes of the observation group
            """
            self.enable_corruption = False  # disable observation value corruption
            self.concatenate_terms = False  # disable observation item connection
    # observation groups
    # create policy observation group instance
    policy: PolicyCfg = PolicyCfg()

@configclass
class TerminationsCfg:
    # check if the object is out of the working range
    success = DoneTerm(func=mdp.reset_object_estimate)# use task completion check function

@configclass
class RewardsCfg:
    reward = RewTerm(func=mdp.compute_reward,weight=1.0)

@configclass
class EventCfg:
    # metal / plastic finish of the robot materials (the URDF only carries colors)
    robot_material_finish = EventTermCfg(func=apply_material_finish, mode="startup", params={"asset_name": "robot"})
    # write the robot's default joint positions (lift height, open grippers, ...) on every reset;
    # without it the joints start at 0 (lift fully retracted, grippers closed)
    reset_robot_joints = EventTermCfg(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={"position_range": (1.0, 1.0), "velocity_range": (0.0, 0.0), "asset_cfg": SceneEntityCfg("robot")},
    )
    reset_object = EventTermCfg(
        func=mdp.reset_root_state_uniform,  # use uniform distribution reset function
        mode="reset",   # set event mode to reset
        params={
            # position range parameter
            "pose_range": {
                "x": [-0.05, 0.05],  # x axis position range: -0.05 to 0.0 meter
                "y": [-0.05, 0.05],   # y axis position range: 0.0 to 0.05 meter
            },
            # speed range parameter (empty dictionary means using default value)
            "velocity_range": {},
            # specify
            "asset_cfg": SceneEntityCfg("object"),
        },
    )


@configclass
class PickPlaceR1DDEX1JointEnvCfg(UnitreeManagerBasedRLEnvCfg):
    """Unitree R1-D (wheeled base, Dex1 grippers) pick place environment configuration class
    inherits from UnitreeManagerBasedRLEnvCfg, defines all configuration parameters for the entire environment
    """

    # 1. scene settings
    scene: ObjectTableSceneCfg = ObjectTableSceneCfg(num_envs=1, # environment number: 1
                                                     env_spacing=2.5, # environment spacing: 2.5 meter
                                                     replicate_physics=True # enable physics replication
                                                     )
    # basic settings
    observations: ObservationsCfg = ObservationsCfg()   # observation configuration
    actions: ActionsCfg = ActionsCfg()                  # action configuration
    # MDP settings
    # 3. MDP settings
    terminations: TerminationsCfg = TerminationsCfg()    # termination configuration
    events = EventCfg()                                  # event configuration
    commands = None # command manager
    rewards: RewardsCfg = RewardsCfg()  # reward manager
    curriculum = None # curriculum manager
    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 2
        self.episode_length_s = 20.0
        # simulation settings
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.sim.physics.bounce_threshold_velocity = 0.01
        self.sim.physics.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physics.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physics.friction_correlation_distance = 0.00625




        # create event manager
        self.event_manager = SimpleEventManager()

        # register "reset object" event
        self.event_manager.register("reset_object_self", SimpleEvent(
            func=lambda env: base_mdp.reset_root_state_uniform(
                env,
                torch.arange(env.num_envs, device=str(env.device)),
                pose_range={"x": [-0.05, 0.05], "y": [0.0, 0.05]},
                velocity_range={},
                asset_cfg=SceneEntityCfg("object"),
            )
        ))
        self.event_manager.register("reset_all_self", SimpleEvent(
            func=lambda env: base_mdp.reset_scene_to_default(
                env,
                torch.arange(env.num_envs, device=str(env.device)))
        ))