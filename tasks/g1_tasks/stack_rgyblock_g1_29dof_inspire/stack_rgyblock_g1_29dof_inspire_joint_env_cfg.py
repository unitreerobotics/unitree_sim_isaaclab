# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
import tempfile
import torch
from dataclasses import MISSING



import isaaclab.envs.mdp as base_mdp
from tasks.common_config.env_configs import UnitreeManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.utils import configclass
from isaaclab.assets import ArticulationCfg
from . import mdp
# use Isaac Lab native event system

from tasks.common_config import  G1RobotPresets, CameraPresets  # isort: skip
from tasks.common_event.event_manager import SimpleEvent, SimpleEventManager, BatchObjectEvent, MultiObjectEvent

# import public scene configuration
from tasks.common_scene.base_scene_stack_rgyblock import TableRedGreenYellowBlockSceneCfg

##
# Scene definition
##

@configclass
class ObjectTableSceneCfg(TableRedGreenYellowBlockSceneCfg):
    """object table scene configuration class
    
    inherits from G1SingleObjectSceneCfg, gets the complete G1 robot scene configuration
    can add task-specific scene elements or override default configurations here
    """
    
    # Humanoid robot w/ arms higher
    # 5. humanoid robot configuration 
    robot: ArticulationCfg = G1RobotPresets.g1_29dof_inspire_base_fix(init_pos=(-4.2, -3.7, 0.76),
        init_rot=(0, 0, -0.7071, 0.7071))


    # 6. add camera configuration 
    front_camera = CameraPresets.g1_front_camera()
    front_right_camera = CameraPresets.g1_front_right_camera()
    left_wrist_camera = CameraPresets.left_inspire_wrist_camera(calibrated=True)
    right_wrist_camera = CameraPresets.right_inspire_wrist_camera(calibrated=True)

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
    """Observation specifications for the MDP."""
    """defines all available observation information
    """
    @configclass
    class PolicyCfg(ObsGroup):
        """policy group observation configuration class
        defines all state observation values for policy decision
        inherit from ObsGroup base class 
        """
        # 1. robot joint state observation
        robot_joint_state = ObsTerm(func=mdp.get_robot_boy_joint_states)
        # 2. gripper joint state observation 
        robot_inspire_state = ObsTerm(func=mdp.get_robot_inspire_joint_states)

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
    reset_red_block = EventTermCfg(
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
            # specify the object to reset
            "asset_cfg": SceneEntityCfg("red_block"),
        },
    )
    reset_yellow_block = EventTermCfg(
        func=mdp.reset_root_state_uniform,  # use uniform distribution reset function
        mode="reset",   # set event mode to reset
        params={
            "pose_range": {
                "x": [-0.05, 0.05],  # x axis position range: -0.05 to 0.0 meter
                "y": [-0.05, 0.05],   # y axis position range: 0.0 to 0.05 meter
            },
            # speed range parameter (empty dictionary means using default value)
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("yellow_block"),
        },
    )   
    reset_green_block = EventTermCfg(
        func=mdp.reset_root_state_uniform,  # use uniform distribution reset function
        mode="reset",   # set event mode to reset
        params={
            "pose_range": {
                "x": [-0.05, 0.05],  # x axis position range: -0.05 to 0.0 meter
                "y": [-0.05, 0.05],   # y axis position range: 0.0 to 0.05 meter
            },
            # speed range parameter (empty dictionary means using default value)
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("green_block"),
        },
    )   

@configclass
class StackRgyBlockG129InspireBaseFixEnvCfg(UnitreeManagerBasedRLEnvCfg):
    """Unitree G1 robot stack red, yellow, green block environment configuration class
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
        self.decimation = 2
        self.episode_length_s = 20.0
        # simulation settings
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.sim.physics.bounce_threshold_velocity = 0.01
        self.sim.physics.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physics.gpu_total_aggregate_pairs_capacity = 32 * 1024
        self.sim.physics.friction_correlation_distance = 0.003
        self.sim.physics.enable_ccd = True
        self.sim.physics.min_position_iteration_count = 16
        self.sim.physics.min_velocity_iteration_count = 4
        # create event manager
        self.event_manager = SimpleEventManager() 

        self.event_manager.register_multi_object_reset(
            name="reset_object_self",
            object_names=["red_block", "yellow_block", "green_block"],
            pose_ranges={"x": [-0.05, 0.05], "y": [-0.05, 0.05]},  # 所有物体使用相同范围
            velocity_ranges={}
        )
        
        # 方法2：如果需要为不同物体设置不同的重置范围，可以使用这种方式
        # self.event_manager.register("reset_object_self", BatchObjectEvent(
        #     object_names=["red_block", "yellow_block", "green_block"],
        #     pose_ranges={
        #         "red_block": {"x": [-0.05, 0.05], "y": [-0.05, 0.05]},
        #         "yellow_block": {"x": [-0.03, 0.03], "y": [-0.03, 0.03]},
        #         "green_block": {"x": [-0.08, 0.08], "y": [-0.08, 0.08]}
        #     },
        #     velocity_ranges={}
        # ))
        
        # 方法3：使用MultiObjectEvent的详细配置方式
        # self.event_manager.register("reset_object_self", MultiObjectEvent(
        #     reset_configs=[
        #         {
        #             "asset_cfg": SceneEntityCfg("red_block"),
        #             "pose_range": {"x": [-0.05, 0.05], "y": [-0.05, 0.05]},
        #             "velocity_range": {}
        #         },
        #         {
        #             "asset_cfg": SceneEntityCfg("yellow_block"),
        #             "pose_range": {"x": [-0.03, 0.03], "y": [-0.03, 0.03]},
        #             "velocity_range": {}
        #         },
        #         {
        #             "asset_cfg": SceneEntityCfg("green_block"),
        #             "pose_range": {"x": [-0.08, 0.08], "y": [-0.08, 0.08]},
        #             "velocity_range": {}
        #         }
        #     ]
        # ))
        
        self.event_manager.register("reset_all_self", SimpleEvent(
            func=lambda env: base_mdp.reset_scene_to_default(
                env,
                torch.arange(env.num_envs, device=str(env.device)))
        ))