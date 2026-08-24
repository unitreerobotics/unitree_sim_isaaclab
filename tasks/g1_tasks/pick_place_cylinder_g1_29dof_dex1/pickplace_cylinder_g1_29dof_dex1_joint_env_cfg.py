# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
import tempfile
import os
import time
import torch
from dataclasses import MISSING, dataclass

from pink.tasks import FrameTask

import isaaclab.envs.mdp as base_mdp
import isaaclab.sim as sim_utils
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.utils import configclass
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.actuators.actuator_cfg import ImplicitActuatorCfg
from isaaclab.sensors import ContactSensorCfg
from . import mdp
# use Isaac Lab native event system

from tasks.common_config import  G1RobotPresets, CameraPresets  # isort: skip
from tasks.common_event.event_manager import SimpleEvent
# import public scene configuration
from tasks.common_scene.base_scene_randomized_pickplace_cfg import RandomizedRoomPickPlaceSceneCfg
from tasks.utils.room_randomizer import randomize_pickplace_room_layout
from tasks.utils.room_randomizer.room_events import reset_target_on_current_table
from tasks.utils.room_randomizer.pickplace_config import (
    HOSPITAL_TABLE_PROP_NAMES,
    WALL_PROP_NAMES,
    register_randomized_room_reset_events,
)
from tasks.utils.room_randomizer.constants import TABLE_FALLBACK_X, TABLE_FALLBACK_Y
from tasks.common_scene.base_scene_pickplace_cylindercfg import (
    hospital_hand_sanitizer_cfg,
    hospital_medicine_bottle_cfg,
    project_root,
)


RIDGEBACK_USD = (
    f"{project_root}/assets/robots/ridgeback_base_only.usda"
)

FIXED_TELEOP_TABLE_POS = (TABLE_FALLBACK_X, TABLE_FALLBACK_Y, -0.2)


@dataclass
class RidgebackAssistantState:
    phase: str = "waiting"
    grasp_candidate: tuple[str, str] | None = None
    grasp_since: float | None = None
    grasp_object_name: str | None = None
    placement_since: float | None = None
    demo_side: str | None = None
    demo_at: float | None = None


def _require_single_teleop_env(env) -> None:
    if env.num_envs != 1:
        raise ValueError(
            "DDS teleoperation and the Ridgeback assistant require num_envs == 1; "
            f"received {env.num_envs}"
        )


def _assistant_state(env, env_id: int = 0) -> RidgebackAssistantState:
    states = getattr(env, "_ridgeback_assistant_states", None)
    if states is None or env_id not in states:
        raise RuntimeError("Ridgeback assistant has not been initialized by the full reset")
    return states[env_id]


def reset_ridgeback_assistant(
    env, env_ids: torch.Tensor | None, asset_cfg: SceneEntityCfg = SceneEntityCfg("ridgeback")
):
    """Return Ridgeback to its behind-G1 waiting pose and clear its state machine."""
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    ridgeback = env.scene[asset_cfg.name]
    joint_pos = ridgeback.data.default_joint_pos[env_ids].clone()
    joint_vel = torch.zeros_like(joint_pos)
    ridgeback.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)
    ridgeback.set_joint_position_target(joint_pos, env_ids=env_ids)
    ridgeback.set_joint_velocity_target(joint_vel, env_ids=env_ids)
    states = getattr(env, "_ridgeback_assistant_states", {})
    demo_side = os.getenv("RIDGEBACK_ASSISTANT_DEMO", "").strip().lower()
    demo_side = demo_side if demo_side in ("left", "right") else None
    for env_id_tensor in env_ids:
        env_id = int(env_id_tensor.item())
        states[env_id] = RidgebackAssistantState(
            demo_side=demo_side,
            demo_at=time.monotonic() + 3.0 if demo_side else None,
        )
    env._ridgeback_assistant_states = states
    env._ridgeback_assistant_error_reported = False
    print("[ridgeback assistant] reset -> waiting behind G1", flush=True)


def update_ridgeback_assistant(env):
    """Drive the assistant through robot-local targets from the room layout state."""
    _require_single_teleop_env(env)
    ridgeback = env.scene["ridgeback"]
    robot = env.scene["robot"]
    graspable_names = ("object", "hand_sanitizer", "gauze_box", "specimen_cup")
    layout_states = getattr(env, "_room_layout_state", None)
    if not layout_states or 0 not in layout_states:
        raise RuntimeError("Ridgeback assistant requires the current randomized room layout state")
    layout = layout_states[0]
    assistant = _assistant_state(env)

    if not hasattr(env, "_ridgeback_left_hand_id"):
        left_ids, _ = robot.find_bodies("left_hand_base_link")
        right_ids, _ = robot.find_bodies("right_hand_base_link")
        env._ridgeback_left_hand_id = int(left_ids[0])
        env._ridgeback_right_hand_id = int(right_ids[0])
        ridgeback_base_ids, _ = ridgeback.find_bodies("base_link")
        env._ridgeback_base_body_id = int(ridgeback_base_ids[0])
        print("[ridgeback assistant] active; waiting for a stable grasp", flush=True)

    phase = assistant.phase
    if phase == "waiting":
        authored_left = robot.data.body_pos_w[0, env._ridgeback_left_hand_id]
        authored_right = robot.data.body_pos_w[0, env._ridgeback_right_hand_id]
        robot_x, robot_y, _ = layout.robot_pose.position
        cos_yaw = torch.cos(torch.tensor(layout.robot_pose.yaw, device=env.device))
        sin_yaw = torch.sin(torch.tensor(layout.robot_pose.yaw, device=env.device))

        def local_lateral(hand_pos):
            dx = hand_pos[0] - robot_x
            dy = hand_pos[1] - robot_y
            return -sin_yaw * dx + cos_yaw * dy

        # Resolve handedness in the randomized robot frame.  Positive local Y
        # is left, irrespective of the selected wall or world orientation.
        if float(local_lateral(authored_left)) >= float(local_lateral(authored_right)):
            hands = {"left": authored_left, "right": authored_right}
        else:
            hands = {"left": authored_right, "right": authored_left}
        candidate = None
        candidate_distance = float("inf")
        for object_name in graspable_names:
            graspable = env.scene[object_name]
            object_pos = graspable.data.root_pos_w[0]
            placement = layout.tabletop_placements.get(object_name)
            if placement is None:
                continue
            initial_z = layout.packing_table_pose.position[2] + placement.local_pose[2]
            left_dist = float(torch.linalg.vector_norm(object_pos - hands["left"]))
            right_dist = float(torch.linalg.vector_norm(object_pos - hands["right"]))
            nearest = "left" if left_dist < right_dist else "right"
            nearest_dist = min(left_dist, right_dist)
            # Require a real, sustained pickup.  This rejects a hand merely
            # passing near an object and small collision/spawn disturbances.
            lifted = float(object_pos[2]) > initial_z + 0.080
            if lifted and nearest_dist < 0.18 and nearest_dist < candidate_distance:
                candidate = (nearest, object_name)
                candidate_distance = nearest_dist
        now = time.monotonic()
        if assistant.demo_side and assistant.demo_at is not None and now >= assistant.demo_at:
            candidate = (assistant.demo_side, "object")
            assistant.grasp_candidate = candidate
            assistant.grasp_since = now - 1.0
            assistant.demo_side = None
            print(f"[ridgeback assistant demo] simulating {candidate[0]}-hand grasp", flush=True)
        if candidate != assistant.grasp_candidate:
            assistant.grasp_candidate = candidate
            assistant.grasp_since = now if candidate else None
        elif candidate and assistant.grasp_since is not None and now - assistant.grasp_since >= 0.35:
            candidate_side, candidate_object = candidate
            assistant.grasp_object_name = candidate_object
            assistant.phase = "staging"
            phase = "staging"
            print(
                f"[ridgeback assistant] {candidate_object} in {candidate_side} hand confirmed; "
                f"approaching {candidate_side} side",
                flush=True,
            )

    if phase in ("staging", "side"):
        side = assistant.grasp_candidate[0]
        target_name = f"staging_{side}" if phase == "staging" else f"delivery_{side}"
        target = torch.tensor([layout.ridgeback_joint_targets[target_name]], device=env.device)
        ridgeback.set_joint_position_target(target)
        current = ridgeback.data.joint_pos[0]
        if float(torch.max(torch.abs(current - target[0]))) < 0.10:
            if phase == "staging":
                assistant.phase = "side"
                print("[ridgeback assistant] staging point reached; moving beside G1", flush=True)
            else:
                assistant.phase = "holding"
                assistant.placement_since = None
                ridgeback.set_joint_position_target(target)
                ridgeback.set_joint_velocity_target(torch.zeros_like(target))
                print("[ridgeback assistant] side delivery pose reached; holding", flush=True)
    elif phase == "holding":
        side = assistant.grasp_candidate[0]
        target = torch.tensor(
            [layout.ridgeback_joint_targets[f"delivery_{side}"]], device=env.device
        )
        # Keep the chassis fully stationary beside G1.  Returning is unlocked
        # only after the real object has remained inside the basket.
        ridgeback.set_joint_position_target(target)
        ridgeback.set_joint_velocity_target(torch.zeros_like(target))
        now = time.monotonic()
        base_pos = ridgeback.data.body_pos_w[0, env._ridgeback_base_body_id]
        base_quat = ridgeback.data.body_quat_w[0, env._ridgeback_base_body_id]
        carried_object = env.scene[assistant.grasp_object_name or "object"]
        object_pos = carried_object.data.root_pos_w[0]
        delta_x = object_pos[0] - base_pos[0]
        delta_y = object_pos[1] - base_pos[1]
        # Convert the object position into Ridgeback's local frame.  The basket
        # is 0.47 x 0.31 m internally and follows base_link yaw.
        w, x, y, z = base_quat
        base_yaw = torch.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        cos_yaw = torch.cos(base_yaw)
        sin_yaw = torch.sin(base_yaw)
        local_x = cos_yaw * delta_x + sin_yaw * delta_y
        local_y = -sin_yaw * delta_x + cos_yaw * delta_y
        relative_height = float(object_pos[2] - base_pos[2])
        object_speed = float(torch.linalg.vector_norm(carried_object.data.root_lin_vel_w[0]))
        left_pos = robot.data.body_pos_w[0, env._ridgeback_left_hand_id]
        right_pos = robot.data.body_pos_w[0, env._ridgeback_right_hand_id]
        hand_distance = min(
            float(torch.linalg.vector_norm(object_pos - left_pos)),
            float(torch.linalg.vector_norm(object_pos - right_pos)),
        )
        # Bottom top is z=0.335 and rim top is z=0.515 relative to base_link.
        # Margins reject objects resting on a wall/rim instead of inside it.
        in_basket = (
            abs(float(local_x)) < 0.205
            and abs(float(local_y)) < 0.125
            and 0.345 < relative_height < 0.500
            and object_speed < 0.12
            and hand_distance > 0.16
        )
        demo_placed = False
        if in_basket:
            if assistant.placement_since is None:
                # Demo has already waited three seconds; real placement still
                # needs a stability window to reject momentary fly-through.
                assistant.placement_since = now - (1.0 if demo_placed else 0.0)
            elif now - assistant.placement_since >= 0.80:
                assistant.phase = "returning_side"
                print("[ridgeback assistant] placement confirmed; returning behind G1", flush=True)
        else:
            assistant.placement_since = None
    elif phase in ("returning_side", "returning_home"):
        side = assistant.grasp_candidate[0]
        if phase == "returning_side":
            target = torch.tensor(
                [layout.ridgeback_joint_targets[f"staging_{side}"]], device=env.device
            )
        else:
            target = torch.tensor([layout.ridgeback_joint_targets["waiting"]], device=env.device)
        ridgeback.set_joint_position_target(target)
        current = ridgeback.data.joint_pos[0]
        if float(torch.max(torch.abs(current - target[0]))) < 0.10:
            if phase == "returning_side":
                assistant.phase = "returning_home"
                print("[ridgeback assistant] cleared G1; returning to home pose", flush=True)
            else:
                object_name = assistant.grasp_object_name
                if object_name:
                    reset_target_on_current_table(
                        env, torch.tensor([0], device=env.device), asset_name=object_name
                    )
                    print(
                        f"[ridgeback assistant] {object_name} reset to tabletop for next cycle",
                        flush=True,
                    )
                assistant.phase = "waiting"
                assistant.grasp_candidate = None
                assistant.grasp_since = None
                assistant.grasp_object_name = None
                assistant.placement_since = None
                ridgeback.set_joint_position_target(target)
                print("[ridgeback assistant] home behind G1; waiting", flush=True)


def respawn_dropped_object(
    env,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    floor_height: float = 0.32,
):
    """Respawn only the bottle after it falls to the floor.

    The Ridgeback basket is lower than the table, so the threshold deliberately
    sits below the basket instead of treating every off-table placement as a
    failure.
    """
    object_asset = env.scene[asset_cfg.name]
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    root_pos = object_asset.data.root_pos_w[env_ids]
    ridgeback = env.scene["ridgeback"]
    base_ids, _ = ridgeback.find_bodies("base_link")
    ridgeback_pos = ridgeback.data.body_pos_w[env_ids, int(base_ids[0])]

    # A bottle is successful when it is in the Ridgeback basket, even though
    # the basket is lower than the table.  Everything near floor height is a
    # genuine drop.  The radial check also catches a bottle that comes to rest
    # on low hospital geometry instead of reaching the collision ground plane.
    basket_xy_distance = torch.linalg.vector_norm(root_pos[:, :2] - ridgeback_pos[:, :2], dim=1)
    in_basket = (basket_xy_distance < 0.42) & (root_pos[:, 2] > 0.30)
    low_or_lost = (root_pos[:, 2] < floor_height) | (
        (root_pos[:, 2] < 0.46) & ~in_basket
    )
    invalid = ~torch.isfinite(root_pos).all(dim=1)
    dropped = low_or_lost | invalid
    dropped_ids = env_ids[dropped]
    if len(dropped_ids) == 0:
        return

    before = root_pos[dropped].clone()
    reset_target_on_current_table(env, dropped_ids, asset_name=asset_cfg.name)
    after = object_asset.data.root_pos_w[dropped_ids]
    print(
        "[bottle respawn] dropped/lost: "
        f"{before[0].tolist()} -> {after[0].tolist()}",
        flush=True,
    )


def never_terminate(env) -> torch.Tensor:
    """Keep teleoperation running; dropped bottles are handled independently."""
    return torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)


def reset_all_teleop_scene(
    env,
    env_ids: torch.Tensor | None,
    randomize_table_position: bool | None = None,
):
    """Reset teleoperation and optionally change the persistent table switch."""
    if randomize_table_position is not None:
        env._teleop_randomize_table_position = bool(randomize_table_position)
    randomize_table_position = bool(
        getattr(env, "_teleop_randomize_table_position", False)
    )
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    base_mdp.reset_scene_to_default(env, env_ids)
    randomize_pickplace_room_layout(
        env,
        env_ids,
        wall_prop_names=WALL_PROP_NAMES,
        table_prop_names=HOSPITAL_TABLE_PROP_NAMES,
        min_table_objects=len(HOSPITAL_TABLE_PROP_NAMES),
        randomize_table_position=randomize_table_position,
    )
    reset_ridgeback_assistant(env, env_ids)
    mode = "full randomization" if randomize_table_position else "fixed table"
    print(f"[reset all] teleoperation scene restored ({mode})", flush=True)


##
# Scene definition
##

@configclass
class ObjectTableSceneCfg(RandomizedRoomPickPlaceSceneCfg):
    """object table scene configuration class
    inherits from G1SingleObjectSceneCfg, gets the complete G1 robot scene configuration
    can add task-specific scene elements or override default configurations here
    """
    
    # Humanoid robot w/ arms higher
    # 5. humanoid robot configuration 
    robot: ArticulationCfg = G1RobotPresets.g1_29dof_dex1_base_fix()
    # Preserve Shidan's medicine-bottle physics and mass configuration while
    # the shared randomizer owns its table-local pose.
    object: RigidObjectCfg = hospital_medicine_bottle_cfg()
    # The stock fixed-base asset hard-locks every waist joint with zero velocity
    # and kp/kd=10000.  Release yaw for VR torso turning while keeping waist
    # roll/pitch and the complete lower body fixed.
    robot.actuators.pop("waist", None)
    robot.actuators["waist_yaw_teleop"] = ImplicitActuatorCfg(
        joint_names_expr=["waist_yaw_joint"],
        effort_limit_sim=350.0,
        velocity_limit_sim=2.5,
        stiffness=260.0,
        damping=18.0,
    )
    robot.actuators["waist_roll_pitch_lock"] = ImplicitActuatorCfg(
        joint_names_expr=["waist_roll_joint", "waist_pitch_joint"],
        effort_limit_sim=1000.0,
        velocity_limit_sim=0.1,
        stiffness=10000.0,
        damping=10000.0,
    )

    ridgeback: ArticulationCfg = ArticulationCfg(
        prim_path="/World/envs/env_.*/Ridgeback",
        spawn=sim_utils.UsdFileCfg(
            usd_path=RIDGEBACK_USD,
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                articulation_enabled=True,
                enabled_self_collisions=False,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            # Fixed articulation root waits behind G1.  Planar joints move the
            # chassis from here when the assistant state machine is triggered.
            # About 1.70 m behind G1 while no object is being carried.
            pos=(-0.15, -1.80, 0.0328),
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
        actuators={
            "base_translation": ImplicitActuatorCfg(
                joint_names_expr=["dummy_base_prismatic_.*_joint"],
                effort_limit_sim=1600.0,
                velocity_limit_sim=0.55,
                stiffness=500.0,
                damping=90.0,
            ),
            "base_yaw": ImplicitActuatorCfg(
                joint_names_expr=["dummy_base_revolute_z_joint"],
                effort_limit_sim=900.0,
                velocity_limit_sim=0.75,
                stiffness=320.0,
                damping=55.0,
            ),
        },
    )

    # Open collision basket mounted directly to the Ridgeback chassis.  Since
    # it is a child of base_link it follows the randomized chassis pose.
    ridgeback_basket = AssetBaseCfg(
        prim_path="/World/envs/env_.*/Ridgeback/base_link/Basket",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/ridgeback_basket.usda",
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.31),
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
    )

    # Additional hospital tabletop props.  They are separate rigid bodies, so
    # they provide meaningful visual clutter and can also be grasped or moved.
    hand_sanitizer: RigidObjectCfg = hospital_hand_sanitizer_cfg(
        # Rightmost item in the front 1x4 row. Its geometry still ends before
        # the built-in container's x~=0.26 left boundary.
        init_pos=(0.07, 0.40, 0.875),
    )
    gauze_box = RigidObjectCfg(
        prim_path="/World/envs/env_.*/GauzeBox",
        init_state=RigidObjectCfg.InitialStateCfg(
            # Third item in the front 1x4 row.
            pos=(-0.18, 0.40, 0.838), rot=(1.0, 0.0, 0.0, 0.0)
        ),
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/hospital_gauze_box.usda",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                linear_damping=1.5, angular_damping=3.0,
                max_linear_velocity=5.0, max_angular_velocity=10.0,
                max_depenetration_velocity=0.25,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.10),
        ),
    )
    specimen_cup = RigidObjectCfg(
        prim_path="/World/envs/env_.*/SpecimenCup",
        init_state=RigidObjectCfg.InitialStateCfg(
            # Second item in the front 1x4 row.
            pos=(-0.43, 0.40, 0.845), rot=(1.0, 0.0, 0.0, 0.0)
        ),
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/hospital_specimen_cup.usda",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                linear_damping=1.5, angular_damping=3.0,
                max_linear_velocity=5.0, max_angular_velocity=10.0,
                max_depenetration_velocity=0.25,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.08),
        ),
    )


    # 6. add camera configuration 
    front_camera = CameraPresets.g1_front_camera()
    left_wrist_camera = CameraPresets.left_gripper_wrist_camera()
    right_wrist_camera = CameraPresets.right_gripper_wrist_camera()

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
    """
    defines all available observation information
    """
    @configclass
    class PolicyCfg(ObsGroup):
        """policy group observation configuration class
        defines all state observation values for policy decision
        inherit from ObsGroup base class 
        """

        robot_joint_state = ObsTerm(func=mdp.get_robot_boy_joint_states)
        robot_gipper_state = ObsTerm(func=mdp.get_robot_gipper_joint_states)

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
    # Teleoperation must not reset G1/Ridgeback when only the bottle falls.
    success = DoneTerm(func=never_terminate)

@configclass
class RewardsCfg:
    reward = RewTerm(func=mdp.compute_reward,weight=1.0)

@configclass
class EventCfg:
    reset_teleop_scene = EventTermCfg(
        func=reset_all_teleop_scene,
        mode="reset",
    )
    respawn_dropped_bottle = EventTermCfg(
        func=respawn_dropped_object,
        mode="interval",
        interval_range_s=(0.10, 0.10),
        is_global_time=True,
        params={"asset_cfg": SceneEntityCfg("object"), "floor_height": 0.32},
    )


@configclass
class PickPlaceG129DEX1BaseFixEnvCfg(ManagerBasedRLEnvCfg):
    """
    inherits from ManagerBasedRLEnvCfg, defines all configuration parameters for the entire environment
    """

    # 1. scene settings
    scene: ObjectTableSceneCfg = ObjectTableSceneCfg(num_envs=1, # environment number: 1
                                                     env_spacing=16.0, # hospital room footprint needs wider spacing
                                                     replicate_physics=True # enable physics replication
                                                     )
    # basic settings
    observations: ObservationsCfg = ObservationsCfg()   # observation configuration
    actions: ActionsCfg = ActionsCfg()                  # action configuration
    # MDP settings

    terminations: TerminationsCfg = TerminationsCfg()    # termination configuration
    events = EventCfg()                                  # event configuration
    commands = None # command manager
    rewards: RewardsCfg = RewardsCfg()  # reward manager
    curriculum = None # curriculum manager
    def __post_init__(self):
        """Post initialization."""
        # Isaac Lab's configclass stores inherited scene fields on the scene
        # instance, not as accessible base-class attributes.
        self.scene.packing_table.init_state.pos = FIXED_TELEOP_TABLE_POS
        # general settings
        self.decimation = 2
        self.episode_length_s = 20.0
        self.viewer.origin_type = "world"
        self.viewer.eye = (-7.5, -3.2, 4.2)
        self.viewer.lookat = (-7.5, -7.6, 0.8)
        # simulation settings
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625
        # Hospital props must remain controllable in the Dex1 parallel jaws.
        # Use the stronger material in every contact pair and eliminate bounce.
        self.sim.physics_material.static_friction = 2.8
        self.sim.physics_material.dynamic_friction = 2.4
        self.sim.physics_material.restitution = 0.0
        self.sim.physics_material.friction_combine_mode = "max"
        self.sim.physics_material.restitution_combine_mode = "min"
        # create event manager
        register_randomized_room_reset_events(self)

        # Preserve the teleoperation-specific manual resets while retaining the
        # target branch's native room-randomization event above.
        self.event_manager.register("reset_object_self", SimpleEvent(
            func=lambda env: reset_target_on_current_table(
                env, torch.arange(env.num_envs, device=env.device)
            )
        ))

        self.event_manager.register("reset_all_self", SimpleEvent(
            # Quest/xr_teleoperate's full-reset button sends DDS category 2.
            # That explicit action opts back into table-group randomization.
            func=lambda env: reset_all_teleop_scene(
                env, None, randomize_table_position=True
            )
        ))
        self.event_manager.register(
            "reset_room_fixed_table_self",
            SimpleEvent(
                func=lambda env: reset_all_teleop_scene(
                    env, None, randomize_table_position=False
                )
            ),
        )


@configclass
class ObjectTableWholebodySceneCfg(ObjectTableSceneCfg):
    """Hospital scene with the free-root G1 used by the locomotion policy."""

    robot: ArticulationCfg = G1RobotPresets.g1_29dof_dex1_wholebody(
        init_pos=(-0.15, -0.10, 0.8),
        # Preserve the task's calibrated +90-degree yaw toward the table.
        init_rot=(0.7071, 0.0, 0.0, 0.7071),
    )
    contact_forces = ContactSensorCfg(
        prim_path="/World/envs/env_.*/Robot/.*",
        history_length=10,
        track_air_time=True,
        debug_vis=False,
    )


@configclass
class PickPlaceHospitalG129DEX1WholebodyEnvCfg(PickPlaceG129DEX1BaseFixEnvCfg):
    """Movable G1 variant retaining the hospital task and Ridgeback basket."""

    scene: ObjectTableWholebodySceneCfg = ObjectTableWholebodySceneCfg(
        num_envs=1,
        env_spacing=16.0,
        replicate_physics=True,
    )

    def __post_init__(self):
        super().__post_init__()
        self.decimation = 4
        self.sim.render_interval = self.decimation
        self.scene.contact_forces.update_period = self.sim.dt
