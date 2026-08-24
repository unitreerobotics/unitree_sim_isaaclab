# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
Hospital-room medicine-bottle pick-place task for G1 + Dex1 joint control.

This is an *independent* task: it does NOT modify any existing warehouse task.
The scene reuses the hospital RoomShell (new_base_room.usda) from
RandomizedRoomPickPlaceSceneCfg. The calibrated task group stays fixed while
the wall props use the shared room randomizer:

  * table / robot / object poses reuse the known-good warehouse manipulation geometry
    translated by T = (-1.7, -3.3, 0) so the table sits in the open hospital
    interior at (-6.0, -7.5). That preserves the robot<->table<->object relative
    poses the IK, cameras and grasp are already calibrated for.
  * wall props are spawned separately and placed from
    tasks/utils/room_randomizer/constants.py on every full reset.

Tunable numbers that may need a visual pass in Isaac Sim are tagged  # TUNE.
"""

from __future__ import annotations

import os
from collections.abc import Callable
from dataclasses import dataclass

import torch
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

import isaaclab.sim as sim_utils
import isaaclab.envs.mdp as base_mdp
from isaaclab.sim import schemas
from isaaclab.sim.spawners.from_files import from_files as file_spawners
from isaaclab.sim.utils import bind_physics_material, clone, get_current_stage
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.actuators.actuator_cfg import ImplicitActuatorCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass

from . import mdp
from tasks.common_config import CameraBaseCfg, CameraPresets, G1RobotPresets  # isort: skip
from tasks.common_event.event_manager import SimpleEvent, SimpleEventManager
from tasks.common_scene.base_scene_randomized_pickplace_cfg import (
    RandomizedRoomPickPlaceSceneCfg,
)
from tasks.utils.room_randomizer import (
    StaticClusterMember,
    TabletopSpawnRegion,
    randomize_pickplace_room_layout,
)
from tasks.utils.room_randomizer.room_events import (
    Pose2D,
    reset_target_on_current_table,
)
from tasks.utils.room_randomizer.constants import (
    BBox,
    RIDGEBACK_CONTAINER_RISER_HEIGHT,
    TablePropMeta,
    ridgeback_static_arc_pose,
)
from tasks.utils.room_randomizer.placement_utils import (
    build_root_state,
    offset_from_yaw_batched,
)
from tasks.utils.room_randomizer.pickplace_config import (
    WALL_PROP_NAMES,
)
from tools.camera_optics import HOSPITAL_FRONT_CAMERA_OPTICS
from tools.medical_object_catalog import (
    MEDICAL_OBJECT_SPECS,
    OBJECT_ROLES_ENV,
    ROLE_DISTRACTOR,
    ROLE_IMPORTANT,
    parse_roles,
)

project_root = os.environ.get("PROJECT_ROOT")

MEDICAL_OBJECT_ROLES = parse_roles(os.environ.get(OBJECT_ROLES_ENV))
IMPORTANT_MEDICAL_OBJECT_NAMES = tuple(
    spec.scene_name
    for spec in MEDICAL_OBJECT_SPECS
    if MEDICAL_OBJECT_ROLES[spec.scene_name] == ROLE_IMPORTANT
)
DISTRACTOR_MEDICAL_OBJECT_NAMES = tuple(
    spec.scene_name
    for spec in MEDICAL_OBJECT_SPECS
    if MEDICAL_OBJECT_ROLES[spec.scene_name] == ROLE_DISTRACTOR
)
ACTIVE_MEDICAL_OBJECT_NAMES = (
    IMPORTANT_MEDICAL_OBJECT_NAMES + DISTRACTOR_MEDICAL_OBJECT_NAMES
)

RIDGEBACK_OMNIVERSE_USD = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/"
    "Isaac/Robots/Clearpath/RidgebackUr/ridgeback_ur5.usd"
)
CRATE_OMNIVERSE_USD = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/"
    "Isaac/Environments/Simple_Warehouse/Props/SM_CratePlastic_D_02.usd"
)
RIDGEBACK_DISABLED_PRIM_NAMES = {
    "ur_arm_shoulder_pan_joint",
    "ur_arm_shoulder_link",
    "ur_arm_upper_arm_link",
    "ur_arm_forearm_link",
    "ur_arm_wrist_1_link",
    "ur_arm_wrist_2_link",
    "ur_arm_wrist_3_link",
}

# The Ridgeback body is turned 180 degrees.  Its crate and riser are
# counter-transformed in base_link so they keep their original world pose
# relative to G1 and the room.
RIDGEBACK_BODY_YAW_OFFSET = torch.pi
CRATE_LOCAL_ROT = (0.7046465942, 0.0, 0.0, 0.7095584382)
CRATE_RISER_SIZE = (0.46, 0.35, RIDGEBACK_CONTAINER_RISER_HEIGHT)
RIDGEBACK_BODY_COLOR = (0.15, 0.15, 0.15)
# The original crate floor was at 28.576 cm above Ridgeback's base_link.  The
# riser sits below it and raises the entire container by its configured height.
CRATE_LOCAL_POS = (-0.22877, -0.00612, 0.43576)
CRATE_RISER_LOCAL_POS = (
    CRATE_LOCAL_POS[0],
    CRATE_LOCAL_POS[1],
    CRATE_LOCAL_POS[2] - RIDGEBACK_CONTAINER_RISER_HEIGHT * 0.5,
)


# ----------------------------------------------------------------------------
# Task-local medical asset policy.
# ----------------------------------------------------------------------------
GRASP_PHYSICS_MATERIAL = sim_utils.RigidBodyMaterialCfg(
    friction_combine_mode="max",
    restitution_combine_mode="min",
    static_friction=2.5,
    dynamic_friction=2.0,
    restitution=0.0,
)

FINGER_PHYSICS_MATERIAL = sim_utils.RigidBodyMaterialCfg(
    friction_combine_mode="max",
    restitution_combine_mode="min",
    static_friction=3.0,
    dynamic_friction=2.0,
    restitution=0.0,
)
FINGER_COLLISION_PROPERTIES = sim_utils.CollisionPropertiesCfg(
    collision_enabled=True,
    contact_offset=0.001,
    rest_offset=0.0,
)
PILL_GRASP_PAD_RADII = {"left": 0.009, "right": 0.010}
DEX1_OPEN_JOINT_POSITION = -0.02
DISTAL_COLLIDER_LOCAL_CENTERS = {
    "Link1_3": (0.001997, 0.022747, -0.014200),
    "Link2_3": (-0.001997, 0.022747, -0.014200),
}


@clone
def _spawn_grasp_ready_dex1_usd(
    prim_path: str,
    cfg,
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
    **kwargs,
) -> Usd.Prim:
    """Spawn Dex1 and harden its exact finger hulls for manipulation."""
    root_prim = file_spawners._spawn_from_usd_file(
        prim_path, cfg.usd_path, cfg, translation, orientation
    )
    stage = get_current_stage()
    material_path = f"{prim_path}/GraspFingerPhysicsMaterial"
    FINGER_PHYSICS_MATERIAL.func(material_path, FINGER_PHYSICS_MATERIAL)

    # The Dex1 USD stores each exact finger hull in an instanced collision
    # branch. Make those branches editable so their contact properties and
    # materials can be overridden without changing the shared robot asset.
    instance_paths = [
        str(child.GetPath())
        for child in Usd.PrimRange(root_prim)
        if child.IsInstance()
        and (
            "/left_hand_Link" in str(child.GetPath())
            or "/right_hand_Link" in str(child.GetPath())
        )
    ]
    for child_path in instance_paths:
        stage.GetPrimAtPath(child_path).SetInstanceable(False)

    finger_branch_paths = [
        str(child.GetPath())
        for child in Usd.PrimRange(stage.GetPrimAtPath(prim_path))
        if child.GetName() == "collisions"
        and (
            "/left_hand_Link" in str(child.GetPath())
            or "/right_hand_Link" in str(child.GetPath())
        )
    ]

    finger_collider_paths: list[str] = []
    for child_path in finger_branch_paths:
        child = stage.GetPrimAtPath(child_path)

        branch_meshes = [
            descendant
            for descendant in Usd.PrimRange(child)
            if descendant.IsA(UsdGeom.Mesh)
        ]
        if len(branch_meshes) != 1:
            raise ValueError(
                f"Dex1 finger collision branch {child_path!r} must contain exactly one mesh; "
                f"found {[str(mesh.GetPath()) for mesh in branch_meshes]}"
            )

        mesh_prim = branch_meshes[0]
        if mesh_prim.IsInstanceProxy():
            raise ValueError(
                f"Dex1 finger collider remained instanced: {mesh_prim.GetPath()}"
            )

        UsdPhysics.CollisionAPI.Apply(child).CreateCollisionEnabledAttr().Set(True)
        UsdPhysics.MeshCollisionAPI.Apply(child).CreateApproximationAttr().Set(
            UsdPhysics.Tokens.convexHull
        )
        PhysxSchema.PhysxCollisionAPI.Apply(child)
        finger_collider_paths.append(child_path)

    per_hand_counts = {
        side: sum(f"/{side}_hand_Link" in path for path in finger_collider_paths)
        for side in ("left", "right")
    }
    if per_hand_counts != {"left": 6, "right": 6}:
        raise ValueError(
            f"Dex1 must expose six exact collision hulls per hand; found {per_hand_counts}"
        )

    # Natural-scale pill bottles are narrower than the stock Dex1 aperture at
    # its mapped full-close target. Add invisible spherical pads at the exact
    # distal-collider centers so the existing command can pinch them without
    # resizing either source asset or changing any controller mapping.
    grasp_pad_paths: list[str] = []
    for side in ("left", "right"):
        for link_name, local_center in DISTAL_COLLIDER_LOCAL_CENTERS.items():
            pad_path = f"{prim_path}/{side}_hand_{link_name}/PillGraspPad"
            pad = UsdGeom.Sphere.Define(stage, pad_path)
            pad.CreateRadiusAttr().Set(PILL_GRASP_PAD_RADII[side])
            UsdGeom.Xformable(pad.GetPrim()).AddTranslateOp().Set(
                Gf.Vec3d(*local_center)
            )
            UsdPhysics.CollisionAPI.Apply(pad.GetPrim()).CreateCollisionEnabledAttr().Set(
                True
            )
            PhysxSchema.PhysxCollisionAPI.Apply(pad.GetPrim())
            grasp_pad_paths.append(pad_path)

    for collider_path in finger_collider_paths + grasp_pad_paths:
        schemas.modify_collision_properties(
            collider_path, FINGER_COLLISION_PROPERTIES, stage=stage
        )
        bind_physics_material(collider_path, material_path, stage=stage)

    return root_prim


@clone
def _spawn_graspable_hospital_usd(
    prim_path: str,
    cfg,
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
    **kwargs,
) -> Usd.Prim:
    """Turn one NVIDIA prop mesh into a convex-decomposed dynamic rigid body."""
    spawn_cfg = cfg.copy()
    spawn_cfg.rigid_props = None
    spawn_cfg.collision_props = None
    spawn_cfg.mass_props = None
    root_prim = file_spawners._spawn_from_usd_file(
        prim_path, cfg.usd_path, spawn_cfg, translation, orientation
    )
    stage = get_current_stage()

    collider_prims = [
        child for child in Usd.PrimRange(root_prim) if child.IsA(UsdGeom.Mesh)
    ]
    if len(collider_prims) != 1:
        raise ValueError(
            f"Hospital grasp prop {cfg.usd_path!r} must contain exactly one mesh; "
            f"found {[str(child.GetPath()) for child in collider_prims]}"
        )

    collider_prim = collider_prims[0]
    UsdPhysics.CollisionAPI.Apply(collider_prim).CreateCollisionEnabledAttr().Set(True)
    UsdPhysics.MeshCollisionAPI.Apply(collider_prim).CreateApproximationAttr().Set(
        UsdPhysics.Tokens.convexDecomposition
    )
    PhysxSchema.PhysxCollisionAPI.Apply(collider_prim)
    collider_path = str(collider_prim.GetPath())

    if cfg.rigid_props is not None:
        schemas.define_rigid_body_properties(prim_path, cfg.rigid_props, stage=stage)
    if cfg.mass_props is not None:
        schemas.define_mass_properties(prim_path, cfg.mass_props, stage=stage)

    rigid_roots = [
        child
        for child in Usd.PrimRange(root_prim)
        if child.HasAPI(UsdPhysics.RigidBodyAPI)
    ]
    authored_mass = UsdPhysics.MassAPI(root_prim).GetMassAttr().Get()
    if rigid_roots != [root_prim] or authored_mass is None or float(authored_mass) <= 0.0:
        raise ValueError(
            f"Hospital grasp prop {cfg.usd_path!r} requires one positive-mass root rigid body"
        )

    material_path = f"{prim_path}/GraspPhysicsMaterial"
    GRASP_PHYSICS_MATERIAL.func(material_path, GRASP_PHYSICS_MATERIAL)
    if cfg.collision_props is not None:
        schemas.modify_collision_properties(
            collider_path, cfg.collision_props, stage=stage
        )
    bind_physics_material(collider_path, material_path, stage=stage)

    return root_prim


@configclass
class GraspableHospitalUsdFileCfg(sim_utils.UsdFileCfg):
    """USD spawner configuration for convex-decomposed NVIDIA grasp props."""

    func: Callable = _spawn_graspable_hospital_usd


@clone
def _spawn_static_ridgeback_usd(
    prim_path: str,
    cfg,
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
    **kwargs,
) -> Usd.Prim:
    """Import the Omniverse Ridgeback mesh as one base-only kinematic body."""
    spawn_cfg = cfg.copy()
    spawn_cfg.rigid_props = None
    spawn_cfg.collision_props = None
    root_prim = file_spawners._spawn_from_usd_file(
        prim_path, cfg.usd_path, spawn_cfg, translation, orientation
    )
    stage = get_current_stage()

    for child in list(Usd.PrimRange(root_prim)):
        if child.IsA(UsdPhysics.Joint):
            # Keep the referenced prim alive but make it a plain transform.
            # Deactivating/removing a joint leaves expired handles in Isaac's
            # stage parser; jointEnabled=false is still parsed as a joint.
            child.SetTypeName("Xform")
        if child.HasAPI(UsdPhysics.ArticulationRootAPI):
            child.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        if child.HasAPI(PhysxSchema.PhysxArticulationAPI):
            child.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
        if child != root_prim:
            if child.HasAPI(UsdPhysics.RigidBodyAPI):
                child.RemoveAPI(UsdPhysics.RigidBodyAPI)
            if child.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
                child.RemoveAPI(PhysxSchema.PhysxRigidBodyAPI)

        ancestor = child
        belongs_to_arm = False
        while ancestor.IsValid() and ancestor != root_prim:
            if ancestor.GetName() in RIDGEBACK_DISABLED_PRIM_NAMES:
                belongs_to_arm = True
                break
            ancestor = ancestor.GetParent()
        if belongs_to_arm:
            imageable = UsdGeom.Imageable(child)
            if imageable:
                imageable.MakeInvisible()
            if child.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(child).CreateCollisionEnabledAttr().Set(False)

    if not any(child.IsA(UsdGeom.Mesh) for child in Usd.PrimRange(root_prim)):
        raise ValueError(f"Ridgeback asset {cfg.usd_path!r} has no active mesh geometry")
    if not any(
        child.HasAPI(UsdPhysics.CollisionAPI) for child in Usd.PrimRange(root_prim)
    ):
        raise ValueError(f"Ridgeback asset {cfg.usd_path!r} has no collision geometry")

    schemas.define_rigid_body_properties(prim_path, cfg.rigid_props, stage=stage)
    if cfg.collision_props is not None:
        schemas.modify_collision_properties(prim_path, cfg.collision_props, stage=stage)
    return root_prim


def _static_ridgeback_cfg(
    prim_path: str,
    pos: tuple[float, float, float],
    rot: tuple[float, float, float, float],
) -> RigidObjectCfg:
    return RigidObjectCfg(
        prim_path=prim_path,
        spawn=sim_utils.UsdFileCfg(
            func=_spawn_static_ridgeback_usd,
            usd_path=RIDGEBACK_OMNIVERSE_USD,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=True,
                kinematic_enabled=True,
                disable_gravity=True,
                linear_damping=10.0,
                angular_damping=10.0,
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
            ),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=pos,
            rot=rot,
        ),
    )


@clone
def _spawn_static_crate_usd(
    prim_path: str,
    cfg,
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
    **kwargs,
) -> Usd.Prim:
    """Import an Omniverse crate as mesh/collision children of its platform."""
    spawn_cfg = cfg.copy()
    spawn_cfg.rigid_props = None
    root_prim = file_spawners._spawn_from_usd_file(
        prim_path, cfg.usd_path, spawn_cfg, translation, orientation
    )

    # The crate is deliberately not a second physics body. Its mesh and
    # collision geometry inherit the Ridgeback base's kinematic transform.
    for child in list(Usd.PrimRange(root_prim)):
        if child.IsA(UsdPhysics.Joint):
            child.SetTypeName("Xform")
        if child.HasAPI(UsdPhysics.ArticulationRootAPI):
            child.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        if child.HasAPI(PhysxSchema.PhysxArticulationAPI):
            child.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
        if child.HasAPI(UsdPhysics.RigidBodyAPI):
            child.RemoveAPI(UsdPhysics.RigidBodyAPI)
        if child.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
            child.RemoveAPI(PhysxSchema.PhysxRigidBodyAPI)

    if not any(child.IsA(UsdGeom.Mesh) for child in Usd.PrimRange(root_prim)):
        raise ValueError(f"Crate asset {cfg.usd_path!r} has no active mesh geometry")
    if not any(
        child.HasAPI(UsdPhysics.CollisionAPI) for child in Usd.PrimRange(root_prim)
    ):
        raise ValueError(f"Crate asset {cfg.usd_path!r} has no collision geometry")
    return root_prim


def _ridgeback_crate_cfg(
    prim_path: str,
    local_x: float,
    local_y: float,
    local_z: float,
    local_rot: tuple[float, float, float, float],
) -> AssetBaseCfg:
    return AssetBaseCfg(
        prim_path=prim_path,
        spawn=sim_utils.UsdFileCfg(
            func=_spawn_static_crate_usd,
            usd_path=CRATE_OMNIVERSE_USD,
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            # The crate's local transform counteracts the reversed Ridgeback,
            # preserving the same world pose beside G1 at every arc point.
            pos=(local_x, local_y, local_z),
            rot=local_rot,
        ),
    )


def _ridgeback_crate_riser_cfg(
    prim_path: str,
    local_pos: tuple[float, float, float],
    local_rot: tuple[float, float, float, float],
) -> AssetBaseCfg:
    """Create the configured-height dark Ridgeback-coloured support below the crate."""
    return AssetBaseCfg(
        prim_path=prim_path,
        spawn=sim_utils.CuboidCfg(
            size=CRATE_RISER_SIZE,
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=RIDGEBACK_BODY_COLOR,
                metallic=0.5,
            ),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=local_pos, rot=local_rot),
    )


@dataclass(frozen=True)
class HospitalPropSpec:
    usd_path: str
    scale: float
    mass: float
    min_z: float
    bbox: BBox


TABLETOP_SURFACE_Z = 0.794
TABLETOP_SPAWN_CLEARANCE = 0.003
HOSPITAL_PROP_SPECS = {
    catalog_spec.scene_name: HospitalPropSpec(
        usd_path=catalog_spec.asset_url,
        scale=1.0,
        mass=catalog_spec.mass,
        min_z=catalog_spec.min_z,
        bbox=BBox(*catalog_spec.bbox_half_xy),
    )
    for catalog_spec in MEDICAL_OBJECT_SPECS
}

MEDICAL_OBJECT_TABLE_PROP_META_OVERRIDES = {
    name: TablePropMeta(bbox=spec.bbox, dynamic=True, mandatory=True)
    for name, spec in HOSPITAL_PROP_SPECS.items()
}


def _hospital_prop_cfg(
    name: str,
    prim_path: str,
    init_xy: tuple[float, float],
) -> RigidObjectCfg:
    spec = HOSPITAL_PROP_SPECS[name]
    init_z = (
        TABLETOP_SURFACE_Z
        - spec.min_z * spec.scale
        + TABLETOP_SPAWN_CLEARANCE
    )
    return RigidObjectCfg(
        prim_path=prim_path,
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(init_xy[0], init_xy[1], init_z),
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
        spawn=GraspableHospitalUsdFileCfg(
            usd_path=spec.usd_path,
            scale=(spec.scale, spec.scale, spec.scale),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                linear_damping=1.5,
                angular_damping=3.0,
                max_linear_velocity=5.0,
                max_angular_velocity=10.0,
                max_depenetration_velocity=0.25,
                solver_position_iteration_count=16,
                solver_velocity_iteration_count=4,
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
                contact_offset=0.001,
                rest_offset=0.0,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=spec.mass),
        ),
    )


def _selected_hospital_prop_cfg(
    name: str,
    prim_path: str,
    init_xy: tuple[float, float],
) -> RigidObjectCfg | None:
    """Spawn a MedicalObjects asset only when the GUI assigns it a role."""
    if name not in ACTIVE_MEDICAL_OBJECT_NAMES:
        return None
    return _hospital_prop_cfg(name, prim_path, init_xy)


# ----------------------------------------------------------------------------
# Fixed-layout world coordinates (hospital room interior).
# Derived from the warehouse task translated by T = (-1.7, -3.3, 0).
# ----------------------------------------------------------------------------
TABLE_POS = (-6.0, -7.5, -0.2)          # TUNE: open interior of new_base_room.usda
ROBOT_POS = (-5.9, -7.0, 0.76)          # +Y of table by 0.5 m, same as warehouse
ROBOT_ROT = (0.7071, 0.0, 0.0, -0.7071)  # yaw -90deg, robot faces the table

# The first spawn is 20 degrees off G1's rear centre.  At the calibrated
# 10 cm edge gap, the next 5-degree points touch the packing table, while the
# rear-centre points remain no-spawn positions.
RIDGEBACK_POS = (-6.190717, -6.201272, 0.0328)
RIDGEBACK_ROT = (0.5735764364, 0.0, 0.0, 0.8191520443)
RIDGEBACK_ARC_BBOX = BBox(half_w=0.50, half_d=0.42)


def _next_static_logistics_cluster(env) -> tuple[StaticClusterMember, ...]:
    """Cycle one crate-carrying Ridgeback through the equal-radius rear arc."""
    previous_index = getattr(env, "_hospital_ridgeback_arc_index", None)
    if previous_index is None:
        index = 0  # left-rear point: close to G1 without directly trailing it
    else:
        index = previous_index + 1
    env._hospital_ridgeback_arc_index = index
    robot_local_xy, yaw_offset = ridgeback_static_arc_pose(index)
    return (
        StaticClusterMember(
            asset_name="ridgeback",
            robot_local_xy=robot_local_xy,
            yaw_offset=yaw_offset + float(RIDGEBACK_BODY_YAW_OFFSET),
            # The requested full arc deliberately includes points that may
            # overlap the table/G1 OBB proxies.
            bbox=RIDGEBACK_ARC_BBOX,
            allow_protected_overlap=True,
        ),
    )
HAND_REACHABLE_TABLETOP_REGION = TabletopSpawnRegion(
    # G1 is at table-local (0.10, 0.50), facing toward decreasing Y.
    # These bounds sit directly in front of the two default Dex1 jaws and keep
    # every footprint inside the 96-degree front-camera horizontal FOV.
    x_min=-0.32,
    x_max=0.18,
    y_min=-0.16,
    y_max=0.12,
)
COMPACT_TABLETOP_OBJECT_MARGIN = 0.015

FULL_MEDICAL_TABLETOP_REGION = TabletopSpawnRegion(
    x_min=-0.85,
    x_max=0.18,
    y_min=-0.28,
    y_max=0.28,
)
# Keep the established hand-reachable layout for up to six active objects.
# Larger selections use the rest of the physical tabletop so all ten authored
# MedicalObjects assets can be placed without overlap.
ACTIVE_MEDICAL_TABLETOP_REGION = (
    HAND_REACHABLE_TABLETOP_REGION
    if len(ACTIVE_MEDICAL_OBJECT_NAMES) <= 6
    else FULL_MEDICAL_TABLETOP_REGION
)


def _apply_hospital_front_camera_optics(env) -> None:
    """Keep the streamed USD camera at the optics saved from the Isaac viewport."""
    stage = get_current_stage()
    optics = HOSPITAL_FRONT_CAMERA_OPTICS
    vertical_aperture = optics.horizontal_aperture * optics.height / optics.width

    for env_id in range(env.num_envs):
        camera_path = f"/World/envs/env_{env_id}/Robot/d435_link/front_cam"
        camera_prim = stage.GetPrimAtPath(camera_path)
        if not camera_prim.IsValid() or not camera_prim.IsA(UsdGeom.Camera):
            raise RuntimeError(f"Front camera prim is missing or invalid: {camera_path}")

        usd_camera = UsdGeom.Camera(camera_prim)
        usd_camera.GetFocalLengthAttr().Set(optics.focal_length)
        usd_camera.GetFocusDistanceAttr().Set(optics.focus_distance)
        usd_camera.GetHorizontalApertureAttr().Set(optics.horizontal_aperture)
        usd_camera.GetVerticalApertureAttr().Set(vertical_aperture)

    if not getattr(env, "_hospital_front_camera_optics_reported", False):
        print(
            "[front_camera] enforced streamed POV: "
            f"{optics.width}x{optics.height}, focal={optics.focal_length}, "
            f"horizontal_aperture={optics.horizontal_aperture}, "
            f"h_fov={optics.horizontal_fov_degrees:.3f}, "
            f"v_fov={optics.vertical_fov_degrees:.3f}",
            flush=True,
        )
        env._hospital_front_camera_optics_reported = True


def reset_hospital_teleop_scene(
    env,
    env_ids: torch.Tensor | None,
    randomize_table_position: bool | None = None,
) -> None:
    """Reset the Quest scene and optionally change its persistent table switch."""
    if randomize_table_position is not None:
        env._teleop_randomize_table_position = bool(randomize_table_position)
    randomize_table_position = bool(
        getattr(env, "_teleop_randomize_table_position", False)
    )
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)

    base_mdp.reset_scene_to_default(env, env_ids)
    _apply_hospital_front_camera_optics(env)
    static_cluster = _next_static_logistics_cluster(env)
    randomize_pickplace_room_layout(
        env,
        env_ids,
        wall_prop_names=WALL_PROP_NAMES,
        table_prop_names=list(ACTIVE_MEDICAL_OBJECT_NAMES),
        min_table_objects=len(ACTIVE_MEDICAL_OBJECT_NAMES),
        randomize_table_position=randomize_table_position,
        table_prop_meta_overrides=MEDICAL_OBJECT_TABLE_PROP_META_OVERRIDES,
        tabletop_spawn_region=ACTIVE_MEDICAL_TABLETOP_REGION,
        tabletop_object_margin=COMPACT_TABLETOP_OBJECT_MARGIN,
        static_cluster_members=static_cluster,
    )
    mode = "full randomization" if randomize_table_position else "fixed table"
    print(f"[Meta Quest reset] hospital scene restored ({mode})", flush=True)


def reset_hospital_tabletop_props(env) -> None:
    """Respawn every selected object without moving the table, robot, room, or bins."""
    env_ids = torch.arange(env.num_envs, device=env.device)
    for asset_name in ACTIVE_MEDICAL_OBJECT_NAMES:
        reset_target_on_current_table(
            env,
            env_ids,
            asset_name=asset_name,
            table_prop_meta_overrides=MEDICAL_OBJECT_TABLE_PROP_META_OVERRIDES,
            tabletop_spawn_region=ACTIVE_MEDICAL_TABLETOP_REGION,
            tabletop_object_margin=COMPACT_TABLETOP_OBJECT_MARGIN,
        )
    print(
        f"[Meta Quest reset] {len(ACTIVE_MEDICAL_OBJECT_NAMES)} selected tabletop props respawned",
        flush=True,
    )


def _yaw_from_quaternion_wxyz(quaternion: torch.Tensor) -> torch.Tensor:
    """Return one planar yaw angle per scalar-first quaternion."""
    w, x, y, z = quaternion.unbind(dim=-1)
    return torch.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def reset_hospital_ridgeback_arc(env) -> None:
    """Move only the hospital Ridgeback to the next requested arc point."""
    env_ids = torch.arange(env.num_envs, device=env.device)
    (member,) = _next_static_logistics_cluster(env)
    robot = env.scene["robot"]
    ridgeback = env.scene[member.asset_name]
    origins = env.scene.env_origins
    robot_positions = robot.data.root_pos_w[env_ids] - origins[env_ids]
    robot_yaws = _yaw_from_quaternion_wxyz(robot.data.root_quat_w[env_ids])
    ridgeback_positions = offset_from_yaw_batched(
        robot_positions,
        robot_yaws,
        member.robot_local_xy[0],
        member.robot_local_xy[1],
        float(ridgeback.data.default_root_state[0, 2] - origins[0, 2]),
    )
    ridgeback_yaws = robot_yaws + member.yaw_offset

    ridgeback_state = build_root_state(
        ridgeback_positions,
        ridgeback_yaws,
        origins,
        env_ids,
        ridgeback.data.default_root_state,
    )
    ridgeback.write_root_pose_to_sim(ridgeback_state[:, :7], env_ids=env_ids)

    layout_states = getattr(env, "_room_layout_state", {})
    for env_idx, env_id_tensor in enumerate(env_ids):
        state = layout_states.get(int(env_id_tensor.item()))
        if state is not None:
            state.static_cluster_poses[member.asset_name] = Pose2D(
                position=tuple(float(value) for value in ridgeback_state[env_idx, :3]),
                yaw=float(ridgeback_yaws[env_idx]),
            )
    print("[Meta Quest reset] Ridgeback advanced to the next G1-facing arc point", flush=True)


def reset_hospital_room_fixed_table(env) -> None:
    """Scramble the hospital room while restoring the authored table anchor."""
    env_ids = torch.arange(env.num_envs, device=env.device)
    env._teleop_randomize_table_position = False
    base_mdp.reset_scene_to_default(env, env_ids)
    _apply_hospital_front_camera_optics(env)
    static_cluster = _next_static_logistics_cluster(env)
    randomize_pickplace_room_layout(
        env,
        env_ids,
        wall_prop_names=WALL_PROP_NAMES,
        table_prop_names=list(ACTIVE_MEDICAL_OBJECT_NAMES),
        min_table_objects=len(ACTIVE_MEDICAL_OBJECT_NAMES),
        randomize_table_position=False,
        table_prop_meta_overrides=MEDICAL_OBJECT_TABLE_PROP_META_OVERRIDES,
        tabletop_spawn_region=ACTIVE_MEDICAL_TABLETOP_REGION,
        tabletop_object_margin=COMPACT_TABLETOP_OBJECT_MARGIN,
        static_cluster_members=static_cluster,
    )
    print("[Meta Quest reset] hospital room scrambled (fixed table)", flush=True)


##
# Scene definition
##
@configclass
class HospitalMedicineBottleSceneCfg(RandomizedRoomPickPlaceSceneCfg):
    """Hospital table/G1 scene with a bottle and static logistics platforms."""

    # --- repurpose inherited entities -------------------------------------
    # table: reposition the inherited kinematic PackingTable into the interior
    packing_table = RigidObjectCfg(
        prim_path="/World/envs/env_.*/PackingTable",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/PackingTable/PackingTable.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=TABLE_POS),
    )

    # Every MedicalObjects child has a scene field. The GUI role map decides
    # which configs exist for this process; important and distractor objects
    # use identical physical spawning and differ only in success evaluation.
    object = None
    coffee_cup = _selected_hospital_prop_cfg(
        "coffee_cup",
        prim_path="/World/envs/env_.*/CoffeeCup",
        init_xy=(-6.34, -7.36),
    )
    pill_bottle_t: RigidObjectCfg | None = _selected_hospital_prop_cfg(
        "pill_bottle_t",
        prim_path="/World/envs/env_.*/PillBottleT",
        init_xy=(-6.02, -7.42),
    )
    pill_bottle_v: RigidObjectCfg | None = _selected_hospital_prop_cfg(
        "pill_bottle_v",
        prim_path="/World/envs/env_.*/PillBottleV",
        init_xy=(-6.15, -7.42),
    )
    cup_half_full: RigidObjectCfg | None = _selected_hospital_prop_cfg(
        "cup_half_full",
        prim_path="/World/envs/env_.*/CupHalfFull",
        init_xy=(-5.72, -7.36),
    )
    medical_bottle_a: RigidObjectCfg | None = _selected_hospital_prop_cfg(
        "medical_bottle_a",
        prim_path="/World/envs/env_.*/MedicalBottleA",
        init_xy=(-6.27, -7.42),
    )
    plastic_cup: RigidObjectCfg | None = _selected_hospital_prop_cfg(
        "plastic_cup",
        prim_path="/World/envs/env_.*/PlasticCup",
        init_xy=(-5.82, -7.62),
    )
    medical_bottle_f: RigidObjectCfg | None = _selected_hospital_prop_cfg(
        "medical_bottle_f",
        prim_path="/World/envs/env_.*/MedicalBottleF",
        init_xy=(-5.87, -7.42),
    )
    marker_blue: RigidObjectCfg | None = _selected_hospital_prop_cfg(
        "marker_blue",
        prim_path="/World/envs/env_.*/MarkerBlue",
        init_xy=(-6.22, -7.58),
    )
    marker_yellow: RigidObjectCfg | None = _selected_hospital_prop_cfg(
        "marker_yellow",
        prim_path="/World/envs/env_.*/MarkerYellow",
        init_xy=(-5.98, -7.58),
    )
    felt_pen_pink: RigidObjectCfg | None = _selected_hospital_prop_cfg(
        "felt_pen_pink",
        prim_path="/World/envs/env_.*/FeltPenPink",
        init_xy=(-6.44, -7.60),
    )

    blue_cube = None
    yellow_cube = None

    # One reversed static Ridgeback occupies a rear-arc point. Its crate and
    # riser are base_link children counter-transformed to remain fixed in the
    # same G1/room-relative pose.
    ridgeback: RigidObjectCfg = _static_ridgeback_cfg(
        "/World/envs/env_.*/Ridgeback",
        RIDGEBACK_POS,
        RIDGEBACK_ROT,
    )
    ridgeback_crate_riser: AssetBaseCfg = _ridgeback_crate_riser_cfg(
        "/World/envs/env_.*/Ridgeback/base_link/CrateRiser",
        local_pos=CRATE_RISER_LOCAL_POS,
        local_rot=CRATE_LOCAL_ROT,
    )
    ridgeback_crate: AssetBaseCfg = _ridgeback_crate_cfg(
        "/World/envs/env_.*/Ridgeback/base_link/Crate",
        local_x=CRATE_LOCAL_POS[0],
        local_y=CRATE_LOCAL_POS[1],
        local_z=CRATE_LOCAL_POS[2],
        local_rot=CRATE_LOCAL_ROT,
    )

    # --- robot + cameras ---------------------------------------------------
    robot: ArticulationCfg = G1RobotPresets.g1_29dof_dex1_base_fix(
        init_pos=ROBOT_POS, init_rot=ROBOT_ROT
    )
    # Both fingers of both hands begin fully open. Scene resets restore this
    # pose; an index-trigger press then commands the matching pair closed.
    robot.init_state = robot.init_state.replace(
        joint_pos={
            **robot.init_state.joint_pos,
            "left_hand_Joint1_1": DEX1_OPEN_JOINT_POSITION,
            "left_hand_Joint2_1": DEX1_OPEN_JOINT_POSITION,
            "right_hand_Joint1_1": DEX1_OPEN_JOINT_POSITION,
            "right_hand_Joint2_1": DEX1_OPEN_JOINT_POSITION,
        }
    )
    robot.spawn = robot.spawn.replace(func=_spawn_grasp_ready_dex1_usd)
    # This task alone releases waist yaw and pitch for the Quest right stick:
    # yaw turns toward either rear crate and pitch lowers the hands over it.
    # Keep waist roll and the complete fixed lower body locked.
    robot.actuators = dict(robot.actuators)
    # The Quest index trigger is a binary hold/release command. The low drive
    # friction lets the authored jaw stroke finish visibly instead of stalling
    # halfway; speed and force remain capped for stable lightweight-prop contact.
    robot.actuators["hands"] = ImplicitActuatorCfg(
        joint_names_expr=[
            "left_hand_Joint1_1",
            "left_hand_Joint2_1",
            "right_hand_Joint1_1",
            "right_hand_Joint2_1",
        ],
        effort_limit_sim=12.0,
        velocity_limit_sim=0.5,
        stiffness=600.0,
        damping=8.0,
        friction=0.0,
    )
    robot.actuators.pop("waist", None)
    robot.actuators["waist_yaw_pitch_teleop"] = ImplicitActuatorCfg(
        joint_names_expr=["waist_yaw_joint", "waist_pitch_joint"],
        effort_limit_sim=350.0,
        velocity_limit_sim=2.5,
        stiffness=260.0,
        damping=18.0,
    )
    robot.actuators["waist_roll_lock"] = ImplicitActuatorCfg(
        joint_names_expr=["waist_roll_joint"],
        effort_limit_sim=1000.0,
        velocity_limit_sim=0.1,
        stiffness=10000.0,
        damping=10000.0,
    )
    # Optics tuned on /Robot/d435_link/front_cam in the Isaac Sim viewport
    # (Screenshot-2026-08-22_15-38-27.png). The default mount transform already
    # matches the captured translate=(0,0,0), Euler=(90,-90,0) pose.
    front_camera = CameraBaseCfg.get_camera_config(
        height=HOSPITAL_FRONT_CAMERA_OPTICS.height,
        width=HOSPITAL_FRONT_CAMERA_OPTICS.width,
        focal_length=HOSPITAL_FRONT_CAMERA_OPTICS.focal_length,
        focus_distance=HOSPITAL_FRONT_CAMERA_OPTICS.focus_distance,
        horizontal_aperture=HOSPITAL_FRONT_CAMERA_OPTICS.horizontal_aperture,
    )
    left_wrist_camera = CameraPresets.left_gripper_wrist_camera()
    right_wrist_camera = CameraPresets.right_gripper_wrist_camera()

    # spectator camera (GUI only): warehouse view translated to look at the table
    world_camera = CameraBaseCfg.get_camera_config(
        prim_path="/World/PerspectiveCamera",
        pos_offset=(-5.8, -8.2, 1.8),       # TUNE: GUI-only viewpoint
        rot_offset=(-0.3173, 0.94833, 0.0, 0.0),
    )

    # These authored desk props are outside /World/MedicalObjects.
    desk_lamp = None
    box_portable = None


##
# MDP settings
##
@configclass
class ActionsCfg:
    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot", joint_names=[".*"], scale=1.0, use_default_offset=True
    )


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        robot_joint_state = ObsTerm(func=mdp.get_robot_boy_joint_states)
        robot_gipper_state = ObsTerm(func=mdp.get_robot_gipper_joint_states)
        camera_image = ObsTerm(func=mdp.get_camera_image)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    policy: PolicyCfg = PolicyCfg()


@configclass
class TerminationsCfg:
    success = DoneTerm(func=mdp.all_important_objects_contained)


@configclass
class RewardsCfg:
    reward = RewTerm(
        func=mdp.compute_important_object_reward,
        weight=1.0,
    )


@configclass
class EventCfg:
    reset_teleop_scene = EventTermCfg(
        func=reset_hospital_teleop_scene,
        mode="reset",
    )


@configclass
class PickPlaceMedicineBottleHospitalG129DEX1EnvCfg(ManagerBasedRLEnvCfg):
    """G1 + Dex1 medicine-bottle pick-place in a randomized hospital room."""

    scene: HospitalMedicineBottleSceneCfg = HospitalMedicineBottleSceneCfg(
        num_envs=1, env_spacing=16.0, replicate_physics=True
    )
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events = EventCfg()
    commands = None
    rewards: RewardsCfg = RewardsCfg()
    curriculum = None

    def __post_init__(self):
        """Post initialization (PhysX/sim settings copied from the warehouse task)."""
        self.decimation = 2
        self.episode_length_s = 20.0
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 32 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625
        # GPU rigid-body CCD is unsupported in this Isaac Sim configuration;
        # bounded jaw velocity and convex-decomposed contacts prevent tunneling.
        self.sim.physx.enable_ccd = False
        self.sim.physx.gpu_constraint_solver_heavy_spring_enabled = True
        self.sim.physx.num_substeps = 2
        # The scored props are about 27--29 mm across. A 15 mm contact skin
        # made opposing jaw contacts appear far too early and injected large
        # depenetration impulses.  Keep the broad default and the task-local
        # finger/object offsets in the low-millimetre range.
        self.sim.physx.contact_offset = 0.003
        self.sim.physx.rest_offset = 0.0
        self.sim.physx.num_position_iterations = 16
        self.sim.physx.num_velocity_iterations = 4

        # Custom event manager (matches the warehouse manipulation task).
        self.event_manager = SimpleEventManager()
        self.event_manager.register("reset_object_self", SimpleEvent(
            func=reset_hospital_tabletop_props
        ))
        self.event_manager.register(
            "reset_ridgeback_arc_self",
            SimpleEvent(func=reset_hospital_ridgeback_arc),
        )
        self.event_manager.register("reset_all_self", SimpleEvent(
            # Quest/xr_teleoperate's full-reset button sends DDS category 2.
            func=lambda env: reset_hospital_teleop_scene(
                env, None, randomize_table_position=True
            )
        ))
        self.event_manager.register(
            "reset_room_fixed_table_self",
            SimpleEvent(func=reset_hospital_room_fixed_table),
        )
