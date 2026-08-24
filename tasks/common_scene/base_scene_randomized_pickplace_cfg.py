# base_scene_randomized_pickplace_cfg.py
# Randomized room scene configuration for G1 pick and place tasks.

from __future__ import annotations

import os
from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import schemas
from isaaclab.sim.spawners.from_files import from_files as file_spawners
from isaaclab.sim.utils import clone, get_current_stage
from isaaclab.utils import configclass
from isaaclab.utils.assets import check_usd_path_with_timeout

from tasks.common_config import CameraBaseCfg
from tasks.utils.room_randomizer.constants import ASSET_PATHS, ROOM_SHELL_USD, FLOOR_Z, DESK_OBJECT_Z, DESK_LAMP_Z

project_root = os.environ.get("PROJECT_ROOT")

# ------------------------------------------------------------------
# Helpers for spawning USD assets and authoring root rigid body APIs
# ------------------------------------------------------------------

def _ensure_mesh_colliders(root_prim: Usd.Prim) -> None:
    """Use the visual meshes as real colliders when the asset has no authored colliders."""
    has_collider = any(prim.HasAPI(UsdPhysics.CollisionAPI) for prim in Usd.PrimRange(root_prim))
    if has_collider:
        return

    for prim in Usd.PrimRange(root_prim):
        if prim.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(prim)
            PhysxSchema.PhysxCollisionAPI.Apply(prim)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr().Set("none")


def _ensure_root_rigid_body(prim_path: str, cfg: sim_utils.UsdFileCfg) -> None:
    """Make the referenced USD a single Isaac Lab RigidObject rooted at prim_path."""
    stage = get_current_stage()
    root_prim = stage.GetPrimAtPath(prim_path)
    if not root_prim.IsValid():
        raise ValueError(f"Prim path '{prim_path}' is not valid.")

    for prim in Usd.PrimRange(root_prim):
        if prim == root_prim:
            continue
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
        if prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
            prim.RemoveAPI(PhysxSchema.PhysxRigidBodyAPI)

    schemas.define_rigid_body_properties(prim_path, cfg.rigid_props, stage=stage)
    _ensure_mesh_colliders(root_prim)

    if cfg.collision_props is not None:
        schemas.modify_collision_properties(prim_path, cfg.collision_props, stage=stage)
    if cfg.mass_props is not None:
        schemas.define_mass_properties(prim_path, cfg.mass_props, stage=stage)


@clone
def _spawn_real_rigid_usd(
    prim_path: str,
    cfg: sim_utils.UsdFileCfg,
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
    **kwargs,
) -> Usd.Prim:
    """Spawn a detailed USD asset and author a single root rigid body for Isaac Lab."""
    if cfg.rigid_props is None:
        raise ValueError("_spawn_real_rigid_usd requires cfg.rigid_props.")

    if not check_usd_path_with_timeout(cfg.usd_path):
        raise FileNotFoundError(f"USD file not found at path: '{cfg.usd_path}'.")

    spawn_cfg = cfg.copy()
    spawn_cfg.rigid_props = None
    spawn_cfg.collision_props = None
    spawn_cfg.mass_props = None

    prim = file_spawners._spawn_from_usd_file(prim_path, cfg.usd_path, spawn_cfg, translation, orientation)
    _ensure_root_rigid_body(prim_path, cfg)
    return prim


def _kinematic_usd_cfg(usd_path: str) -> sim_utils.UsdFileCfg:
    """UsdFileCfg with kinematic rigid-body properties."""
    return sim_utils.UsdFileCfg(
        func=_spawn_real_rigid_usd,
        usd_path=usd_path,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            kinematic_enabled=True,
            disable_gravity=True,
            linear_damping=10.0,
            angular_damping=10.0,
        ),
        collision_props=sim_utils.CollisionPropertiesCfg(),
    )


def tabletop_cube_cfg(color: tuple[float, float, float]) -> sim_utils.CuboidCfg:
    """Create a small tabletop cube tuned for stable robot handling."""
    return sim_utils.CuboidCfg(
        size=(0.06, 0.06, 0.06),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.05,
            angular_damping=0.1,
            max_linear_velocity=5.0,
            max_angular_velocity=20.0,
            max_depenetration_velocity=1.0,
        ),
        mass_props=sim_utils.MassPropertiesCfg(mass=0.1),
        collision_props=sim_utils.CollisionPropertiesCfg(
            collision_enabled=True,
            contact_offset=0.005,
            rest_offset=0.0,
        ),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color, metallic=0),
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="max",
            restitution_combine_mode="min",
            static_friction=1.0,
            dynamic_friction=0.8,
            restitution=0.0,
        ),
    )


# ------------------------------------------------------------------
# Interactive scene configuration
# ------------------------------------------------------------------

@configclass
class RandomizedRoomPickPlaceSceneCfg(InteractiveSceneCfg):
    """Hospital/Office room scene layout for pick and place tasks."""

    # Dome light
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )

    # Room Shell USD
    room_shell = AssetBaseCfg(
        prim_path="/World/envs/env_.*/RoomShell",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(ROOM_SHELL_USD),
        ),
    )

    # Main Packing Table (kinematic desk)
    packing_table = RigidObjectCfg(
        prim_path="/World/envs/env_.*/PackingTable",
        spawn=_kinematic_usd_cfg(f"{project_root}/assets/objects/PackingTable/PackingTable.usd"),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.55, -0.2)),
    )

    # Target Object (cylinder as default, overwritten by task if needed)
    object = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Object",
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-0.35, 0.40, 0.84), rot=(1, 0, 0, 0)),
        spawn=sim_utils.CylinderCfg(
            radius=0.018,
            height=0.35,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.4),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.15, 0.15, 0.15), metallic=1.0),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="max",
                restitution_combine_mode="min",
                static_friction=1.5,
                dynamic_friction=1.5,
                restitution=0.0,
            ),
        ),
    )

    # --- Wall Props ---
    medical_cabinet = RigidObjectCfg(
        prim_path="/World/envs/env_.*/MedicalCabinet",
        spawn=_kinematic_usd_cfg(ASSET_PATHS["SM_MedicalCabinet_01a"]),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-3.0, -10.0, FLOOR_Z)),
    )

    shelf_set = RigidObjectCfg(
        prim_path="/World/envs/env_.*/ShelfSet",
        spawn=_kinematic_usd_cfg(ASSET_PATHS["SM_ShelfSet_01a"]),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-4.32, -10.76, FLOOR_Z)),
    )

    supply_cabinet = RigidObjectCfg(
        prim_path="/World/envs/env_.*/SupplyCabinet",
        spawn=_kinematic_usd_cfg(ASSET_PATHS["SM_SupplyCabinet_01c"]),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-5.78, -10.91, FLOOR_Z)),
    )

    supply_cart_a = RigidObjectCfg(
        prim_path="/World/envs/env_.*/SupplyCartA",
        spawn=_kinematic_usd_cfg(ASSET_PATHS["SM_SupplyCart_02a"]),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-6.50, -10.95, FLOOR_Z)),
    )

    supply_cart_b = RigidObjectCfg(
        prim_path="/World/envs/env_.*/SupplyCartB",
        spawn=_kinematic_usd_cfg(ASSET_PATHS["SM_SupplyCart_03a"]),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-7.20, -11.0, FLOOR_Z)),
    )

    trash_can = RigidObjectCfg(
        prim_path="/World/envs/env_.*/TrashCan",
        spawn=_kinematic_usd_cfg(ASSET_PATHS["SM_TrashCan"]),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-8.10, -10.91, FLOOR_Z)),
    )

    plant_a = RigidObjectCfg(
        prim_path="/World/envs/env_.*/PlantA",
        spawn=_kinematic_usd_cfg(ASSET_PATHS["SM_Plant01"]),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-2.80, -5.50, FLOOR_Z)),
    )

    plant_b = RigidObjectCfg(
        prim_path="/World/envs/env_.*/PlantB",
        spawn=_kinematic_usd_cfg(ASSET_PATHS["SM_Plant02"]),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-2.89, -6.35, FLOOR_Z)),
    )

    # --- Tabletop Distractors ---
    coffee_cup = RigidObjectCfg(
        prim_path="/World/envs/env_.*/CoffeeCup",
        spawn=_kinematic_usd_cfg(ASSET_PATHS["SM_CoffeeToGo"]),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-7.0, -7.5, DESK_OBJECT_Z)),
    )

    desk_lamp = RigidObjectCfg(
        prim_path="/World/envs/env_.*/DeskLamp",
        spawn=_kinematic_usd_cfg(ASSET_PATHS["SM_Lamp02"]),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-7.2, -7.3, DESK_LAMP_Z)),
    )

    box_portable = RigidObjectCfg(
        prim_path="/World/envs/env_.*/BoxPortable",
        spawn=_kinematic_usd_cfg(ASSET_PATHS["SM_BoxPortableC"]),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-6.8, -7.7, DESK_OBJECT_Z)),
    )

    blue_cube = RigidObjectCfg(
        prim_path="/World/envs/env_.*/BlueCube",
        spawn=tabletop_cube_cfg((0.0, 0.2, 1.0)),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-7.1, -7.4, DESK_OBJECT_Z), rot=(1, 0, 0, 0)),
    )

    yellow_cube = RigidObjectCfg(
        prim_path="/World/envs/env_.*/YellowCube",
        spawn=tabletop_cube_cfg((1.0, 1.0, 0.0)),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-7.0, -7.6, DESK_OBJECT_Z), rot=(1, 0, 0, 0)),
    )

    # Cameras
    world_camera = CameraBaseCfg.get_camera_config(
        prim_path="/World/PerspectiveCamera",
        pos_offset=(-7.3, -8.2, 1.8),
        rot_offset=(-0.3173, 0.94833, 0.0, 0.0),
        focal_length=16.5,
    )
