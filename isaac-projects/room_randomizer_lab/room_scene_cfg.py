# room_scene_cfg.py
# Declarative scene configuration for the hospital room environment.
# Each randomizable object is a separate named field so the event term
# can access it via env.scene["field_name"].

from __future__ import annotations

from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import CameraCfg
from isaaclab.utils import configclass
import isaaclab.sim as sim_utils
from isaaclab.sim import schemas
from isaaclab.sim.spawners.from_files import from_files as file_spawners
from isaaclab.sim.utils import clone, get_current_stage
from isaaclab.utils.assets import check_usd_path_with_timeout
from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics

from .constants import ASSET_PATHS, DESK_OBJECT_Z, FLOOR_Z, ROBOT_Z
from .paths import ROOM_SHELL_USD


# ------------------------------------------------------------------
# Helpers for real USD rigid-object configs
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
    """UsdFileCfg with kinematic rigid-body properties (won't fall)."""
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


# ------------------------------------------------------------------
# Scene configuration
# ------------------------------------------------------------------

@configclass
class RoomSceneCfg(InteractiveSceneCfg):
    """Hospital room scene with all randomizable objects as named fields.

    The event term repositions every object on each episode reset.
    Default ``init_state`` positions are safe starting points; the randomizer
    overwrites them immediately.
    """

    # --- static environment -------------------------------------------

    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(),
    )

    dome_light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0),
    )

    # Room shell: the full template USD with walls, floor, ceiling.
    # Original authored props inside it should have their visibility
    # disabled in the USD file itself (pre-processing step).
    room_shell = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/RoomShell",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(ROOM_SHELL_USD),
        ),
    )

    # --- table group (kinematic) --------------------------------------

    desk = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Desk",
        spawn=_kinematic_usd_cfg(ASSET_PATHS["SM_Desk_04a"]),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-7.0, -7.5, FLOOR_Z)),
    )

    chair = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Chair",
        spawn=_kinematic_usd_cfg(ASSET_PATHS["SM_Chair_04a"]),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-7.0, -9.15, FLOOR_Z)),
    )

    ridgeback = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Ridgeback",
        spawn=sim_utils.UsdFileCfg(
            usd_path=ASSET_PATHS["RidgebackUr"],
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                articulation_enabled=True,
                enabled_self_collisions=False,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(pos=(-6.1, -5.95, ROBOT_Z)),
        actuators={},
    )

    # --- wall props (kinematic) ---------------------------------------

    medical_cabinet = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/MedicalCabinet",
        spawn=_kinematic_usd_cfg(ASSET_PATHS["SM_MedicalCabinet_01a"]),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-3.0, -10.0, FLOOR_Z)),
    )

    shelf_set = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/ShelfSet",
        spawn=_kinematic_usd_cfg(ASSET_PATHS["SM_ShelfSet_01a"]),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-4.32, -10.76, FLOOR_Z)),
    )

    supply_cabinet = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/SupplyCabinet",
        spawn=_kinematic_usd_cfg(ASSET_PATHS["SM_SupplyCabinet_01c"]),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-5.78, -10.91, FLOOR_Z)),
    )

    supply_cart_a = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/SupplyCartA",
        spawn=_kinematic_usd_cfg(ASSET_PATHS["SM_SupplyCart_02a"]),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-6.50, -10.95, FLOOR_Z)),
    )

    supply_cart_b = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/SupplyCartB",
        spawn=_kinematic_usd_cfg(ASSET_PATHS["SM_SupplyCart_03a"]),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-7.20, -11.0, FLOOR_Z)),
    )

    trash_can = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/TrashCan",
        spawn=_kinematic_usd_cfg(ASSET_PATHS["SM_TrashCan"]),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-8.10, -10.91, FLOOR_Z)),
    )

    plant_a = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/PlantA",
        spawn=_kinematic_usd_cfg(ASSET_PATHS["SM_Plant01"]),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-2.80, -5.50, FLOOR_Z)),
    )

    plant_b = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/PlantB",
        spawn=_kinematic_usd_cfg(ASSET_PATHS["SM_Plant02"]),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-2.89, -6.35, FLOOR_Z)),
    )

    # --- tabletop objects (real USD props on desk surface) --------------

    coffee_cup = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/CoffeeCup",
        spawn=_kinematic_usd_cfg(ASSET_PATHS["SM_CoffeeToGo"]),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-7.0, -7.5, DESK_OBJECT_Z)),
    )

    desk_lamp = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/DeskLamp",
        spawn=_kinematic_usd_cfg(ASSET_PATHS["SM_Lamp02"]),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-7.2, -7.3, DESK_OBJECT_Z)),
    )

    box_portable = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/BoxPortable",
        spawn=_kinematic_usd_cfg(ASSET_PATHS["SM_BoxPortableC"]),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-6.8, -7.7, DESK_OBJECT_Z)),
    )

    # --- cameras ------------------------------------------------------

    top_down_camera = CameraCfg(
        prim_path="{ENV_REGEX_NS}/TopDownCamera",
        update_period=0.0,
        height=720,
        width=1280,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0,
            focus_distance=400.0,
            horizontal_aperture=20.955,
            clipping_range=(0.1, 1.0e5),
        ),
        offset=CameraCfg.OffsetCfg(
            pos=(-5.5, -9.1, 16.8),
            rot=(0.7071068, 0.0, 0.7071068, 0.0),  # look down
            convention="world",
        ),
    )
