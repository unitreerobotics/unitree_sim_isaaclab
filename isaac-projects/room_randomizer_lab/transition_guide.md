# Transition Reference: `nbr_gen.py` → Isaac Lab `ManagerBasedEnv`

> [!NOTE]
> This document records the architectural decisions and mappings from the original procedural `nbr_gen.py` script to the current Isaac Lab implementation. The migration is **complete** — this serves as a reference for understanding the final design.

---

## 1. Architecture Summary

The original `nbr_gen.py` created rooms procedurally by directly manipulating USD prims. The Isaac Lab version uses a declarative configuration + event-driven reset pattern.

| Aspect | Original `nbr_gen.py` | Current Isaac Lab |
|---|---|---|
| **Scene creation** | Imperative: `define_xform()`, `add_payload_prim()`, `set_xform()` | Declarative: `@configclass RoomSceneCfg` with `RigidObjectCfg` fields |
| **Environment cloning** | Manual grid loop with `ROOM_SPACING_X/Y` | Automatic: `num_envs=20`, `env_spacing=16.0` |
| **Randomization timing** | Once at build (baked into USD) | Every reset via event term |
| **Collision detection** | Circle-packing + predefined slots | OBB + SAT + continuous zone sampling |
| **Math backend** | Python `random` + scalar math | Hybrid: Python `random` for sampling, `torch` for state writes |
| **State representation** | USD xformOps | Root state tensors: `(N, 13)` = pos + quat + velocities |
| **Physics bodies** | Static USD meshes | Real USD rigid objects (kinematic, gravity disabled) |
| **Physics device** | N/A (no simulation) | GPU PhysX (`cuda:0`, Fabric enabled) |

---

## 2. What Got Deleted

All of these elements from `nbr_gen.py` were replaced by Isaac Lab's built-in mechanisms:

- `define_xform()`, `add_payload_prim()`, `set_xform()`, `add_internal_reference_prim()`, `add_internal_reference_xform()` — replaced by `RigidObjectCfg` declarations
- Grid layout loop (`create_room_instance()`, `ROOM_SPACING_X/Y`, `GENERATED_START_OFFSET`) — replaced by `num_envs` + `env_spacing`
- `hide_*` functions — replaced by `z = -100` despawning pattern
- `build_ui()` / `main()` — replaced by Isaac Lab's `AppLauncher`
- `PlacementSlot` and predefined slot lists — replaced by continuous `WallZone` sampling
- Circle-packing collision (`is_free()`, spacing_radius) — replaced by OBB + SAT

---

## 3. Component Mapping (Final)

### 3.1 Scene Definition

Each randomizable object is a named field in [RoomSceneCfg](file:///Users/cezarioa/Projects/isaac-projects/room_randomizer_lab/room_scene_cfg.py):

| Original `nbr_gen.py` Object | Scene Cfg Field | Asset Type | Prim Path |
|---|---|---|---|
| Room structure (`/World/Environment`) | `room_shell` | `AssetBaseCfg` + `UsdFileCfg` | `{ENV}/RoomShell` |
| Ground/lights | `ground`, `dome_light` | `AssetBaseCfg` | `/World/ground`, `/World/light` |
| SM_Desk_04a | `desk` | `RigidObjectCfg` + real USD | `{ENV}/Desk` |
| SM_Chair_04a | `chair` | `RigidObjectCfg` + real USD | `{ENV}/Chair` |
| ridgeback_ur5 | `ridgeback` | `ArticulationCfg` | `{ENV}/Ridgeback` |
| SM_MedicalCabinet_01a | `medical_cabinet` | `RigidObjectCfg` + real USD | `{ENV}/MedicalCabinet` |
| SM_ShelfSet_01a | `shelf_set` | `RigidObjectCfg` + real USD | `{ENV}/ShelfSet` |
| SM_SupplyCabinet_01c | `supply_cabinet` | `RigidObjectCfg` + real USD | `{ENV}/SupplyCabinet` |
| SM_SupplyCart_02a | `supply_cart_a` | `RigidObjectCfg` + real USD | `{ENV}/SupplyCartA` |
| SM_SupplyCart_03a | `supply_cart_b` | `RigidObjectCfg` + real USD | `{ENV}/SupplyCartB` |
| SM_TrashCan | `trash_can` | `RigidObjectCfg` + real USD | `{ENV}/TrashCan` |
| SM_Plant01 | `plant_a` | `RigidObjectCfg` + real USD | `{ENV}/PlantA` |
| SM_Plant02 | `plant_b` | `RigidObjectCfg` + real USD | `{ENV}/PlantB` |
| SM_CoffeeToGo | `coffee_cup` | `RigidObjectCfg` + real USD | `{ENV}/CoffeeCup` |
| SM_Lamp02 | `desk_lamp` | `RigidObjectCfg` + real USD | `{ENV}/DeskLamp` |
| SM_BoxPortableC | `box_portable` | `RigidObjectCfg` + real USD | `{ENV}/BoxPortable` |
| Top-down camera | `top_down_camera` | `CameraCfg` + `PinholeCameraCfg` | `{ENV}/TopDownCamera` |

> [!IMPORTANT]
> All furniture props are loaded as **real USD meshes** from the Omniverse CDN using `_spawn_real_rigid_usd()`. This function ensures a single root `RigidBodyAPI` and auto-adds mesh colliders. The original props inside the room shell are hidden on first reset via `_hide_duplicate_visual_props()`.

### 3.2 State Writing: `set_xform()` → `write_root_state_to_sim()`

```python
# ORIGINAL (imperative USD)
set_xform(path, translate=(x, y, z), yaw_deg=90.0)

# CURRENT (Isaac Lab tensor API)
root_state = build_root_state(pos, yaw_rad, env_origins, env_ids, default_state)
asset.write_root_state_to_sim(root_state, env_ids=env_ids)
```

The [build_root_state()](file:///Users/cezarioa/Projects/isaac-projects/room_randomizer_lab/placement_utils.py#L232-L251) helper:
1. Clones the asset's default state for the given env IDs.
2. Adds `env_origins` offsets to positions (local → world).
3. Converts yaw to `(w, x, y, z)` quaternion via [yaw_to_quat()](file:///Users/cezarioa/Projects/isaac-projects/room_randomizer_lab/placement_utils.py#L148-L161).
4. Zeroes velocities.

### 3.3 Duplicate Visual Handling

Because the room shell USD contains original authored prop meshes at their default positions, these would visually double with the separately spawned rigid objects. On the **first reset**, `_hide_duplicate_visual_props()` calls `UsdGeom.Imageable.MakeInvisible()` on 10 hardcoded prim paths inside the room shell (desk, chair, and all 8 wall props).

### 3.4 Coordinate System

The original room coordinates were kept as-is (no re-centering):

```
Room bounds: x ∈ [-13.0, -0.5],  y ∈ [-11.25, -5.0]
Floor: z = 0.0
Robot: z = 0.0328  (Ridgeback wheel clearance)
```

All placement math operates in room-local coordinates. The `env_origins` offset is applied only at the final `build_root_state()` call.

### 3.5 Dynamic Object Count

The "despawn underground" pattern from `nbr_gen.py` was carried over:

```python
# Hide an object by moving it far below ground
pos[:, 2] = -100.0  # DESPAWN_Z
asset.write_root_state_to_sim(state, env_ids=env_ids)
```

All environments have the same set of prims. Objects that shouldn't appear in a particular env are teleported to `z = -100`.

---

## 4. Resolved Design Questions

These were open questions during the migration, now resolved:

| Question | Resolution |
|---|---|
| **Room shell loading strategy?** | Load the full `new_base_room.usda` as a single `AssetBaseCfg`. Original authored props are hidden on first reset via `MakeInvisible()`. |
| **Kinematic vs. dynamic objects?** | All furniture uses `_kinematic_usd_cfg()`: `rigid_body_enabled=True`, `kinematic_enabled=True`, `disable_gravity=True`, high damping. Tabletop objects also kinematic. |
| **USD asset hosting?** | Individual prop assets are loaded directly from the Omniverse CDN (S3 URLs). The room shell is loaded from a local path resolved via `paths.py`. |
| **RL or data generation?** | Currently configured as a `ManagerBasedEnv` for future RL training. Still has dummy action/observation managers. |
| **CPU vs GPU PhysX?** | **GPU PhysX** enabled (`sim.device = "cuda:0"`, `sim.use_fabric = True`). Real USD rigid objects with proper root body authoring work with GPU tensor views. |
| **Tabletop objects?** | Fully active. Coffee cup, desk lamp, and portable box are spawned as kinematic rigid objects and placed on the desk surface (Phase 3 of the randomizer). |

---

## 5. Current File Structure

```
room_randomizer_lab/
├── __init__.py               # Exports RoomEnvCfg, RoomSceneCfg
├── constants.py              # OBB sizes, wall zones, orbit offsets, asset paths
├── placement_utils.py        # SAT collision, quaternion math, build_root_state
├── room_scene_cfg.py         # @configclass RoomSceneCfg — 18 asset fields (real USD)
├── room_events.py            # 3-phase placement + duplicate visual hiding
├── room_env_cfg.py           # @configclass RoomEnvCfg — master config (GPU PhysX)
├── run_randomizer.py         # Launcher script (AppLauncher + sim loop + camera capture)
├── paths.py                  # Room shell USD path resolution
├── test_placement.py         # Standalone matplotlib OBB test (no Isaac Sim)
├── get_bounding_boxes.py     # USD bounding box computation utility
├── execution_flow.md         # Step-by-step runtime trace
├── project_architecture.md   # Architecture overview + RL roadmap
└── transition_guide.md       # This file — migration reference
```
