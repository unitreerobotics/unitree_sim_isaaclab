# Project Architecture & Roadmap

This document provides a high-level overview of the `room_randomizer_lab` project as it currently stands, and outlines the missing pieces required to turn this from a "randomizing simulation" into a fully functional Reinforcement Learning (RL) training environment.

> **Last updated:** 2026-06-23 | **Real USD rigid objects + GPU PhysX architecture**

## 1. Ensemble Picture (Current Architecture)

The system is built using Isaac Lab's declarative `ManagerBasedEnv` framework. The core responsibility of the current codebase is **domain randomization** (resetting the environment into a new, collision-free state at the start of every episode).

### Real USD Rigid Object Architecture

The environment spawns each furniture prop as a full-detail **USD rigid object** directly into the scene:

- **Wall props and table group objects** use `_spawn_real_rigid_usd()`, a custom spawner that loads the detailed USD mesh, authors a single root `RigidBodyAPI`, and ensures child mesh colliders exist. All furniture is **kinematic** (gravity disabled, high damping) so it stays where placed.
- **The Ridgeback robot** is an `ArticulationCfg` loaded from the Omniverse CDN.
- **Tabletop objects** (coffee cup, desk lamp, portable box) are kinematic rigid objects placed on the desk surface.
- **The room shell** (`new_base_room.usda`) provides walls, floor, and ceiling. The original authored props inside it are **hidden** at the first reset via `_hide_duplicate_visual_props()` to avoid visual doubling.
- **GPU PhysX** is enabled (`sim.device = "cuda:0"`, `sim.use_fabric = True`).

```
                          run_randomizer.py
                                |
                           Initializes
                                |
                                v
               RoomEnvCfg (device: cuda:0, fabric: on)
                  /          |          \            \
                 v            v           v            v
          RoomSceneCfg     randomize_     Observation   Action
          (18 assets)      room_layout   Manager       Manager
              |              (3-phase     (DUMMY)       (DUMMY)
              |               OBB)
              |               / \
              |              v   v
              |     constants.py  placement_utils.py
              |     (OBB Sizes,   (SAT Collision,
              |      Wall Zones,  Quaternion Math)
              |      Offsets)
              |
    +---------+---------+
    |         |         |
    v         v         v
  Real USD   new_base   Omniverse CDN
  Rigid      _room.usda (Hospital, Office,
  Objects    (shell)     Warehouse props)
```

### Module Breakdown

| Module | Responsibility |
|--------|---------------|
| [constants.py](constants.py) | Room geometry, OBB sizes (`BBox`), wall zone definitions (`WallZone`), wall prop metadata (`WallPropMeta`), tabletop prop metadata, orbit offsets, asset USD paths |
| [placement_utils.py](placement_utils.py) | Pure math: SAT collision (`obb_overlap`), room bounds check (`obb_inside_room`), yaw-to-quaternion (`yaw_to_quat`), coordinate transforms (`offset_from_yaw`, `build_root_state`) |
| [room_scene_cfg.py](room_scene_cfg.py) | Isaac Lab scene definition. Each furniture prop is a `RigidObjectCfg` spawned via `_spawn_real_rigid_usd()`. The Ridgeback is an `ArticulationCfg`. Kinematic prop configs use `_kinematic_usd_cfg`. |
| [room_events.py](room_events.py) | The "Director". On reset: hides duplicate visual props, then runs 3-phase OBB placement (wall props → table group → tabletop objects) |
| [room_env_cfg.py](room_env_cfg.py) | Master configuration tying Scene + Events + sim settings. Uses GPU PhysX with Fabric. |
| [run_randomizer.py](run_randomizer.py) | Executable launcher. Boots Isaac Sim, injects dummy managers, runs reset/step loop with optional camera capture. |
| [paths.py](paths.py) | Location-independent path resolution for the room shell USD file. |
| [test_placement.py](test_placement.py) | Standalone test (no Isaac Sim). Runs 6 room randomizations with matplotlib OBB visualization. |
| [get_bounding_boxes.py](get_bounding_boxes.py) | Utility to compute axis-aligned bounding boxes from USD assets on the Omniverse CDN. |

### Asset Summary

| Category | Count | Spawn Type | Notes |
|----------|------:|------------|-------|
| Static (ground, light, room shell) | 3 | GroundPlane / DomeLight / UsdFile | Room shell is `new_base_room.usda` |
| Table group (desk, chair, ridgeback) | 3 | RigidObjectCfg / ArticulationCfg | Full USD meshes, kinematic |
| Wall props | 8 | RigidObjectCfg | Full USD meshes from CDN, kinematic |
| Tabletop objects | 3 | RigidObjectCfg | Coffee cup, desk lamp, portable box |
| Camera | 1 | PinholeCamera | Top-down view for debugging |
| **Total scene fields** | **18** | | |

---

## 2. What Is Left for Full RL Functionality?

The environment perfectly randomizes the room layout, but the "Agent" (the Ridgeback robot) has no brain, no eyes, and no muscles. To use this for Reinforcement Learning (RL), you need to implement the following:

### A. Action Manager (Muscles)
Currently, the environment uses a dummy action space. You need to configure how the RL agent controls the Ridgeback robot.
* **Task:** Define the `ActionManagerCfg` in `room_env_cfg.py`.
* **Implementation:** The Ridgeback is already an `ArticulationCfg`. Configure a `DifferentialDriveActionCfg` or `JointVelocityActionCfg` targeting its wheel joints.

### B. Observation Manager (Eyes)
Currently, the environment returns a dummy observation. The RL agent needs to know its state to make decisions.
* **Task:** Define the `ObservationManagerCfg` in `room_env_cfg.py`.
* **Implementation:** Add observation terms:
  * Proprioception: The robot's current velocity.
  * State: The robot's position and heading.
  * Task: Relative distance/vector to the target.
  * Exteroception: (Optional) 2D Lidar or camera data for vision-based navigation.

### C. Reward Manager (Motivation)
RL agents learn via rewards and penalties. This is currently missing.
* **Task:** Define a `RewardManagerCfg`.
* **Implementation:** Reward terms such as:
  * Progress reward for moving closer to target.
  * Success reward for reaching the desk.
  * Collision penalty for hitting wall props or chair.
  * Time penalty to encourage efficiency.

### D. Termination Manager (Episode Rules)
The simulation needs to know when an episode is "over" so it can trigger the `randomize_room_layout` event again.
* **Task:** Define a `TerminationManagerCfg`.
* **Implementation:**
  * Terminate on reaching the desk.
  * Terminate on max time limit (e.g., 500 steps).
  * Terminate on collision or out-of-bounds.

### E. RL Wrapper & Training Loop
Once the environment has Actions, Observations, and Rewards, connect it to an RL library.
* **Task:** Create a `train.py` script.
* **Implementation:** Use `skrl` or `rsl_rl`. Wrap the Isaac Lab environment with their vector env wrappers and run PPO training.

---

## 3. Data Flow on Reset

```
  ManagerBasedEnv           randomize_room_layout          placement_utils       GPU PhysX
       |                            |                           |                   |
       |--- on_reset(env_ids) ----->|                           |                   |
       |                            |                           |                   |
       |                     [First reset only]                 |                   |
       |                            |--- MakeInvisible -------->|                   |
       |                            |    (10 duplicate prims)   |                   |
       |                            |                           |                   |
       |                     [Phase 1: Wall Props]              |                   |
       |                            |   (repeat x8 wall props)  |                   |
       |                            |--- OBB sampling + SAT --->|                   |
       |                            |--- write_root_state ------|------------------>|
       |                            |                           |                   |
       |                     [Phase 2: Table Group]             |                   |
       |                            |--- sample desk + orbit -->|                   |
       |                            |--- write_root_state ------|------------------>|
       |                            |    (desk, chair, robot)   |                   |
       |                            |                           |                   |
       |                     [Phase 3: Tabletop Objects]        |                   |
       |                            |   (repeat x3 objects)     |                   |
       |                            |--- rejection-sample ----->|                   |
       |                            |--- write_root_state ------|------------------>|
       |                            |                           |                   |
```

---

## 4. Current File Structure

```
room_randomizer_lab/
├── __init__.py               # Exports RoomEnvCfg, RoomSceneCfg
├── constants.py              # OBB sizes, wall zones, orbit offsets, asset paths
├── placement_utils.py        # SAT collision, quaternion math, build_root_state
├── room_scene_cfg.py         # @configclass RoomSceneCfg — 18 asset fields (real USD)
├── room_events.py            # 3-phase placement + duplicate visual hiding
├── room_env_cfg.py           # @configclass RoomEnvCfg — master config (GPU PhysX)
├── run_randomizer.py         # Launcher script (AppLauncher + sim loop)
├── paths.py                  # Room shell USD path resolution
├── test_placement.py         # Standalone matplotlib OBB test (no Isaac Sim)
├── get_bounding_boxes.py     # USD bounding box computation utility
├── execution_flow.md         # Step-by-step runtime trace
├── project_architecture.md   # This file — architecture overview + RL roadmap
└── transition_guide.md       # Migration reference from original nbr_gen.py
```
