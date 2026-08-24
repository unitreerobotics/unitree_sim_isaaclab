# room_env_cfg.py
# Top-level environment configuration for RL training.
# Combines the scene, event terms, and simulation parameters.

from __future__ import annotations

from isaaclab.envs import ManagerBasedEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.utils import configclass

from .room_events import randomize_room_layout
from .room_scene_cfg import RoomSceneCfg


# ------------------------------------------------------------------
# Event configuration
# ------------------------------------------------------------------

@configclass
class RoomEventCfg:
    """Events triggered on episode reset."""

    randomize_layout = EventTerm(
        func=randomize_room_layout,
        mode="reset",
        params={
            "wall_prop_names": [
                "medical_cabinet",
                "shelf_set",
                "supply_cabinet",
                "supply_cart_a",
                "supply_cart_b",
                "trash_can",
                "plant_a",
                "plant_b",
            ],
            "table_prop_names": [
                "coffee_cup",
                "desk_lamp",
                "box_portable",
            ],
            "min_table_objects": 2,
        },
    )


# ------------------------------------------------------------------
# Environment configuration
# ------------------------------------------------------------------

@configclass
class RoomEnvCfg(ManagerBasedEnvCfg):
    """Hospital room environment for pick-and-place RL training.

    On every episode reset, the full room layout is re-randomized:
    - Wall props are shuffled into wall slots.
    - The table group (desk + chair + ridgeback) is placed at a random
      position and yaw, constrained to the room interior.
    - 2–3 tabletop objects are scattered on the desk surface.

    Observation and action managers should be added here for the
    specific RL task (e.g., camera observations, gripper actions).
    """

    scene: RoomSceneCfg = RoomSceneCfg(num_envs=20, env_spacing=16.0)
    events: RoomEventCfg = RoomEventCfg()

    # TODO: Add observation and action managers for your RL task.
    # observations = ObservationsCfg()
    # actions = ActionsCfg()

    def __post_init__(self):
        """Simulation defaults."""
        self.decimation = 2
        self.sim.dt = 1.0 / 120.0
        # PhysX GPU tensors are enabled since we are using clean USD assets from CDN
        # instead of the duplicated ones with embedded collision schemas.
        self.sim.device = "cuda:0"
        self.sim.use_fabric = True

        # Viewer defaults for debugging.
        self.viewer.origin_type = "world"
        self.viewer.eye = (-7.5, -3.2, 4.2)
        self.viewer.lookat = (-7.5, -7.6, 0.8)
