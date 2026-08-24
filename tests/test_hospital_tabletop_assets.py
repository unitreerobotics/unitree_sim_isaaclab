from __future__ import annotations

import importlib.util
import math
import random
import re
import sys
import types
import unittest
from pathlib import Path

import torch
from pxr import Usd, UsdGeom, UsdPhysics

from tools.medical_object_catalog import (
    MEDICAL_OBJECT_SPECS,
    ROLE_IMPORTANT,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
ASSET_DIR = REPO_ROOT / "assets" / "objects"
ROOM_RANDOMIZER_DIR = REPO_ROOT / "tasks" / "utils" / "room_randomizer"
ROOM_RANDOMIZER_TEST_PACKAGE = "_hospital_tabletop_room_randomizer_test"
GOAL_PATH = (
    REPO_ROOT
    / "tasks/g1_tasks/pickplace_medicine_bottle_hospital_g1_29dof_dex1"
    / "mdp/container_goal.py"
)


def _load_room_randomizer_module(name: str):
    package = sys.modules.get(ROOM_RANDOMIZER_TEST_PACKAGE)
    if package is None:
        package = types.ModuleType(ROOM_RANDOMIZER_TEST_PACKAGE)
        package.__path__ = [str(ROOM_RANDOMIZER_DIR)]
        sys.modules[ROOM_RANDOMIZER_TEST_PACKAGE] = package
    qualified_name = f"{ROOM_RANDOMIZER_TEST_PACKAGE}.{name}"
    spec = importlib.util.spec_from_file_location(
        qualified_name, ROOM_RANDOMIZER_DIR / f"{name}.py"
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load room-randomizer module {name}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[qualified_name] = module
    spec.loader.exec_module(module)
    return module


def _load_goal_module():
    module_name = "_hospital_pill_container_goal_test"
    spec = importlib.util.spec_from_file_location(module_name, GOAL_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load pill-container goal module")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


class HospitalTabletopAssetTests(unittest.TestCase):
    CONFIG_PATH = (
        REPO_ROOT
        / "tasks/g1_tasks/pickplace_medicine_bottle_hospital_g1_29dof_dex1"
        / "pickplace_medicine_bottle_hospital_g1_29dof_dex1_joint_env_cfg.py"
    )

    @classmethod
    def setUpClass(cls):
        cls.config_source = cls.CONFIG_PATH.read_text(encoding="utf-8")

    def test_meta_quest_hospital_scene_has_all_requested_props(self):
        expected_props = tuple(spec.scene_name for spec in MEDICAL_OBJECT_SPECS)
        for prop_name in expected_props:
            self.assertRegex(
                self.config_source,
                rf"\b{prop_name}(?:\s*:\s*RigidObjectCfg\s*\|\s*None)?\s*="
                rf"\s*_selected_hospital_prop_cfg\(",
            )
            self.assertIn(f'"{prop_name}"', self.config_source)
        self.assertIn("object = None", self.config_source)
        self.assertIn("blue_cube = None", self.config_source)
        self.assertIn("yellow_cube = None", self.config_source)
        self.assertIn(
            "ACTIVE_MEDICAL_OBJECT_NAMES = (\n"
            "    IMPORTANT_MEDICAL_OBJECT_NAMES + DISTRACTOR_MEDICAL_OBJECT_NAMES\n)",
            self.config_source,
        )

    def test_screenshot_prop_specs_are_explicit_and_natural_scale(self):
        self.assertEqual(len(MEDICAL_OBJECT_SPECS), 10)
        self.assertEqual(len({spec.scene_name for spec in MEDICAL_OBJECT_SPECS}), 10)
        for spec in MEDICAL_OBJECT_SPECS:
            with self.subTest(prop=spec.scene_name):
                self.assertTrue(spec.asset_url.endswith(f"/{spec.prim_name}.usd"))
                self.assertGreater(spec.mass, 0.0)
                self.assertGreater(spec.bbox_half_xy[0], 0.0)
                self.assertGreater(spec.bbox_half_xy[1], 0.0)
        self.assertIn("for catalog_spec in MEDICAL_OBJECT_SPECS", self.config_source)
        self.assertIn("usd_path=catalog_spec.asset_url", self.config_source)
        self.assertIn("scale=(spec.scale, spec.scale, spec.scale)", self.config_source)

    def test_every_prop_uses_a_convex_decomposition_rigid_body(self):
        spawner = re.search(
            r"def _spawn_graspable_hospital_usd[\s\S]+?(?=\n\n@configclass)",
            self.config_source,
        )
        self.assertIsNotNone(spawner)
        source = spawner.group(0)
        self.assertIn("UsdPhysics.CollisionAPI.Apply(collider_prim)", source)
        self.assertIn("UsdPhysics.MeshCollisionAPI.Apply(collider_prim)", source)
        self.assertIn("UsdPhysics.Tokens.convexDecomposition", source)
        self.assertIn("PhysxSchema.PhysxCollisionAPI.Apply(collider_prim)", source)
        self.assertIn("schemas.define_rigid_body_properties", source)
        self.assertIn("schemas.define_mass_properties", source)
        self.assertIn("rigid_roots != [root_prim]", source)
        self.assertNotIn("UsdGeom.Cylinder", source)

    def test_only_the_two_pill_bottles_need_dex1_safe_aperture(self):
        important = [
            spec for spec in MEDICAL_OBJECT_SPECS if spec.default_role == ROLE_IMPORTANT
        ]
        self.assertEqual(
            [spec.scene_name for spec in important],
            ["pill_bottle_t", "pill_bottle_v"],
        )
        for spec in important:
            self.assertLessEqual(2.0 * max(spec.bbox_half_xy), 0.030)

    def test_randomized_yaw_can_preserve_a_base_orientation(self):
        _load_room_randomizer_module("constants")
        placement_utils = _load_room_randomizer_module("placement_utils")
        default_state = torch.zeros((2, 13), dtype=torch.float32)
        default_state[:, 3] = 1.0
        result = placement_utils.build_root_state(
            pos=torch.zeros((2, 3), dtype=torch.float32),
            yaw_rad=torch.tensor([0.0, math.pi / 2], dtype=torch.float32),
            env_origins=torch.zeros((2, 3), dtype=torch.float32),
            env_ids=torch.tensor([0, 1]),
            default_state=default_state,
            base_orientation_wxyz=(0.70710678, 0.70710678, 0.0, 0.0),
        )
        expected = torch.tensor(
            [
                [0.70710678, 0.70710678, 0.0, 0.0],
                [0.5, 0.5, 0.5, 0.5],
            ],
            dtype=torch.float32,
        )
        torch.testing.assert_close(result[:, 3:7], expected, atol=1.0e-6, rtol=0.0)

    def test_reliable_grasp_contact_parameters_are_configured(self):
        for text in (
            'friction_combine_mode="max"',
            'restitution_combine_mode="min"',
            "static_friction=2.5",
            "dynamic_friction=2.0",
            "restitution=0.0",
            "contact_offset=0.001",
            "rest_offset=0.0",
            "linear_damping=1.5",
            "angular_damping=3.0",
            "solver_position_iteration_count=16",
            "solver_velocity_iteration_count=4",
            "func: Callable = _spawn_graspable_hospital_usd",
            "robot.spawn = robot.spawn.replace(func=_spawn_grasp_ready_dex1_usd)",
            "table_prop_meta_overrides=MEDICAL_OBJECT_TABLE_PROP_META_OVERRIDES",
            "tabletop_spawn_region=ACTIVE_MEDICAL_TABLETOP_REGION",
            "static_cluster_members=static_cluster",
        ):
            self.assertIn(text, self.config_source)

    def test_dex1_finger_contacts_get_task_local_material_and_offsets(self):
        spawner = re.search(
            r"def _spawn_grasp_ready_dex1_usd[\s\S]+?(?=\n\n@clone\ndef _spawn_graspable_hospital_usd)",
            self.config_source,
        )
        self.assertIsNotNone(spawner)
        spawner = spawner.group(0)
        self.assertIn('"/left_hand_Link"', spawner)
        self.assertIn('"/right_hand_Link"', spawner)
        self.assertIn(
            "stage.GetPrimAtPath(child_path).SetInstanceable(False)", spawner
        )
        self.assertIn("UsdPhysics.CollisionAPI.Apply(child)", spawner)
        self.assertIn("UsdPhysics.MeshCollisionAPI.Apply(child)", spawner)
        self.assertIn("UsdPhysics.Tokens.convexHull", spawner)
        self.assertIn("PhysxSchema.PhysxCollisionAPI.Apply(child)", spawner)
        self.assertIn('{"left": 6, "right": 6}', spawner)
        self.assertIn("schemas.modify_collision_properties(", spawner)
        self.assertIn("bind_physics_material(", spawner)
        self.assertIn("FINGER_COLLISION_PROPERTIES", spawner)
        self.assertIn("FINGER_PHYSICS_MATERIAL", self.config_source)
        self.assertIn("static_friction=3.0", self.config_source)
        self.assertIn(
            'PILL_GRASP_PAD_RADII = {"left": 0.009, "right": 0.010}',
            self.config_source,
        )
        self.assertIn("pad.CreateRadiusAttr().Set(PILL_GRASP_PAD_RADII[side])", spawner)
        self.assertIn("UsdGeom.Sphere.Define(stage, pad_path)", spawner)
        self.assertIn("finger_collider_paths + grasp_pad_paths", spawner)

    def test_dex1_jaw_drive_is_bounded_for_contact_stability(self):
        for expected in (
            'robot.actuators["hands"] = ImplicitActuatorCfg(',
            "effort_limit_sim=12.0",
            "velocity_limit_sim=0.5",
            "stiffness=600.0",
            "damping=8.0",
            "friction=0.0",
            "self.sim.physx.enable_ccd = False",
        ):
            self.assertIn(expected, self.config_source)

    def test_all_dex1_fingers_start_fully_open(self):
        self.assertIn("DEX1_OPEN_JOINT_POSITION = -0.02", self.config_source)
        for joint_name in (
            "left_hand_Joint1_1",
            "left_hand_Joint2_1",
            "right_hand_Joint1_1",
            "right_hand_Joint2_1",
        ):
            self.assertIn(
                f'"{joint_name}": DEX1_OPEN_JOINT_POSITION',
                self.config_source,
            )

    def test_fixed_table_startup_places_every_configured_tabletop_prop(self):
        reset_source = re.search(
            r"def reset_hospital_teleop_scene[\s\S]+?(?=\ndef reset_hospital_tabletop_props)",
            self.config_source,
        )
        self.assertIsNotNone(reset_source)
        reset_source = reset_source.group(0)
        self.assertIn("randomize_pickplace_room_layout(", reset_source)
        self.assertIn("table_prop_names=list(ACTIVE_MEDICAL_OBJECT_NAMES)", reset_source)
        self.assertIn(
            "min_table_objects=len(ACTIVE_MEDICAL_OBJECT_NAMES)", reset_source
        )
        self.assertIn(
            "randomize_table_position=randomize_table_position", reset_source
        )
        self.assertIn(
            "table_prop_meta_overrides=MEDICAL_OBJECT_TABLE_PROP_META_OVERRIDES",
            reset_source,
        )
        self.assertIn(
            "tabletop_spawn_region=ACTIVE_MEDICAL_TABLETOP_REGION", reset_source
        )
        self.assertIn(
            "tabletop_object_margin=COMPACT_TABLETOP_OBJECT_MARGIN", reset_source
        )
        self.assertNotIn("randomize_wall_props_layout", reset_source)

    def test_scene_replaces_the_legacy_target_with_screenshot_props(self):
        scene_block = re.search(
            r"class HospitalMedicineBottleSceneCfg[\s\S]+?(?=\n\n##\n# MDP)",
            self.config_source,
        )
        self.assertIsNotNone(scene_block)
        scene_block = scene_block.group(0)
        self.assertIn("object = None", scene_block)
        for entity in (spec.scene_name for spec in MEDICAL_OBJECT_SPECS):
            self.assertRegex(
                scene_block,
                rf"{entity}(?::\s*RigidObjectCfg\s*\|\s*None)?\s*="
                rf"\s*_selected_hospital_prop_cfg\(",
            )
        self.assertNotIn("tabletop_cube_cfg", self.config_source)
        self.assertNotIn("_redblock_spawn_cfg", self.config_source)

    def test_compact_spawn_region_is_inside_the_handward_table_area(self):
        region = re.search(
            r"HAND_REACHABLE_TABLETOP_REGION\s*=\s*TabletopSpawnRegion\(([\s\S]+?)\n\)",
            self.config_source,
        )
        self.assertIsNotNone(region)
        values = {
            key: float(value)
            for key, value in re.findall(
                r"(x_min|x_max|y_min|y_max)=(-?[0-9.]+)", region.group(1)
            )
        }
        self.assertEqual(
            values,
            {"x_min": -0.32, "x_max": 0.18, "y_min": -0.16, "y_max": 0.12},
        )
        self.assertIn("COMPACT_TABLETOP_OBJECT_MARGIN = 0.015", self.config_source)
        self.assertLess(
            (values["x_max"] - values["x_min"])
            * (values["y_max"] - values["y_min"]),
            0.18,
        )

    def test_compact_region_places_all_six_across_many_seeds(self):
        constants = _load_room_randomizer_module("constants")
        placement = _load_room_randomizer_module("placement_utils")
        props = (
            constants.BBox(0.014611, 0.014611),
            constants.BBox(0.013665, 0.013665),
            constants.BBox(0.031000, 0.031000),
            constants.BBox(0.030050, 0.030050),
            constants.BBox(0.058500, 0.012140),
            constants.BBox(0.058500, 0.012140),
        )
        bounds = (-0.32, 0.18, -0.16, 0.12)
        for seed in range(1000):
            rng = random.Random(seed)
            placed = []
            for bbox in props:
                for _ in range(300):
                    box = placement.make_obb(
                        rng.uniform(bounds[0], bounds[1]),
                        rng.uniform(bounds[2], bounds[3]),
                        bbox,
                        rng.uniform(0.0, 2.0 * math.pi),
                    )
                    corners = placement.obb_corners(*box)
                    if any(
                        x < bounds[0]
                        or x > bounds[1]
                        or y < bounds[2]
                        or y > bounds[3]
                        for x, y in corners
                    ):
                        continue
                    if placement.obb_overlap_any(box, placed, margin=0.015):
                        continue
                    placed.append(box)
                    break
                else:
                    self.fail(f"seed {seed} could not place all six props")

            # Robot table-local origin is (0.10, 0.50), facing -Y. Check the
            # complete OBB corners rather than only object centers.
            half_horizontal_fov = math.radians(96.0255750084 * 0.5)
            for box in placed:
                for x, y in placement.obb_corners(*box):
                    horizontal_angle = math.atan2(abs(x - 0.10), 0.50 - y)
                    self.assertLess(horizontal_angle, half_horizontal_fov)

    def test_object_only_reset_respawns_all_six_on_the_current_table(self):
        reset_source = re.search(
            r"def reset_hospital_tabletop_props[\s\S]+?(?=\ndef reset_hospital_room_fixed_table)",
            self.config_source,
        )
        self.assertIsNotNone(reset_source)
        source = reset_source.group(0)
        self.assertIn("for asset_name in ACTIVE_MEDICAL_OBJECT_NAMES", source)
        self.assertIn("asset_name=asset_name", source)
        self.assertIn(
            "table_prop_meta_overrides=MEDICAL_OBJECT_TABLE_PROP_META_OVERRIDES",
            source,
        )
        self.assertIn("tabletop_spawn_region=ACTIVE_MEDICAL_TABLETOP_REGION", source)
        self.assertIn("tabletop_object_margin=COMPACT_TABLETOP_OBJECT_MARGIN", source)
        self.assertNotIn("reset_scene_to_default", source)
        self.assertIn(
            'self.event_manager.register("reset_object_self", SimpleEvent(',
            self.config_source,
        )
        self.assertIn("func=reset_hospital_tabletop_props", self.config_source)

    def test_goal_tracks_the_one_parented_crate_in_its_live_frame(self):
        goal = _load_goal_module()
        num_envs = 3
        identity = torch.tensor((1.0, 0.0, 0.0, 0.0)).repeat(num_envs, 1)
        ridgeback_quat = identity.clone()
        ridgeback_quat[1] = torch.tensor(
            (math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))
        )
        ridgeback_pos = torch.tensor(
            ((1.0, 2.0, 0.1), (4.0, -3.0, 0.2), (-2.0, 1.0, 0.0))
        )

        def crate_point(parent_pos, parent_quat, crate_index, local_point):
            local_pos = torch.tensor(goal.CRATE_LOCAL_POSITIONS[crate_index]).repeat(
                num_envs, 1
            )
            local_quat = torch.tensor(
                goal.CRATE_LOCAL_ORIENTATIONS[crate_index]
            ).repeat(num_envs, 1)
            crate_pos = parent_pos + goal.quaternion_apply(parent_quat, local_pos)
            crate_quat = goal.quaternion_multiply(parent_quat, local_quat)
            point = torch.tensor(local_point).repeat(num_envs, 1)
            return crate_pos + goal.quaternion_apply(crate_quat, point)

        crate_inside = crate_point(
            ridgeback_pos, ridgeback_quat, 0, (0.0, 0.0, 0.05)
        )
        bottle_t_center = crate_inside.clone()
        bottle_v_center = torch.stack(
            (crate_inside[0], torch.tensor((50.0, 50.0, 50.0)), crate_inside[2])
        )

        def rigid_data(center, center_offset, root_quat=identity):
            offset = torch.tensor(center_offset).repeat(num_envs, 1)
            root_pos = center - goal.quaternion_apply(root_quat, offset)
            return types.SimpleNamespace(root_pos_w=root_pos, root_quat_w=root_quat)

        scene = {
            "pill_bottle_t": types.SimpleNamespace(
                data=rigid_data(
                    bottle_t_center, goal.PILL_BOTTLE_LOCAL_CENTERS[0]
                )
            ),
            "pill_bottle_v": types.SimpleNamespace(
                data=rigid_data(
                    bottle_v_center, goal.PILL_BOTTLE_LOCAL_CENTERS[1]
                )
            ),
            "ridgeback": types.SimpleNamespace(
                data=types.SimpleNamespace(
                    root_pos_w=ridgeback_pos, root_quat_w=ridgeback_quat
                )
            ),
        }
        env = types.SimpleNamespace(scene=scene, num_envs=num_envs)
        torch.testing.assert_close(
            goal.pill_bottles_contained(env),
            torch.tensor(((True, True), (True, False), (True, True))),
        )
        torch.testing.assert_close(
            goal.both_pill_bottles_contained(env),
            torch.tensor((True, False, True)),
        )
        self.assertFalse(env._teleop_randomize_table_position)
        self.assertTrue(env._hospital_success_reset_pending)

    def test_reward_and_termination_use_the_gui_important_object_goal(self):
        self.assertIn(
            "success = DoneTerm(func=mdp.all_important_objects_contained)",
            self.config_source,
        )
        self.assertIn("func=mdp.compute_important_object_reward", self.config_source)
        self.assertNotIn("post_min_x", self.config_source)
        rewards_source = GOAL_PATH.with_name("rewards.py").read_text(encoding="utf-8")
        self.assertIn("mean(dim=-1)", rewards_source)

    def test_one_static_ridgeback_carries_one_omniverse_crate(self):
        self.assertEqual(self.config_source.count("_static_ridgeback_cfg("), 2)
        self.assertEqual(self.config_source.count("_ridgeback_crate_cfg("), 2)
        self.assertIn('asset_name="ridgeback"', self.config_source)
        self.assertIn("/Ridgeback/base_link/Crate", self.config_source)
        self.assertIn("RidgebackUr/ridgeback_ur5.usd", self.config_source)
        self.assertIn("SM_CratePlastic_D_02.usd", self.config_source)
        self.assertIn("kinematic_enabled=True", self.config_source)
        self.assertIn("disable_gravity=True", self.config_source)
        self.assertNotIn("ArticulationCfg(\n        prim_path=\"/World/envs/env_.*/Ridgeback", self.config_source)

    def test_ridgeback_uses_a_fixed_radius_rear_arc_and_faces_g1(self):
        constants = _load_room_randomizer_module("constants")
        placement = _load_room_randomizer_module("placement_utils")
        self.assertEqual(len(constants.RIDGEBACK_STATIC_ARC_ANGLES), 14)
        self.assertAlmostEqual(constants.RIDGEBACK_STATIC_ARC_RADIUS, 0.85)
        self.assertAlmostEqual(constants.RIDGEBACK_STATIC_TIP_CLEARANCE_MAX, 0.10)
        self.assertEqual(
            tuple(round(math.degrees(angle)) for angle in constants.RIDGEBACK_STATIC_ARC_ANGLES),
            (-20, -15, 15, 20, -25, -30, -35, -40, -45, 25, 30, 35, 40, 45),
        )
        for index in range(len(constants.RIDGEBACK_STATIC_ARC_ANGLES)):
            with self.subTest(arc_index=index):
                local_xy, yaw_offset = constants.ridgeback_static_arc_pose(index)
                radius = math.hypot(*local_xy)
                self.assertAlmostEqual(radius, constants.RIDGEBACK_STATIC_ARC_RADIUS)
                self.assertLessEqual(
                    constants.ridgeback_static_arc_tip_clearance(index),
                    constants.RIDGEBACK_STATIC_TIP_CLEARANCE_MAX,
                )
                self.assertAlmostEqual(
                    constants.ridgeback_static_arc_tip_clearance(index),
                    0.10,
                )
                self.assertAlmostEqual(
                    radius - constants.RIDGEBACK_BBOX.half_w - constants.ROBOT_BBOX.half_w,
                    0.10,
                )
                self.assertLessEqual(local_xy[0], 1.0e-9)
                self.assertNotAlmostEqual(local_xy[1], 0.0)
                self.assertAlmostEqual(math.cos(yaw_offset), -local_xy[0] / radius)
                self.assertAlmostEqual(math.sin(yaw_offset), -local_xy[1] / radius)

    def test_fixed_startup_cluster_uses_the_rear_arc_centre(self):
        for expected in (
            "TABLE_POS = (-6.0, -7.5, -0.2)",
            "ROBOT_POS = (-5.9, -7.0, 0.76)",
            "RIDGEBACK_POS = (-6.190717, -6.201272, 0.0328)",
            "RIDGEBACK_ROT = (0.5735764364, 0.0, 0.0, 0.8191520443)",
            'pill_bottle_t: RigidObjectCfg | None = _selected_hospital_prop_cfg(',
            'pill_bottle_v: RigidObjectCfg | None = _selected_hospital_prop_cfg(',
            'medical_bottle_a: RigidObjectCfg | None = _selected_hospital_prop_cfg(',
            'medical_bottle_f: RigidObjectCfg | None = _selected_hospital_prop_cfg(',
            'marker_blue: RigidObjectCfg | None = _selected_hospital_prop_cfg(',
            'marker_yellow: RigidObjectCfg | None = _selected_hospital_prop_cfg(',
            "pos_offset=(-5.8, -8.2, 1.8)",
        ):
            with self.subTest(expected=expected):
                self.assertIn(expected, self.config_source)

    def test_front_camera_uses_saved_viewport_optics(self):
        camera = re.search(
            r"front_camera\s*=\s*CameraBaseCfg\.get_camera_config\(([\s\S]+?)\n\s*\)",
            self.config_source,
        )
        self.assertIsNotNone(camera)
        camera = camera.group(1)
        self.assertIn("height=HOSPITAL_FRONT_CAMERA_OPTICS.height", camera)
        self.assertIn("width=HOSPITAL_FRONT_CAMERA_OPTICS.width", camera)
        self.assertIn("focal_length=HOSPITAL_FRONT_CAMERA_OPTICS.focal_length", camera)
        self.assertIn("focus_distance=HOSPITAL_FRONT_CAMERA_OPTICS.focus_distance", camera)
        self.assertIn(
            "horizontal_aperture=HOSPITAL_FRONT_CAMERA_OPTICS.horizontal_aperture",
            camera,
        )
        self.assertIn("Euler=(90,-90,0)", self.config_source)

    def test_front_camera_optics_are_reasserted_on_scene_resets(self):
        optics_update = re.search(
            r"def _apply_hospital_front_camera_optics[\s\S]+?(?=\ndef reset_hospital_teleop_scene)",
            self.config_source,
        )
        self.assertIsNotNone(optics_update)
        optics_update = optics_update.group(0)
        for expected in (
            "GetFocalLengthAttr().Set(optics.focal_length)",
            "GetFocusDistanceAttr().Set(optics.focus_distance)",
            "GetHorizontalApertureAttr().Set(optics.horizontal_aperture)",
            "GetVerticalApertureAttr().Set(vertical_aperture)",
        ):
            with self.subTest(expected=expected):
                self.assertIn(expected, optics_update)

        self.assertEqual(
            self.config_source.count("\n    _apply_hospital_front_camera_optics(env)"),
            2,
        )

    def test_hospital_quest_releases_yaw_and_pitch_but_locks_roll(self):
        scene = re.search(
            r"class HospitalMedicineBottleSceneCfg[\s\S]+?(?=\n\n##\n# MDP)",
            self.config_source,
        )
        self.assertIsNotNone(scene)
        scene = scene.group(0)
        for expected in (
            'robot.actuators.pop("waist", None)',
            'robot.actuators["waist_yaw_pitch_teleop"]',
            'joint_names_expr=["waist_yaw_joint", "waist_pitch_joint"]',
            'robot.actuators["waist_roll_lock"]',
            'joint_names_expr=["waist_roll_joint"]',
        ):
            with self.subTest(expected=expected):
                self.assertIn(expected, scene)

    def test_ridgeback_container_stays_parented_to_the_arc_platform(self):
        self.assertEqual(self.config_source.count("_ridgeback_crate_cfg("), 2)
        self.assertEqual(self.config_source.count("_ridgeback_crate_riser_cfg("), 2)
        self.assertIn("pos=(local_x, local_y, local_z)", self.config_source)
        self.assertIn("RIDGEBACK_CONTAINER_RISER_HEIGHT,", self.config_source)
        self.assertIn(
            "CRATE_RISER_SIZE = (0.46, 0.35, RIDGEBACK_CONTAINER_RISER_HEIGHT)",
            self.config_source,
        )
        self.assertIn("RIDGEBACK_BODY_COLOR = (0.15, 0.15, 0.15)", self.config_source)
        self.assertIn("/Ridgeback/base_link/CrateRiser", self.config_source)
        self.assertIn("RIDGEBACK_BODY_YAW_OFFSET = torch.pi", self.config_source)
        self.assertIn("CRATE_LOCAL_POS = (-0.22877, -0.00612, 0.43576)", self.config_source)
        self.assertIn("yaw_offset=yaw_offset + float(RIDGEBACK_BODY_YAW_OFFSET)", self.config_source)
        self.assertIn("CRATE_RISER_LOCAL_POS", self.config_source)
        self.assertIn("local_rot=CRATE_LOCAL_ROT", self.config_source)
        self.assertIn("_next_static_logistics_cluster", self.config_source)

        goal_source = GOAL_PATH.read_text(encoding="utf-8")
        self.assertIn("(-0.22877, -0.00612, 0.43576)", goal_source)
        self.assertIn("(0.7046465942, 0.0, 0.0, 0.7095584382)", goal_source)


if __name__ == "__main__":
    unittest.main()
