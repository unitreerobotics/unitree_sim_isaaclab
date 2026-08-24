from __future__ import annotations

import os
import re
import unittest
from pathlib import Path
from types import SimpleNamespace

from tools.meta_quest import (
    CAMERA_SENSOR_NAMES,
    META_QUEST_REDBLOCK_PROFILES,
    MetaQuestConfigurationError,
    configure_meta_quest,
)
from tools.camera_optics import HOSPITAL_FRONT_CAMERA_OPTICS
from tools.data_convert import convert_to_gripper_range, convert_to_joint_range
from tools.teleimager_compat import configure_camera_transports


def _args(task: str, **overrides):
    values = {
        "meta_quest": True,
        "task": task,
        "no_render": False,
        "replay_data": False,
        "enable_dex1_dds": False,
        "enable_dex3_dds": False,
        "enable_inspire_dds": False,
        "robot_type": "g129",
        "action_source": "dds",
        "enable_cameras": False,
        "camera_include": "",
        "camera_write_interval": None,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


class MetaQuestRedBlockTests(unittest.TestCase):
    def test_dex1_dds_conversion_reaches_the_validated_full_stroke(self):
        self.assertAlmostEqual(convert_to_joint_range(5.4), -0.02)
        self.assertAlmostEqual(convert_to_joint_range(0.0), 0.0245)
        self.assertAlmostEqual(convert_to_gripper_range(-0.02), 5.4)
        self.assertAlmostEqual(convert_to_gripper_range(0.0245), 0.0)

    def test_dds_gripper_targets_compensate_the_action_default_offset(self):
        source = (
            Path(__file__).resolve().parents[1]
            / "action_provider/action_provider_dds.py"
        ).read_text(encoding="utf-8")
        self.assertIn("self._gripper_default_offsets", source)
        self.assertIn("gp_vals = gp_vals - self._gripper_default_offsets", source)

    def test_all_registered_redblock_tasks_have_profiles(self):
        repo_root = Path(__file__).resolve().parents[1]
        registered = set()
        for init_file in (repo_root / "tasks").rglob("__init__.py"):
            source = init_file.read_text(encoding="utf-8")
            task_ids = [
                task_id
                for task_id in re.findall(r'id\s*=\s*["\']([^"\']+)["\']', source)
                if any(
                    token in task_id.lower()
                    for token in ("redblock", "medicinebottle")
                )
            ]
            registered.update(task_ids)
            if task_ids:
                config_sources = "\n".join(
                    path.read_text(encoding="utf-8")
                    for path in init_file.parent.glob("*env_cfg.py")
                )
                for sensor_name in (
                    "front_camera",
                    "left_wrist_camera",
                    "right_wrist_camera",
                    "camera_image",
                ):
                    self.assertIn(sensor_name, config_sources, f"{task_ids}: missing {sensor_name}")
        self.assertEqual(registered, set(META_QUEST_REDBLOCK_PROFILES))

    def test_every_verified_task_enables_matching_hand_and_cameras(self):
        for task, expected in META_QUEST_REDBLOCK_PROFILES.items():
            with self.subTest(task=task):
                args = _args(task)
                environ = {}
                profile = configure_meta_quest(args, environ)

                self.assertEqual(profile, expected)
                self.assertEqual(args.robot_type, expected.robot_type)
                self.assertTrue(getattr(args, expected.hand_flag))
                self.assertEqual(
                    sum(
                        bool(getattr(args, flag))
                        for flag in ("enable_dex1_dds", "enable_dex3_dds", "enable_inspire_dds")
                    ),
                    1,
                )
                self.assertTrue(args.enable_cameras)
                self.assertEqual(args.camera_include, CAMERA_SENSOR_NAMES)
                self.assertEqual(args.camera_write_interval, 1)
                self.assertEqual(environ["TELEIMAGER_DISABLE_WEBRTC"], "1")
                if expected.quest_view_preset is None:
                    self.assertNotIn("TELEIMAGER_HEAD_QUEST_VIEW_PRESET", environ)
                else:
                    self.assertEqual(
                        environ["TELEIMAGER_HEAD_QUEST_VIEW_PRESET"],
                        expected.quest_view_preset,
                    )
                self.assertEqual(
                    environ.get("TELEIMAGER_ENABLE_SIM_TORSO_CONTROL") == "1",
                    expected.sim_torso_control,
                )

    def test_no_render_is_rejected_but_headless_is_not(self):
        task = "Isaac-PickPlace-RedBlock-G129-Dex1-Joint"
        with self.assertRaisesRegex(MetaQuestConfigurationError, "Use --headless"):
            configure_meta_quest(_args(task, no_render=True), {})

        args = _args(task, headless=True)
        self.assertIsNotNone(configure_meta_quest(args, {}))

    def test_wrong_hand_is_rejected(self):
        task = "Isaac-PickPlace-RedBlock-G129-Dex3-Joint"
        with self.assertRaisesRegex(MetaQuestConfigurationError, "requires --enable_dex3_dds"):
            configure_meta_quest(_args(task, enable_dex1_dds=True), {})

    def test_replay_is_rejected(self):
        task = "Isaac-PickPlace-RedBlock-G129-Dex1-Joint"
        with self.assertRaisesRegex(MetaQuestConfigurationError, "live DDS mode"):
            configure_meta_quest(_args(task, replay_data=True), {})

    def test_unverified_task_is_rejected(self):
        with self.assertRaisesRegex(MetaQuestConfigurationError, "does not have a verified profile"):
            configure_meta_quest(_args("Isaac-Unknown-Task"), {})

    def test_non_quest_launch_is_untouched(self):
        args = _args("Isaac-Unknown-Task", meta_quest=False)
        environ = dict(os.environ)
        before = vars(args).copy(), environ.copy()
        self.assertIsNone(configure_meta_quest(args, environ))
        self.assertEqual(vars(args), before[0])
        self.assertEqual(environ, before[1])

    def test_pinned_teleimager_config_disables_direct_webrtc(self):
        config = {
            "head_camera": {"enable_zmq": True, "enable_webrtc": True},
            "left_wrist_camera": {"enable_zmq": True, "enable_webrtc": True},
            "right_wrist_camera": {"enable_zmq": True, "enable_webrtc": True},
        }

        configured = configure_camera_transports(
            config, {"TELEIMAGER_DISABLE_WEBRTC": "1"}
        )

        self.assertTrue(all(camera["enable_zmq"] for camera in configured.values()))
        self.assertTrue(all(not camera["enable_webrtc"] for camera in configured.values()))

    def test_hospital_profile_adds_arm_panel_and_sim_torso_control(self):
        task = "Isaac-PickPlace-MedicineBottle-Hospital-G129-Dex1-Joint"
        environ = {}
        configure_meta_quest(_args(task), environ)
        config = {
            "head_camera": {"enable_zmq": True, "enable_webrtc": True},
            "left_wrist_camera": {"enable_zmq": True, "enable_webrtc": True},
            "right_wrist_camera": {"enable_zmq": True, "enable_webrtc": True},
        }

        configured = configure_camera_transports(config, environ)

        self.assertEqual(
            configured["head_camera"]["quest_view"],
            {
                "preset": "arm_work_panel",
                "horizontal_fov_degrees": 82.0,
                "distance": 2.0,
                "vertical_offset": -0.10,
            },
        )
        self.assertEqual(
            configured["head_camera"]["sim_torso_control"],
            {
                "enabled": True,
                "deadzone": 0.15,
                "max_yaw_speed_rad_s": 0.80,
                "max_yaw_angle_rad": 2.20,
                "max_pitch_speed_rad_s": 0.35,
                "min_pitch_angle_rad": 0.0,
                "max_pitch_angle_rad": 0.45,
            },
        )

    def test_hospital_front_camera_optics_match_the_saved_isaac_view(self):
        optics = HOSPITAL_FRONT_CAMERA_OPTICS
        self.assertEqual(
            (
                optics.focal_length,
                optics.focus_distance,
                optics.horizontal_aperture,
                optics.width,
                optics.height,
            ),
            (4.5, 400.0, 10.0, 640, 480),
        )
        self.assertAlmostEqual(optics.horizontal_fov_degrees, 96.0255750084)
        self.assertAlmostEqual(optics.vertical_fov_degrees, 79.6111421845)

    def test_hospital_torso_yaw_and_pitch_are_unlocked_and_mapped_from_dds(self):
        repo_root = Path(__file__).resolve().parents[1]
        task_source = (
            repo_root
            / "tasks/g1_tasks/pickplace_medicine_bottle_hospital_g1_29dof_dex1"
            / "pickplace_medicine_bottle_hospital_g1_29dof_dex1_joint_env_cfg.py"
        ).read_text(encoding="utf-8")
        provider_source = (
            repo_root / "action_provider/action_provider_dds.py"
        ).read_text(encoding="utf-8")

        self.assertIn(
            'joint_names_expr=["waist_yaw_joint", "waist_pitch_joint"]',
            task_source,
        )
        self.assertIn('joint_names_expr=["waist_roll_joint"]', task_source)
        self.assertIn('self._waist_yaw_source_index = 12', provider_source)
        self.assertIn('self._waist_pitch_source_index = 14', provider_source)
        self.assertIn(
            'full_action[self._waist_pitch_target_index] = '
            'self._positions_buf[self._waist_pitch_source_index]',
            provider_source,
        )

    def test_hospital_full_reset_enables_table_randomization(self):
        repo_root = Path(__file__).resolve().parents[1]
        config_path = (
            repo_root
            / "tasks/g1_tasks/pickplace_medicine_bottle_hospital_g1_29dof_dex1"
            / "pickplace_medicine_bottle_hospital_g1_29dof_dex1_joint_env_cfg.py"
        )
        source = config_path.read_text(encoding="utf-8")

        self.assertIn("randomize_table_position: bool | None = None", source)
        self.assertIn("env._teleop_randomize_table_position", source)
        self.assertIn("env_ids: torch.Tensor | None,", source)
        self.assertNotIn('params={"randomize_table_position": None}', source)
        self.assertRegex(
            source,
            r'register\("reset_all_self"[\s\S]+randomize_table_position=True',
        )
        self.assertRegex(
            source,
            r'def reset_hospital_tabletop_props[\s\S]+for asset_name in '
            r'ACTIVE_MEDICAL_OBJECT_NAMES[\s\S]+reset_target_on_current_table',
        )

    def test_quest_fixed_table_room_reset_is_routed_to_both_hospital_tasks(self):
        repo_root = Path(__file__).resolve().parents[1]
        sim_source = (repo_root / "sim_main.py").read_text(encoding="utf-8")
        medicine_bottle_source = (
            repo_root
            / "tasks/g1_tasks/pickplace_medicine_bottle_hospital_g1_29dof_dex1"
            / "pickplace_medicine_bottle_hospital_g1_29dof_dex1_joint_env_cfg.py"
        ).read_text(encoding="utf-8")
        ridgeback_source = (
            repo_root
            / "tasks/g1_tasks/pick_place_cylinder_g1_29dof_dex1"
            / "pickplace_cylinder_g1_29dof_dex1_joint_env_cfg.py"
        ).read_text(encoding="utf-8")

        self.assertRegex(
            sim_source,
            r"reset_category == '3'[\s\S]+reset_room_fixed_table_self",
        )
        for source in (medicine_bottle_source, ridgeback_source):
            self.assertIn('"reset_room_fixed_table_self"', source)
            self.assertIn("randomize_table_position=False", source)
        self.assertIn(
            "env._teleop_randomize_table_position = False",
            medicine_bottle_source,
        )

    def test_quest_a_routes_to_the_hospital_ridgeback_arc_reset(self):
        repo_root = Path(__file__).resolve().parents[1]
        sim_source = (repo_root / "sim_main.py").read_text(encoding="utf-8")
        medicine_bottle_source = (
            repo_root
            / "tasks/g1_tasks/pickplace_medicine_bottle_hospital_g1_29dof_dex1"
            / "pickplace_medicine_bottle_hospital_g1_29dof_dex1_joint_env_cfg.py"
        ).read_text(encoding="utf-8")

        self.assertRegex(
            sim_source,
            r"reset_category == '4'[\s\S]+reset_ridgeback_arc_self",
        )
        self.assertIn('"reset_ridgeback_arc_self"', medicine_bottle_source)
        self.assertIn("def reset_hospital_ridgeback_arc", medicine_bottle_source)

    def test_hospital_success_notifies_quest_torso_recenter(self):
        repo_root = Path(__file__).resolve().parents[1]
        sim_source = (repo_root / "sim_main.py").read_text(encoding="utf-8")
        goal_source = (
            repo_root
            / "tasks/g1_tasks/pickplace_medicine_bottle_hospital_g1_29dof_dex1"
            / "mdp/container_goal.py"
        ).read_text(encoding="utf-8")
        self.assertIn('"rt/isaaclab/hospital_success"', sim_source)
        self.assertIn('String_(data="reset_like_y")', sim_source)
        self.assertIn("env._hospital_success_reset_pending = True", goal_source)
        self.assertIn("env._teleop_randomize_table_position = False", goal_source)


if __name__ == "__main__":
    unittest.main()
