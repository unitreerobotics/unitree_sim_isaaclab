from __future__ import annotations

import importlib.util
import subprocess
import sys
import unittest
from pathlib import Path

from tools.medical_object_catalog import (
    MEDICAL_OBJECT_SPECS,
    ROLE_DISTRACTOR,
    ROLE_EXCLUDED,
    ROLE_IMPORTANT,
    default_roles,
    parse_roles,
    roles_json,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
SELECTOR_PATH = REPO_ROOT / "tools/select_table_objects.py"
MEDICAL_OBJECTS_USDA = Path("/home/vilmos/isaac-sim/isaac-projects/new_base_room.usda")
LAUNCHER_PATH = REPO_ROOT / "tools/start_redblocks_meta_quest.sh"
HOSPITAL_CONFIG_PATH = (
    REPO_ROOT
    / "tasks/g1_tasks/pickplace_medicine_bottle_hospital_g1_29dof_dex1"
    / "pickplace_medicine_bottle_hospital_g1_29dof_dex1_joint_env_cfg.py"
)


def _load_selector_module():
    module_name = "_table_object_selector_test"
    spec = importlib.util.spec_from_file_location(module_name, SELECTOR_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load table-object selector")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


class TableObjectSelectorTests(unittest.TestCase):
    def test_pool_is_every_asset_in_the_authored_medical_objects_scope(self):
        selector = _load_selector_module()
        objects = selector.medical_objects_from_usda(MEDICAL_OBJECTS_USDA)
        self.assertEqual(
            [(item.prim_name, item.scene_name, item.asset_url) for item in objects],
            [
                (spec.prim_name, spec.scene_name, spec.asset_url)
                for spec in MEDICAL_OBJECT_SPECS
            ],
        )

    def test_list_mode_is_headless_and_includes_default_roles(self):
        result = subprocess.run(
            [
                sys.executable,
                str(SELECTOR_PATH),
                "--usd",
                str(MEDICAL_OBJECTS_USDA),
                "--list",
            ],
            check=True,
            capture_output=True,
            text=True,
            cwd=REPO_ROOT,
        )
        self.assertEqual(
            result.stdout.splitlines(),
            [
                f"{spec.scene_name}\t{spec.prim_name}\t{spec.default_role}"
                for spec in MEDICAL_OBJECT_SPECS
            ],
        )
        self.assertEqual(result.stderr, "")

    def test_roles_round_trip_and_require_an_important_object(self):
        roles = {spec.scene_name: ROLE_EXCLUDED for spec in MEDICAL_OBJECT_SPECS}
        roles["coffee_cup"] = ROLE_IMPORTANT
        roles["felt_pen_pink"] = ROLE_DISTRACTOR
        self.assertEqual(parse_roles(roles_json(roles)), roles)
        with self.assertRaisesRegex(ValueError, "at least one important"):
            parse_roles('{"important":[],"distractor":["coffee_cup"]}')

    def test_default_roles_preserve_the_previous_six_object_task(self):
        roles = default_roles()
        self.assertEqual(
            {name for name, role in roles.items() if role == ROLE_IMPORTANT},
            {"pill_bottle_t", "pill_bottle_v"},
        )
        self.assertEqual(
            {name for name, role in roles.items() if role == ROLE_DISTRACTOR},
            {"medical_bottle_a", "medical_bottle_f", "marker_blue", "marker_yellow"},
        )

    def test_launcher_exports_roles_only_to_the_simulator(self):
        source = LAUNCHER_PATH.read_text(encoding="utf-8")
        simulator_block = source.split("run_simulator()", 1)[1].split(
            "run_xr_bridge()", 1
        )[0]
        bridge_block = source.split("run_xr_bridge()", 1)[1].split(
            "launch_terminal()", 1
        )[0]

        self.assertIn(
            'object_roles="$(python3 "$TABLE_OBJECT_SELECTOR" --usd "$MEDICAL_OBJECTS_USDA")"',
            source,
        )
        self.assertIn('export HOSPITAL_OBJECT_ROLES="$object_roles"', simulator_block)
        self.assertNotIn("HOSPITAL_OBJECT_ROLES", bridge_block)
        expected_command = "\n".join(
            (
                "python sim_main.py \\",
                '        --device "$device" \\',
                '        "${render_args[@]}" \\',
                "        --meta_quest \\",
                '        --task "$task"',
            )
        )
        self.assertIn(expected_command, simulator_block)

    def test_hospital_task_uses_roles_for_spawning_placement_and_goal(self):
        source = HOSPITAL_CONFIG_PATH.read_text(encoding="utf-8")
        self.assertIn("MEDICAL_OBJECT_ROLES = parse_roles", source)
        self.assertIn("IMPORTANT_MEDICAL_OBJECT_NAMES", source)
        self.assertIn("DISTRACTOR_MEDICAL_OBJECT_NAMES", source)
        self.assertIn("ACTIVE_MEDICAL_OBJECT_NAMES", source)
        self.assertIn("table_prop_names=list(ACTIVE_MEDICAL_OBJECT_NAMES)", source)
        self.assertIn("ACTIVE_MEDICAL_TABLETOP_REGION", source)
        self.assertIn("func=mdp.all_important_objects_contained", source)
        self.assertIn("func=mdp.compute_important_object_reward", source)
        for spec in MEDICAL_OBJECT_SPECS:
            self.assertIn(f'"{spec.scene_name}",', source)


if __name__ == "__main__":
    unittest.main()
