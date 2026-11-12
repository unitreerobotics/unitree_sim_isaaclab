"""Exercise the actual observation function with CPU tensors, without Isaac/DDS."""

import ast
import __future__
from pathlib import Path
import random
from types import SimpleNamespace
import unittest

import torch


SOURCE = Path(__file__).resolve().parents[1] / "tasks/common_observations/g1_29dof_state.py"


def load_observation():
    tree = ast.parse(SOURCE.read_text())
    functions = {"get_robot_boy_joint_names", "get_robot_boy_joint_states"}
    nodes = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name in functions]
    cache = next(node.value for node in tree.body if isinstance(node, ast.Assign)
                 and any(isinstance(t, ast.Name) and t.id == "_obs_cache" for t in node.targets))
    namespace = {"torch": torch, "_obs_cache": ast.literal_eval(cache)}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE), "exec",
                 flags=__future__.annotations.compiler_flag), namespace)
    return namespace


def environment(names, batch=1):
    positions = torch.arange(batch * len(names), dtype=torch.float32).reshape(batch, len(names))
    data = SimpleNamespace(joint_names=names, joint_pos=positions,
                           joint_vel=positions + 1000, applied_torque=positions + 2000)
    return SimpleNamespace(scene={"robot": SimpleNamespace(data=data)})


class BodyJointOrderTests(unittest.TestCase):
    def setUp(self):
        self.ns = load_observation()
        self.body = self.ns["get_robot_boy_joint_names"]()

    def check_order(self, names, batch=1):
        env = environment(names, batch)
        actual = self.ns["get_robot_boy_joint_states"](env, enable_dds=False)
        indices = [names.index(name) for name in self.body]
        data = env.scene["robot"].data
        expected = torch.cat([data.joint_pos[:, indices], data.joint_vel[:, indices],
                              data.applied_torque[:, indices]], dim=1)
        torch.testing.assert_close(actual, expected)
        self.assertEqual(tuple(actual.shape), (batch, 87))

    def test_body_only_order(self):
        self.check_order(self.body)

    def test_interleaved_finger_joints_and_batch(self):
        names = self.body + [f"finger_{i}" for i in range(22)]
        random.Random(42).shuffle(names)
        self.check_order(names, batch=3)

    def test_cache_is_invalidated_for_another_order_on_same_device(self):
        self.check_order(self.body)
        self.check_order(list(reversed(self.body)))

    def test_missing_body_joint_is_reported(self):
        with self.assertRaisesRegex(ValueError, "left_hip_pitch_joint"):
            self.ns["get_robot_boy_joint_states"](environment(self.body[1:]), enable_dds=False)

    def test_duplicate_names_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "unique"):
            self.ns["get_robot_boy_joint_states"](environment(self.body + [self.body[0]]), enable_dds=False)


if __name__ == "__main__":
    unittest.main()
