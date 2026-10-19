"""CPU-only shutdown regression tests; no Isaac Sim or DDS is imported."""

import multiprocessing
from pathlib import Path
import subprocess
import sys
import time
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.process_cleanup import cleanup_child_processes


def sleep_child():
    time.sleep(60)


class ProcessCleanupTests(unittest.TestCase):
    def test_cleanup_leaves_unrelated_matching_command_alive(self):
        unrelated = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)", "sim_main.py"])
        owned = multiprocessing.get_context("spawn").Process(target=sleep_child)
        owned.start()
        try:
            cleanup_child_processes(timeout=0.1)
            self.assertFalse(owned.is_alive())
            self.assertIsNone(unrelated.poll())
        finally:
            if owned.is_alive():
                owned.kill()
                owned.join()
            unrelated.terminate()
            unrelated.wait(timeout=5)

    def test_no_children_is_safe(self):
        with patch("tools.process_cleanup.multiprocessing.active_children", return_value=[]):
            cleanup_child_processes(timeout=0)

    def test_exited_child_is_only_joined(self):
        child = Mock()
        child.is_alive.return_value = False
        with patch("tools.process_cleanup.multiprocessing.active_children", return_value=[child]):
            cleanup_child_processes(timeout=0)
        child.terminate.assert_not_called()
        child.kill.assert_not_called()

    def test_unresponsive_owned_child_is_killed(self):
        child = Mock()
        child.is_alive.return_value = True
        with patch("tools.process_cleanup.multiprocessing.active_children", return_value=[child]):
            cleanup_child_processes(timeout=0)
        child.terminate.assert_called_once()
        child.kill.assert_called_once()


if __name__ == "__main__":
    unittest.main()
