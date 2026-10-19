"""CPU-only cleanup tests, with DDS transport replaced by test doubles."""

import ast
import __future__
from multiprocessing import shared_memory
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import Mock
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from dds.dds_base import DDSObject
from dds.sharedmemorymanager import SharedMemoryManager


class DummyDDS(DDSObject):
    def dds_publisher(self):
        pass

    def dds_subscriber(self, *args):
        pass

    def setup_publisher(self):
        pass

    def setup_subscriber(self):
        pass


def manager_class():
    source = ROOT / "dds/dds_master.py"
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "DDSManager")
    ns = {"threading": threading, "time": time, "ChannelFactoryInitialize": Mock()}
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(source), "exec",
                 flags=__future__.annotations.compiler_flag), ns)
    return ns["DDSManager"]


class DDSCleanupTests(unittest.TestCase):
    def test_channels_and_owned_memory_are_released_idempotently(self):
        node = DummyDDS()
        publisher, subscriber = Mock(), Mock()
        memory = SharedMemoryManager(size=128)
        name = memory.get_name()
        node.publisher, node.subscriber, node.input_shm = publisher, subscriber, memory
        node.stop_communication()
        node.stop_communication()
        publisher.Close.assert_called_once()
        subscriber.Close.assert_called_once()
        self.assertIsNone(node.input_shm)
        with self.assertRaises(FileNotFoundError):
            shared_memory.SharedMemory(name=name)

    def test_attached_memory_is_not_unlinked(self):
        owner = shared_memory.SharedMemory(create=True, size=128)
        try:
            attached = SharedMemoryManager(name=owner.name, size=128)
            attached.cleanup()
            attached.cleanup()
            still_present = shared_memory.SharedMemory(name=owner.name)
            still_present.close()
        finally:
            owner.close()
            owner.unlink()

    def test_one_close_error_does_not_skip_other_resources(self):
        node = DummyDDS()
        node.subscriber = Mock()
        node.subscriber.Close.side_effect = RuntimeError("transport failure")
        publisher, memory = Mock(), Mock()
        node.publisher, node.output_shm = publisher, memory
        with self.assertRaises(RuntimeError):
            node.stop_communication()
        publisher.Close.assert_called_once()
        memory.cleanup.assert_called_once()

    def test_worker_has_stopped_before_resources_close(self):
        manager = manager_class()()
        manager.publishing_running = True
        def worker():
            while manager.publishing_running:
                time.sleep(0.001)
        thread = threading.Thread(target=worker)
        manager.publish_thread = thread
        node = Mock()
        node.stop_communication.side_effect = lambda: self.assertFalse(thread.is_alive())
        manager.objects["test"] = node
        thread.start()
        manager.stop_all_communication()
        node.stop_communication.assert_called_once()

    def test_blocked_worker_keeps_resources_alive_for_retry(self):
        manager = manager_class()()
        manager.publish_thread = Mock()
        manager.publish_thread.is_alive.return_value = True
        node = Mock()
        manager.objects["test"] = node
        with self.assertRaises(RuntimeError):
            manager.stop_all_communication()
        node.stop_communication.assert_not_called()

    def test_cleanup_continues_after_one_node_fails(self):
        manager = manager_class()()
        bad, good = Mock(), Mock()
        bad.stop_communication.side_effect = RuntimeError("failed")
        manager.objects.update(bad=bad, good=good)
        manager.stop_all_communication()
        good.stop_communication.assert_called_once()


class ImageCleanupTests(unittest.TestCase):
    def test_writer_unlinks_only_memory_it_created(self):
        import numpy as np
        from tools.shared_memory_utils import MultiImageWriter
        name = "test_" + uuid.uuid4().hex
        writer = MultiImageWriter()
        try:
            writer.write_images({name: np.zeros((8, 8, 3), dtype=np.uint8)})
            shm_name = next(iter(writer.shms))
            writer.close()
            writer.close()
            with self.assertRaises(FileNotFoundError):
                shared_memory.SharedMemory(name=shm_name)
        finally:
            writer.close()

    def test_writer_does_not_unlink_an_existing_segment(self):
        import numpy as np
        from tools.shared_memory_utils import MultiImageWriter
        image_name = "test_" + uuid.uuid4().hex
        owner = shared_memory.SharedMemory(create=True, size=4096, name=f"isaac_{image_name}_image_shm")
        writer = MultiImageWriter()
        try:
            writer.write_images({image_name: np.zeros((8, 8, 3), dtype=np.uint8)})
            writer.close()
            attached = shared_memory.SharedMemory(name=owner.name)
            attached.close()
        finally:
            writer.close()
            owner.close()
            owner.unlink()

    def test_camera_worker_stops_before_writer_is_closed(self):
        import queue
        source = ROOT / "tasks/common_observations/camera_state.py"
        fn = next(n for n in ast.parse(source.read_text()).body
                  if isinstance(n, ast.FunctionDef) and n.name == "close_camera_writer")
        pending = queue.Queue(maxsize=1)
        worker = threading.Thread(target=pending.get)
        writer = Mock()
        writer.close.side_effect = lambda: self.assertFalse(worker.is_alive())
        ns = {"queue": queue, "_async_started": True, "_async_thread": worker,
              "_async_queue": pending, "multi_image_writer": writer}
        exec(compile(ast.Module(body=[fn], type_ignores=[]), str(source), "exec"), ns)
        worker.start()
        ns["close_camera_writer"]()
        writer.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
