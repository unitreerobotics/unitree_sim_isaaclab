"""Asynchronous MP4 recording for Isaac Lab robot cameras.

The recorder deliberately discovers cameras from the live scene instead of
hard-coding front/wrist sensor names.  A Quest recording session therefore
contains the front camera and any additional camera attached to the robot in
the selected task.
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from queue import Queue
from threading import Thread
from typing import Any

import cv2
import numpy as np


class QuestCameraRecorder:
    """Write one timestamped MP4 file per robot camera without stalling sim."""

    def __init__(self, desktop_directory: Path, frames_per_second: float = 30.0):
        self.desktop_directory = desktop_directory.expanduser()
        self.desktop_directory.mkdir(parents=True, exist_ok=True)
        self.frames_per_second = max(1.0, float(frames_per_second))
        self.is_recording = False
        self.session_directory: Path | None = None
        self._next_capture_time = 0.0
        self._queue: Queue[dict[str, np.ndarray] | None] = Queue(maxsize=2)
        self._writers: dict[str, cv2.VideoWriter] = {}
        self._worker = Thread(target=self._write_frames, daemon=True)
        self._worker.start()

    def toggle(self) -> tuple[bool, Path | None]:
        """Start a fresh session or stop and finalize the current one."""
        if self.is_recording:
            self.stop()
            return False, self.session_directory
        return True, self.start()

    def start(self) -> Path:
        """Create the Desktop session directory; frames begin on next sim step."""
        session_name = datetime.now().strftime("recording_%Y-%m-%d_%H-%M-%S-%f")
        self.session_directory = self.desktop_directory / session_name
        self.session_directory.mkdir(parents=True, exist_ok=False)
        (self.session_directory / "README.txt").write_text(
            "One MP4 file is created for every camera available on G1.\n"
            "Press Quest X again to finalize the videos.\n",
            encoding="utf-8",
        )
        self._next_capture_time = 0.0
        self.is_recording = True
        return self.session_directory

    def capture(self, env: Any, now: float) -> None:
        """Queue one synchronized RGB frame from every live robot camera."""
        if not self.is_recording or now < self._next_capture_time:
            return
        self._next_capture_time = now + 1.0 / self.frames_per_second
        frames: dict[str, np.ndarray] = {}
        for name, sensor in getattr(env.scene, "sensors", {}).items():
            camera_name = name.lower()
            if "camera" not in camera_name or camera_name.startswith("world_"):
                continue
            try:
                image = sensor.data.output["rgb"][0]
                if image.device.type != "cpu":
                    image = image.cpu()
                frame = image.numpy()
                if frame.ndim != 3 or frame.shape[-1] not in (3, 4):
                    continue
                frames[name] = np.ascontiguousarray(frame).copy()
            except Exception as exc:
                print(f"[Quest recording] skipped {name}: {exc}", flush=True)
        if not frames:
            return
        try:
            if self._queue.full():
                self._queue.get_nowait()
                self._queue.task_done()
            self._queue.put_nowait(frames)
        except Exception:
            # Dropping an old frame is preferable to delaying teleoperation.
            pass

    def stop(self) -> None:
        """Flush queued frames and finalize each MP4 container."""
        if not self.is_recording:
            return
        self.is_recording = False
        self._queue.join()
        self._release_writers()

    def close(self) -> None:
        """Finalize an active session during simulator shutdown."""
        self.stop()
        self._queue.put(None)
        self._worker.join(timeout=5.0)

    def _write_frames(self) -> None:
        while True:
            frames = self._queue.get()
            try:
                if frames is None:
                    return
                for name, frame in frames.items():
                    writer = self._writers.get(name)
                    if writer is None:
                        writer = self._create_writer(name, frame)
                        if writer is None:
                            continue
                        self._writers[name] = writer
                    writer.write(self._as_bgr(frame))
            finally:
                self._queue.task_done()

    def _create_writer(self, camera_name: str, frame: np.ndarray) -> cv2.VideoWriter | None:
        if self.session_directory is None:
            return None
        height, width = frame.shape[:2]
        path = self.session_directory / f"{camera_name}.mp4"
        writer = cv2.VideoWriter(
            str(path),
            cv2.VideoWriter_fourcc(*"mp4v"),
            self.frames_per_second,
            (width, height),
        )
        if writer.isOpened():
            return writer
        writer.release()
        print(f"[Quest recording] could not open {path}", flush=True)
        return None

    @staticmethod
    def _as_bgr(frame: np.ndarray) -> np.ndarray:
        if frame.shape[-1] == 4:
            return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    def _release_writers(self) -> None:
        for writer in self._writers.values():
            writer.release()
        self._writers.clear()
