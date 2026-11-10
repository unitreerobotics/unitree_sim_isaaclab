# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Run the teleimager image server in its own process.

The image server reads the camera images from shared memory and publishes them over ZMQ / WebRTC (JPEG
decoding, stereo concatenation, re-encoding, video encoding for WebRTC). Running it in the simulator process,
its threads compete with the simulation loop for the Python GIL. The images already go through shared memory,
so a separate process needs no other change.

The server is started with ``subprocess`` (a fresh interpreter running this module), not ``multiprocessing``:
spawn would re-import ``sim_main.py`` - and start a second simulation - in the child.
"""

from __future__ import annotations

import os
import subprocess
import sys
import threading

_READY = "IMAGE_SERVER_READY"
_FAILED = "IMAGE_SERVER_FAILED"


def run_image_server(config_path: str | None = None):
    """Start the teleimager server for the simulator cameras in this process and return it.

    Config: ``config_path``, else $TELEIMAGER_CONFIG, else the simulator config bundled with teleimager
    (head stereo 1280x480 on ZMQ 55555 / WebRTC 60001, wrists on 55556 / 60002 and 55557 / 60003).

    Raises:
        OSError: a port is already in use.
    """
    from teleimager.server import run_isaacsim_server

    return run_isaacsim_server(config_path)


class ImageServerProcess:
    """Image server in a child process; ``start()`` raises if the server could not be started (e.g. port in use)."""

    def __init__(self, startup_timeout: float = 30.0):
        self._timeout = startup_timeout
        self._proc: subprocess.Popen | None = None

    def start(self) -> "ImageServerProcess":
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        env = dict(os.environ, PYTHONUNBUFFERED="1", PYTHONPATH=os.pathsep.join(filter(None, [root, os.environ.get("PYTHONPATH")])))
        self._proc = subprocess.Popen(
            [sys.executable, "-m", "tools.image_server_process"], cwd=root, env=env,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
        )
        result: dict = {}
        done = threading.Event()

        def pump():  # forward the server output; detect the startup result
            for line in self._proc.stdout:
                if not done.is_set():
                    if line.startswith(_READY):
                        result["ok"] = True
                        done.set()
                        continue
                    if line.startswith(_FAILED):
                        result["error"] = line[len(_FAILED):].strip()
                        done.set()
                        continue
                sys.stdout.write("[image_server] " + line)
            done.set()

        threading.Thread(target=pump, name="image_server_log", daemon=True).start()
        if not done.wait(self._timeout):
            self.stop()
            raise RuntimeError(f"image server did not start within {self._timeout:.0f} s")
        if not result.get("ok"):
            self.stop()
            raise RuntimeError(f"image server failed to start: {result.get('error', 'process exited')}")
        return self

    def stop(self) -> None:
        if self._proc is not None and self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        self._proc = None


def _main() -> None:
    import signal

    try:
        server = run_image_server()
    except BaseException as e:  # noqa: BLE001 - report every startup failure to the parent
        print(f"{_FAILED} {type(e).__name__}: {e}", flush=True)
        sys.exit(1)
    signal.signal(signal.SIGTERM, lambda *_: server.stop())
    signal.signal(signal.SIGINT, lambda *_: server.stop())
    print(_READY, flush=True)
    server.wait_until_stopped()


if __name__ == "__main__":
    _main()
