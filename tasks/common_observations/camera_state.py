# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""
camera state
"""     

from __future__ import annotations

from typing import TYPE_CHECKING
import torch
import sys
import os

# add the project root directory to the path, so that the shared memory tool can be imported
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
from tools.shared_memory_utils import MultiImageWriter

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

# create the global multi-image shared memory writer
multi_image_writer = MultiImageWriter()

def set_writer_options(enable_jpeg: bool = False, jpeg_quality: int = 85, skip_cvtcolor: bool = False):
    try:
        multi_image_writer.set_options(enable_jpeg=enable_jpeg, jpeg_quality=jpeg_quality, skip_cvtcolor=skip_cvtcolor)
        print(f"[camera_state] writer options: jpeg={enable_jpeg}, quality={jpeg_quality}, skip_cvtcolor={skip_cvtcolor}")
    except Exception as e:
        print(f"[camera_state] failed to set writer options: {e}")


_camera_cache = {
    'available_cameras': None,
    'camera_keys': None,
    'last_scene_id': None,
    'frame_step': 0,
    'write_interval_steps': 1,  # write at most every N env steps (a new frame is written only once anyway)
    'last_frame': None,
}

# shared memory name -> scene camera name
_CAMERA_IMAGES = (
    ("head", "front_camera"),  # head stereo left eye
    ("head_right", "front_right_camera"),  # head stereo right eye
    ("left", "left_wrist_camera"),
    ("right", "right_wrist_camera"),
)

_return_placeholder = None


def _camera_frame(camera) -> int | None:
    """Render frame counter of a camera (Isaac Lab 3), None if not available."""
    try:
        frame = camera.frame
        frame = frame.torch if hasattr(frame, "torch") else frame
        return int(frame[0])
    except Exception:
        return None


def get_camera_image(
    env: ManagerBasedRLEnv,
) -> dict:
    """Write the camera images to shared memory (read by the image server).

    Images are written only when the cameras rendered a new frame (render rate: sim_main --render_fps), and
    synchronously: the former writer thread hardly got the GIL while the main thread was stepping the simulation
    and published only ~19 fps; one write (4 x 640x480, JPEG) takes ~2 ms.
    """
    global _return_placeholder
    if _return_placeholder is None:
        _return_placeholder = torch.zeros((1, 480, 640, 3))

    scene_id = id(env.scene)
    if _camera_cache['last_scene_id'] != scene_id:
        keys = list(env.scene.keys())
        _camera_cache['camera_keys'] = keys
        _camera_cache['available_cameras'] = [name for name in keys if "camera" in name.lower()]
        _camera_cache['last_scene_id'] = scene_id
        _camera_cache['last_frame'] = None
    camera_keys = _camera_cache['camera_keys']

    _camera_cache['frame_step'] = (_camera_cache['frame_step'] + 1) % max(1, _camera_cache['write_interval_steps'])
    if _camera_cache['frame_step'] != 0:
        return _return_placeholder

    pairs = [(img, cam) for img, cam in _CAMERA_IMAGES if cam in camera_keys]
    if not pairs:
        # fallback: the first three cameras of the scene as head / left / right
        pairs = list(zip(("head", "left", "right"), _camera_cache['available_cameras'][:3]))
        if not pairs:
            return _return_placeholder

    # Isaac Lab 3 cameras update lazily: accessing .data renders the cameras whose update_period has elapsed
    # (all of them in one RTX frame). Then skip if no new frame was rendered since the last write.
    for _, cam_name in pairs:
        env.scene[cam_name].data
    frame = _camera_frame(env.scene[pairs[0][1]])
    if frame is not None:
        if frame == _camera_cache['last_frame']:
            return _return_placeholder
        _camera_cache['last_frame'] = frame

    images = {}
    for img_name, cam_name in pairs:
        rgb = env.scene[cam_name].data.output["rgb"]
        rgb = (rgb.torch if hasattr(rgb, "torch") else rgb)[0]
        images[img_name] = rgb.numpy() if rgb.device.type == "cpu" else rgb.cpu().numpy()
    multi_image_writer.write_images(images)
    return _return_placeholder
