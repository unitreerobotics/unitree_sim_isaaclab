# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
gripper state
"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING
import sys
import os
if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


import torch


_obs_cache = {
    "device": None,
    "batch": None,
    "gripper_idx_t": None,
    "gripper_idx_batch": None,
    "pos_buf": None,
    "vel_buf": None,
    "torque_buf": None,
    "dds_last_ms": 0,
    "dds_min_interval_ms": 20,
}

def get_robot_girl_joint_names() -> list[str]:
    return [
        "right_hand_Joint1_1",
        "left_hand_Joint1_1",
    ]

# global variable to cache the DDS instance
_gripper_dds = None
_dds_initialized = False

def _get_gripper_dds_instance():
    """get the DDS instance, delay initialization"""
    global _gripper_dds, _dds_initialized

    if not _dds_initialized or _gripper_dds is None:
        try:
            # dynamically import the DDS module
            sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), 'dds'))
            from dds.dds_master import dds_manager

            _gripper_dds = dds_manager.get_object("dex1")
            print("[Observations] DDS communication instance obtained")

            # register the cleanup function
            import atexit
            def cleanup_dds():
                try:
                    if _gripper_dds:
                        dds_manager.unregister_object("dex1")
                        print("[gripper_state] DDS communication closed correctly")
                except Exception as e:
                    print(f"[gripper_state] Error closing DDS: {e}")
            atexit.register(cleanup_dds)

        except Exception as e:
            print(f"[Observations] Failed to get DDS instances: {e}")
            _gripper_dds = None

        _dds_initialized = True

    return _gripper_dds

def initialize_gripper_dds():
    """explicitly initialize the DDS communication

    this function can be called manually to initialize the DDS communication,
    instead of relying on delayed initialization
    """
    return _get_gripper_dds_instance()


def get_robot_gipper_joint_states(
    env: ManagerBasedRLEnv,
    enable_dds: bool = True,
) -> torch.Tensor:
    """get the robot gripper joint states and publish them to DDS

    Args:
        env: ManagerBasedRLEnv - reinforcement learning environment instance
        enable_dds: bool - whether to enable the DDS publish function

    返回:
        torch.Tensor
    """
    # get the gripper joint states
    joint_pos = env.scene["robot"].data.joint_pos.torch
    joint_vel = env.scene["robot"].data.joint_vel.torch
    joint_torque = env.scene["robot"].actuators.applied_effort.torch
    device = joint_pos.device
    batch = joint_pos.shape[0]


    global _obs_cache
    if _obs_cache["device"] != device or _obs_cache["gripper_idx_t"] is None:
        gripper_joint_indices = [31, 29]
        _obs_cache["gripper_idx_t"] = torch.tensor(gripper_joint_indices, dtype=torch.long, device=device)
        _obs_cache["device"] = device
        _obs_cache["batch"] = None
    idx_t = _obs_cache["gripper_idx_t"]
    n = idx_t.numel()


    if _obs_cache["batch"] != batch or _obs_cache["gripper_idx_batch"] is None:
        _obs_cache["gripper_idx_batch"] = idx_t.unsqueeze(0).expand(batch, n)
        _obs_cache["pos_buf"] = torch.empty(batch, n, device=device, dtype=joint_pos.dtype)
        _obs_cache["vel_buf"] = torch.empty(batch, n, device=device, dtype=joint_pos.dtype)
        _obs_cache["torque_buf"] = torch.empty(batch, n, device=device, dtype=joint_pos.dtype)
        _obs_cache["batch"] = batch

    idx_batch = _obs_cache["gripper_idx_batch"]
    pos_buf = _obs_cache["pos_buf"]
    vel_buf = _obs_cache["vel_buf"]
    torque_buf = _obs_cache["torque_buf"]


    try:
        torch.gather(joint_pos, 1, idx_batch, out=pos_buf)
        torch.gather(joint_vel, 1, idx_batch, out=vel_buf)
        torch.gather(joint_torque, 1, idx_batch, out=torque_buf)
    except TypeError:
        pos_buf.copy_(torch.gather(joint_pos, 1, idx_batch))
        vel_buf.copy_(torch.gather(joint_vel, 1, idx_batch))
        torque_buf.copy_(torch.gather(joint_torque, 1, idx_batch))

    # publish to DDS (only publish the data of the first environment)
    if enable_dds and len(pos_buf) > 0:
        try:
            import time
            now_ms = int(time.time() * 1000)
            if now_ms - _obs_cache["dds_last_ms"] >= _obs_cache["dds_min_interval_ms"]:
                gripper_dds = _get_gripper_dds_instance()
                if gripper_dds:
                    pos = pos_buf[0].contiguous().cpu().numpy()
                    vel = vel_buf[0].contiguous().cpu().numpy()
                    torque = torque_buf[0].contiguous().cpu().numpy()
                    right_pos = pos[:1]
                    left_pos = pos[1:]
                    right_vel = vel[:1]
                    left_vel = vel[1:]
                    right_torque = torque[:1]
                    left_torque = torque[1:]
                    # write the gripper state to shared memory
                    gripper_dds.write_gripper_state(left_pos, left_vel, left_torque, right_pos, right_vel, right_torque)
                    _obs_cache["dds_last_ms"] = now_ms
        except Exception as e:
            print(f"[gripper_state] Failed to write to shared memory: {e}")

    return pos_buf

