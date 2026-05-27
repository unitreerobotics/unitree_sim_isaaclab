# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""
dex3 state
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
    "hand_idx_t": None,
    "hand_idx_batch": None,
    "pos_buf": None,
    "vel_buf": None,
    "torque_buf": None,
    "dds_last_ms": 0,
    "dds_min_interval_ms": 20,
}

def get_robot_girl_joint_names() -> list[str]:
    return [
        # hand joints (14)
        # left hand (7)
        "left_hand_thumb_0_joint",
        "left_hand_thumb_1_joint",
        "left_hand_thumb_2_joint",
        "left_hand_middle_0_joint",
        "left_hand_middle_1_joint",
        "left_hand_index_0_joint",
        "left_hand_index_1_joint",
        # right hand (7)
        "right_hand_thumb_0_joint",
        "right_hand_thumb_1_joint",
        "right_hand_thumb_2_joint",
        "right_hand_middle_0_joint",
        "right_hand_middle_1_joint",
        "right_hand_index_0_joint",
        "right_hand_index_1_joint",
    ]

# global variable to cache the DDS instance
_dex3_dds = None
_dds_initialized = False

def _get_dex3_dds_instance():
    """get the DDS instance, delay initialization"""
    global _dex3_dds, _dds_initialized
    
    if not _dds_initialized or _dex3_dds is None:
        try:
            # dynamically import the DDS module
            sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), 'dds'))
            from dds.dds_master import dds_manager
            
            _dex3_dds = dds_manager.get_object("dex3")
            print("[Observations Dex3] DDS communication instance obtained")
            
            # register the cleanup function
            import atexit
            def cleanup_dds():
                try:
                    if _dex3_dds:
                        dds_manager.unregister_object("dex3")
                        print("[dex3_state] DDS communication closed correctly")
                except Exception as e:
                    print(f"[dex3_state] Error closing DDS: {e}")
            atexit.register(cleanup_dds)
            
        except Exception as e:
            print(f"[Observations Dex3] Failed to get DDS instances: {e}")
            _dex3_dds = None
        
        _dds_initialized = True
    
    return _dex3_dds

def get_robot_dex3_joint_states(
    env: ManagerBasedRLEnv,
    enable_dds: bool = True,
) -> torch.Tensor:
    """get the robot gripper joint states and publish them to DDS
    
    Args:
        env: ManagerBasedRLEnv - reinforcement learning environment instance
        enable_dds: bool - whether to enable the DDS publish function
    
    Returns:
        torch.Tensor
    """
    # get the gripper joint positions, velocities, torques
    # HANDSIM PATCH 2026-05-26: see g1_29dof_state.py — same prepare_terms device mix.
    device = env.device
    joint_pos = env.scene["robot"].data.joint_pos.to(device)
    joint_vel = env.scene["robot"].data.joint_vel.to(device)
    joint_torque = env.scene["robot"].data.applied_torque.to(device)
    batch = joint_pos.shape[0]
    

    global _obs_cache
    if _obs_cache["device"] != device or _obs_cache["hand_idx_t"] is None:
        gripper_joint_indices=[31, 37, 41, 30, 36, 29, 35, 34, 40, 42, 33, 39, 32, 38]
        _obs_cache["hand_idx_t"] = torch.tensor(gripper_joint_indices, dtype=torch.long, device=device)
        _obs_cache["device"] = device
        _obs_cache["batch"] = None
    idx_t = _obs_cache["hand_idx_t"]
    n = idx_t.numel()


    if _obs_cache["batch"] != batch or _obs_cache["hand_idx_batch"] is None:
        _obs_cache["hand_idx_batch"] = idx_t.unsqueeze(0).expand(batch, n)
        _obs_cache["pos_buf"] = torch.empty(batch, n, device=device, dtype=joint_pos.dtype)
        _obs_cache["vel_buf"] = torch.empty(batch, n, device=device, dtype=joint_pos.dtype)
        _obs_cache["torque_buf"] = torch.empty(batch, n, device=device, dtype=joint_pos.dtype)
        _obs_cache["batch"] = batch

    idx_batch = _obs_cache["hand_idx_batch"]
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
                dex3_dds = _get_dex3_dds_instance()
                if dex3_dds:
                    pos = pos_buf[0].contiguous().cpu().numpy()
                    vel = vel_buf[0].contiguous().cpu().numpy()
                    torque = torque_buf[0].contiguous().cpu().numpy()
                    left_pos = pos[:7]
                    right_pos = pos[7:]
                    left_vel = vel[:7]
                    right_vel = vel[7:]
                    left_torque = torque[:7]
                    right_torque = torque[7:]
                    dex3_dds.write_hand_states(left_pos, left_vel, left_torque, right_pos, right_vel, right_torque)
                    _obs_cache["dds_last_ms"] = now_ms
        except Exception as e:
            print(f"dex3_state [dex3_state] Failed to write to DDS: {e}")
    
    return pos_buf

