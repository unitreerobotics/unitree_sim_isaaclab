# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""
g1_29dof state
"""     
from __future__ import annotations

import torch
from typing import TYPE_CHECKING
import os

import sys
from multiprocessing import shared_memory

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


import torch

def get_robot_boy_joint_names() -> list[str]:
    return [
        # leg joints (12)
        # left leg (6)
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",
        # right leg (6)
        "right_hip_pitch_joint",
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_knee_joint",
        "right_ankle_pitch_joint",
        "right_ankle_roll_joint",
        # waist joints (3)
        "waist_yaw_joint",
        "waist_roll_joint",
        "waist_pitch_joint",

        # arm joints (14)
        # left arm (7)
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
        # right arm (7)
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    ]

def get_robot_arm_joint_names() -> list[str]:
    return [
        # arm joints (14)
        # left arm (7)
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
        # right arm (7)
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    ]

# global variable to cache the DDS instance
from dds.dds_master import dds_manager
_g1_robot_dds = None
_dds_initialized = False

# 观测缓存：索引张量与DDS限速（50FPS）+ 预分配缓冲
_obs_cache = {
    "device": None,
    "batch": None,
    "boy_idx_t": None,
    "boy_idx_batch": None,
    "pos_buf": None,
    "vel_buf": None,
    "torque_buf": None,
    "combined_buf": None,
    "dds_last_ms": 0,
    "dds_min_interval_ms": 20,
}

# IMU 加速度缓存：用于通过速度差分计算加速度
# IMU acceleration cache: for computing acceleration via velocity differentiation
_imu_acc_cache = {
    "prev_vel": None,
    "dt": 0.01,
    "initialized": False,
}

def _get_g1_robot_dds_instance():
    """get the DDS instance, delay initialization"""
    global _g1_robot_dds, _dds_initialized
    
    if not _dds_initialized or _g1_robot_dds is None:
        try:
            # dynamically import the DDS module
            sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), 'dds'))
            from dds.dds_master import dds_manager
            print(f"dds_manager: {dds_manager}")
            _g1_robot_dds = dds_manager.get_object("g129")
            print("[g1_state] G1 robot DDS communication instance obtained")
            
            # register the cleanup function
            import atexit
            def cleanup_dds():
                try:
                    if _g1_robot_dds:
                        dds_manager.unregister_object("g129")
                        print("[g1_state] DDS communication closed correctly")
                except Exception as e:
                    print(f"[g1_state] Error closing DDS: {e}")
            atexit.register(cleanup_dds)
            
        except Exception as e:
            print(f"[g1_state] Failed to get G1 robot DDS instance: {e}")
            _g1_robot_dds = None
        
        _dds_initialized = True
    
    return _g1_robot_dds

def get_robot_boy_joint_states(
    env: ManagerBasedRLEnv,
    enable_dds: bool = True,
) -> torch.Tensor:
    """get the robot body joint states, positions and velocities
    
    Args:
        env: ManagerBasedRLEnv - reinforcement learning environment instance
        enable_dds: bool - whether to enable the DDS publish function
    
    Returns:
        torch.Tensor
        - the first 29 elements are joint positions
        - the middle 29 elements are joint velocities
        - the last 29 elements are joint torques
    """
    # get all joint states
    joint_pos = env.scene["robot"].data.joint_pos
    joint_vel = env.scene["robot"].data.joint_vel
    joint_torque = env.scene["robot"].data.applied_torque  # use applied_torque to get joint torques
    device = joint_pos.device
    batch = joint_pos.shape[0]

    # 预计算并缓存索引张量（列索引）
    global _obs_cache
    if _obs_cache["device"] != device or _obs_cache["boy_idx_t"] is None:
        boy_joint_indices = [0, 3, 6, 9, 13, 17, 1, 4, 7, 10, 14, 18, 2, 5, 8, 11, 15, 19, 21, 23, 25, 27, 12, 16, 20, 22, 24, 26, 28]
        _obs_cache["boy_idx_t"] = torch.tensor(boy_joint_indices, dtype=torch.long, device=device)
        _obs_cache["device"] = device
        _obs_cache["batch"] = None  # force re-init batch-shaped buffers

    idx_t = _obs_cache["boy_idx_t"]
    n = idx_t.numel()

    # 预分配/复用 batch 形状索引与输出缓冲
    if _obs_cache["batch"] != batch or _obs_cache["boy_idx_batch"] is None:
        _obs_cache["boy_idx_batch"] = idx_t.unsqueeze(0).expand(batch, n)
        _obs_cache["pos_buf"] = torch.empty(batch, n, device=device, dtype=joint_pos.dtype)
        _obs_cache["vel_buf"] = torch.empty(batch, n, device=device, dtype=joint_pos.dtype)
        _obs_cache["torque_buf"] = torch.empty(batch, n, device=device, dtype=joint_pos.dtype)
        _obs_cache["combined_buf"] = torch.empty(batch, n * 3, device=device, dtype=joint_pos.dtype)
        _obs_cache["batch"] = batch

    idx_batch = _obs_cache["boy_idx_batch"]
    pos_buf = _obs_cache["pos_buf"]
    vel_buf = _obs_cache["vel_buf"]
    torque_buf = _obs_cache["torque_buf"]
    combined_buf = _obs_cache["combined_buf"]

    # 使用 gather(out=...) 填充，避免新张量分配
    try:
        torch.gather(joint_pos, 1, idx_batch, out=pos_buf)
        torch.gather(joint_vel, 1, idx_batch, out=vel_buf)
        torch.gather(joint_torque, 1, idx_batch, out=torque_buf)
    except TypeError:
        pos_buf.copy_(torch.gather(joint_pos, 1, idx_batch))
        vel_buf.copy_(torch.gather(joint_vel, 1, idx_batch))
        torque_buf.copy_(torch.gather(joint_torque, 1, idx_batch))

    # 组合为一个缓冲，避免 cat 分配
    combined_buf[:, 0:n].copy_(pos_buf)
    combined_buf[:, n:2*n].copy_(vel_buf)
    combined_buf[:, 2*n:3*n].copy_(torque_buf)

    # write to DDS（限速发布，避免高频CPU拷贝）
    if enable_dds and combined_buf.shape[0] > 0:
        try:
            import time
            now_ms = int(time.time() * 1000)
            if now_ms - _obs_cache["dds_last_ms"] >= _obs_cache["dds_min_interval_ms"]:
                g1_robot_dds = _get_g1_robot_dds_instance()
                if g1_robot_dds:
                    imu_data = get_robot_imu_data(env)
                    if imu_data.shape[0] > 0:
                        g1_robot_dds.write_robot_state(
                            pos_buf[0].contiguous().cpu().numpy(),
                            vel_buf[0].contiguous().cpu().numpy(),
                            torque_buf[0].contiguous().cpu().numpy(),
                            imu_data[0].contiguous().cpu().numpy(),
                        )
                        _obs_cache["dds_last_ms"] = now_ms
        except Exception as e:
            print(f"[g1_state] Error writing robot state to DDS: {e}")
    
    return combined_buf


def quat_to_rot_matrix(q):
    """
    q: [B,4] assumed (w,x,y,z)
    returns R: [B,3,3] such that v_world = R @ v_body
    """
    w = q[:, 0:1]
    x = q[:, 1:2]
    y = q[:, 2:3]
    z = q[:, 3:4]

    # precompute
    ww = w * w
    xx = x * x
    yy = y * y
    zz = z * z
    wx = w * x
    wy = w * y
    wz = w * z
    xy = x * y
    xz = x * z
    yz = y * z

    # rotation matrix elements
    r00 = ww + xx - yy - zz
    r01 = 2 * (xy - wz)
    r02 = 2 * (xz + wy)

    r10 = 2 * (xy + wz)
    r11 = ww - xx + yy - zz
    r12 = 2 * (yz - wx)

    r20 = 2 * (xz - wy)
    r21 = 2 * (yz + wx)
    r22 = ww - xx - yy + zz

    R = torch.cat([
        torch.cat([r00, r01, r02], dim=1).unsqueeze(1),
        torch.cat([r10, r11, r12], dim=1).unsqueeze(1),
        torch.cat([r20, r21, r22], dim=1).unsqueeze(1),
    ], dim=1)  # [B,3,3] but built transposed blocks; fix shape next

    # Currently R is [B,3,3] where rows are correct; reshape properly:
    R = R.view(-1, 3, 3)
    return R

def ensure_quat_w_first(quat, assume_w_first=None):
    """
    quat: [B,4] unknown ordering.
    If assume_w_first is True/False enforce; if None do heuristic:
      - if mean(abs(quat[:,0])) > 0.9 -> likely w in index 0 (w,x,y,z)
      - elif mean(abs(quat[:,3])) > 0.9 -> likely (x,y,z,w) so reorder
      - else keep as is (user should verify).
    Returns quat_wxyz: [B,4] (w,x,y,z)
    """
    if assume_w_first is True:
        return quat
    if assume_w_first is False:
        # reorder xyzw -> wxyz
        return torch.cat([quat[:, 3:4], quat[:, 0:3]], dim=1)

    # heuristic:
    b = quat.shape[0]
    mean0 = torch.mean(torch.abs(quat[:, 0]))
    mean3 = torch.mean(torch.abs(quat[:, 3]))
    if mean0 > 0.9:
        return quat  # already w first
    if mean3 > 0.9:
        return torch.cat([quat[:, 3:4], quat[:, 0:3]], dim=1)
    # ambiguous: default to w-first but warn (can't print here reliably for all contexts)
    return quat

def get_robot_imu_data(env, use_torso_imu: bool = True, quat_w_first: bool = None) -> torch.Tensor:
    """
    Returns [batch, 13] = pos(world,3) | quat(w,x,y,z) | acc_body(3) | gyro_body(3)
    - accel/gyro are in IMU/body frame (proper accelerometer reading)
    - quat_w_first: if None do heuristic, if True assume input quat already (w,x,y,z),
                    if False assume input quat is (x,y,z,w)
    """
    data = env.scene["robot"].data
    global _imu_acc_cache

    # --- dt ---
    dt = _imu_acc_cache["dt"]
    try:
        if hasattr(env, "physics_dt"):
            dt = float(env.physics_dt)
        elif hasattr(env, "step_dt"):
            dt = float(env.step_dt)
        elif hasattr(env, "dt"):
            dt = float(env.dt)
    except Exception:
        pass
    if dt <= 0:
        dt = _imu_acc_cache["dt"]

    # --- extract pose & vel ---
    if use_torso_imu:
        try:
            body_names = data.body_names
            imu_idx = body_names.index("imu_in_torso")
            body_pose = data.body_link_pose_w  # [B, N, 7]
            body_vel = data.body_link_vel_w    # [B, N, 6]
            pos = body_pose[:, imu_idx, :3]
            quat = body_pose[:, imu_idx, 3:7]
            lin_vel = body_vel[:, imu_idx, :3]
            ang_vel_world = body_vel[:, imu_idx, 3:6]
        except ValueError:
            use_torso_imu = False

    if not use_torso_imu:
        root_state = data.root_state_w  # [B, 13]
        pos = root_state[:, :3]
        quat = root_state[:, 3:7]
        lin_vel = root_state[:, 7:10]
        ang_vel_world = root_state[:, 10:13]

    # device/dtype consistency
    device = lin_vel.device if isinstance(lin_vel, torch.Tensor) else torch.device("cpu")
    quat = quat.to(device)
    lin_vel = lin_vel.to(device)
    ang_vel_world = ang_vel_world.to(device)

    # initialize prev_vel if needed
    if _imu_acc_cache["prev_vel"] is None:
        _imu_acc_cache["prev_vel"] = lin_vel.clone().detach().to(device)
        _imu_acc_cache["initialized"] = False
    else:
        if _imu_acc_cache["prev_vel"].device != device:
            _imu_acc_cache["prev_vel"] = _imu_acc_cache["prev_vel"].to(device)

    # compute a_world
    a_world = (lin_vel - _imu_acc_cache["prev_vel"]) / dt  # [B,3]

    # gravity in world frame (z-up convention)
    g_world = torch.zeros_like(a_world)
    g_world[:, 2] = -9.81

    # subtract gravity (proper acceleration in world frame)
    a_world_corrected = a_world - g_world  # [B,3]

    # prepare quaternion in (w,x,y,z)
    quat_wxyz = ensure_quat_w_first(quat, assume_w_first=quat_w_first)

    # build rotation matrices R_body->world ; to convert world->body use R^T
    R_body_to_world = quat_to_rot_matrix(quat_wxyz)  # [B,3,3]
    R_world_to_body = R_body_to_world.transpose(1, 2)  # [B,3,3]

    # rotate a_world_corrected to body: a_body = R_world_to_body @ a_world_corrected
    a_body = torch.bmm(R_world_to_body, a_world_corrected.unsqueeze(-1)).squeeze(-1)  # [B,3]

    # rotate angular velocity to body frame as well (if ang_vel_world is indeed in world-frame)
    omega_body = torch.bmm(R_world_to_body, ang_vel_world.unsqueeze(-1)).squeeze(-1)

    # handle first frame: prefer returning only gravity-compensated static reading
    if not _imu_acc_cache["initialized"]:
        # set a_body to rotation of -g_world (so accelerometer reads gravity in body coords)
        a_body = torch.bmm(R_world_to_body, (-g_world).unsqueeze(-1)).squeeze(-1)
        _imu_acc_cache["initialized"] = True

    # update cache
    _imu_acc_cache["prev_vel"] = lin_vel.clone().detach()
    _imu_acc_cache["dt"] = dt

    imu_data = torch.cat([pos, quat_wxyz, a_body, omega_body], dim=1)
    return imu_data
