# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""DDS action provider for the Unitree G1-D (wheeled base + lift column + G1 upper body, Dex1 grippers).

Same DDS interface as the real G1-D (xr_teleoperate ``mobile-teleop``), so the teleoperation client
drives the simulated robot unchanged:

* rt/lowcmd (``g129`` DDS object): motors 15..28 -> arm joints, motor 12 -> waist yaw (torso_Joint).
  Waist commands are only applied once a lowcmd was received.
* Waist pitch (Yaw_Joint): rt/waistpitchcmd (``g1d_waist_pitch``, unitree_hg MotorCmd_.q) like the real G1-D
  flagship (unirobot ``motor_topics``). Until that topic is received, lowcmd motor 13 (xr_teleoperate layout) is
  used when it is enabled (mode 1), so a lowcmd that leaves it unused (unirobot) does not move the waist.
* Dex1 grippers, real "internal" wiring (unirobot G1-D / R1-D): lowcmd motors 31 (left) / 33 (right), motor q in
  [0 closed, 5.4 open], used when the slot is enabled (mode 1 and kp > 0); they take precedence over
* rt/dex1/*/cmd (``dex1``, "external" wiring): the DDS object delivers the G1 Dex1 joint range [-0.02 open,
  0.024 closed]; it is mapped to the G1-D finger travel [0.041 open, 0 closed] and applied to both fingers.
* rt/cmd_hispeed (``g1d_mobile``): lift speed command z in [-1, 1] is integrated into the lift height
  target (both stages extend equally, 0 .. 0.42 m in total).
* rt/cmd_vel_no_limit (``g1d_mobile``): linear.x / angular.z set the base velocity (forward along the
  heading / yaw rate). The floating base hovers 3 mm above the floor and is velocity-controlled every
  control step (height and tilt are held), so it follows the command and is stopped by obstacles.
* The simulated lift height and the actual base odometry are published back through ``g1d_mobile``
  (rt/hispeed_state, rt/slamware_ros_sdk_server_node/odom).

Actions are sent relative to the action term's default offset (``use_default_offset=True``).
"""

import math
import time
from typing import Optional

import torch

from action_provider.action_base import ActionProvider
from dds.dds_master import dds_manager
from robots.unitree_g1d import (
    G1D_CMD_VEL_TIMEOUT,
    G1D_LIFT_JOINTS,
    G1D_LIFT_MAX_SPEED,
    G1D_LIFT_STAGE_MAX,
    G1D_WAIST_PITCH_JOINT,
    G1D_WAIST_YAW_JOINT,
)
from tasks.common_observations.g1d_state import DEX1_LOWCMD_SLOTS, ARM_JOINTS, dex1_motor_to_g1d_finger, g1_dex1_to_g1d_finger

_ARM_DDS_OFFSET = 15


def _yaw_of(q_xyzw: torch.Tensor) -> float:
    x, y, z, w = (float(v) for v in q_xyzw)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class G1DDDSActionProvider(ActionProvider):
    """G1-D provider. Wheeled robots with the same structure (e.g. R1-D) subclass it and override the
    class attributes below (joint names / lowcmd slots / lift / mobile DDS object)."""

    ROBOT = "G1-D"
    LOWCMD_EXTRA = {G1D_WAIST_YAW_JOINT: 12}  # joint -> lowcmd motor (besides the arms)
    LOWCMD_GATED = {G1D_WAIST_PITCH_JOINT: 13}  # like LOWCMD_EXTRA, but only applied while the slot is enabled (mode 1)
    WAIST_PITCH_DDS = "g1d_waist_pitch"  # DDS object with get_waist_pitch_command(); None: no separate pitch topic
    WAIST_PITCH_JOINT = G1D_WAIST_PITCH_JOINT
    LIFT_JOINTS = G1D_LIFT_JOINTS
    LIFT_STAGE_MAX = G1D_LIFT_STAGE_MAX
    LIFT_MAX_SPEED = G1D_LIFT_MAX_SPEED
    CMD_TIMEOUT = G1D_CMD_VEL_TIMEOUT
    MOBILE_DDS = "g1d_mobile"  # DDS object with get_mobile_command() / write_mobile_state(); None: base / lift fixed
    LIFT_MAX_TOTAL = None  # column soft limit [m]; None: LIFT_STAGE_MAX * number of stages
    # (accel, decel) slew limits of the base / column commands; None: commands are applied directly
    SLEW_LINEAR = None  # m/s^2
    SLEW_ANGULAR = None  # rad/s^2
    SLEW_COLUMN = None  # m/s^2

    def __init__(self, env, args_cli):
        super().__init__(f"{type(self).__name__}")
        self.env = env
        device = str(env.device)
        self._device = device
        self.robot_dds = dds_manager.get_object("g129")
        self.gripper_dds = dds_manager.get_object("dex1") if args_cli.enable_dex1_dds else None
        self.mobile_dds = dds_manager.get_object(self.MOBILE_DDS) if self.MOBILE_DDS else None
        self.pitch_dds = dds_manager.get_object(self.WAIST_PITCH_DDS) if self.WAIST_PITCH_DDS else None
        self._pitch_topic_seen = False
        self._start_time = time.time()  # ignore commands left in the (named) shared memory by an earlier run
        if args_cli.enable_dex3_dds or args_cli.enable_inspire_dds:
            print(f"[{self.name}] {self.ROBOT} has Dex1 grippers; --enable_dex3_dds / --enable_inspire_dds are ignored")

        self.robot = robot = env.scene["robot"]
        names = list(robot.data.joint_names)
        idx = {n: i for i, n in enumerate(names)}
        self._arm_target = torch.tensor([idx[n] for n in ARM_JOINTS], dtype=torch.long, device=device)
        self._arm_source = torch.arange(_ARM_DDS_OFFSET, _ARM_DDS_OFFSET + len(ARM_JOINTS), device=device)
        self._waist = {idx[j]: slot for j, slot in self.LOWCMD_EXTRA.items()}
        self._waist_gated = {idx[j]: slot for j, slot in self.LOWCMD_GATED.items()}
        self._pitch_id = idx[self.WAIST_PITCH_JOINT] if self.pitch_dds else None
        self._lift_ids = [idx[j] for j in self.LIFT_JOINTS]
        self._fingers = {
            side: torch.tensor([idx[f"{side}_dex1_finger_joint_1"], idx[f"{side}_dex1_finger_joint_2"]], dtype=torch.long, device=device)
            for side in ("left", "right")
        }

        term = env.action_manager.get_term("joint_pos")
        self._offset = torch.as_tensor(term._offset, device=device).reshape(-1, len(names))[0].float()
        self._scale = torch.as_tensor(term._scale, device=device).float()
        self._dt = float(env.step_dt)
        self._env_ids = torch.tensor([0], device=device)
        self._last_episode_step = None
        print(
            f"[{self.name}] DDS initialized: arms + waist + dex1 gripper (lowcmd 31/33){' + rt/dex1' if self.gripper_dds else ''}"
            f"{' + rt/waistpitchcmd' if self.pitch_dds else ''}"
            f"{' + mobile base / lift' if self.mobile_dds else ' (base / lift held: ' + (f'no {self.MOBILE_DDS} DDS object' if self.MOBILE_DDS else 'no mobile DDS interface yet') + ')'}"
        )

    # ------------------------------------------------------------------ helpers
    def _reset_targets(self):
        """(Re)start from the default pose, e.g. after an environment reset."""
        data = self.robot.data
        self._targets = data.default_joint_pos.torch[0].clone().float().to(self._device)
        self._lift_height = float(sum(self._targets[i] for i in self._lift_ids))
        root = data.root_link_pose_w.torch[0].clone().to(self._device)  # odom frame origin = pose after reset
        self._home_pos = root[:3].clone()
        self._home_yaw = _yaw_of(root[3:7])
        # hover 3 mm above the resting height: the wheels / casters do not touch the floor, the base is
        # carried by the velocity controller below (no wheel slip / friction effects)
        self._base_z = float(root[2]) + 0.003
        self._v = self._w = self._h_speed = 0.0  # slew-limited commands
        self._stall_time, self._stall_dir = 0.0, 0  # column stall protection

    def _slew(self, cur, target, limits):
        """Move ``cur`` towards ``target`` with (accel, decel) limits per second (None: no limit)."""
        if limits is None:
            return target
        accel, decel = limits
        speeding_up = abs(target) > abs(cur) and (cur == 0.0 or (target > 0) == (cur > 0))
        step = (accel if speeding_up else decel) * self._dt
        return cur + max(-step, min(step, target - cur))

    def _apply_lift(self, cmd, now):
        """Integrate the rt/cmd_hispeed speed command into the lift height target."""
        h_speed = 0.0
        if cmd and now - cmd.get("height_stamp", 0.0) < self.CMD_TIMEOUT:
            h_speed = max(-1.0, min(1.0, float(cmd.get("height_speed", 0.0)))) * self.LIFT_MAX_SPEED
        h_speed = self._h_speed = self._slew(self._h_speed, h_speed, self.SLEW_COLUMN)
        jp = self.robot.data.joint_pos.torch[0]
        actual = float(sum(jp[i] for i in self._lift_ids))
        # stall protection: commanded but (almost) not moving for > 0.5 s (e.g. the hands pressed onto the table)
        # -> lock that direction until the opposite direction is commanded (do not keep pushing into the obstacle)
        sign = (h_speed > 0) - (h_speed < 0)
        if self._stall_dir and sign == -self._stall_dir:
            self._stall_dir, self._stall_time = 0, 0.0
        if self._stall_dir and sign == self._stall_dir:
            h_speed = self._h_speed = 0.0
            self._lift_height = actual
        else:
            actual_v = float(sum(self.robot.data.joint_vel.torch[0][i] for i in self._lift_ids))
            # only judge once the (slew-limited) command is at least half the maximum speed
            commanded = abs(h_speed) >= 0.5 * self.LIFT_MAX_SPEED
            moving = abs(actual_v) >= 0.2 * abs(h_speed)
            self._stall_time = self._stall_time + self._dt if (commanded and not moving) else 0.0
            if self._stall_time > 0.5:
                self._stall_dir, self._stall_time = sign, 0.0
                print(f"[{self.name}] lift stalled ({'up' if sign > 0 else 'down'}), direction locked")
        target = self._lift_height + h_speed * self._dt
        if h_speed != 0.0:  # do not let the target run away from the (velocity-limited) column
            target = min(max(target, actual - 0.01), actual + 0.01)
        top = self.LIFT_MAX_TOTAL if self.LIFT_MAX_TOTAL is not None else self.LIFT_STAGE_MAX * len(self._lift_ids)
        self._lift_height = min(max(target, 0.0), top)
        for i in self._lift_ids:
            self._targets[i] = self._lift_height / len(self._lift_ids)

    def _apply_base(self, cmd, now):
        """Drive the base with the rt/cmd_vel_no_limit twist.

        The root link velocity is set every control step: the commanded forward / yaw velocity, plus
        corrections that keep the base at its hover height and upright. Physics integrates it, so the
        base is stopped by obstacles (e.g. the table) instead of being pushed through them.
        """
        v = w = 0.0
        if cmd and now - cmd.get("move_stamp", 0.0) < self.CMD_TIMEOUT:
            v, w = float(cmd.get("lin_x", 0.0)), float(cmd.get("ang_z", 0.0))
        v = self._v = self._slew(self._v, v, self.SLEW_LINEAR)
        w = self._w = self._slew(self._w, w, self.SLEW_ANGULAR)
        pose = self.robot.data.root_link_pose_w.torch[0]
        z = float(pose[2])
        x, y, qz, qw = (float(c) for c in pose[3:7])
        yaw = _yaw_of(pose[3:7])
        # tilt error: rotation from the current orientation to upright with the same yaw (small angles)
        roll = math.atan2(2.0 * (qw * x + y * qz), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (qw * y - qz * x))))
        k = 1.0 / (2.0 * self._dt)  # converge in ~2 control steps
        cy, sy = math.cos(yaw), math.sin(yaw)
        wx_b, wy_b = -k * roll, -k * pitch
        vel = torch.tensor(
            [[v * cy, v * sy, k * (self._base_z - z), cy * wx_b - sy * wy_b, sy * wx_b + cy * wy_b, w]],
            device=self._device,
        )
        self.robot.write_root_link_velocity_to_sim_index(root_velocity=vel, env_ids=self._env_ids)

    def _publish_mobile_state(self):
        """Lift height / speed and base odometry (actual simulated motion, in the odom frame)."""
        data = self.robot.data
        jp, jv = data.joint_pos.torch[0], data.joint_vel.torch[0]
        height = float(sum(jp[i] for i in self._lift_ids))
        height_speed = float(sum(jv[i] for i in self._lift_ids))
        pose = data.root_link_pose_w.torch[0]
        dx, dy = float(pose[0] - self._home_pos[0]), float(pose[1] - self._home_pos[1])
        c, s = math.cos(self._home_yaw), math.sin(self._home_yaw)
        yaw = _yaw_of(pose[3:7])
        rel_yaw = math.atan2(math.sin(yaw - self._home_yaw), math.cos(yaw - self._home_yaw))
        lin = data.root_link_lin_vel_w.torch[0]
        v = float(lin[0]) * math.cos(yaw) + float(lin[1]) * math.sin(yaw)
        w = float(data.root_link_ang_vel_w.torch[0, 2])
        self.mobile_dds.write_mobile_state(height, height_speed, c * dx + s * dy, -s * dx + c * dy, rel_yaw, v, w)

    # ------------------------------------------------------------------ main
    def get_action(self, env) -> Optional[torch.Tensor]:
        try:
            step = int(env.episode_length_buf[0])
            if self._last_episode_step is None or step < self._last_episode_step:
                self._reset_targets()  # first call or the environment was reset
            self._last_episode_step = step

            internal_grippers = {}  # side -> Dex1 motor q from lowcmd motors 31 / 33
            if self.robot_dds:
                cmd = self.robot_dds.get_robot_command()
                if cmd and "motor_cmd" in cmd:
                    mc = cmd["motor_cmd"]
                    positions = mc["positions"]
                    modes = mc.get("modes") or [1] * len(positions)
                    kps = mc.get("kp") or [0.0] * len(positions)
                    slots = list(self._waist.values()) + list(self._waist_gated.values())
                    n = max([_ARM_DDS_OFFSET + len(ARM_JOINTS)] + [slot + 1 for slot in slots])
                    if len(positions) >= n:
                        q = torch.tensor(positions[:n], dtype=torch.float32, device=self._device)
                        self._targets.index_copy_(0, self._arm_target, q.index_select(0, self._arm_source))
                        for j, slot in self._waist.items():
                            self._targets[j] = q[slot]
                        if not self._pitch_topic_seen:
                            for j, slot in self._waist_gated.items():
                                if modes[slot] == 1:
                                    self._targets[j] = q[slot]
                    for side, slot in DEX1_LOWCMD_SLOTS.items():
                        if slot < len(positions) and modes[slot] == 1 and kps[slot] > 0.0:
                            internal_grippers[side] = positions[slot]
            if self.pitch_dds:
                cmd = self.pitch_dds.get_waist_pitch_command()
                if cmd and "q" in cmd and float(cmd.get("stamp", 0.0)) >= self._start_time:
                    if not self._pitch_topic_seen:
                        print(f"[{self.name}] waist pitch commanded through rt/waistpitchcmd (lowcmd motor 13 ignored)")
                    self._pitch_topic_seen = True
                    self._targets[self._pitch_id] = float(cmd["q"])
            if self.gripper_dds:
                cmd = self.gripper_dds.get_gripper_command()
                if cmd:
                    for side in ("left", "right"):
                        p = cmd.get(f"{side}_gripper_cmd", {}).get("positions", [])
                        if p and side not in internal_grippers:
                            self._targets[self._fingers[side]] = float(g1_dex1_to_g1d_finger(float(p[0])))
            for side, q_motor in internal_grippers.items():
                self._targets[self._fingers[side]] = dex1_motor_to_g1d_finger(q_motor)
            # without a mobile DDS object the base is still held in place and the lift at its target
            cmd, now = (self.mobile_dds.get_mobile_command() if self.mobile_dds else None), time.time()
            self._apply_lift(cmd, now)
            self._apply_base(cmd, now)
            if self.mobile_dds:
                self._publish_mobile_state()
            return ((self._targets - self._offset) / self._scale).unsqueeze(0)
        except Exception as e:
            print(f"[{self.name}] Get DDS action failed: {e}")
            return None

    def cleanup(self):
        try:
            for dds in (self.robot_dds, self.gripper_dds, self.mobile_dds, self.pitch_dds):
                if dds:
                    dds.stop_communication()
        except Exception as e:
            print(f"[{self.name}] Clean up DDS resources failed: {e}")
