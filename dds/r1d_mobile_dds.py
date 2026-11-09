# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
DDS bridge for the Unitree R1-D base and lift column (R1-A5/A7D "twist" module topics).

Subscribed:
    rt/cmd_vel       geometry_msgs/Twist   NORMALIZED [-1, 1]: linear.x (x gear vmax 0.5/1.0/1.5 m/s),
                                           linear.z (column, x 0.08 m/s, up +), angular.z (x 1.2 rad/s, left +)
Published (50 Hz, from the simulated motion):
    rt/state_vel     geometry_msgs/Twist   measured velocities, normalized like rt/cmd_vel, with dead zones
                                           (linear.x / linear.z 0.005, angular.z 0.02); other fields 0
    rt/height_state  geometry_msgs/Point32 z = column height above the zero point [mm]; x = y = 0
    rt/height_alarm  unitree_go/Error      source = 200, state bits: 0 NotHomed, 1 Lower, 2 Upper, 3 Stall

The commands go to the simulation in physical units through shared memory ("isaac_r1d_mobile_cmd"),
in the format of the G1-D mobile bridge (lin_x [m/s], ang_z [rad/s], height_speed [-1, 1]); the state
comes back through "isaac_r1d_mobile_state".
"""

import time
from typing import Any, Dict, Optional

from dds.dds_base import DDSObject
from robots.unitree_r1d import R1D_GEAR, R1D_GEAR_VMAX, R1D_LIFT_MAX_SPEED, R1D_LIFT_SOFT_MAX, R1D_MAX_ANGULAR_VEL
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import geometry_msgs_msg_dds__Point32_, geometry_msgs_msg_dds__Twist_
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Point32_, Twist_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import Error_

TOPIC_CMD = "rt/cmd_vel"
TOPIC_STATE_VEL = "rt/state_vel"
TOPIC_HEIGHT_STATE = "rt/height_state"
TOPIC_HEIGHT_ALARM = "rt/height_alarm"
ALARM_SOURCE = 200
BIT_LOWER, BIT_UPPER = 1 << 1, 1 << 2
DEADZONE_LINEAR, DEADZONE_ANGULAR = 0.005, 0.02
SOFT_LIMIT_EPS = 0.0005  # [m] "at or beyond" a soft limit


def _clamp1(v: float) -> float:
    return max(-1.0, min(1.0, float(v)))


def _dead(v: float, dz: float) -> float:
    return 0.0 if abs(v) < dz else v


class R1DMobileDDS(DDSObject):
    def __init__(self, node_name: str = "r1d_mobile"):
        if hasattr(self, "_initialized"):
            return
        super().__init__()
        self.node_name = node_name
        self.vmax = R1D_GEAR_VMAX.get(R1D_GEAR, R1D_GEAR_VMAX["low"])
        self.state_vel_msg = geometry_msgs_msg_dds__Twist_()
        self.height_msg = geometry_msgs_msg_dds__Point32_()
        self.alarm_msg = Error_(source=ALARM_SOURCE, state=0)
        self.state_vel_pub = self.height_pub = self.alarm_pub = None
        self.setup_shared_memory(
            input_shm_name="isaac_r1d_mobile_state", input_size=1024, output_shm_name="isaac_r1d_mobile_cmd", output_size=1024
        )
        self._initialized = True
        print(f"[{self.node_name}] R1-D base / column DDS node initialized (gear '{R1D_GEAR}', vmax {self.vmax} m/s)")

    def setup_publisher(self) -> bool:
        try:
            self.state_vel_pub = ChannelPublisher(TOPIC_STATE_VEL, Twist_)
            self.state_vel_pub.Init()
            self.height_pub = ChannelPublisher(TOPIC_HEIGHT_STATE, Point32_)
            self.height_pub.Init()
            self.alarm_pub = ChannelPublisher(TOPIC_HEIGHT_ALARM, Error_)
            self.alarm_pub.Init()
            print(f"[{self.node_name}] publishers initialized ({TOPIC_STATE_VEL}, {TOPIC_HEIGHT_STATE}, {TOPIC_HEIGHT_ALARM})")
            return True
        except Exception as e:
            print(f"[{self.node_name}] publisher initialization failed: {e}")
            return False

    def setup_subscriber(self) -> bool:
        try:
            self.cmd_sub = ChannelSubscriber(TOPIC_CMD, Twist_)
            self.cmd_sub.Init(lambda msg: self.dds_subscriber(msg, "cmd"), 1)
            print(f"[{self.node_name}] command subscriber initialized ({TOPIC_CMD})")
            return True
        except Exception as e:
            print(f"[{self.node_name}] subscriber initialization failed: {e}")
            return False

    # ---------------------------------------------------------------- DDS -> sim (physical units)
    def dds_subscriber(self, msg: Any, datatype: str = None) -> None:
        try:
            now = time.time()
            cmd = {
                "lin_x": _clamp1(msg.linear.x) * self.vmax,
                "ang_z": _clamp1(msg.angular.z) * R1D_MAX_ANGULAR_VEL,
                "height_speed": _clamp1(msg.linear.z),  # the provider scales it by R1D_LIFT_MAX_SPEED
                "move_stamp": now,
                "height_stamp": now,
            }
            if self.output_shm:
                self.output_shm.write_data(cmd)
        except Exception as e:
            print(f"[{self.node_name}] error processing command: {e}")

    def get_mobile_command(self) -> Optional[Dict[str, float]]:
        return self.output_shm.read_data() if self.output_shm else None

    # ---------------------------------------------------------------- sim -> DDS
    def write_mobile_state(self, height: float, height_speed: float, x: float, y: float, yaw: float, lin_x: float, ang_z: float):
        if self.input_shm:
            self.input_shm.write_data({"height": height, "height_speed": height_speed, "lin_x": lin_x, "ang_z": ang_z})

    def dds_publisher(self) -> None:
        try:
            data = self.input_shm.read_data() if self.input_shm else None
            if not data:
                return
            v = self.state_vel_msg
            v.linear.x = _dead(_clamp1(data["lin_x"] / self.vmax), DEADZONE_LINEAR)
            v.linear.y = 0.0
            v.linear.z = _dead(_clamp1(data["height_speed"] / R1D_LIFT_MAX_SPEED), DEADZONE_LINEAR)
            v.angular.x = v.angular.y = 0.0
            v.angular.z = _dead(_clamp1(data["ang_z"] / R1D_MAX_ANGULAR_VEL), DEADZONE_ANGULAR)
            self.height_msg.x = self.height_msg.y = 0.0
            self.height_msg.z = float(max(0.0, min(R1D_LIFT_SOFT_MAX, data["height"])) * 1000.0)
            h = float(data["height"])
            self.alarm_msg.state = BIT_LOWER if h <= SOFT_LIMIT_EPS else (BIT_UPPER if h >= R1D_LIFT_SOFT_MAX - SOFT_LIMIT_EPS else 0)
            if self.state_vel_pub:
                self.state_vel_pub.Write(v)
            if self.height_pub:
                self.height_pub.Write(self.height_msg)
            if self.alarm_pub:
                self.alarm_pub.Write(self.alarm_msg)
        except Exception as e:
            print(f"[{self.node_name}] error publishing state: {e}")
