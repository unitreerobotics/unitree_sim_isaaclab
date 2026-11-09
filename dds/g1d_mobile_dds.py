# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
DDS bridge for the Unitree G1-D mobile base and lift column (same topics as the real robot,
see xr_teleoperate ``mobile-teleop`` / ``teleop/robot_control/mobile_control.py``).

Subscribed (commands from the teleoperation client):
    rt/cmd_hispeed         geometry_msgs/Point32   z = lift speed command in [-1, 1] (0 = hold)
    rt/cmd_vel_no_limit    geometry_msgs/Twist     linear.x [m/s], angular.z [rad/s]

Published (simulated robot state):
    rt/hispeed_state                          geometry_msgs/Point32   y = lift height [m], z = lift speed [m/s]
    rt/slamware_ros_sdk_server_node/odom      nav_msgs/Odometry       base pose (odom frame) and twist

Commands are passed to the simulation through shared memory ("isaac_g1d_mobile_cmd"), the state
comes back through "isaac_g1d_mobile_state" (written by the G1-D action provider).
"""

import math
import time
from typing import Any, Dict, Optional

from dds.dds_base import DDSObject
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import geometry_msgs_msg_dds__Point32_, nav_msgs_msg_dds__Odometry_
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Point32_, Twist_
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_

TOPIC_HEIGHT_CMD = "rt/cmd_hispeed"
TOPIC_HEIGHT_STATE = "rt/hispeed_state"
TOPIC_MOVE_CMD = "rt/cmd_vel_no_limit"
TOPIC_MOVE_STATE = "rt/slamware_ros_sdk_server_node/odom"


class G1DMobileDDS(DDSObject):
    def __init__(self, node_name: str = "g1d_mobile"):
        if hasattr(self, "_initialized"):
            return
        super().__init__()
        self.node_name = node_name
        self._cmd = {"height_speed": 0.0, "height_stamp": 0.0, "lin_x": 0.0, "ang_z": 0.0, "move_stamp": 0.0}
        self.height_state_msg = geometry_msgs_msg_dds__Point32_()
        self.odom_msg = nav_msgs_msg_dds__Odometry_()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_link"
        self.height_state_publisher = None
        self.odom_publisher = None
        self.setup_shared_memory(
            input_shm_name="isaac_g1d_mobile_state",  # state from Isaac Lab -> DDS
            input_size=1024,
            output_shm_name="isaac_g1d_mobile_cmd",  # DDS commands -> Isaac Lab
            output_size=1024,
        )
        self._initialized = True
        print(f"[{self.node_name}] G1-D mobile base / lift DDS node initialized")

    # ---------------------------------------------------------------- DDS setup
    def setup_publisher(self) -> bool:
        try:
            self.height_state_publisher = ChannelPublisher(TOPIC_HEIGHT_STATE, Point32_)
            self.height_state_publisher.Init()
            self.odom_publisher = ChannelPublisher(TOPIC_MOVE_STATE, Odometry_)
            self.odom_publisher.Init()
            print(f"[{self.node_name}] state publishers initialized ({TOPIC_HEIGHT_STATE}, {TOPIC_MOVE_STATE})")
            return True
        except Exception as e:
            print(f"[{self.node_name}] state publisher initialization failed: {e}")
            return False

    def setup_subscriber(self) -> bool:
        try:
            self.height_cmd_subscriber = ChannelSubscriber(TOPIC_HEIGHT_CMD, Point32_)
            self.height_cmd_subscriber.Init(lambda msg: self.dds_subscriber(msg, "height"), 1)
            self.move_cmd_subscriber = ChannelSubscriber(TOPIC_MOVE_CMD, Twist_)
            self.move_cmd_subscriber.Init(lambda msg: self.dds_subscriber(msg, "move"), 1)
            print(f"[{self.node_name}] command subscribers initialized ({TOPIC_HEIGHT_CMD}, {TOPIC_MOVE_CMD})")
            return True
        except Exception as e:
            print(f"[{self.node_name}] command subscriber initialization failed: {e}")
            return False

    # ---------------------------------------------------------------- DDS -> sim
    def dds_subscriber(self, msg: Any, datatype: str = None) -> None:
        try:
            now = time.time()
            if datatype == "height":
                self._cmd["height_speed"] = float(msg.z)
                self._cmd["height_stamp"] = now
            elif datatype == "move":
                self._cmd["lin_x"] = float(msg.linear.x)
                self._cmd["ang_z"] = float(msg.angular.z)
                self._cmd["move_stamp"] = now
            if self.output_shm:
                self.output_shm.write_data(dict(self._cmd))
        except Exception as e:
            print(f"[{self.node_name}] error processing command: {e}")

    def get_mobile_command(self) -> Optional[Dict[str, float]]:
        """Latest command: height_speed [-1, 1], lin_x [m/s], ang_z [rad/s] and their wall-clock stamps."""
        return self.output_shm.read_data() if self.output_shm else None

    # ---------------------------------------------------------------- sim -> DDS
    def write_mobile_state(self, height: float, height_speed: float, x: float, y: float, yaw: float, lin_x: float, ang_z: float):
        if self.input_shm:
            self.input_shm.write_data(
                {"height": height, "height_speed": height_speed, "x": x, "y": y, "yaw": yaw, "lin_x": lin_x, "ang_z": ang_z}
            )

    def dds_publisher(self) -> None:
        try:
            data = self.input_shm.read_data() if self.input_shm else None
            if not data:
                return
            self.height_state_msg.y = float(data["height"])
            self.height_state_msg.z = float(data["height_speed"])
            if self.height_state_publisher:
                self.height_state_publisher.Write(self.height_state_msg)

            o = self.odom_msg
            now = time.time()
            o.header.stamp.sec = int(now)
            o.header.stamp.nanosec = int((now - int(now)) * 1e9)
            o.pose.pose.position.x = float(data["x"])
            o.pose.pose.position.y = float(data["y"])
            o.pose.pose.position.z = 0.0
            o.pose.pose.orientation.x = 0.0
            o.pose.pose.orientation.y = 0.0
            o.pose.pose.orientation.z = math.sin(0.5 * float(data["yaw"]))
            o.pose.pose.orientation.w = math.cos(0.5 * float(data["yaw"]))
            o.twist.twist.linear.x = float(data["lin_x"])
            o.twist.twist.angular.z = float(data["ang_z"])
            if self.odom_publisher:
                self.odom_publisher.Write(o)
        except Exception as e:
            print(f"[{self.node_name}] error publishing state: {e}")
