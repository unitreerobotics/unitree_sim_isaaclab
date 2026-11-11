# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
DDS bridge for the Unitree G1-D waist pitch motor (same topics as the real G1-D flagship).

On the real G1-D the waist pitch motor (body motor 14) is not driven through rt/lowcmd but through its own
topics (unirobot ``motor_topics``: ``waist_pitch_joint`` -> rt/waistpitchstate / rt/waistpitchcmd):

Subscribed:
    rt/waistpitchcmd     unitree_hg/MotorCmd_     q = waist pitch target [rad] (positive = bend forward)
Published:
    rt/waistpitchstate   unitree_hg/MotorState_   q / dq / tau_est of the simulated waist pitch joint

Commands go to the simulation through shared memory ("isaac_g1d_waist_pitch_cmd", with the receive time),
the state comes back through "isaac_g1d_waist_pitch_state" (written by the G1-D state observation).
"""

import time
from typing import Any, Dict, Optional

from dds.dds_base import DDSObject
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__MotorState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import MotorCmd_, MotorState_

TOPIC_CMD = "rt/waistpitchcmd"
TOPIC_STATE = "rt/waistpitchstate"


class G1DWaistPitchDDS(DDSObject):
    def __init__(self, node_name: str = "g1d_waist_pitch"):
        if hasattr(self, "_initialized"):
            return
        super().__init__()
        self.node_name = node_name
        self.state_msg = unitree_hg_msg_dds__MotorState_()
        self.state_msg.mode = 1
        self.state_publisher = None
        self.setup_shared_memory(
            input_shm_name="isaac_g1d_waist_pitch_state",  # state from Isaac Lab -> DDS
            input_size=512,
            output_shm_name="isaac_g1d_waist_pitch_cmd",  # DDS command -> Isaac Lab
            output_size=512,
        )
        self._initialized = True
        print(f"[{self.node_name}] G1-D waist pitch DDS node initialized ({TOPIC_CMD} / {TOPIC_STATE})")

    # ---------------------------------------------------------------- DDS setup
    def setup_publisher(self) -> bool:
        try:
            self.state_publisher = ChannelPublisher(TOPIC_STATE, MotorState_)
            self.state_publisher.Init()
            print(f"[{self.node_name}] state publisher initialized ({TOPIC_STATE})")
            return True
        except Exception as e:
            print(f"[{self.node_name}] state publisher initialization failed: {e}")
            return False

    def setup_subscriber(self) -> bool:
        try:
            self.cmd_subscriber = ChannelSubscriber(TOPIC_CMD, MotorCmd_)
            self.cmd_subscriber.Init(lambda msg: self.dds_subscriber(msg, ""), 1)
            print(f"[{self.node_name}] command subscriber initialized ({TOPIC_CMD})")
            return True
        except Exception as e:
            print(f"[{self.node_name}] command subscriber initialization failed: {e}")
            return False

    # ---------------------------------------------------------------- DDS -> sim
    def dds_subscriber(self, msg: MotorCmd_, datatype: str = None) -> None:
        try:
            if self.output_shm:
                self.output_shm.write_data(
                    {"q": float(msg.q), "mode": int(msg.mode), "kp": float(msg.kp), "stamp": time.time()}
                )
        except Exception as e:
            print(f"[{self.node_name}] error processing command: {e}")

    def get_waist_pitch_command(self) -> Optional[Dict[str, Any]]:
        """Latest command: q [rad], mode, kp and the wall-clock receive time ``stamp``; None if none received."""
        return self.output_shm.read_data() if self.output_shm else None

    # ---------------------------------------------------------------- sim -> DDS
    def write_waist_pitch_state(self, q: float, dq: float, tau: float):
        if self.input_shm:
            self.input_shm.write_data({"q": float(q), "dq": float(dq), "tau": float(tau)})

    def dds_publisher(self) -> None:
        try:
            data = self.input_shm.read_data() if self.input_shm else None
            if not data:
                return
            self.state_msg.q = float(data["q"])
            self.state_msg.dq = float(data["dq"])
            self.state_msg.tau_est = float(data["tau"])
            if self.state_publisher:
                self.state_publisher.Write(self.state_msg)
        except Exception as e:
            print(f"[{self.node_name}] error publishing state: {e}")
