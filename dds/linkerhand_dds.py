# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""LinkerHand O6 DDS (simulation only; the real hand is driven over CAN by the LinkerHand SDK).

* rt/linkerhand/{left,right}/cmd   (unitree_go MotorCmds_, 6 motors): ``q`` = raw angle 0..255 per joint in the SDK
  order thumb_flex, thumb_abd, index, middle, ring, pinky (255 = open, see robots/linkerhand_o6.py)
* rt/linkerhand/{left,right}/state (unitree_go MotorStates_, 6 motors, same order): ``q`` raw angle, ``dq`` raw / s,
  ``tau_est`` joint torque [Nm]
"""

import time
from typing import Any, Dict, Optional

from dds.dds_base import DDSObject
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_

from robots.linkerhand_o6 import O6_NUM_MOTORS

SIDES = ("left", "right")


class LinkerHandDDS(DDSObject):
    def __init__(self, node_name: str = "linkerhand"):
        if hasattr(self, "_initialized"):
            return
        super().__init__()
        self.node_name = node_name
        self.states = {}
        for side in SIDES:
            msg = MotorStates_()
            msg.states = [unitree_go_msg_dds__MotorState_() for _ in range(O6_NUM_MOTORS)]
            self.states[side] = msg
        self.publishers = {}
        self.commands = {f"{side}_hand_cmd": {} for side in SIDES}
        self._initialized = True
        self.setup_shared_memory(
            input_shm_name="isaac_linkerhand_state", input_size=2048,
            output_shm_name="isaac_linkerhand_cmd", output_size=2048,
        )
        print(f"[{self.node_name}] LinkerHand DDS node initialized")

    def setup_publisher(self) -> bool:
        try:
            for side in SIDES:
                self.publishers[side] = ChannelPublisher(f"rt/linkerhand/{side}/state", MotorStates_)
                self.publishers[side].Init()
            print(f"[{self.node_name}] LinkerHand state publisher initialized")
            return True
        except Exception as e:
            print(f"linkerhand_dds [{self.node_name}] state publisher initialization failed: {e}")
            return False

    def setup_subscriber(self) -> bool:
        try:
            self._subscribers = []
            for side in SIDES:
                sub = ChannelSubscriber(f"rt/linkerhand/{side}/cmd", MotorCmds_)
                sub.Init(lambda msg, side=side: self.dds_subscriber(msg, side), 1)
                self._subscribers.append(sub)
            print(f"[{self.node_name}] LinkerHand command subscriber initialized")
            return True
        except Exception as e:
            print(f"linkerhand_dds [{self.node_name}] command subscriber initialization failed: {e}")
            return False

    def dds_subscriber(self, msg: Any, datatype: str = None) -> None:
        try:
            raw = [float(c.q) for c in msg.cmds[:O6_NUM_MOTORS]]
            if len(raw) < O6_NUM_MOTORS:
                return
            self.commands[f"{datatype}_hand_cmd"] = {"positions": raw, "stamp": time.time()}
            if self.output_shm:
                self.output_shm.write_data(self.commands)
        except Exception as e:
            print(f"linkerhand_dds [{self.node_name}] error processing {datatype} command: {e}")

    def dds_publisher(self) -> None:
        try:
            data = self.input_shm.read_data() or {}
            for side in SIDES:
                hand = data.get(f"{side}_hand")
                if not hand or side not in self.publishers:
                    continue
                msg = self.states[side]
                for i, (q, dq, tau) in enumerate(zip(hand["positions"], hand["velocities"], hand["torques"])):
                    msg.states[i].q, msg.states[i].dq, msg.states[i].tau_est = float(q), float(dq), float(tau)
                self.publishers[side].Write(msg)
        except Exception as e:
            print(f"linkerhand_dds [{self.node_name}] error publishing state: {e}")

    def get_hand_commands(self) -> Optional[Dict[str, Any]]:
        """{"left_hand_cmd": {"positions": [6 raw], "stamp": t}, "right_hand_cmd": {...}} or None."""
        return self.output_shm.read_data() if self.output_shm else None

    def write_hand_states(self, states: Dict[str, Dict[str, list]]) -> None:
        """``states``: side -> {"positions": [6 raw], "velocities": [6], "torques": [6]}."""
        if self.input_shm:
            self.input_shm.write_data({f"{side}_hand": v for side, v in states.items()})
