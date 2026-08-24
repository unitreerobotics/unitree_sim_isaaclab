# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""
Run command DDS communication class
Specialized in receiving the run command
"""

import threading
import time
from typing import Any, Dict, Optional
from dds.dds_base import DDSObject
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_


class RunCommandDDS(DDSObject):
    """Run command DDS node (singleton pattern)"""
    
    def __init__(self,node_name:str="run_command_dds"):
        """Initialize the run command DDS node"""
        # avoid duplicate initialization
        if hasattr(self, '_initialized'):
            return
        super().__init__()
        
        self._initialized = True
        self.node_name = node_name
        self.last_command_time = 0.0
        # setup the shared memory
        self.setup_shared_memory(
            output_shm_name="isaac_run_command_cmd", 
            input_shm_name="isaac_run_command_state",
            output_size=512, 
            input_size=3072,
            outputshm_flag=True,
            inputshm_flag=True,
        )
        print(f"[{self.node_name}] Run command DDS node initialized")

    
    def setup_publisher(self) -> bool:
        """Setup the run command publisher (this node is mainly used for subscribe, the publisher is optional)"""
        pass
    
    def setup_subscriber(self) -> bool:
        """Setup the run command subscriber"""
        try:
            self.subscriber = ChannelSubscriber("rt/run_command/cmd", String_)
            self.subscriber.Init(lambda msg: self.dds_subscriber(msg, ""), 1)
            
            print(f"[{self.node_name}] Run command subscriber initialized")
            return True
        except Exception as e:
            print(f"run_command_dds [{self.node_name}] Failed to initialize the run command subscriber: {e}")
            return False
    
    
    def dds_publisher(self) -> Any:
        """Process the publish data (this node is mainly used for subscribe, the publish function is optional)"""
        pass
    
    def dds_subscriber(self, msg: String_,datatype:str=None) -> Dict[str, Any]:
        """Process the subscribe data"""
        try:
            cmd_data = {
                "run_command": msg.data
            }
            self.output_shm.write_data(cmd_data)
            self.last_command_time = time.monotonic()
        except Exception as e:
            print(f"run_command_dds [{self.node_name}] Failed to process the subscribe data: {e}")
            return {}
    
    def get_run_command(self) -> Optional[Dict[str, Any]]:
        """Get the run command
        
        Returns:
            Dict: the run command, if no command return None
        """
        if self.output_shm:
            return self.output_shm.read_data()
        return None
    
    def write_run_command(self, flag_category):
        """Write the run command to the shared memory
        
        Args:
            positions: the run command, if no command return None
        """
        try:
            # prepare the reset pose data
            cmd_data = {
                "run_command":flag_category
            }
            
            # write the reset pose data to the shared memory
            if self.output_shm:
                self.output_shm.write_data(cmd_data)
                
        except Exception as e:
            print(f"run_command_dds [{self.node_name}] Failed to write the run command: {e}")
    
    def write_run_state(self, ang_vel,projected_gravity,joint_pos,joint_vel):
        """Write the robot state to the shared memory
        
        Args:
            ang_vel: the angular velocity list or torch.Tensor
            projected_gravity: the projected gravity list or torch.Tensor
            joint_pos: the joint position list or torch.Tensor
            joint_vel: the joint velocity list or torch.Tensor
        """
        if self.input_shm is None:
            return
        try:
            state_data = {
                "ang_vel": ang_vel.tolist() if hasattr(ang_vel, 'tolist') else ang_vel,
                "projected_gravity": projected_gravity.tolist() if hasattr(projected_gravity, 'tolist') else projected_gravity,
                "joint_pos": joint_pos.tolist() if hasattr(joint_pos, 'tolist') else joint_pos,
                "joint_vel": joint_vel.tolist() if hasattr(joint_vel, 'tolist') else joint_vel,
            }
            self.input_shm.write_data(state_data)
        except Exception as e:
            print(f"run_command_dds [{self.node_name}] Error writing robot state: {e}")
    def get_run_state(self):
        if self.input_shm is None:
            return None
        state_data = self.input_shm.read_data()
        if state_data is None:
            return None
        return state_data
