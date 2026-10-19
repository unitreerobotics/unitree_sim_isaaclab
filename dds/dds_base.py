# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
from abc import ABC, abstractmethod
from dds.sharedmemorymanager import SharedMemoryManager
from typing import Any
class DDSObject(ABC):
    def __init__(self):
        self.publishing = False
        self.subscribing = False
    
    @abstractmethod
    def dds_publisher(self) -> None:
        pass
    @abstractmethod
    def dds_subscriber(self,msg:Any,datatype:str=None) -> None:
        """Process subscribe data"""
        pass
    @abstractmethod
    def setup_subscriber(): 
        """Process hand command"""
        pass
    @abstractmethod
    def setup_publisher():
        """Process hand command"""
        pass
    def setup_shared_memory(self, input_shm_name: str = None, output_shm_name: str = None, 
                           input_size: int = 4096, output_size: int = 4096,inputshm_flag:bool=True,outputshm_flag:bool=True):
        """Setup shared memory
        
        Args:
            input_shm_name: input shared memory name
            output_shm_name: output shared memory name
            input_size: input shared memory size
            output_size: output shared memory size
        """
        if inputshm_flag:
            if input_shm_name:
                self.input_shm = SharedMemoryManager(input_shm_name, input_size)
                print(f"[{self.node_name}] Input shared memory: {self.input_shm.get_name()}")
            else:
                self.input_shm = SharedMemoryManager(size=input_size)
                print(f"[{self.node_name}] Input shared memory: {self.input_shm.get_name()}")
        if outputshm_flag:
            if output_shm_name:
                self.output_shm = SharedMemoryManager(output_shm_name, output_size)
                print(f"[{self.node_name}] Output shared memory: {self.output_shm.get_name()}")
            else:
                self.output_shm = SharedMemoryManager(size=output_size)
                print(f"[{self.node_name}] Output shared memory: {self.output_shm.get_name()}")
    def stop_communication(self):
        self.publishing = False
        self.subscribing = False
        errors = []
        for name in ("subscriber", "publisher", "input_shm", "output_shm"):
            resource = getattr(self, name, None)
            if resource is None:
                continue
            method = "cleanup" if name.endswith("shm") else "Close"
            close = getattr(resource, method, None)
            try:
                if callable(close):
                    close()
                setattr(self, name, None)
            except Exception as error:
                errors.append(error)
        if errors:
            raise RuntimeError("Failed to close one or more DDS resources") from errors[0]
