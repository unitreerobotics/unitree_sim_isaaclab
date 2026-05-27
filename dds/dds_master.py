# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
import time
import threading
from typing import Dict, List, Optional
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from dds.dds_base import DDSObject



class DDSManager:    
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super(DDSManager, cls).__new__(cls)
        return cls._instance
    
    def __init__(self):
        """Init DDSManager"""
        if hasattr(self, '_initialized'):
            return
        
        self._initialized = True
        self.publishing_running = False
        self.subscribing_running = False
        
        self.objects: Dict[str, DDSObject] = {}
        
        self.publish_thread: Optional[threading.Thread] = None
        self.subscribe_thread: Optional[threading.Thread] = None
        
        # publish object cache and frequency control (Hz→interval)
        self._pub_list: List[str] = []
        self._pub_interval: Dict[str, float] = {}
        self._pub_next_ts: Dict[str, float] = {}
        self._default_pub_interval: float = 0.01  # 100Hz default

        self.dds_initialized = False
        self._init_dds()
        print("[DDSManager] DDSManager initialized")
    
    def _parse_object_name(self, name: str) -> tuple[str, str]:
        """Parse object name"""
        if ':' in name:
            parts = name.split(':', 1)
            return parts[0], parts[1]
        else:
            return "", name
    
    def _init_dds(self) -> bool:
        """Init DDS system"""
        if self.dds_initialized:
            return True
        
        try:
            # HANDSIM PATCH 2026-05-26: explicit "lo" interface so cross-process
            # DDS discovery works on Ubuntu 24.04. Autodetermine picked an
            # interface that lacked working multicast, so external publishers
            # (e.g. WBC composer, GR00T-driven goal sender) couldn't reach the
            # sim's rt/lowstate subscribers. cyclonedds falls back to unicast
            # on lo and that works fine for local cross-process IPC.
            ChannelFactoryInitialize(1, "lo")
            self.dds_initialized = True
            print("[DDSManager] DDS system initialized (forced lo interface)")
            return True
        except Exception as e:
            print(f"[DDSManager] DDS system initialization failed: {e}")
            return False
    
    def register_object(self, name: str, obj: DDSObject) -> bool:
        """Register DDS object"""
        if name in self.objects:
            print(f"[DDSManager] object '{name}' already exists")
            return False
        
        try:
            category, obj_name = self._parse_object_name(name)
            
            self.objects[name] = obj
            
            print(f"[DDSManager] register object '{name}' success (category: {category or 'No category'})")
            
            # default frequency
            self._pub_interval[name] = self._default_pub_interval
            self._pub_next_ts[name] = 0.0
            return True
        except Exception as e:
            print(f"[DDSManager] register object '{name}' failed: {e}")
            return False
    
    def unregister_object(self, name: str) -> bool:
        """Unregister DDS object"""
        if name not in self.objects:
            print(f"[DDSManager] object '{name}' not found")
            return False
        
        obj = self.objects[name]
        obj.publishing = False
        obj.subscribing = False
        
        del self.objects[name]
        self._pub_interval.pop(name, None)
        self._pub_next_ts.pop(name, None)
        if name in self._pub_list:
            self._pub_list.remove(name)
        print(f"[DDSManager] unregister object '{name}' success")
        return True
    
    def get_object(self, name: str) -> Optional[DDSObject]:
        """Get specified object"""
        obj = self.objects.get(name)
        if obj is None:
            print(f"[DDSManager] object '{name}' not found, objects: {self.objects.keys()}")
            return None
        return obj
    
    def get_objects_by_category(self, category: str) -> Dict[str, DDSObject]:
        """Get all objects by category"""
        result = {}
        for full_name, obj in self.objects.items():
            cat, obj_name = self._parse_object_name(full_name)
            if cat == category:
                result[obj_name] = obj
        return result
    
    def set_publish_rate(self, name: str, hz: float) -> None:
        """Set publish rate (Hz) for a specific object"""
        if name in self.objects and hz > 0:
            self._pub_interval[name] = 1.0 / hz
            # make the next cycle take effect immediately
            self._pub_next_ts[name] = 0.0
            print(f"[DDSManager] set publish rate for '{name}' to {hz}Hz")
    
    def set_default_publish_rate(self, hz: float) -> None:
        if hz > 0:
            self._default_pub_interval = 1.0 / hz
            for name in self.objects.keys():
                if name not in self._pub_interval:
                    self._pub_interval[name] = self._default_pub_interval
            print(f"[DDSManager] default publish rate set to {hz}Hz")

    def _publish_loop(self) -> None:
        """Publish loop thread"""
        print("[DDSManager] publish loop thread started")
        
        while self.publishing_running:
            try:
                now = time.perf_counter()
                next_due = None
                for name in self._pub_list:
                    obj = self.objects.get(name)
                    if obj is None or not obj.publishing:
                        continue
                    interval = self._pub_interval.get(name, self._default_pub_interval)
                    due = self._pub_next_ts.get(name, 0.0)
                    if now >= due:
                        try:
                            obj.dds_publisher()
                        except Exception as e:
                            print(f"[DDSManager] object '{name}' publish failed: {e}")
                        # schedule next
                        self._pub_next_ts[name] = now + interval
                    # track earliest due
                    nd = self._pub_next_ts.get(name, now + interval)
                    if next_due is None or nd < next_due:
                        next_due = nd
                # dynamic sleep until the nearest due, minimum lower bound
                if next_due is not None:
                    sleep_time = max(0.0002, next_due - time.perf_counter())
                    time.sleep(sleep_time)
                else:
                    time.sleep(0.001)
                
            except Exception as e:
                print(f"[DDSManager] publish loop error: {e}")
                time.sleep(0.01)
        
        print("[DDSManager] publish loop thread stopped")
    
    def start_publishing(self,enable_publish_names:List[str]=None):
        """Start publishing"""
        self._pub_list.clear()
        for name, obj in self.objects.items():
            if enable_publish_names is None or name in enable_publish_names:
                obj.setup_publisher()
                obj.publishing = True
                self._pub_list.append(name)
        self.publishing_running = True
        
        self.publish_thread = threading.Thread(target=self._publish_loop)
        self.publish_thread.daemon = True
        self.publish_thread.start()
        print(f"[DDSManager] manager started, managing {len(self._pub_list)} publishing objects")
    def stop_publishing(self):
        """Stop publishing"""
        for name, obj in self.objects.items():
            obj.publishing = False
        self.publishing_running = False
    def stop_subscribing(self):
        """Stop subscribing"""
        for name, obj in self.objects.items():
            obj.subscribing = False
        self.subscribing_running = False
    def start_subscribing(self,enable_subscribe_names:List[str]=None):
        """Start subscribing"""
        for name, obj in self.objects.items():  
            if enable_subscribe_names is None or name in enable_subscribe_names:
                obj.setup_subscriber()
                obj.subscribing = True


    def stop_all_communication(self):
        for name, obj in self.objects.items():
            obj.stop_communication()    
            self.publishing_running=False
            self.subscribing_running=False
# global singleton instance
dds_manager = DDSManager()
