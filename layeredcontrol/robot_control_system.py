# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""
A layered robot control system
"""

import time
from typing import Optional, Dict, Any
import torch
from dataclasses import dataclass
from action_provider.action_base import ActionProvider


@dataclass
class ControlConfig:
    """minimal control configuration"""
    step_hz: int = 500  # the frequency of the low-level execution
    replay_mode: bool = False
    use_rl_action_mode: bool = False


class RobotController:
    """robot controller
    """
    
    def __init__(self, env, config: ControlConfig):
        self.env = env
        self.config = config
        self.action_provider: Optional[ActionProvider] = None
        self.is_running = False
        
        
        # minimal frequency control
        self._step_interval = 1.0 / config.step_hz
        self._last_step_time = 0.0
        
        all_joint_names = env.scene["robot"].data.joint_names
        self._last_action = torch.zeros(len(all_joint_names), device=env.device)
        
        
        # pre-calculate the sleep threshold (avoid calculating every time)
        self._sleep_threshold = 0.0002
        self._sleep_adjustment = 0.0001
        
        # minimal statistics
        self.step_count = 0
        self._start_time = 0.0
        
        # minimal performance analysis
        self._profile_counter = 0
        self._profile_interval = 2000  # reduce the printing frequency
        
        # cache the function reference (reduce the lookup overhead)
        self._perf_counter = time.perf_counter
        self._time_sleep = time.sleep
        
        print(f"  - control frequency: {config.step_hz}Hz")
    
    def set_action_provider(self, provider: ActionProvider):
        """set the action provider"""
        if self.action_provider:
            self.action_provider.stop()
            self.action_provider.cleanup()
        
        self.action_provider = provider
        print(f"[SimpleController] set the action provider: {provider.name}")
    
    def start(self):
        """start the controller"""
        if self.is_running:
            return
        
        self.is_running = True
        self._start_time = time.time()
        self._last_step_time = self._perf_counter()
        
        # start the action provider
        if self.action_provider:
            self.action_provider.start()
        
        print("[SimpleController] the controller is started")
    
    def stop(self):
        """stop the controller"""
        if not self.is_running:
            return
            
        self.is_running = False
        
        if self.action_provider:
            self.action_provider.stop()
        
        print("[SimpleController] the controller is stopped")
    
    def step(self):
        """minimal control step - zero thread competition"""
        if not self.is_running:
            return
        
        # use the cached function reference
        perf_counter = self._perf_counter
        step_start = perf_counter()
        
        # 1. minimal action acquisition (synchronous, zero thread competition, pre-calculated strategy)
        action_start = perf_counter()
        action = None
        
        # try to get the action from the action provider
        if self.action_provider:
            action = self.action_provider.get_action(self.env)
            if action is not None:
                self._last_action = action
        
        # if no action is obtained, use the pre-calculated fallback strategy
        if action is None:
            action = self._last_action

        action_time = perf_counter() - action_start
        
        # 2. direct environment step
        env_start = perf_counter()
        with torch.inference_mode():
            if self.config.replay_mode or self.config.use_rl_action_mode:
                pass
                # self.env.sim.render()
            else:
                step_result = self.env.step(action)
                if len(step_result) >= 4:
                    _, _, terminated, time_outs = step_result[:4]
                    if torch.any(terminated) or torch.any(time_outs):
                        reset_kind = "timeout" if torch.any(time_outs) else "termination"
                        print(f"[AUTO_RESET] env reset triggered by {reset_kind}", flush=True)
            env_time = perf_counter() - env_start
            
            self.step_count += 1
        
        # 3. minimal frequency control (no rendering overhead, use the pre-calculated threshold)
        sleep_start = perf_counter()
        current_time = perf_counter()
        if self._last_step_time > 0:
            elapsed = current_time - self._last_step_time
            sleep_needed = self._step_interval - elapsed
            if sleep_needed > self._sleep_threshold:  # use the pre-calculated threshold
                self._time_sleep(sleep_needed - self._sleep_adjustment)  # use the pre-calculated adjustment value
        self._last_step_time = current_time
        sleep_time = perf_counter() - sleep_start
        
        # 4. minimal performance print
        self._profile_counter += 1
        if self._profile_counter >= self._profile_interval:
            total_time = perf_counter() - step_start
            print(f"[Performance] A:{action_time*1000:.1f}ms, E:{env_time*1000:.1f}ms, S:{sleep_time*1000:.1f}ms, T:{total_time*1000:.1f}ms")
            self._profile_counter = 0
    def cleanup(self):
        """clean up the resources"""
        self.stop()
        if self.action_provider:
            self.action_provider.cleanup()
    
    def set_profiling(self, enabled: bool, interval: int = 2000):
        """set the performance analysis"""
        if enabled:
            self._profile_interval = interval
        else:
            self._profile_interval = 999999999  # actually disable the printing
