# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
from action_provider.action_base import ActionProvider
from typing import Optional
import torch
from dds.dds_master import dds_manager
import os
import onnxruntime as ort
from dds.sharedmemorymanager import SharedMemoryManager
import time
import threading
from isaaclab.utils.buffers import CircularBuffer,DelayBuffer
import os
import ast
project_root = os.environ.get("PROJECT_ROOT")
class DDSRLActionProvider(ActionProvider):
    """Action provider based on DDS"""
    
    def __init__(self,env, args_cli):
        super().__init__("DDSActionProvider")
        self.enable_robot = args_cli.robot_type
        self.enable_gripper = args_cli.enable_dex1_dds
        self.enable_dex3 = args_cli.enable_dex3_dds
        self.enable_inspire = args_cli.enable_inspire_dds
        self.wh = args_cli.enable_wholebody_dds
        self.policy_path = f"{project_root}/"+args_cli.model_path
        self.env = env
        if self.env.num_envs != 1:
            raise ValueError(
                "DDS Wholebody teleoperation requires num_envs == 1; "
                f"received {self.env.num_envs}"
            )
        # Initialize DDS communication
        self.robot_dds = None
        self.gripper_dds = None
        self.dex3_dds = None
        self.inspire_dds = None
        self.run_command = None
        self._setup_dds()
        self._setup_joint_mapping()
        self.policy = self.load_policy(self.policy_path)
        
        # 预计算索引张量与复用缓冲
        device = self.env.device
        if hasattr(self, "arm_joint_mapping") and self.arm_joint_mapping:
            self._arm_target_indices = [self.joint_to_index[name] for name in self.arm_joint_mapping.keys()]
            self._arm_source_indices = [idx + 15 for idx in self.arm_joint_mapping.values()]
            self._arm_target_idx_t = torch.tensor(self._arm_target_indices, dtype=torch.long, device=device)
            self._arm_source_idx_t = torch.tensor(self._arm_source_indices, dtype=torch.long, device=device)
        if self.enable_gripper:
            self._gripper_target_indices = [self.joint_to_index[name] for name in self.gripper_joint_mapping.keys()]
            self._gripper_source_indices = [idx for idx in self.gripper_joint_mapping.values()]
            self._gripper_target_idx_t = torch.tensor(self._gripper_target_indices, dtype=torch.long, device=device)
            self._gripper_source_idx_t = torch.tensor(self._gripper_source_indices, dtype=torch.long, device=device)
        if self.enable_dex3:
            self._left_hand_target_indices = [self.joint_to_index[name] for name in self.left_hand_joint_mapping.keys()]
            self._left_hand_source_indices = [idx for idx in self.left_hand_joint_mapping.values()]
            self._right_hand_target_indices = [self.joint_to_index[name] for name in self.right_hand_joint_mapping.keys()]
            self._right_hand_source_indices = [idx for idx in self.right_hand_joint_mapping.values()]
            self._left_hand_target_idx_t = torch.tensor(self._left_hand_target_indices, dtype=torch.long, device=device)
            self._left_hand_source_idx_t = torch.tensor(self._left_hand_source_indices, dtype=torch.long, device=device)
            self._right_hand_target_idx_t = torch.tensor(self._right_hand_target_indices, dtype=torch.long, device=device)
            self._right_hand_source_idx_t = torch.tensor(self._right_hand_source_indices, dtype=torch.long, device=device)
        if self.enable_inspire:
            self._inspire_target_indices = [self.joint_to_index[name] for name in self.inspire_hand_joint_mapping.keys()]
            self._inspire_source_indices = [idx for idx in self.inspire_hand_joint_mapping.values()]
            self._inspire_special_target_indices = [self.joint_to_index[name] for name in self.special_joint_mapping.keys()]
            self._inspire_special_source_indices = [spec[0] for spec in self.special_joint_mapping.values()]
            self._inspire_special_scales = torch.tensor([spec[1] for spec in self.special_joint_mapping.values()], dtype=torch.float32)
            self._inspire_target_idx_t = torch.tensor(self._inspire_target_indices, dtype=torch.long, device=device)
            self._inspire_source_idx_t = torch.tensor(self._inspire_source_indices, dtype=torch.long, device=device)
            self._inspire_special_target_idx_t = torch.tensor(self._inspire_special_target_indices, dtype=torch.long, device=device)
            self._inspire_special_source_idx_t = torch.tensor(self._inspire_special_source_indices, dtype=torch.long, device=device)
            self._inspire_special_scales_t = self._inspire_special_scales.to(device)
        
        self._full_action_buf = torch.zeros(len(self.all_joint_names), device=device, dtype=torch.float32)
        self._positions_buf = torch.empty(29, device=device, dtype=torch.float32)
        if self.enable_gripper:
            self._gripper_buf = torch.empty(2, device=device, dtype=torch.float32)
        if self.enable_dex3:
            self._left_hand_buf = torch.empty(len(self._left_hand_source_indices), device=device, dtype=torch.float32)
            self._right_hand_buf = torch.empty(len(self._right_hand_source_indices), device=device, dtype=torch.float32)
        if self.enable_inspire:
            self._inspire_buf = torch.empty(12, device=device, dtype=torch.float32)
        
    def _setup_dds(self):
        """Setup DDS communication"""
        print(f"enable_robot: {self.enable_robot}")
        print(f"enable_gripper: {self.enable_gripper}")
        print(f"enable_dex3: {self.enable_dex3}")
        try:
            if self.enable_robot == "g129":
                self.robot_dds = dds_manager.get_object("g129")
            if self.enable_gripper:
                self.gripper_dds = dds_manager.get_object("dex1")
            elif self.enable_dex3:
                self.dex3_dds = dds_manager.get_object("dex3")
            elif self.enable_inspire:
                self.inspire_dds = dds_manager.get_object("inspire")
            if self.wh:
                self.run_command_dds = dds_manager.get_object("run_command")
            print(f"[{self.name}] DDS communication initialized")
        except Exception as e:
            print(f"[{self.name}] DDS initialization failed: {e}")
    
    def _setup_joint_mapping(self):
        """Setup joint mapping"""
        if self.wh:
            self.action_joint_names = [
            'left_hip_pitch_joint', 
            'right_hip_pitch_joint', 
            'left_hip_roll_joint', 
            'right_hip_roll_joint', 
            'left_hip_yaw_joint', 
            'right_hip_yaw_joint', 
            'left_knee_joint', 
            'right_knee_joint', 
            'left_ankle_pitch_joint',
            'right_ankle_pitch_joint',
            'left_ankle_roll_joint',
            'right_ankle_roll_joint'
            ]
            self.waist_joint_mapping = [
                'waist_yaw_joint',
                'waist_roll_joint',
                'waist_pitch_joint',
            ]
            self.arm_joint_names = [
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
            self.old_action_joints_names = [
            'left_hip_pitch_joint', 
            'right_hip_pitch_joint', 
            'waist_yaw_joint', 
            'left_hip_roll_joint', 
            'right_hip_roll_joint', 
            'waist_roll_joint',
            'left_hip_yaw_joint', 
            'right_hip_yaw_joint', 
            'waist_pitch_joint', 
            'left_knee_joint', 
            'right_knee_joint', 
            'left_shoulder_pitch_joint',
            'right_shoulder_pitch_joint', 
            'left_ankle_pitch_joint', 
            'right_ankle_pitch_joint',
            'left_shoulder_roll_joint', 
            'right_shoulder_roll_joint', 
            'left_ankle_roll_joint', 
            'right_ankle_roll_joint', 
            'left_shoulder_yaw_joint', 
            'right_shoulder_yaw_joint', 
            'left_elbow_joint', 
            'right_elbow_joint', 
            'left_wrist_roll_joint', 
            'right_wrist_roll_joint', 
            'left_wrist_pitch_joint', 
            'right_wrist_pitch_joint', 
            'left_wrist_yaw_joint', 
            'right_wrist_yaw_joint',]
        if self.enable_robot == "g129":
            self.arm_joint_mapping = {
                "left_shoulder_pitch_joint": 0,
                "left_shoulder_roll_joint": 1,
                "left_shoulder_yaw_joint": 2,
                "left_elbow_joint": 3,
                "left_wrist_roll_joint": 4,
                "left_wrist_pitch_joint": 5,
                "left_wrist_yaw_joint": 6,
                "right_shoulder_pitch_joint": 7,
                "right_shoulder_roll_joint": 8,
                "right_shoulder_yaw_joint": 9,
                "right_elbow_joint": 10,
                "right_wrist_roll_joint": 11,
                "right_wrist_pitch_joint": 12,
                "right_wrist_yaw_joint": 13
            }
        if self.enable_gripper:
            self.gripper_joint_mapping = {
                "left_hand_Joint1_1": 1,
                "left_hand_Joint2_1": 1,
                "right_hand_Joint1_1": 0,
                "right_hand_Joint2_1": 0,
            }
        if self.enable_dex3:
            self.left_hand_joint_mapping = {
                "left_hand_thumb_0_joint":0,
                "left_hand_thumb_1_joint":1,
                "left_hand_thumb_2_joint":2,
                "left_hand_middle_0_joint":3,
                "left_hand_middle_1_joint":4,
                "left_hand_index_0_joint":5,
                "left_hand_index_1_joint":6}
            self.right_hand_joint_mapping = {
                "right_hand_thumb_0_joint":0,     
                "right_hand_thumb_1_joint":1,
                "right_hand_thumb_2_joint":2,
                "right_hand_middle_0_joint":3,
                "right_hand_middle_1_joint":4,
                "right_hand_index_0_joint":5,
                "right_hand_index_1_joint":6}
        if self.enable_inspire:
            self.inspire_hand_joint_mapping = {
                "R_pinky_proximal_joint":0,
                "R_ring_proximal_joint":1,
                "R_middle_proximal_joint":2,
                "R_index_proximal_joint":3,
                "R_thumb_proximal_pitch_joint":4,
                "R_thumb_proximal_yaw_joint":5,
                "L_pinky_proximal_joint":6,
                "L_ring_proximal_joint":7,
                "L_middle_proximal_joint":8,
                "L_index_proximal_joint":9,
                "L_thumb_proximal_pitch_joint":10,
                "L_thumb_proximal_yaw_joint":11,
            }
            self.special_joint_mapping = {
                "L_index_intermediate_joint":[9,1],
                "L_middle_intermediate_joint":[8,1],
                "L_pinky_intermediate_joint":[6,1],
                "L_ring_intermediate_joint":[7,1],
                "L_thumb_intermediate_joint":[10,1.5],
                "L_thumb_distal_joint":[10,2.4],

                "R_index_intermediate_joint":[3,1],
                "R_middle_intermediate_joint":[2,1],
                "R_pinky_intermediate_joint":[0,1],
                "R_ring_intermediate_joint":[1,1],
                "R_thumb_intermediate_joint":[4,1.5],
                "R_thumb_distal_joint":[4,2.4],
            }
        self.all_joint_names = self.env.scene["robot"].data.joint_names
        self.joint_to_index = {name: i for i, name in enumerate(self.all_joint_names)}
        self.arm_action_pose = [self.joint_to_index[name] for name in self.arm_joint_mapping.keys()]
        self.arm_action_pose_indices = [self.arm_joint_mapping[name] for name in self.arm_joint_mapping.keys()]
        self.action_to_indices=[]
        for action_joint in self.action_joint_names:
            if action_joint in self.all_joint_names:
                self.action_to_indices.append(self.all_joint_names.index(action_joint))
            else:
                raise ValueError(f"action joint '{action_joint}' not in all joint list")
        self.waist_to_all_indices = []
        for waist_joint in self.waist_joint_mapping:
            if waist_joint in self.all_joint_names:
                self.waist_to_all_indices.append(self.all_joint_names.index(waist_joint))
            else:
                raise ValueError(f"waist joint '{waist_joint}' not in all joint list")

        self.arm_to_all_indices=[]
        for arm_joint in self.arm_joint_names:
            if arm_joint in self.all_joint_names:
                self.arm_to_all_indices.append(self.all_joint_names.index(arm_joint))
            else:
                raise ValueError(f"arm joint '{arm_joint}' not in all joint list")
        self.default_waist_positions = self.env.scene["robot"].data.default_joint_pos[:, self.waist_to_all_indices]
        self.default_action_positions = self.env.scene["robot"].data.default_joint_pos
        self.default_action_velocities = self.env.scene["robot"].data.default_joint_vel
        self.all_obs_indices = self.action_to_indices + self.arm_to_all_indices
        self.old_action_indices = []
        for old_action_joint in self.old_action_joints_names:
            if old_action_joint in self.all_joint_names:
                self.old_action_indices.append(self.all_joint_names.index(old_action_joint))
            else:
                raise ValueError(f"action joint '{old_action_joint}' not in all joint list")
        self.arm_action = []
        self.obs_scales = {"ang_vel":1.0, "projected_gravity":1.0, "commands":1.0, 
                           "joint_pos":1.0, "joint_vel":1.0, "actions":1.0}
        self.ang_vel = self.env.scene["robot"].data.root_ang_vel_b                      
        self.projected_gravity = self.env.scene["robot"].data.projected_gravity_b       
        self.joint_pos = self.env.scene["robot"].data.joint_pos
        self.joint_vel = self.env.scene["robot"].data.joint_vel
        self.actor_obs_buffer = CircularBuffer(
            max_len=10, batch_size=1, device=self.env.device
        )
        self.num_envs =1
        self.clip_obs = 100
        self.num_actions_all = self.env.scene["robot"].data.default_joint_pos[:,self.old_action_indices].shape[1]  
        self.action_buffer = DelayBuffer(
            5, self.num_envs, device=self.env.device
        )
        self.action_buffer.compute(
            torch.zeros(self.num_envs, self.num_actions_all, dtype=torch.float, device=self.env.device, requires_grad=False)
        )
        self.clip_actions = 100
        self.action_scale = 0.25
        self.sim_step_counter = 0
    def load_policy(self,path):
        ext = os.path.splitext(path)[1].lower()
        if ext==".onnx":
            return self.load_onnx_policy(path)
        elif ext==".pt":
            return self.load_jit_pt_policy(path)

    def load_jit_pt_policy(self,path):
        return torch.jit.load(path)

    def load_onnx_policy(self,path):
        model = ort.InferenceSession(path)
        def run_inference(input_tensor):
            ort_inputs = {model.get_inputs()[0].name: input_tensor.cpu().numpy()}
            ort_outs = model.run(None, ort_inputs)
            return torch.tensor(ort_outs[0], device=self.env.device)
        return run_inference
    def compute_current_observations(self):
        command = [0,0,0,0.8]  
        run_command = self.run_command_dds.get_run_command()
        command_is_fresh = (
            time.monotonic() - self.run_command_dds.last_command_time < 0.25
        )
        if command_is_fresh and run_command and 'run_command' in run_command:
            run_command_data = run_command['run_command']
            
            if isinstance(run_command_data, str):
                try:
                    run_command_list = ast.literal_eval(run_command_data)
                    if isinstance(run_command_list, list) and len(run_command_list) >= 4:
                        command[0] = float(run_command_list[0])
                        command[1] = float(run_command_list[1])
                        command[2] = float(run_command_list[2])
                        command[3] = float(run_command_list[3])
                except (ValueError, SyntaxError) as e:
                    print(f"[WARNING] cannot parse run_command string: {run_command_data}, error: {e}")
            else:
                try:
                    command[0] = float(run_command_data[0])
                    command[1] = float(run_command_data[1])
                    command[2] = float(run_command_data[2])
                    command[3] = float(run_command_data[3])
                except (IndexError, TypeError) as e:
                    print(f"[WARNING] cannot parse run_command data: {run_command_data}, error: {e}")
            
      
        # command = [0.5,0.0,0.7,0.8]
        command = torch.tensor(command, device=self.env.device, dtype=torch.float32)
        
        if command.dim() == 1:
            command = command.unsqueeze(0)  # [4] -> [1, 4]
        self.ang_vel = self.env.scene["robot"].data.root_ang_vel_b                      
        self.projected_gravity = self.env.scene["robot"].data.projected_gravity_b       
        self.joint_pos = self.env.scene["robot"].data.joint_pos
        self.joint_vel = self.env.scene["robot"].data.joint_vel
        action = self.action_buffer._circular_buffer.buffer[:, -1, :]     
        current_actor_obs = torch.cat(
        [
            self.ang_vel * self.obs_scales["ang_vel"],
            self.projected_gravity * self.obs_scales["projected_gravity"],
            command * self.obs_scales["commands"],
            (self.joint_pos[:, self.all_obs_indices] - self.default_action_positions[:, self.all_obs_indices]) * self.obs_scales["joint_pos"],
            (self.joint_vel[:, self.all_obs_indices] - self.default_action_velocities[:, self.all_obs_indices]) * self.obs_scales["joint_vel"],
            action * self.obs_scales["actions"],  # [29] -> [1, 29]
        ],
        dim=-1,
    )
        return current_actor_obs
    def compute_observations(self):

        current_actor_obs = self.compute_current_observations()

        self.actor_obs_buffer.append(current_actor_obs)
        actor_obs = self.actor_obs_buffer.buffer.reshape(self.num_envs, -1)
        actor_obs = torch.clip(actor_obs, -self.clip_obs, self.clip_obs)
        return actor_obs
    
    def run_policy(self):
        current_actor_obs = self.compute_observations()
        action = self.policy(current_actor_obs)
        return action
    def get_action(self, env) -> Optional[torch.Tensor]:
        """Get action from DDS"""
        try:
            full_action = self._full_action_buf
            full_action.zero_()
            action_data = self.run_policy()

            # RL 输出与腰部默认位姿
            full_action[self.action_to_indices] = action_data
            full_action[self.waist_to_all_indices] = self.default_waist_positions
            # 机器人指令（若有）
            if self.enable_robot == "g129" and self.robot_dds:
                cmd_data = self.robot_dds.get_robot_command()
                if cmd_data and 'motor_cmd' in cmd_data:
                    positions = cmd_data['motor_cmd']['positions']
                    if len(positions) >= 29 and hasattr(self, "_arm_source_idx_t"):
                        self._positions_buf[:29].copy_(torch.tensor(positions[:29], dtype=torch.float32, device=self.env.device))
                        arm_vals = self._positions_buf.index_select(0, self._arm_source_idx_t)
                        full_action.index_copy_(0, self._arm_target_idx_t, arm_vals)
                        # Quest right stick publishes waist yaw at G1 motor 12.
                        full_action[self.waist_to_all_indices[0]] = self._positions_buf[12]
            # 延时/裁剪/缩放
            delayed_actions = self.action_buffer.compute(full_action[self.old_action_indices].unsqueeze(0))
            cliped_actions = torch.clip(delayed_actions[:,self.action_to_indices], -self.clip_actions, self.clip_actions).to(self.env.device)
            full_action[self.action_to_indices] = cliped_actions * self.action_scale + self.default_action_positions[:, self.action_to_indices]
            
            # 夹爪/手指（若有）
            if self.gripper_dds and hasattr(self, "_gripper_source_idx_t"):
                gripper_cmd = self.gripper_dds.get_gripper_command()
                if gripper_cmd:
                    left_gripper_cmd = gripper_cmd.get('left_gripper_cmd', {})
                    right_gripper_cmd = gripper_cmd.get('right_gripper_cmd', {})
                    left_gripper_positions = left_gripper_cmd.get('positions', [])
                    right_gripper_positions = right_gripper_cmd.get('positions', [])
                    gripper_positions = right_gripper_positions + left_gripper_positions
                    if len(gripper_positions) >= 2:
                        self._gripper_buf.copy_(torch.tensor(gripper_positions[:2], dtype=torch.float32, device=self.env.device))
                        gp_vals = self._gripper_buf.index_select(0, self._gripper_source_idx_t)
                        full_action.index_copy_(0, self._gripper_target_idx_t, gp_vals)
            elif self.dex3_dds and hasattr(self, "_left_hand_source_idx_t"):
                hand_cmds = self.dex3_dds.get_hand_commands()
                if hand_cmds:
                    left_hand_cmd = hand_cmds.get('left_hand_cmd', {})
                    right_hand_cmd = hand_cmds.get('right_hand_cmd', {})
                    if left_hand_cmd and right_hand_cmd:
                        left_positions = left_hand_cmd.get('positions', [])
                        right_positions = right_hand_cmd.get('positions', [])
                        if len(left_positions) >= len(self._left_hand_buf) and len(right_positions) >= len(self._right_hand_buf):
                            self._left_hand_buf.copy_(torch.tensor(left_positions[:len(self._left_hand_buf)], dtype=torch.float32, device=self.env.device))
                            self._right_hand_buf.copy_(torch.tensor(right_positions[:len(self._right_hand_buf)], dtype=torch.float32, device=self.env.device))
                            l_vals = self._left_hand_buf.index_select(0, self._left_hand_source_idx_t)
                            r_vals = self._right_hand_buf.index_select(0, self._right_hand_source_idx_t)
                            full_action.index_copy_(0, self._left_hand_target_idx_t, l_vals)
                            full_action.index_copy_(0, self._right_hand_target_idx_t, r_vals)
            elif self.inspire_dds and hasattr(self, "_inspire_source_idx_t"):
                inspire_cmds = self.inspire_dds.get_inspire_hand_command()
                if inspire_cmds and 'positions' in inspire_cmds:
                        inspire_cmds_positions = inspire_cmds['positions']
                        if len(inspire_cmds_positions) >= 12:
                            self._inspire_buf.copy_(torch.tensor(inspire_cmds_positions[:12], dtype=torch.float32, device=self.env.device))
                            base_vals = self._inspire_buf.index_select(0, self._inspire_source_idx_t)
                            full_action.index_copy_(0, self._inspire_target_idx_t, base_vals)
                            special_vals = self._inspire_buf.index_select(0, self._inspire_special_source_idx_t) * self._inspire_special_scales_t
                            full_action.index_copy_(0, self._inspire_special_target_idx_t, special_vals)
            # 同步仿真多步
            for _ in range(4):
                self.env.scene["robot"].set_joint_position_target(full_action) 
                self.env.scene.write_data_to_sim()                           
                self.env.sim.step(render=False)                              
                self.env.scene.update(dt=self.env.physics_dt)                    

            self.env.sim.render()
            self.env.observation_manager.compute()
            
        except Exception as e:
            print(f"[{self.name}] Get DDS action failed: {e}")
            return None
    
    def _convert_to_joint_range(self, value):
        """Convert gripper control value to joint angle"""
        input_min, input_max = 0.0, 5.6
        output_min, output_max = 0.03, -0.02
        value = max(input_min, min(input_max, value))
        return output_min + (output_max - output_min) * (value - input_min) / (input_max - input_min)
    
    def cleanup(self):
        """Clean up DDS resources"""
        try:
            if self.robot_dds:
                self.robot_dds.stop_communication()
            if self.gripper_dds:
                self.gripper_dds.stop_communication()
            if self.dex3_dds:
                self.dex3_dds.stop_communication()
            if self.inspire_dds:
                self.inspire_dds.stop_communication()
        except Exception as e:
            print(f"[{self.name}] Clean up DDS resources failed: {e}")
