# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
from action_provider.action_base import ActionProvider
from typing import Optional
import torch
from dds.dds_master import dds_manager
class DDSActionProvider(ActionProvider):
    """Action provider based on DDS"""
    
    def __init__(self,env, args_cli):
        super().__init__("DDSActionProvider")
        self.enable_robot = args_cli.robot_type
        self.enable_gripper = args_cli.enable_dex1_dds
        self.enable_dex3 = args_cli.enable_dex3_dds
        self.enable_inspire = args_cli.enable_inspire_dds
        self.env = env
        if self.env.num_envs != 1:
            raise ValueError(
                "DDS teleoperation requires num_envs == 1; "
                f"received {self.env.num_envs}"
            )
        # Initialize DDS communication
        self.robot_dds = None
        self.gripper_dds = None
        self.dex3_dds = None
        self.inspire_dds = None
        self._setup_dds()
        self._setup_joint_mapping()
    
    def _setup_dds(self):
        """Setup DDS communication"""
        print(f"enable_robot: {self.enable_robot}")
        print(f"enable_gripper: {self.enable_gripper}")
        print(f"enable_dex3: {self.enable_dex3}")
        try:
            if self.enable_robot == "g129" or self.enable_robot == "h1_2":
                self.robot_dds = dds_manager.get_object("g129")
            if self.enable_gripper:
                self.gripper_dds = dds_manager.get_object("dex1")
            elif self.enable_dex3:
                self.dex3_dds = dds_manager.get_object("dex3")
            elif self.enable_inspire:
                self.inspire_dds = dds_manager.get_object("inspire")
            print(f"[{self.name}] DDS communication initialized")
        except Exception as e:
            print(f"[{self.name}] DDS initialization failed: {e}")
    
    def _setup_joint_mapping(self):
        """Setup joint mapping"""
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
            self.all_joint_names = self.env.scene["robot"].data.joint_names
            self.joint_to_index = {name: i for i, name in enumerate(self.all_joint_names)}
            self.arm_action_pose = [self.joint_to_index[name] for name in self.arm_joint_mapping.keys()]
            self.arm_action_pose_indices = [self.arm_joint_mapping[name] for name in self.arm_joint_mapping.keys()]
            self._arm_target_indices = [self.joint_to_index[name] for name in self.arm_joint_mapping.keys()]
            self._arm_source_indices = [idx + 15 for idx in self.arm_joint_mapping.values()]  # source data from positions[15:]
            self._waist_yaw_target_index = self.joint_to_index["waist_yaw_joint"]
            self._waist_yaw_source_index = 12
            self._waist_pitch_target_index = self.joint_to_index["waist_pitch_joint"]
            self._waist_pitch_source_index = 14
        elif self.enable_robot == "h1_2":
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
            print(f"self.env.scene['robot'].data.joint_names: {self.env.scene['robot'].data.joint_names}")
            self.all_joint_names = self.env.scene["robot"].data.joint_names
            self.joint_to_index = {name: i for i, name in enumerate(self.all_joint_names)}
            self.arm_action_pose = [self.joint_to_index[name] for name in self.arm_joint_mapping.keys()]
            self.arm_action_pose_indices = [self.arm_joint_mapping[name] for name in self.arm_joint_mapping.keys()]
            self._arm_target_indices = [self.joint_to_index[name] for name in self.arm_joint_mapping.keys()]
            self._arm_source_indices = [idx + 13 for idx in self.arm_joint_mapping.values()]  # source data from positions[13:]
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

        
        # precompute indices (for vectorization)

        if self.enable_gripper:
            self._gripper_target_indices = [self.joint_to_index[name] for name in self.gripper_joint_mapping.keys()]
            self._gripper_source_indices = [idx for idx in self.gripper_joint_mapping.values()]
        if self.enable_dex3:
            self._left_hand_target_indices = [self.joint_to_index[name] for name in self.left_hand_joint_mapping.keys()]
            self._left_hand_source_indices = [idx for idx in self.left_hand_joint_mapping.values()]
            self._right_hand_target_indices = [self.joint_to_index[name] for name in self.right_hand_joint_mapping.keys()]
            self._right_hand_source_indices = [idx for idx in self.right_hand_joint_mapping.values()]
        if self.enable_inspire:
            self._inspire_target_indices = [self.joint_to_index[name] for name in self.inspire_hand_joint_mapping.keys()]
            self._inspire_source_indices = [idx for idx in self.inspire_hand_joint_mapping.values()]
            self._inspire_special_target_indices = [self.joint_to_index[name] for name in self.special_joint_mapping.keys()]
            self._inspire_special_source_indices = [spec[0] for spec in self.special_joint_mapping.values()]
            self._inspire_special_scales = torch.tensor([spec[1] for spec in self.special_joint_mapping.values()], dtype=torch.float32)
        
        device = self.env.device
        self._arm_target_idx_t = torch.tensor(self._arm_target_indices, dtype=torch.long, device=device)
        self._arm_source_idx_t = torch.tensor(self._arm_source_indices, dtype=torch.long, device=device)
        if self.enable_gripper:
            self._gripper_target_idx_t = torch.tensor(self._gripper_target_indices, dtype=torch.long, device=device)
            self._gripper_source_idx_t = torch.tensor(self._gripper_source_indices, dtype=torch.long, device=device)
            # DDS carries absolute Dex1 joint targets. JointPositionActionCfg
            # adds the robot's default pose, so remove that pose here to keep
            # the commanded physical target exact (notably -0.02 -> 0.0245).
            self._gripper_default_offsets = self.env.scene["robot"].data.default_joint_pos[0].index_select(
                0, self._gripper_target_idx_t
            ).clone()
        if self.enable_dex3:
            self._left_hand_target_idx_t = torch.tensor(self._left_hand_target_indices, dtype=torch.long, device=device)
            self._left_hand_source_idx_t = torch.tensor(self._left_hand_source_indices, dtype=torch.long, device=device)
            self._right_hand_target_idx_t = torch.tensor(self._right_hand_target_indices, dtype=torch.long, device=device)
            self._right_hand_source_idx_t = torch.tensor(self._right_hand_source_indices, dtype=torch.long, device=device)
        if self.enable_inspire:
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
    
    def get_action(self, env) -> Optional[torch.Tensor]:
        """Get action from DDS"""
        try:

            full_action = self._full_action_buf
            full_action.zero_()
            if self.enable_robot == "g129" and self.robot_dds:
                cmd_data = self.robot_dds.get_robot_command()
                if cmd_data and 'motor_cmd' in cmd_data:
                    positions = cmd_data['motor_cmd']['positions']
                    if len(positions) >= 29:
                        self._positions_buf[:29].copy_(torch.tensor(positions[:29], dtype=torch.float32, device=self.env.device))
                        arm_vals = self._positions_buf.index_select(0, self._arm_source_idx_t)
                        full_action.index_copy_(0, self._arm_target_idx_t, arm_vals)
                        full_action[self._waist_yaw_target_index] = self._positions_buf[self._waist_yaw_source_index]
                        full_action[self._waist_pitch_target_index] = self._positions_buf[self._waist_pitch_source_index]
            elif self.enable_robot == "h1_2" and self.robot_dds:
                cmd_data = self.robot_dds.get_robot_command()
                if cmd_data and 'motor_cmd' in cmd_data:
                    positions = cmd_data['motor_cmd']['positions']
                    if len(positions) >= 29:
                        self._positions_buf[:29].copy_(torch.tensor(positions[:29], dtype=torch.float32, device=self.env.device))
                        arm_vals = self._positions_buf.index_select(0, self._arm_source_idx_t)
                        full_action.index_copy_(0, self._arm_target_idx_t, arm_vals)
            # Get gripper command
            if self.gripper_dds:
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
                        gp_vals = gp_vals - self._gripper_default_offsets
                        full_action.index_copy_(0, self._gripper_target_idx_t, gp_vals)
             
            elif self.dex3_dds:
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
            elif self.inspire_dds:
                inspire_cmds = self.inspire_dds.get_inspire_hand_command()
                if inspire_cmds and 'positions' in inspire_cmds:
                        inspire_cmds_positions = inspire_cmds['positions']
                        if len(inspire_cmds_positions) >= 12:
                            self._inspire_buf.copy_(torch.tensor(inspire_cmds_positions[:12], dtype=torch.float32, device=self.env.device))
                            base_vals = self._inspire_buf.index_select(0, self._inspire_source_idx_t)
                            full_action.index_copy_(0, self._inspire_target_idx_t, base_vals)
                            special_vals = self._inspire_buf.index_select(0, self._inspire_special_source_idx_t) * self._inspire_special_scales_t
                            full_action.index_copy_(0, self._inspire_special_target_idx_t, special_vals)
            return full_action.unsqueeze(0)
            
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
