#!/usr/bin/env python3
"""
使用 pynput 库实现键盘控制，并集成夹爪控制和场景重置
"""

import time
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

import threading
from pynput import keyboard


class G1JointIndex:
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28


ARM_JOINTS = (
    G1JointIndex.LeftShoulderPitch,
    G1JointIndex.LeftShoulderRoll,
    G1JointIndex.LeftShoulderYaw,
    G1JointIndex.LeftElbow,
    G1JointIndex.LeftWristRoll,
    G1JointIndex.LeftWristPitch,
    G1JointIndex.LeftWristYaw,
    G1JointIndex.RightShoulderPitch,
    G1JointIndex.RightShoulderRoll,
    G1JointIndex.RightShoulderYaw,
    G1JointIndex.RightElbow,
    G1JointIndex.RightWristRoll,
    G1JointIndex.RightWristPitch,
    G1JointIndex.RightWristYaw,
)


ARM_NEUTRAL_TARGETS = {
    G1JointIndex.LeftShoulderPitch: 0.0,
    G1JointIndex.LeftShoulderRoll: 0.45,
    G1JointIndex.LeftShoulderYaw: 0.0,
    G1JointIndex.LeftElbow: 0.9,
    G1JointIndex.LeftWristRoll: 0.0,
    G1JointIndex.LeftWristPitch: 0.0,
    G1JointIndex.LeftWristYaw: 0.0,
    G1JointIndex.RightShoulderPitch: 0.0,
    G1JointIndex.RightShoulderRoll: -0.45,
    G1JointIndex.RightShoulderYaw: 0.0,
    G1JointIndex.RightElbow: 0.9,
    G1JointIndex.RightWristRoll: 0.0,
    G1JointIndex.RightWristPitch: 0.0,
    G1JointIndex.RightWristYaw: 0.0,
}


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def build_arm_targets(forward, lift):
    """Approximate both-hand Cartesian intent with bounded shoulder/elbow targets."""
    forward = clamp(float(forward), -0.55, 0.55)
    lift = clamp(float(lift), -0.35, 0.75)

    targets = ARM_NEUTRAL_TARGETS.copy()
    targets[G1JointIndex.LeftShoulderPitch] = clamp(forward, -0.8, 0.8)
    targets[G1JointIndex.RightShoulderPitch] = clamp(forward, -0.8, 0.8)
    targets[G1JointIndex.LeftShoulderRoll] = clamp(0.45 + lift, 0.1, 1.3)
    targets[G1JointIndex.RightShoulderRoll] = clamp(-0.45 - lift, -1.3, -0.1)
    targets[G1JointIndex.LeftElbow] = clamp(0.9 + 0.35 * lift - 0.2 * forward, 0.2, 1.8)
    targets[G1JointIndex.RightElbow] = clamp(0.9 + 0.35 * lift - 0.2 * forward, 0.2, 1.8)
    targets[G1JointIndex.LeftWristPitch] = clamp(-0.25 * lift, -0.5, 0.5)
    targets[G1JointIndex.RightWristPitch] = clamp(-0.25 * lift, -0.5, 0.5)
    return targets


class LowPassFilter:
    def __init__(self, alpha=0.15):
        self.alpha = alpha
        self._value = 0.0
        self._last_value = 0.0

    def update(self, new_value, max_accel=1.5):
        delta = new_value - self._last_value
        delta = clamp(delta, -max_accel, max_accel)
        filtered = self.alpha * (self._last_value + delta) + (1 - self.alpha) * self._value
        self._last_value = filtered
        self._value = filtered
        return self._value


class KeyboardController:
    def __init__(self, reset_publisher=None):
        self.reset_publisher = reset_publisher
        self.control_params = {
            'x_vel': 0.0,
            'y_vel': 0.0,
            'yaw_vel': 0.0,
            'height': 0.0,
            'left_gripper': 5.4,
            'right_gripper': 5.4,
            'arm_forward': 0.0,
            'arm_lift': 0.0,
            'arm_active': False
        }
        
        # Key increment step size   
        self.increment = 0.05
        
        # control range
        self.ranges = {
            'x_vel': (-0.6, 1.0),    # forward velocity
            'y_vel': (-0.5, 0.5),   # lateral velocity
            'yaw_vel': (-1.57, 1.57), # yaw velocity
            'height': (-0.5, 0.0),   # height
            'left_gripper': (0.0, 5.4),
            'right_gripper': (0.0, 5.4),
            'arm_forward': (-0.55, 0.55),
            'arm_lift': (-0.35, 0.75)
        }
        
        # key state
        self.key_states = {
            'w': False,  # forward
            's': False,  # backward
            'a': False,  # left
            'd': False,  # right
            'z': False,  # left rotation
            'x': False,  # right rotation
            'c': False,  # crouch
            'u': False,  # left gripper open
            'i': False,  # left gripper close
            'o': False,  # right gripper open
            'p': False,  # right gripper close
            'f': False,  # hands forward
            'b': False,  # hands backward
            'y': False,  # hands up
            'h': False,  # hands down
            'n': False,  # neutral arm pose
        }
        
        self.param_lock = threading.Lock()
        self.running = True

        self._filters = {
            'x_vel': LowPassFilter(alpha=0.3),
            'y_vel': LowPassFilter(alpha=0.3),
            'yaw_vel': LowPassFilter(alpha=0.3),
            'height': LowPassFilter(alpha=0.3)
        }

        self._default_values = {
            'x_vel': 0.0,
            'y_vel': 0.0,
            'yaw_vel': 0.0,
            'height': 0.0
        }

        # Start threads
        self._control_thread = threading.Thread(target=self._control_update)
        self._control_thread.daemon = True
        self._control_thread.start()

        # Start keyboard listener
        self._start_keyboard_listener()

    def _start_keyboard_listener(self):
        """start keyboard listener"""
        def on_press(key):
            """key press event"""
            try:
                key_char = key.char.lower() if hasattr(key, 'char') and key.char else None
                
                with self.param_lock:
                    if key_char in self.key_states:
                        if not self.key_states[key_char]:
                            self.key_states[key_char] = True
                            print(f"[KEY] {key_char.upper()}: press")
                    elif key_char == 'r':
                        print("[KEY] R: publish reset category 1 (Room Randomization)")
                        if self.reset_publisher:
                            publish_reset_category("1", self.reset_publisher)
                    elif key_char == 't':
                        print("[KEY] T: publish reset category 2 (Full Scene Reset & Randomize)")
                        if self.reset_publisher:
                            publish_reset_category("2", self.reset_publisher)
                    elif key_char == 'q':
                        print("exit program...")
                        self.running = False
                        return False  # stop listening
                        
            except AttributeError:
                # handle special keys
                pass

        def on_release(key):
            """按键释放事件"""
            try:
                key_char = key.char.lower() if hasattr(key, 'char') and key.char else None
                
                with self.param_lock:
                    if key_char in self.key_states:
                        if self.key_states[key_char]:
                            self.key_states[key_char] = False
                            print(f"[KEY] {key_char.upper()}: release")
                            
            except AttributeError:
                # handle special keys
                pass

        # start keyboard listener
        self.listener = keyboard.Listener(
            on_press=on_press,
            on_release=on_release
        )
        self.listener.start()
        
        print("keyboard listener started...")
        print("W/A/S/D/Z/X/C to move the robot base/crouch")
        print("U/I to Open/Close Left Gripper")
        print("O/P to Open/Close Right Gripper")
        print("F/B to move both hands forward/backward")
        print("Y/H to move both hands up/down")
        print("N to move both arms back to neutral")
        print("R to randomize room layout, T for full scene reset")
        print("press Q key to exit program")

    def _control_update(self):
        """control parameter update thread"""
        while self.running:
            with self.param_lock:
                # update control parameters according to key states
                
                # forward/backward (x_vel)
                if self.key_states['w']:  # forward
                    self.control_params['x_vel'] = min(
                        self.control_params['x_vel'] + self.increment,
                        self.ranges['x_vel'][1]
                    )
                elif self.key_states['s']:  # backward
                    self.control_params['x_vel'] = max(
                        self.control_params['x_vel'] - self.increment,
                        self.ranges['x_vel'][0]
                    )
                else:
                    # release key, gradually return to default value
                    if self.control_params['x_vel'] > 0:
                        self.control_params['x_vel'] = max(0, self.control_params['x_vel'] - self.increment * 2)
                    elif self.control_params['x_vel'] < 0:
                        self.control_params['x_vel'] = min(0, self.control_params['x_vel'] + self.increment * 2)

                # left/right (y_vel)
                if self.key_states['a']:  # left
                    self.control_params['y_vel'] = max(
                        self.control_params['y_vel'] - self.increment,
                        self.ranges['y_vel'][0]
                    )
                elif self.key_states['d']:  # right
                    self.control_params['y_vel'] = min(
                        self.control_params['y_vel'] + self.increment,
                        self.ranges['y_vel'][1]
                    )
                else:
                    # release key, gradually return to default value
                    if self.control_params['y_vel'] > 0:
                        self.control_params['y_vel'] = max(0, self.control_params['y_vel'] - self.increment * 2)
                    elif self.control_params['y_vel'] < 0:
                        self.control_params['y_vel'] = min(0, self.control_params['y_vel'] + self.increment * 2)

                # left/right rotation (yaw_vel)
                if self.key_states['z']:  # left
                    self.control_params['yaw_vel'] = max(
                        self.control_params['yaw_vel'] - self.increment,
                        self.ranges['yaw_vel'][0]
                    )
                elif self.key_states['x']:  # right
                    self.control_params['yaw_vel'] = min(
                        self.control_params['yaw_vel'] + self.increment,
                        self.ranges['yaw_vel'][1]
                    )
                else:
                    # release key, gradually return to default value
                    if self.control_params['yaw_vel'] > 0:
                        self.control_params['yaw_vel'] = max(0, self.control_params['yaw_vel'] - self.increment * 2)
                    elif self.control_params['yaw_vel'] < 0:
                        self.control_params['yaw_vel'] = min(0, self.control_params['yaw_vel'] + self.increment * 2)

                # crouch (height)
                if self.key_states['c']:  # crouch
                    self.control_params['height'] = max(
                        self.control_params['height'] - self.increment,
                        self.ranges['height'][0]
                    )
                else:
                    # release key, gradually return to default value
                    if self.control_params['height'] < 0:
                        self.control_params['height'] = min(0, self.control_params['height'] + self.increment * 2)

                # left gripper control
                if self.key_states['u']:  # open
                    self.control_params['left_gripper'] = 5.4
                elif self.key_states['i']:  # close
                    self.control_params['left_gripper'] = 0.0

                # right gripper control
                if self.key_states['o']:  # open
                    self.control_params['right_gripper'] = 5.4
                elif self.key_states['p']:  # close
                    self.control_params['right_gripper'] = 0.0

                # approximate hand front/back control through arm joints
                if self.key_states['f']:
                    self.control_params['arm_active'] = True
                    self.control_params['arm_forward'] = min(
                        self.control_params['arm_forward'] + self.increment,
                        self.ranges['arm_forward'][1]
                    )
                elif self.key_states['b']:
                    self.control_params['arm_active'] = True
                    self.control_params['arm_forward'] = max(
                        self.control_params['arm_forward'] - self.increment,
                        self.ranges['arm_forward'][0]
                    )

                # approximate hand up/down control through arm joints
                if self.key_states['y']:
                    self.control_params['arm_active'] = True
                    self.control_params['arm_lift'] = min(
                        self.control_params['arm_lift'] + self.increment,
                        self.ranges['arm_lift'][1]
                    )
                elif self.key_states['h']:
                    self.control_params['arm_active'] = True
                    self.control_params['arm_lift'] = max(
                        self.control_params['arm_lift'] - self.increment,
                        self.ranges['arm_lift'][0]
                    )

                if self.key_states['n']:
                    self.control_params['arm_active'] = True
                    self.control_params['arm_forward'] = 0.0
                    self.control_params['arm_lift'] = 0.0

                # round to avoid floating point precision issues
                for key in self.control_params:
                    if isinstance(self.control_params[key], float):
                        self.control_params[key] = round(self.control_params[key], 3)

            time.sleep(0.02)  # 50Hz update frequency

    # === external interface ===

    def get_control_params(self):
        with self.param_lock:
            return self.control_params.copy()

    def get_key_states(self):
        with self.param_lock:
            return self.key_states.copy()
    
    def stop(self):
        """stop keyboard controller"""
        self.running = False
        if hasattr(self, 'listener'):
            self.listener.stop()


def publish_reset_category(category, publisher):
    # construct message
    msg = String_(data=str(category))  # pass data parameter directly during initialization
    publisher.Write(msg)


def init_publisher(topic, message_type):
    print(f"  creating DDS publisher: {topic} ({message_type.__name__})")
    try:
        publisher = ChannelPublisher(topic, message_type)
        publisher.Init()
    except Exception as exc:
        print(f"  failed DDS publisher: {topic} ({message_type.__name__})")
        if "PRECONDITION_NOT_MET" in str(exc):
            print("  hint: another DDS process may already use this topic with a different message type.")
        raise
    return publisher


def publish_gripper_cmd(publisher, q_val):
    msg = MotorCmds_()
    cmd = unitree_go_msg_dds__MotorCmd_()
    cmd.q = float(q_val)
    cmd.dq = 0.0
    cmd.tau = 0.0
    cmd.kp = 80.0
    cmd.kd = 2.0
    msg.cmds.append(cmd)
    publisher.Write(msg)


def publish_arm_cmd(publisher, crc, targets):
    msg = unitree_hg_msg_dds__LowCmd_()
    msg.mode_pr = 0
    msg.mode_machine = 0

    for joint in ARM_JOINTS:
        cmd = msg.motor_cmd[joint]
        cmd.mode = 1
        cmd.q = float(targets[joint])
        cmd.dq = 0.0
        cmd.tau = 0.0
        cmd.kp = 80.0
        cmd.kd = 2.0

    msg.crc = crc.Crc(msg)
    publisher.Write(msg)


if __name__ == "__main__":
    print("=" * 50)
    print("keyboard control instructions (pynput version):")
    print("W: forward    S: backward")
    print("A: left       D: right") 
    print("Z: left rot   X: right rot")
    print("C: crouch     Q: exit program")
    print("U/I: Open/Close Left Gripper")
    print("O/P: Open/Close Right Gripper")
    print("F/B: Move both hands forward/backward")
    print("Y/H: Move both hands up/down")
    print("N: Return both arms to neutral")
    print("R: Trigger room randomization (Category 1)")
    print("T: Trigger full scene reset (Category 2)")
    print("press and hold move keys to increase, release to return to default")
    print("=" * 50)
    
    try:
        # check if pynput library is available
        try:
            from pynput import keyboard
        except ImportError:
            print("error: pynput library missing")
            print("please install: pip install pynput")
            exit(1)
            
        # initialize DDS
        print("initializing DDS communication...")
        ChannelFactoryInitialize(1)
        
        # Publishers
        cmd_publisher = init_publisher("rt/run_command/cmd", String_)
        reset_publisher = init_publisher("rt/reset_pose/cmd", String_)
        left_gripper_publisher = init_publisher("rt/dex1/left/cmd", MotorCmds_)
        right_gripper_publisher = init_publisher("rt/dex1/right/cmd", MotorCmds_)
        arm_publisher = init_publisher("rt/lowcmd", LowCmd_)
        arm_crc = CRC()
        
        print("DDS communication initialized")
        
        print("initializing keyboard controller...")
        keyboard_controller = KeyboardController(reset_publisher=reset_publisher)
        default_height = 0.8
        
        print("=" * 50)
        print("program started, waiting for keyboard input...")
        print("press Ctrl+C to exit program")
        print("=" * 50)
        
        # add counters, only show when command changes
        counter = 0
        last_commands = [0.0, 0.0, 0.0, 0.8]
        last_left_gripper = 5.4
        last_right_gripper = 5.4
        last_arm_forward = 0.0
        last_arm_lift = 0.0
        
        while keyboard_controller.running:
            time.sleep(0.01)
            commands = keyboard_controller.get_control_params()
            commands['height'] = default_height + commands['height']
            
            # convert to list format string [x_vel, y_vel, yaw_vel, height]
            commands_list = [float(commands['x_vel']), -float(commands['y_vel']), -float(commands['yaw_vel']), float(commands['height'])]
            commands_str = str(commands_list)
            
            # only show when the command changes
            counter += 1
            if commands_list != last_commands:
                print(f"commands: {commands_str}")
                last_commands = commands_list.copy()
                
            publish_reset_category(commands_str, cmd_publisher)
            
            # Grippers
            left_q = float(commands['left_gripper'])
            right_q = float(commands['right_gripper'])
            
            if left_q != last_left_gripper:
                print(f"left gripper command: {left_q}")
                last_left_gripper = left_q
            if right_q != last_right_gripper:
                print(f"right gripper command: {right_q}")
                last_right_gripper = right_q
                
            publish_gripper_cmd(left_gripper_publisher, left_q)
            publish_gripper_cmd(right_gripper_publisher, right_q)

            if commands['arm_active']:
                arm_forward = float(commands['arm_forward'])
                arm_lift = float(commands['arm_lift'])
                arm_targets = build_arm_targets(arm_forward, arm_lift)

                if arm_forward != last_arm_forward or arm_lift != last_arm_lift:
                    print(f"hand command: forward={arm_forward}, lift={arm_lift}")
                    last_arm_forward = arm_forward
                    last_arm_lift = arm_lift

                publish_arm_cmd(arm_publisher, arm_crc, arm_targets)
            
    except KeyboardInterrupt:
        print("\nprogram interrupted by user (Ctrl+C)")
        if 'keyboard_controller' in locals():
            keyboard_controller.stop()
    except Exception as e:
        print(f"\nprogram error: {e}")
        if 'keyboard_controller' in locals():
            keyboard_controller.stop()
    
    print("program ended")
