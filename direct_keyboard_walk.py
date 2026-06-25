#!/usr/bin/env python3
import sys
import os
import time
import math
import numpy as np
from types import ModuleType

# -------------------------------------------------------------------------
# 1. ROS Mocking Layer to prevent crashes from robodog_control imports
# -------------------------------------------------------------------------
class DummyRospy(ModuleType):
    def loginfo(self, *args, **kwargs):
        print("[INFO]", *args)
    def logwarn(self, *args, **kwargs):
        print("[WARN]", *args)
    def logerr(self, *args, **kwargs):
        print("[ERROR]", *args)
    def logfatal(self, *args, **kwargs):
        print("[FATAL]", *args)
    
    class Publisher:
        def __init__(self, *args, **kwargs): pass
        def publish(self, *args, **kwargs): pass
        
    class Subscriber:
        def __init__(self, *args, **kwargs): pass
        
    class Time:
        @staticmethod
        def now():
            return time.time()
            
    @staticmethod
    def init_node(*args, **kwargs): pass
    @staticmethod
    def is_shutdown(): return False

sys.modules['rospy'] = DummyRospy('rospy')

class DummyMsg:
    def __init__(self, *args, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)

sys.modules['geometry_msgs'] = DummyRospy('geometry_msgs')
sys.modules['geometry_msgs.msg'] = DummyRospy('geometry_msgs.msg')
sys.modules['geometry_msgs.msg'].Point = DummyMsg

sys.modules['std_msgs'] = DummyRospy('std_msgs')
sys.modules['std_msgs.msg'] = DummyRospy('std_msgs.msg')
sys.modules['std_msgs.msg'].Header = DummyMsg
sys.modules['std_msgs.msg'].Bool = DummyMsg
sys.modules['std_msgs.msg'].Float64 = DummyMsg

sys.modules['robodog_control.msg'] = DummyRospy('robodog_control.msg')
sys.modules['robodog_control.msg'].TaskSpace = DummyMsg
sys.modules['robodog_control.msg'].JointSpace = DummyMsg
sys.modules['robodog_control.msg'].Angle = DummyMsg

# -------------------------------------------------------------------------
# 2. Add package paths to Python path
# -------------------------------------------------------------------------
base_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(base_dir, "robodog_ws", "src", "robodog_control", "src"))
sys.path.append(os.path.join(base_dir, "robodog_ws", "src", "robodog_utilities", "src"))
sys.path.append(os.path.join(base_dir, "robodog_ws", "src", "robodog_hardware_interfacing", "robodog_servo_interfacing", "src"))
sys.path.append(os.path.join(base_dir, "robodog_ws", "src", "robodog_hardware_interfacing", "robodog_peripheral_interfacing", "src"))
sys.path.append(os.path.join(base_dir, "robodog_ws", "src", "robodog_hardware_interfacing", "robodog_input_interfacing", "src"))

# Now import the necessary modules from robodog_control
from robodog_control.Config import Configuration, Leg_linkage
from robodog_control.Kinematics import four_legs_inverse_kinematics
from robodog_control.Gaits import GaitController
from robodog_control.StanceController import StanceController
from robodog_control.SwingLegController import SwingController
from robodog_control.State import BehaviorState, State
from robodog_control.Command import Command
from robodog_utilities.Utilities import clipped_first_order_filter
from transforms3d.euler import euler2mat

# -------------------------------------------------------------------------
# 3. Import hardware communication libraries
# -------------------------------------------------------------------------
try:
    from adafruit_servokit import ServoKit
    servos_available = True
except ImportError:
    print("Warning: adafruit-circuitpython-servokit not installed.")
    servos_available = False

try:
    import board
    import adafruit_bno055
    imu_available = True
except ImportError:
    print("Warning: board or adafruit-bno055 not installed.")
    imu_available = False

import select
import termios
import tty
import threading

pynput_available = False
try:
    from pynput import keyboard
    pynput_available = True
except Exception:
    # If pynput fails to import (e.g. no X server display), define fallback listener
    class MockKeyboard:
        class Key:
            space = 'space'
            esc = 'esc'

        class Listener:
            def __init__(self, on_press, on_release):
                self.on_press = on_press
                self.on_release = on_release
                self.running = False
                self.thread = None
                self.old_settings = None

            def start(self):
                self.running = True
                try:
                    self.old_settings = termios.tcgetattr(sys.stdin)
                except Exception:
                    self.old_settings = None
                self.thread = threading.Thread(target=self._run)
                self.thread.daemon = True
                self.thread.start()

            def stop(self):
                self.running = False
                if self.old_settings:
                    try:
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
                    except Exception:
                        pass

            def _run(self):
                try:
                    tty.setcbreak(sys.stdin.fileno())
                except Exception:
                    # In case stdin is not a tty
                    pass

                class DummyKey:
                    def __init__(self, char=None, special=None):
                        self.char = char
                        self.special = special
                    def __eq__(self, other):
                        if self.special and other:
                            return getattr(other, 'special', None) == self.special
                        return False

                # We keep track of the last pressed movement key to simulate release
                last_key = None
                last_key_time = 0.0

                while self.running:
                    try:
                        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
                        now = time.time()
                        if rlist:
                            char = sys.stdin.read(1)
                            if char == '\x1b': # ESC
                                self.on_press(MockKeyboard.Key.esc)
                            elif char == ' ':
                                self.on_press(MockKeyboard.Key.space)
                            elif char:
                                char_lower = char.lower()
                                if last_key and last_key != char_lower:
                                    self.on_release(DummyKey(char=last_key))
                                self.on_press(DummyKey(char=char_lower))
                                last_key = char_lower
                                last_key_time = now
                        else:
                            if last_key and (now - last_key_time > 0.3):
                                self.on_release(DummyKey(char=last_key))
                                last_key = None
                    except Exception:
                        time.sleep(0.05)

                if self.old_settings:
                    try:
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
                    except Exception:
                        pass

    keyboard = MockKeyboard()

import json

# -------------------------------------------------------------------------
# 4. Helper Linkage Functions (replicating HardwareInterface.py logic)
# -------------------------------------------------------------------------
def calculate_4_bar(th2, a, b, c, d):
    try:
        x_b = a * math.cos(th2)
        y_b = a * math.sin(th2)
        f = math.sqrt((d - x_b)**2 + y_b**2)
        beta = math.acos((f**2 + c**2 - b**2) / (2 * f * c))
        gamma = math.atan2(y_b, d - x_b)
        th4 = math.pi - gamma - beta
        x_c = c * math.cos(th4) + d
        y_c = c * math.sin(th4)
        th3 = math.atan2((y_c - y_b), (x_c - x_b))
        ABC = math.pi - th2 + th3
        BCD = th4 - th3
        CDA = math.pi * 2.0 - th2 - ABC - BCD
        return ABC, BCD, CDA
    except Exception:
        return 0.0, 0.0, 0.0

def lower_leg_angle_to_servo_angle(link, THETA2, THETA3):
    GDE, DEF, EFG = calculate_4_bar(THETA3 + link.lower_leg_bend_angle, link.i, link.h, link.f, link.g)
    CDH = 1.5 * math.pi - THETA2 - GDE - link.EDC
    CDA = CDH + link.gamma
    DAB, ABC, BCD = calculate_4_bar(CDA, link.d, link.a, link.b, link.c)
    THETA0 = DAB + link.gamma
    return THETA0

# -------------------------------------------------------------------------
# 5. Standalone Direct Keyboard Walking System Controller
# -------------------------------------------------------------------------
class DirectKeyboardWalk:
    def __init__(self):
        self.config = Configuration()
        self.linkage = Leg_linkage(self.config)
        
        # Load Calibration settings from JSON
        self.offsets = [0.0] * 12
        self.inversions = [False] * 12
        self.load_calibration()
        
        # Initialize PCA9685 Servo Kit
        if servos_available:
            print("Initializing PCA9685 Servo Kit (16 channels)...")
            self.kit = ServoKit(channels=16)
            for i in range(16):
                self.kit.servo[i].actuation_range = 180
                self.kit.servo[i].set_pulse_width_range(370, 2400)
        else:
            self.kit = None
            print("Running in MOCK mode (No PCA9685 connection).")
            
        # Initialize BNO055 IMU
        self.imu = None
        if imu_available:
            try:
                print("Initializing BNO055 IMU on I2C...")
                i2c_bus = board.I2C()
                self.imu = adafruit_bno055.BNO055_I2C(i2c_bus)
            except Exception as e:
                print(f"IMU Initialization failed: {e}. Running without IMU feedback.")
        
        # Locomotion controllers
        self.gait_controller = GaitController(self.config)
        self.swing_controller = SwingController(self.config)
        self.stance_controller = StanceController(self.config)
        
        # Robot States
        self.state = State()
        self.state.behavior_state = BehaviorState.REST
        self.command = Command()
        
        # Walking settings
        self.step_length = 0.12 # stride
        self.height_level = 0
        self.target_height = -0.205
        
        # Keyboard Control Flags
        self.keys_pressed = set()
        self.running = True

    def load_calibration(self):
        cal_path = os.path.join(base_dir, "calibration_tool", "robodog_calibration_offsets.json")
        if os.path.exists(cal_path):
            try:
                with open(cal_path, "r") as f:
                    data = json.load(f)
                if "offsets" in data:
                    self.offsets = data["offsets"]
                if "inversions" in data:
                    self.inversions = data["inversions"]
                print(f"Loaded calibration offsets successfully from: {cal_path}")
            except Exception as e:
                print(f"Failed to parse calibration offsets file: {e}")
        else:
            print("No calibration offsets file found. Using default zeros.")

    def read_imu_orientation(self):
        if not self.imu:
            return np.array([0.0, 0.0, 0.0])
        try:
            euler = self.imu.euler
            if euler and all(x is not None for x in euler):
                yaw = math.radians(360.0 - euler[0])
                roll = math.radians(-euler[1])
                pitch = math.radians(euler[2])  # Apply standard 30 deg mounting correction
                return np.array([yaw, pitch, roll])
        except Exception:
            pass
        return np.array([0.0, 0.0, 0.0])

    def update_command_from_keys(self):
        # Default velocity & steering
        vx = 0.0
        vy = 0.0
        yaw_rate = 0.0
        
        # Key bindings
        if 'w' in self.keys_pressed:
            vx = self.step_length
        elif 's' in self.keys_pressed:
            vx = -self.step_length
            
        if 'a' in self.keys_pressed:
            vy = 0.04 # strafe left
        elif 'd' in self.keys_pressed:
            vy = -0.04 # strafe right
            
        if 'q' in self.keys_pressed:
            yaw_rate = 0.5 # turn left
        elif 'e' in self.keys_pressed:
            yaw_rate = -0.5 # turn right
            
        self.command.horizontal_velocity = np.array([vx, vy])
        self.command.yaw_rate = yaw_rate

    def on_press(self, key):
        try:
            if hasattr(key, 'char') and key.char:
                char = key.char.lower()
                self.keys_pressed.add(char)
                
                # Height settings
                if char == 'u': # Raise
                    self.height_level = min(3, self.height_level + 1)
                    self.target_height = -0.205 + self.height_level * 0.01
                    print(f"Height raised to: {self.target_height*100:.1f} cm")
                elif char == 'j': # Lower
                    self.height_level = max(-3, self.height_level - 1)
                    self.target_height = -0.205 + self.height_level * 0.01
                    print(f"Height lowered to: {self.target_height*100:.1f} cm")
                    
            elif key == keyboard.Key.space:
                # Toggle Gait state
                if self.state.behavior_state == BehaviorState.REST:
                    self.state.behavior_state = BehaviorState.TROT
                    print("Walking active (TROT mode).")
                else:
                    self.state.behavior_state = BehaviorState.REST
                    print("Standing still (REST mode).")
                    
            elif key == keyboard.Key.esc:
                self.running = False
                print("Exiting...")
        except Exception as e:
            print(f"Error handling key press: {e}")

    def on_release(self, key):
        try:
            if hasattr(key, 'char') and key.char:
                char = key.char.lower()
                if char in self.keys_pressed:
                    self.keys_pressed.remove(char)
        except Exception:
            pass

    def step_gait(self):
        contact_modes = self.gait_controller.contacts(self.state.ticks)
        new_foot_locations = np.zeros((3, 4))
        for leg_index in range(4):
            contact_mode = contact_modes[leg_index]
            if contact_mode == 1:
                new_location = self.stance_controller.next_foot_location(leg_index, self.state, self.command)
            else:
                swing_proportion = (
                    self.gait_controller.subphase_ticks(self.state.ticks) / self.config.swing_ticks
                )
                new_location = self.swing_controller.next_foot_location(
                    swing_proportion,
                    leg_index,
                    self.state,
                    self.command
                )
            new_foot_locations[:, leg_index] = new_location
        return new_foot_locations

    def control_loop(self):
        rate_hz = 50.0
        dt = 1.0 / rate_hz
        
        print("--------------------------------------------------")
        print("RoboDog Standalone Keyboard Walking Controller Active")
        print("Controls:")
        print("  [W] Walk Forward      [S] Walk Backward")
        print("  [A] Strafe Left       [D] Strafe Right")
        print("  [Q] Turn Left         [E] Turn Right")
        print("  [U] Raise Body        [J] Lower Body")
        print("  [Space] Toggle Walk/Stand")
        print("  [ESC] Exit Control Program")
        print("--------------------------------------------------")

        # Start keyboard listener
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

        # Set default stance height
        self.command.height = self.target_height

        while self.running:
            start_time = time.time()
            
            # Read IMU
            self.state.euler_orientation = self.read_imu_orientation()
            
            # Update inputs
            self.update_command_from_keys()
            self.command.height = self.target_height
            
            # Run Stance or Walk Controllers
            if self.state.behavior_state == BehaviorState.TROT:
                self.state.foot_locations = self.step_gait()
                rotated_foot_locations = euler2mat(self.command.roll, self.command.pitch, 0.0) @ self.state.foot_locations
                
                # Compensate using IMU orientation
                yaw, pitch, roll = self.state.euler_orientation
                correction_factor = 0.8
                max_tilt = 0.4
                roll_compensation = correction_factor * np.clip(roll, -max_tilt, max_tilt)
                pitch_compensation = correction_factor * np.clip(pitch, -max_tilt, max_tilt)
                rmat = euler2mat(roll_compensation, pitch_compensation, 0)
                rotated_foot_locations = rmat.T @ rotated_foot_locations
                
                self.state.joint_angles = four_legs_inverse_kinematics(rotated_foot_locations, self.config)
                self.state.ticks += 1
            else:
                # Rest state
                self.state.foot_locations = self.config.default_stance + np.array([0, 0, self.command.height])[:, np.newaxis]
                rotated_foot_locations = euler2mat(self.command.roll, self.command.pitch, 0.0) @ self.state.foot_locations
                
                # Stabilise with IMU orientation
                yaw, pitch, roll = self.state.euler_orientation
                correction_factor = 0.5
                max_tilt = 0.4
                roll_compensation = correction_factor * np.clip(-roll, -max_tilt, max_tilt)
                pitch_compensation = correction_factor * np.clip(-pitch, -max_tilt, max_tilt)
                rmat = euler2mat(roll_compensation, pitch_compensation, 0)
                rotated_foot_locations = rmat.T @ rotated_foot_locations
                
                self.state.joint_angles = four_legs_inverse_kinematics(rotated_foot_locations, self.config)
                self.state.ticks += 1

            # Translate IK angles to PCA9685 raw angles using loaded JSON offsets
            raw_angles = [90.0] * 12
            for leg in range(4):
                # 1. Hip
                hip_ang_deg = math.degrees(self.state.joint_angles[0, leg]) + 90.0
                offset_h = self.offsets[leg * 3]
                inv_h = self.inversions[leg * 3]
                r_h = (180.0 - (hip_ang_deg + offset_h)) if inv_h else (hip_ang_deg + offset_h)
                raw_angles[leg * 3] = max(0.0, min(180.0, r_h))
                
                # 2. Thigh (Upper Leg)
                THETA2 = self.state.joint_angles[1, leg]
                up_ang_deg = math.degrees(THETA2)
                offset_u = self.offsets[leg * 3 + 1]
                inv_u = self.inversions[leg * 3 + 1]
                r_u = (180.0 - (up_ang_deg + offset_u)) if inv_u else (up_ang_deg + offset_u)
                raw_angles[leg * 3 + 1] = max(0.0, min(180.0, r_u))
                
                # 3. Calf (Lower Leg)
                THETA3 = self.state.joint_angles[2, leg]
                THETA0 = lower_leg_angle_to_servo_angle(self.linkage, math.pi/2.0 - THETA2, THETA3 + math.pi/2.0)
                compensated_l = math.degrees(math.pi/2.0 + math.pi - THETA0)
                
                offset_l = self.offsets[leg * 3 + 2]
                inv_l = self.inversions[leg * 3 + 2]
                r_l = (180.0 - (compensated_l + offset_l)) if inv_l else (compensated_l + offset_l)
                raw_angles[leg * 3 + 2] = max(0.0, min(180.0, r_l))

            # Push angles to PCA9685
            if self.kit:
                JOINT_TO_CHANNEL = [14, 13, 12, 10, 9, 8, 2, 1, 0, 6, 5, 4]
                for i in range(12):
                    ch = JOINT_TO_CHANNEL[i]
                    self.kit.servo[ch].angle = raw_angles[i]

            # Sleep to maintain 50 Hz loop
            elapsed = time.time() - start_time
            sleep_time = max(0.0, dt - elapsed)
            time.sleep(sleep_time)

        # Set all servos back to limp/neutral on exit
        if self.kit:
            print("Setting servos to neutral before exiting...")
            JOINT_TO_CHANNEL = [14, 13, 12, 10, 9, 8, 2, 1, 0, 6, 5, 4]
            for i in range(12):
                ch = JOINT_TO_CHANNEL[i]
                self.kit.servo[ch].angle = 90.0
        listener.stop()

if __name__ == "__main__":
    controller = DirectKeyboardWalk()
    controller.control_loop()
