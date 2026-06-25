#!/usr/bin/env python3
import sys
import os
import time
import math
import numpy as np
from types import ModuleType
import select
import termios
import tty
import threading
import json
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import queue

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

# Fallback terminal listener setup
pynput_available = False
try:
    from pynput import keyboard
    pynput_available = True
except Exception:
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
                    pass
                class DummyKey:
                    def __init__(self, char=None, special=None):
                        self.char = char
                        self.special = special
                    def __eq__(self, other):
                        if self.special and other:
                            return getattr(other, 'special', None) == self.special
                        return False
                last_key = None
                last_key_time = 0.0
                while self.running:
                    try:
                        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
                        now = time.time()
                        if rlist:
                            char = sys.stdin.read(1)
                            if char == '\x1b':
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

def calculate_knee_position(angles, config, origin, is_right):
    theta_1, theta_2, _ = angles
    x_local = -config.L2 * math.cos(theta_2)
    y_local = 0.0
    z_local = -config.L2 * math.sin(theta_2)
    
    R1 = math.pi/2 - config.phi
    angle_x = theta_1 + R1
    
    x_hip = x_local
    y_hip = config.L1 * math.cos(theta_1) + y_local * math.cos(angle_x) - z_local * math.sin(angle_x)
    z_hip = config.L1 * math.sin(theta_1) + y_local * math.sin(angle_x) + z_local * math.cos(angle_x)
    
    if is_right:
        y_hip = -y_hip
    return origin + np.array([x_hip, y_hip, z_hip])

# -------------------------------------------------------------------------
# 5. Web Telemetry Server
# -------------------------------------------------------------------------
# Custom Handler to serve SSE streams and HTML UI
class TelemetryServerHandler(BaseHTTPRequestHandler):
    controller = None
    clients = []

    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            html_path = os.path.join(base_dir, 'web', 'index.html')
            try:
                with open(html_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                self.wfile.write(content.encode('utf-8'))
            except Exception as e:
                self.wfile.write(f"Error loading UI: {e}".encode('utf-8'))
        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            
            q = queue.Queue()
            TelemetryServerHandler.clients.append(q)
            try:
                while TelemetryServerHandler.controller and TelemetryServerHandler.controller.running:
                    try:
                        # Non-blocking pop from queue to stream events down
                        data = q.get(timeout=1.0)
                        self.wfile.write(f"data: {json.dumps(data)}\n\n".encode('utf-8'))
                        self.wfile.write(b"\n")
                    except queue.Empty:
                        self.wfile.write(b": ping\n\n")
            except Exception:
                pass
            finally:
                if q in TelemetryServerHandler.clients:
                    TelemetryServerHandler.clients.remove(q)
        else:
            self.send_error(404, 'File not found')

    def do_POST(self):
        if self.path == '/api/control':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            try:
                data = json.loads(post_data.decode('utf-8'))
                if TelemetryServerHandler.controller:
                    if 'limp' in data:
                        TelemetryServerHandler.controller.limp = bool(data['limp'])
                    else:
                        TelemetryServerHandler.controller.limp = False
                    if 'manual_override' in data:
                        TelemetryServerHandler.controller.manual_override = bool(data['manual_override'])
                    if 'override_angles' in data:
                        TelemetryServerHandler.controller.override_angles = [float(x) for x in data['override_angles']]
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(b'{"status":"ok"}')
            except Exception as e:
                self.send_error(400, f"Bad request: {e}")
        elif self.path == '/api/calibration/calculate':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            try:
                data = json.loads(post_data.decode('utf-8'))
                height_cm = float(data.get('height', 20.0))
                leg_idx = int(data.get('leg_index', 0))
                
                if TelemetryServerHandler.controller:
                    TelemetryServerHandler.controller.calibration_active = True
                    TelemetryServerHandler.controller.calibration_leg_idx = leg_idx
                
                if TelemetryServerHandler.controller:
                    config = TelemetryServerHandler.controller.config
                    linkage = TelemetryServerHandler.controller.linkage
                    offsets = TelemetryServerHandler.controller.offsets
                    inversions = TelemetryServerHandler.controller.inversions
                else:
                    config = Configuration()
                    linkage = Leg_linkage(config)
                    offsets = [0.0] * 12
                    inversions = [False] * 12
                
                # Target Pose for Calibration:
                # Hip straight, Thigh horizontal backward, Calf vertical down
                side = -1 if leg_idx in [0, 2] else 1
                origin = config.LEG_ORIGINS[:, leg_idx]
                
                # Knee: shift backward by L2, outward by L1
                knee = np.array([
                    origin[0] - config.L2,
                    origin[1] + side * config.L1,
                    origin[2]
                ])
                
                # Paw: shift backward by L2, outward by L1, down by L3
                paw = np.array([
                    origin[0] - config.L2,
                    origin[1] + side * config.L1,
                    origin[2] - config.L3
                ])
                
                # Theoretical angles for this pose
                hip_phys = 0.0
                hip_servo = 90.0
                thigh_phys = 60.0   # Software angle for horizontal backward (90 deg backward)
                thigh_servo = 110.0 # Software 60 maps to hardware 70/110
                calf_phys = -20.0   # Software angle for vertical down as mounted
                calf_servo = 110.0  # Software -20 maps to hardware 70/110
                
                resp = {
                    "hip_origin": origin.tolist(),
                    "knee": knee.tolist(),
                    "paw": paw.tolist(),
                    "angles": {
                        "hip_phys": hip_phys,
                        "hip_servo": hip_servo,
                        "thigh_phys": thigh_phys,
                        "thigh_servo": thigh_servo,
                        "calf_phys": calf_phys,
                        "calf_servo": calf_servo
                    },
                    "offsets": [offsets[leg_idx*3], offsets[leg_idx*3+1], offsets[leg_idx*3+2]],
                    "inversions": [inversions[leg_idx*3], inversions[leg_idx*3+1], inversions[leg_idx*3+2]]
                }
                
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps(resp).encode('utf-8'))
            except Exception as e:
                self.send_error(400, f"Error calculating calibration: {e}")
        elif self.path == '/api/calibration/save':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            try:
                data = json.loads(post_data.decode('utf-8'))
                leg_idx = int(data['leg_index'])
                new_offsets = [float(x) for x in data['offsets']] # 3 values
                
                # Load existing config, modify, write back
                cal_path = os.path.join(base_dir, "calibration_tool", "robodog_calibration_offsets.json")
                if os.path.exists(cal_path):
                    with open(cal_path, "r") as f:
                        config_data = json.load(f)
                else:
                    config_data = {"offsets": [0.0]*12, "inversions": [False]*12}
                
                config_data["offsets"][leg_idx*3] = round(new_offsets[0], 2)
                config_data["offsets"][leg_idx*3+1] = round(new_offsets[1], 2)
                config_data["offsets"][leg_idx*3+2] = round(new_offsets[2], 2)
                
                with open(cal_path, "w") as f:
                    json.dump(config_data, f, indent=4)
                    
                # Reload calibration in running controller if active
                if TelemetryServerHandler.controller:
                    TelemetryServerHandler.controller.load_calibration()
                    
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(b'{"status":"saved"}')
            except Exception as e:
                self.send_error(400, f"Error saving calibration: {e}")
        elif self.path == '/api/calibration/tune':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            try:
                data = json.loads(post_data.decode('utf-8'))
                leg_idx = int(data['leg_index'])
                temp_offsets = [float(x) for x in data['offsets']]
                
                if TelemetryServerHandler.controller:
                    TelemetryServerHandler.controller.calibration_active = True
                    TelemetryServerHandler.controller.calibration_leg_idx = leg_idx
                    TelemetryServerHandler.controller.offsets[leg_idx*3] = temp_offsets[0]
                    TelemetryServerHandler.controller.offsets[leg_idx*3+1] = temp_offsets[1]
                    TelemetryServerHandler.controller.offsets[leg_idx*3+2] = temp_offsets[2]
                    
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(b'{"status":"tuned"}')
            except Exception as e:
                self.send_error(400, f"Error tuning calibration: {e}")
        elif self.path == '/api/calibration/mode':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            try:
                data = json.loads(post_data.decode('utf-8'))
                active = bool(data.get('active', False))
                leg_idx = int(data.get('leg_index', 0))
                
                if TelemetryServerHandler.controller:
                    TelemetryServerHandler.controller.calibration_active = active
                    TelemetryServerHandler.controller.calibration_leg_idx = leg_idx
                    
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(b'{"status":"ok"}')
            except Exception as e:
                self.send_error(400, f"Error setting calibration mode: {e}")
        else:
            self.send_error(404)

    def log_message(self, format, *args):
        # Suppress server request logging to avoid terminal clutter
        pass

# -------------------------------------------------------------------------
# 6. Standalone Direct Keyboard Walking System Controller with Server
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
            try:
                print("Initializing PCA9685 Servo Kit (16 channels)...")
                self.kit = ServoKit(channels=16)
                for i in range(16):
                    self.kit.servo[i].actuation_range = 180
                    self.kit.servo[i].set_pulse_width_range(370, 2400)
            except Exception as e:
                print(f"Warning: PCA9685 Initialization failed ({e}). Running in MOCK mode.")
                self.kit = None
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
        self.manual_override = False
        self.override_angles = [0.0, 90.0, 0.0] * 4
        self.current_angles = [90.0] * 12
        self.limp = False
        self.calibration_active = False
        self.calibration_leg_idx = 0

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
                roll = math.radians(euler[2])  # Correct mounting roll orientation
                pitch = math.radians(euler[1])
                return np.array([yaw, pitch, roll])
        except Exception:
            pass
        return np.array([0.0, 0.0, 0.0])

    def update_command_from_keys(self):
        vx = 0.0
        vy = 0.0
        yaw_rate = 0.0
        
        if 'w' in self.keys_pressed:
            vx = self.step_length
        elif 's' in self.keys_pressed:
            vx = -self.step_length
            
        if 'a' in self.keys_pressed:
            vy = 0.04
        elif 'd' in self.keys_pressed:
            vy = -0.04
            
        if 'q' in self.keys_pressed:
            yaw_rate = 0.5
        elif 'e' in self.keys_pressed:
            yaw_rate = -0.5
            
        self.command.horizontal_velocity = np.array([vx, vy])
        self.command.yaw_rate = yaw_rate

    def on_press(self, key):
        try:
            if hasattr(key, 'char') and key.char:
                char = key.char.lower()
                self.keys_pressed.add(char)
                
                if char == 'u':
                    self.height_level = min(3, self.height_level + 1)
                    self.target_height = -0.205 + self.height_level * 0.01
                    print(f"Height raised to: {self.target_height*100:.1f} cm")
                elif char == 'j':
                    self.height_level = max(-3, self.height_level - 1)
                    self.target_height = -0.205 + self.height_level * 0.01
                    print(f"Height lowered to: {self.target_height*100:.1f} cm")
                    
            elif key == keyboard.Key.space:
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
        
        # Start Web Telemetry Server
        TelemetryServerHandler.controller = self
        web_server = ThreadingHTTPServer(('0.0.0.0', 8080), TelemetryServerHandler)
        web_thread = threading.Thread(target=web_server.serve_forever)
        web_thread.daemon = True
        web_thread.start()
        
        print("--------------------------------------------------")
        print("RoboDog Standalone Keyboard Walking Controller + Web UI Active")
        print("Web GUI available at: http://localhost:8080 or http://<pi-ip>:8080")
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

            # If in manual override, bypass IK and use manual override angles directly
            if self.manual_override:
                raw_angles = [90.0] * 12
                for leg in range(4):
                    # Software angles from UI
                    sw_h = self.override_angles[leg * 3]
                    sw_u = self.override_angles[leg * 3 + 1]
                    sw_l = self.override_angles[leg * 3 + 2]
                    
                    # 1. Hip: sw_h + 90.0 is the slider value (0 to 180)
                    hip_val = sw_h + 90.0
                    offset_h = self.offsets[leg * 3]
                    inv_h = self.inversions[leg * 3]
                    r_h = (180.0 - (hip_val + offset_h)) if inv_h else (hip_val + offset_h)
                    raw_angles[leg * 3] = max(0.0, min(180.0, r_h))
                    
                    # 2. Thigh: sw_u is the direct slider value (0 to 160)
                    thigh_val = sw_u
                    offset_u = self.offsets[leg * 3 + 1]
                    inv_u = self.inversions[leg * 3 + 1]
                    r_u = (180.0 - (thigh_val + offset_u)) if inv_u else (thigh_val + offset_u)
                    raw_angles[leg * 3 + 1] = max(0.0, min(180.0, r_u))
                    
                    # 3. Calf: sw_l + 20.0 is the kinematics angle (THETA3 = 0 when vertical)
                    # Use parallel linkage math relative to physical mounting
                    theta_upper_rad = math.pi / 2.0 - math.radians(sw_u - 150.0)
                    theta_lower_rad = math.radians(sw_l + 20.0) + math.pi / 2.0
                    THETA0 = lower_leg_angle_to_servo_angle(self.linkage, theta_upper_rad, theta_lower_rad)
                    compensated_l = math.degrees(math.pi / 2.0 + math.pi - THETA0)
                    
                    offset_l = self.offsets[leg * 3 + 2]
                    inv_l = self.inversions[leg * 3 + 2]
                    r_l = (180.0 - (compensated_l + offset_l)) if inv_l else (compensated_l + offset_l)
                    raw_angles[leg * 3 + 2] = max(0.0, min(180.0, r_l))
                
                # Reconstruct joint_angles (kinematic) for 3D model telemetry
                for leg in range(4):
                    sw_h = self.override_angles[leg * 3]
                    sw_u = self.override_angles[leg * 3 + 1]
                    sw_l = self.override_angles[leg * 3 + 2]
                    
                    self.state.joint_angles[0, leg] = math.radians(sw_h)
                    self.state.joint_angles[1, leg] = math.radians(sw_u - 150.0)
                    self.state.joint_angles[2, leg] = math.radians(sw_l + 20.0)
            else:
                # Translate IK angles to PCA9685 raw angles using loaded JSON offsets & mounting logic
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
                    up_ang_deg = math.degrees(THETA2) + 150.0
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

            # Rate limit/interpolate the raw servo angles to prevent sudden jerking (matching arduinocde7-6-2026-v1.ino)
            for i in range(12):
                error = raw_angles[i] - self.current_angles[i]
                if abs(error) > 0.01:
                    abs_err = abs(error)
                    if abs_err > 60:
                        step = 8.0
                    elif abs_err > 30:
                        step = 5.0
                    elif abs_err > 15:
                        step = 3.0
                    elif abs_err > 5:
                        step = 2.0
                    else:
                        step = 1.0

                    if error > 0:
                        self.current_angles[i] = min(raw_angles[i], self.current_angles[i] + step)
                    else:
                        self.current_angles[i] = max(raw_angles[i], self.current_angles[i] - step)

            # Push angles to PCA9685
            if self.kit:
                JOINT_TO_CHANNEL = [14, 13, 12, 10, 9, 8, 2, 1, 0, 6, 5, 4]
                for i in range(12):
                    ch = JOINT_TO_CHANNEL[i]
                    try:
                        if self.limp:
                            self.kit.servo[ch].angle = None
                        elif self.calibration_active:
                            # Only activate the leg being calibrated. Rest are limp!
                            if (i // 3) == self.calibration_leg_idx:
                                self.kit.servo[ch].angle = self.current_angles[i]
                            else:
                                self.kit.servo[ch].angle = None
                        else:
                            self.kit.servo[ch].angle = self.current_angles[i]
                    except Exception as e:
                        # Suppress spam but print warnings for transient connection issues
                        if self.state.ticks % 100 == 0:
                            print(f"[WARN] I2C Servo Write Error (ch {ch}): {e}")

            # Compute software angles list for UI dashboard display
            software_angles = [[0.0]*4 for _ in range(3)]
            for leg in range(4):
                if self.manual_override:
                    software_angles[0][leg] = self.override_angles[leg * 3]
                    software_angles[1][leg] = self.override_angles[leg * 3 + 1]
                    software_angles[2][leg] = self.override_angles[leg * 3 + 2]
                else:
                    software_angles[0][leg] = math.degrees(self.state.joint_angles[0, leg])
                    software_angles[1][leg] = math.degrees(self.state.joint_angles[1, leg]) + 60.0
                    software_angles[2][leg] = math.degrees(self.state.joint_angles[2, leg]) - 20.0

            # Broadcast update to web clients
            state_data = {
                "imu": self.state.euler_orientation.tolist(),
                "joint_angles": self.state.joint_angles.tolist(),
                "software_angles": software_angles,
                "foot_locations": self.state.foot_locations.tolist(),
                "raw_angles": raw_angles,
                "behavior_state": "LIMP" if self.limp else ("MANUAL" if self.manual_override else ("TROT" if self.state.behavior_state == BehaviorState.TROT else "REST")),
                "height": self.target_height,
                "keys": list(self.keys_pressed),
                "manual_override": self.manual_override,
                "override_angles": self.override_angles,
                "limp": self.limp,
                "calibration_active": self.calibration_active,
                "calibration_leg_idx": self.calibration_leg_idx
            }
            for q in list(TelemetryServerHandler.clients):
                try:
                    q.put_nowait(state_data)
                except Exception:
                    pass

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
                try:
                    self.kit.servo[ch].angle = 90.0
                except Exception:
                    pass
        listener.stop()
        web_server.shutdown()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--cali', action='store_true', help='Launch directly in calibration mode')
    parser.add_argument('--raw', action='store_true', help='Launch directly in raw/manual override mode')
    args = parser.parse_args()

    controller = DirectKeyboardWalk()
    if args.cali:
        controller.calibration_active = True
        controller.calibration_leg_idx = 0
    if args.raw:
        controller.manual_override = True
    controller.control_loop()
