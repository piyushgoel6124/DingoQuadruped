#!/usr/bin/env python3
# coding: utf-8
import sys
import os
import math
import json
import threading
import time

# Add package paths to Python path
base_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(base_dir, "robodog_ws", "src", "robodog_control", "src"))
sys.path.append(os.path.join(base_dir, "robodog_ws", "src", "robodog_utilities", "src"))
sys.path.append(os.path.join(base_dir, "robodog_ws", "src", "robodog_hardware_interfacing", "robodog_servo_interfacing", "src"))
sys.path.append(os.path.join(base_dir, "robodog_ws", "src", "robodog_hardware_interfacing", "robodog_servo_interfacing", "src", "robodog_servo_interfacing"))
sys.path.append(os.path.join(base_dir, "robodog_ws", "src", "robodog_hardware_interfacing", "robodog_input_interfacing", "src"))
sys.path.append(os.path.join(base_dir, "robodog_ws", "src", "robodog_hardware_interfacing", "robodog_peripheral_interfacing", "src"))

# Import RL Balancer and split components
from rl_balancer import RLBalancer
from ik_kinematics import KinematicsSolver
from ik_web_server import start_web_server
from ik_control import start_control_loops

# Global RL state
rl_balancer = RLBalancer()

from robodog_control.Config import Configuration, Leg_linkage

# Replicating link configuration & helper functions
config = Configuration()
linkage = Leg_linkage(config)

# Load calibration
offsets = [0.0]*12
inversions = [False]*12

csv_logging_enabled = False
csv_log_path = os.path.join(base_dir, "motor_angles_log.csv")
csv_lock = threading.Lock()

cal_path = os.path.join(base_dir, "calibration_tool", "robodog_calibration_offsets.json")
if os.path.exists(cal_path):
    try:
        with open(cal_path, "r") as f:
            data = json.load(f)
        if "offsets" in data:
            offsets = data["offsets"]
        if "inversions" in data:
            inversions = data["inversions"]
        print(f"Loaded calibration offsets successfully from: {cal_path}")
    except Exception as e:
        print(f"Failed to parse calibration offsets file: {e}")

adc_calib = {
    "angle_min_grid": [[0]*4 for _ in range(3)],
    "angle_max_grid": [[0]*4 for _ in range(3)],
    "adc_min": [[0]*4 for _ in range(3)],
    "adc_max": [[0]*4 for _ in range(3)]
}
adc_calib_path = os.path.join(base_dir, "calibration_tool", "adc_calibration.json")
if os.path.exists(adc_calib_path):
    try:
        with open(adc_calib_path, "r") as f:
            data = json.load(f)
        if "angle_min_grid" in data: adc_calib["angle_min_grid"] = data["angle_min_grid"]
        if "angle_max_grid" in data: adc_calib["angle_max_grid"] = data["angle_max_grid"]
        if "adc_min" in data: adc_calib["adc_min"] = data["adc_min"]
        if "adc_max" in data: adc_calib["adc_max"] = data["adc_max"]
        print(f"Loaded ADC calibration successfully from: {adc_calib_path}")
    except Exception as e:
        print(f"Failed to parse ADC calibration file: {e}")

# Load IMU calibration
imu_pitch_index = 2
imu_roll_index = 1
imu_yaw_index = 0
imu_inv_pitch = False
imu_inv_roll = False
imu_inv_yaw = False
imu_zero_pitch = 0.0
imu_zero_roll = 0.0
imu_zero_yaw = 0.0

imu_calib_path = os.path.join(base_dir, "calibration_tool", "imu_calibration.json")
if os.path.exists(imu_calib_path):
    try:
        with open(imu_calib_path, "r") as f:
            imu_data = json.load(f)
        if "axis_mapping" in imu_data:
            imu_pitch_index = imu_data["axis_mapping"].get("pitch_index", imu_pitch_index)
            imu_roll_index = imu_data["axis_mapping"].get("roll_index", imu_roll_index)
            imu_yaw_index = imu_data["axis_mapping"].get("yaw_index", imu_yaw_index)
        if "inversions" in imu_data:
            imu_inv_pitch = imu_data["inversions"].get("pitch", imu_inv_pitch)
            imu_inv_roll = imu_data["inversions"].get("roll", imu_inv_roll)
            imu_inv_yaw = imu_data["inversions"].get("yaw", imu_inv_yaw)
        if "zero_offsets_deg" in imu_data:
            imu_zero_pitch = imu_data["zero_offsets_deg"].get("pitch", imu_zero_pitch)
            imu_zero_roll = imu_data["zero_offsets_deg"].get("roll", imu_zero_roll)
            imu_zero_yaw = imu_data["zero_offsets_deg"].get("yaw", imu_zero_yaw)
        print(f"Loaded IMU calibration successfully from: {imu_calib_path}")
    except Exception as e:
        print(f"Failed to parse IMU calibration file: {e}")

imu_sensor = None
try:
    import board
    import busio
    import adafruit_bno055
    i2c_bus = busio.I2C(board.SCL, board.SDA)
    imu_sensor = adafruit_bno055.BNO055_I2C(i2c_bus)
    print("BNO055 IMU initialized successfully in IK Visualizer.")
except Exception as e:
    print(f"Warning: Could not initialize BNO055 IMU: {e}")

try:
    from robodog_servo_interfacing.CalibrateServos import nano_controller
    print("NanoController imported successfully in IK Visualizer.")
except Exception as e:
    print(f"Could not import NanoController: {e}")
    nano_controller = None

vis_to_ctrl = {
    0: 1,  # FL
    1: 0,  # FR
    2: 3,  # BL
    3: 2   # BR
}

# 3D offsets for mounting the 4 legs relative to center of body (matching LEG_ORIGINS)
LEG_ORIGINS_3D = [
    [0.11165, 0.061, 0.0],   # FL (Leg 0)
    [0.11165, -0.061, 0.0],  # FR (Leg 1)
    [-0.11165, 0.061, 0.0],  # BL (Leg 2)
    [-0.11165, -0.061, 0.0]  # BR (Leg 3)
]

# Thread-safe global variables for current simulation state
state_lock = threading.Lock()

# Initialize KinematicsSolver
solver = KinematicsSolver(linkage, adc_calib)

# Setup shared state dictionary
state = {
    "base_dir": base_dir,
    "csv_log_path": csv_log_path,
    "csv_lock": csv_lock,
    "state_lock": state_lock,
    "active_mode": 'IK',
    "active_leg": '0',
    "target_x": 0.0,
    "target_y": 0.0,
    "target_z": -0.220,
    "target_fk_angles": [0.0, 46.3 * math.pi / 180.0, -2.9 * math.pi / 180.0],
    "demo_running": False,
    "demo_time": 0.0,
    "hw_enabled": False,
    "ctrl_only_selected": True,
    "use_soft_motion": False,
    "no_hip_walk": False,
    "dither_enabled": False,
    "last_undithered_targets": [0.0] * 12,
    "last_target_change_times": [time.time()] * 12,
    "walk_height": 0.20,
    "diagonal_lift": 0.0,
    "walk_front": 0.05,
    "walk_back": -0.05,
    "walk_lift": 0.04,
    "walk_speed": 0.03,
    "legs_target_x": [0.0] * 4,
    "legs_target_z": [-0.180] * 4,
    "pitch_offset_z_active": 0.0,
    "roll_offset_z_active": 0.0,
    "rl_balancing_enabled": False,
    "rl_balancer": rl_balancer,
    "rl_last_reward": 0.0,
    "rl_last_action": [0.0, 0.0],
    "LEG_ORIGINS_3D": LEG_ORIGINS_3D,
    "last_target_raw": [90.0] * 12,
    "sw_angles": [[90.0, 90.0, 90.0, 90.0], [0.0, 0.0, 0.0, 0.0], [90.0, 90.0, 90.0, 90.0]],
    "offsets": offsets,
    "inversions": inversions,
    "nano_controller": nano_controller,
    "imu_sensor": imu_sensor,
    "imu_pitch_index": imu_pitch_index,
    "imu_roll_index": imu_roll_index,
    "imu_yaw_index": imu_yaw_index,
    "imu_inv_pitch": imu_inv_pitch,
    "imu_inv_roll": imu_inv_roll,
    "imu_inv_yaw": imu_inv_yaw,
    "imu_zero_pitch": imu_zero_pitch,
    "imu_zero_roll": imu_zero_roll,
    "imu_zero_yaw": imu_zero_yaw,
    "vis_to_ctrl": vis_to_ctrl,
    "csv_logging_enabled": csv_logging_enabled,
    "solver": solver,
}

def main():
    # Start soft motion & motor control threads
    start_control_loops(state)
    
    # Run the visualizer web server
    start_web_server(state, port=8081)

if __name__ == "__main__":
    main()
