#!/usr/bin/env python3
# coding: utf-8
import sys
import numpy as np
import math as m
import threading
import time
import json
import os
import re

from nano_controller import NanoController, PIN_TO_JOINT
from web_server import start_web_server

# Global Calibration Offsets and Inversions (Target for Regex replacement)
offsets = np.array(
                    [[0.0, 3.0, 7.0, -7.0],
                    [0.0, 0.0, 0.0, 0.0],
                    [-3.0, 0.0, 0.0, 0.0]])

inversions = np.array(
                    [[False, False, False, False],
                    [False, False, False, False],
                    [False, False, False, False]], dtype=bool)

# Stance settings & positions
calibration_pos = [0, 0, 0]
low = [0, 25, 50]
mid = [0, 42, 30]
high = [0, 50, 20]

position_dict = {
    "cal": calibration_pos,
    "low": low,
    "mid": mid,
    "high": high
}

control_pos = np.array(
                    [[0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [0, 0, 0, 0]], dtype=float)

# RoboDog Interface mock wrapping NanoController
class RoboDogPins:
    def __init__(self):
        self.front_left_hip   = 10
        self.front_left_upper = 9
        self.front_left_lower = 8
        self.front_right_hip   = 14
        self.front_right_upper = 13
        self.front_right_lower = 12
        self.back_left_hip   = 6
        self.back_left_upper = 5
        self.back_left_lower = 4
        self.back_right_hip   = 2
        self.back_right_upper = 1
        self.back_right_lower = 0
        self.pins = np.array([[14,10,2,6], 
                              [13,9,1,5], 
                              [12,8,0,4]])
        self.pwm_max = 2400
        self.pwm_min = 370
        
    def moveAbsAngle(self, servo_pin, angle):
        if servo_pin in PIN_TO_JOINT:
            leg_idx, joint_idx = PIN_TO_JOINT[servo_pin]
            nano_controller.set_servo_angle(leg_idx, joint_idx, angle)

    def relax_all_motors(self):
        pass

RoboDog = RoboDogPins()

# HTML Template Loading
HTML_CONTENT = ""
try:
    ui_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration_ui.html")
    with open(ui_file_path, "r", encoding="utf-8") as f:
        HTML_CONTENT = f.read()
except Exception as e:
    print(f"Error loading HTML UI template: {e}")

def load_json_calibration():
    global offsets, inversions
    json_path = "/home/pi/robodog/calibration_tool/robodog_calibration_offsets.json"
    if os.path.exists(json_path):
        try:
            with open(json_path, "r") as f:
                data = json.load(f)
            if "offsets" in data:
                json_o = data["offsets"]
                for leg in range(4):
                    for joint in range(3):
                        offsets[joint, leg] = json_o[leg * 3 + joint]
            if "inversions" in data:
                json_inv = data["inversions"]
                for leg in range(4):
                    for joint in range(3):
                        inversions[joint, leg] = bool(json_inv[leg * 3 + joint])
            print("Loaded offsets/inversions from JSON:", json_path)
        except Exception as e:
            print("Failed to load JSON calib:", e)

load_json_calibration()

def get_target_angle(leg_idx, joint_idx, offset_val, control_val):
    if joint_idx == 0: # Hip
        base = 90
    elif joint_idx == 1: # Thigh
        base = 0
    else: # Calf
        base = 90
    return base + offset_val + control_val

def init_angles():
    for leg_idx in range(4):
        for joint_idx in range(3):
            val = offsets[joint_idx, leg_idx]
            ctrl = control_pos[joint_idx, leg_idx]
            target_angle = get_target_angle(leg_idx, joint_idx, val, ctrl)
            nano_controller.set_target_angle(leg_idx, joint_idx, target_angle)

def command_servo_angle(servo_pin, target_angle):
    if servo_pin in PIN_TO_JOINT:
        leg_idx, joint_idx = PIN_TO_JOINT[servo_pin]
        nano_controller.set_target_angle(leg_idx, joint_idx, target_angle)

def get_servo_pin(leg_idx, joint_idx):
    if leg_idx == 0: # FR
        return [RoboDog.front_right_hip, RoboDog.front_right_upper, RoboDog.front_right_lower][joint_idx]
    elif leg_idx == 1: # FL
        return [RoboDog.front_left_hip, RoboDog.front_left_upper, RoboDog.front_left_lower][joint_idx]
    elif leg_idx == 2: # BR
        return [RoboDog.back_right_hip, RoboDog.back_right_upper, RoboDog.back_right_lower][joint_idx]
    elif leg_idx == 3: # BL
        return [RoboDog.back_left_hip, RoboDog.back_left_upper, RoboDog.back_left_lower][joint_idx]
    return None

def set_stance(stance_name):
    global calibration_pos
    if stance_name in position_dict:
        calibration_pos = list(position_dict[stance_name])
        for leg_idx in range(4):
            for joint_idx in range(3):
                control_pos[joint_idx, leg_idx] = calibration_pos[joint_idx]
                target_angle = get_target_angle(
                    leg_idx, joint_idx, 
                    offsets[joint_idx, leg_idx], 
                    control_pos[joint_idx, leg_idx]
                )
                pin = get_servo_pin(leg_idx, joint_idx)
                if pin is not None:
                    command_servo_angle(pin, target_angle)
        return "ok", calibration_pos
    return "error", "Invalid Stance"

def update_joint_state(leg_idx, joint_idx, offset=None, control=None, inversion=None):
    if offset is not None:
        offsets[joint_idx, leg_idx] = offset
    if control is not None:
        control_pos[joint_idx, leg_idx] = control
    if inversion is not None:
        inversions[joint_idx, leg_idx] = inversion
        
    target_angle = get_target_angle(
        leg_idx, joint_idx, 
        offsets[joint_idx, leg_idx], 
        control_pos[joint_idx, leg_idx]
    )
    pin = get_servo_pin(leg_idx, joint_idx)
    if pin is not None:
        command_servo_angle(pin, target_angle)
    return target_angle

def save_offsets():
    json_path = "/home/pi/robodog/calibration_tool/robodog_calibration_offsets.json"
    json_data = {
        "offsets": [0.0]*12,
        "inversions": [False]*12
    }
    for leg in range(4):
        for joint in range(3):
            json_data["offsets"][leg * 3 + joint] = float(offsets[joint, leg])
            json_data["inversions"][leg * 3 + joint] = bool(inversions[joint, leg])
    
    os.makedirs(os.path.dirname(json_path), exist_ok=True)
    with open(json_path, "w") as f:
        json.dump(json_data, f, indent=4)
    
    file_path = os.path.abspath(__file__)
    with open(file_path, "r", encoding="utf-8") as f:
        content = f.read()
    
    new_offsets_str = f"offsets = np.array(\n                    [[{offsets[0,0]}, {offsets[0,1]}, {offsets[0,2]}, {offsets[0,3]}],\n                    [{offsets[1,0]}, {offsets[1,1]}, {offsets[1,2]}, {offsets[1,3]}],\n                    [{offsets[2,0]}, {offsets[2,1]}, {offsets[2,2]}, {offsets[2,3]}]])"
    new_inversions_str = f"inversions = np.array(\n                    [[{inversions[0,0]}, {inversions[0,1]}, {inversions[0,2]}, {inversions[0,3]}],\n                    [{inversions[1,0]}, {inversions[1,1]}, {inversions[1,2]}, {inversions[1,3]}],\n                    [{inversions[2,0]}, {inversions[2,1]}, {inversions[2,2]}, {inversions[2,3]}]])"
    
    pattern_offsets = r"offsets\s*=\s*np\.array\(\s*\[\[.*?\]\]\)"
    content_new, count_o = re.subn(pattern_offsets, new_offsets_str, content, flags=re.DOTALL)
    pattern_inversions = r"inversions\s*=\s*np\.array\(\s*\[\[.*?\]\]\)"
    content_new, count_i = re.subn(pattern_inversions, new_inversions_str, content_new, flags=re.DOTALL)
    
    status = "saved"
    if count_o > 0 or count_i > 0:
        with open(file_path, "w", encoding="utf-8") as f:
            f.write(content_new)
    
    hw_path = os.path.join(os.path.dirname(file_path), "HardwareInterface.py")
    if os.path.exists(hw_path):
        with open(hw_path, "r", encoding="utf-8") as f:
            hw_content = f.read()
        new_hw_offsets_str = f"self.physical_calibration_offsets = np.array(\n                    [[{offsets[0,0]}, {offsets[0,1]}, {offsets[0,2]}, {offsets[0,3]}],\n                    [{offsets[1,0]}, {offsets[1,1]}, {offsets[1,2]}, {offsets[1,3]}],\n                    [{offsets[2,0]}, {offsets[2,1]}, {offsets[2,2]}, {offsets[2,3]}]])"
        hw_pattern = r"self\.physical_calibration_offsets\s*=\s*np\.array\(\s*\[\[.*?\]\]\)"
        hw_content_new, hw_count = re.subn(hw_pattern, new_hw_offsets_str, hw_content, flags=re.DOTALL)
        if hw_count > 0:
            with open(hw_path, "w", encoding="utf-8") as f:
                f.write(hw_content_new)
    return status

def calibrate_adc():
    joint_min_limits = [-25, 0, -80]
    joint_max_limits = [25, 110, 80]
    
    def move_smoothly_to(target_limits):
        while True:
            all_reached = True
            for leg_idx in range(4):
                for joint_idx in range(3):
                    current = control_pos[joint_idx, leg_idx]
                    target = target_limits[joint_idx]
                    if current < target:
                        control_pos[joint_idx, leg_idx] = min(current + 5, target)
                        all_reached = False
                    elif current > target:
                        control_pos[joint_idx, leg_idx] = max(current - 5, target)
                        all_reached = False
                        
                    c_val = control_pos[joint_idx, leg_idx]
                    t_angle = get_target_angle(leg_idx, joint_idx, offsets[joint_idx, leg_idx], c_val)
                    servo_pin = [RoboDog.front_right_hip, RoboDog.front_right_upper, RoboDog.front_right_lower][joint_idx] if leg_idx==0 else \
                                [RoboDog.front_left_hip, RoboDog.front_left_upper, RoboDog.front_left_lower][joint_idx] if leg_idx==1 else \
                                [RoboDog.back_right_hip, RoboDog.back_right_upper, RoboDog.back_right_lower][joint_idx] if leg_idx==2 else \
                                [RoboDog.back_left_hip, RoboDog.back_left_upper, RoboDog.back_left_lower][joint_idx]
                    command_servo_angle(servo_pin, t_angle)
            if all_reached:
                break
            time.sleep(0.1)
    
    # Move to min limits smoothly
    move_smoothly_to(joint_min_limits)
    time.sleep(5.0)
    adc_grid_min, _, _ = nano_controller.get_adc_values()
    
    # Move to max limits smoothly
    move_smoothly_to(joint_max_limits)
    time.sleep(5.0)
    adc_grid_max, _, _ = nano_controller.get_adc_values()
    
    # Back to 0
    move_smoothly_to([0, 0, 0])
            
    angle_min_grid = [[0]*4 for _ in range(3)]
    angle_max_grid = [[0]*4 for _ in range(3)]
    for leg_idx in range(4):
        for joint_idx in range(3):
            t_min = get_target_angle(leg_idx, joint_idx, offsets[joint_idx, leg_idx], joint_min_limits[joint_idx])
            t_max = get_target_angle(leg_idx, joint_idx, offsets[joint_idx, leg_idx], joint_max_limits[joint_idx])
            angle_min_grid[joint_idx][leg_idx] = nano_controller.get_adjusted_angle(leg_idx, joint_idx, t_min)
            angle_max_grid[joint_idx][leg_idx] = nano_controller.get_adjusted_angle(leg_idx, joint_idx, t_max)

    mapping_data = {
        "angle_min_grid": angle_min_grid,
        "angle_max_grid": angle_max_grid,
        "adc_min": adc_grid_min,
        "adc_max": adc_grid_max
    }
    
    os.makedirs("/home/pi/robodog/calibration_tool", exist_ok=True)
    with open("/home/pi/robodog/calibration_tool/adc_calibration.json", "w") as f:
        json.dump(mapping_data, f, indent=4)
    return "ok"

# Initialize NanoController
nano_controller = NanoController(inversions)

last_undithered_targets = np.zeros((3, 4))
last_target_change_times = np.zeros((3, 4))

def dither_loop_func():
    global last_undithered_targets, last_target_change_times
    while True:
        try:
            current_time = time.time()
            dither_enabled = state.get("dither_enabled", False)
            for leg_idx in range(4):
                for joint_idx in range(3):
                    val = offsets[joint_idx, leg_idx]
                    ctrl = control_pos[joint_idx, leg_idx]
                    base_target = get_target_angle(leg_idx, joint_idx, val, ctrl)
                    
                    if abs(base_target - last_undithered_targets[joint_idx, leg_idx]) > 0.01:
                        last_undithered_targets[joint_idx, leg_idx] = base_target
                        last_target_change_times[joint_idx, leg_idx] = current_time
                        
                    target_to_send = base_target
                    if dither_enabled and (current_time - last_target_change_times[joint_idx, leg_idx] >= 1.0):
                        dither_val = 1.0 if (int(current_time * 20) % 2 == 0) else 0.0
                        target_to_send = max(0.0, min(180.0, base_target + dither_val))
                    
                    nano_controller.set_target_angle(leg_idx, joint_idx, target_to_send)
        except Exception:
            pass
        time.sleep(0.05)

# Setup shared state dictionary
state = {
    "HTML_CONTENT": HTML_CONTENT,
    "offsets": offsets,
    "inversions": inversions,
    "control_pos": control_pos,
    "calibration_pos": calibration_pos,
    "RoboDog": RoboDog,
    "use_soft_motion": False,
    "dither_enabled": False,
    "nano_controller": nano_controller,
    "position_dict": position_dict,
    "command_servo_angle": command_servo_angle,
    "get_target_angle": get_target_angle,
    "set_stance": set_stance,
    "update_joint": update_joint_state,
    "save_offsets": save_offsets,
    "calibrate_adc": calibrate_adc,
}

if __name__ == "__main__":
    init_angles()
    
    # Start background dither loop thread
    t_dither = threading.Thread(target=dither_loop_func, daemon=True)
    t_dither.start()
    
    # Run the web server
    start_web_server(state, port=8080)
