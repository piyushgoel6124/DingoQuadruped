#!/usr/bin/env python3
import sys
import os
import math
import json
import webbrowser
import threading
import time
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

# Add package paths to Python path
base_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(base_dir, "dingo_ws", "src", "dingo_control", "src"))
sys.path.append(os.path.join(base_dir, "dingo_ws", "src", "dingo_utilities", "src"))
sys.path.append(os.path.join(base_dir, "dingo_ws", "src", "dingo_hardware_interfacing", "dingo_servo_interfacing", "src"))
sys.path.append(os.path.join(base_dir, "dingo_ws", "src", "dingo_hardware_interfacing", "dingo_input_interfacing", "src"))
sys.path.append(os.path.join(base_dir, "dingo_ws", "src", "dingo_hardware_interfacing", "dingo_peripheral_interfacing", "src"))

from dingo_control.Config import Configuration, Leg_linkage

# Replicating link configuration & helper functions
config = Configuration()
linkage = Leg_linkage(config)

# Load calibration
offsets = [0.0]*12
inversions = [False]*12

cal_path = os.path.join(base_dir, "calibration_tool", "dingo_calibration_offsets.json")
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

# Hardware init
try:
    from adafruit_servokit import ServoKit
    kit = ServoKit(channels=16)
    for i in range(16):
        kit.servo[i].actuation_range = 180
        kit.servo[i].set_pulse_width_range(370, 2400)
    servos_available = True
    print("Adafruit ServoKit initialized successfully.")
except Exception as e:
    print(f"Warning: Adafruit ServoKit could not be initialized: {e}")
    kit = None
    servos_available = False

# Mapping
vis_to_ctrl = {
    0: 1,  # FL
    1: 0,  # FR
    2: 3,  # BL
    3: 2   # BR
}

# PCA9685 Channel mapping for joints
JOINT_TO_CHANNEL = [14, 13, 12, 10, 9, 8, 2, 1, 0, 6, 5, 4]

# Keep track of active / last target values for all 12 joints
last_target_raw = [90.0] * 12

# Soft motion variables
target_angles = [None] * 16
current_angles = [None] * 16
use_soft_motion_active = False

# 3D offsets for mounting the 4 legs relative to center of body (matching LEG_ORIGINS)
LEG_ORIGINS_3D = [
    [0.11165, 0.061, 0.0],   # FL (Leg 0)
    [0.11165, -0.061, 0.0],  # FR (Leg 1)
    [-0.11165, 0.061, 0.0],  # BL (Leg 2)
    [-0.11165, -0.061, 0.0]  # BR (Leg 3)
]

# Thread-safe global variables for current simulation state
state_lock = threading.Lock()
active_mode = 'IK' # 'IK' or 'FK'
active_leg = '0'   # '0'..'3' or 'all'
target_x = 0.0
target_y = 0.0
target_z = -0.180
target_fk_angles = [0.0, 46.3 * math.pi / 180.0, -2.9 * math.pi / 180.0]
demo_running = False
demo_time = 0.0
hw_enabled = False
ctrl_only_selected = True
use_soft_motion = False

# Target lists for all 4 legs
legs_target_x = [0.0] * 4
legs_target_z = [-0.180] * 4

config_L1 = 0.05162
config_L2 = 0.130
config_L3 = 0.13814
config_phi = 73.917 * math.pi / 180.0

def point_to_rad(p1, p2):
    theta = math.atan2(p2, p1)
    return (theta + 2 * math.pi) % (2 * math.pi)

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

def solve_ik_in_python(x, y, z, leg_index):
    is_right = 0 if (leg_index == 1 or leg_index == 3) else 1
    
    inputY = y
    if is_right:
        inputY = -inputY

    r_body_foot = [x, inputY, z]
    
    # Step 2: Rotate origin frame
    R1 = math.pi/2 - config_phi
    c = math.cos(-R1)
    s = math.sin(-R1)
    rx = r_body_foot[0]
    ry = c*r_body_foot[1] - s*r_body_foot[2]
    rz = s*r_body_foot[1] + c*r_body_foot[2]
    
    # Step 3: Calculate theta_1
    len_A = math.sqrt(ry * ry + rz * rz)
    is_out_of_bounds = False

    if len_A == 0:
        len_A = 0.0001
        
    ratio = (math.sin(config_phi) * config_L1) / len_A
    if ratio > 1.0:
        ratio = 1.0
        is_out_of_bounds = True
    elif ratio < -1.0:
        ratio = -1.0
        is_out_of_bounds = True

    a_1 = point_to_rad(ry, rz)
    a_2 = math.asin(ratio)
    a_3 = math.pi - a_2 - config_phi               

    theta_1 = a_1 + a_3
    if theta_1 >= 2 * math.pi:
        theta_1 = theta_1 % (2 * math.pi)
    
    # Step 4: Translate and rotate to 2D plane
    offset = [0.0, config_L1 * math.cos(theta_1), config_L1 * math.sin(theta_1)]
    translated_frame = [
        rx - offset[0],
        ry - offset[1],
        rz - offset[2]
    ]
    
    R2 = theta_1 + config_phi - math.pi / 2
    c2 = math.cos(-R2)
    s2 = math.sin(-R2)
    
    rx_ = translated_frame[0]
    ry_ = c2*translated_frame[1] - s2*translated_frame[2]
    rz_ = s2*translated_frame[1] + c2*translated_frame[2]
    
    len_B = math.sqrt(rx_ * rx_ + rz_ * rz_)
    
    # Step 5: Solve 2D Knee and Thigh angles
    if len_B >= (config_L2 + config_L3):
        len_B = (config_L2 + config_L3) * 0.999
        is_out_of_bounds = True
    if len_B < abs(config_L2 - config_L3):
        len_B = abs(config_L2 - config_L3) * 1.001
        is_out_of_bounds = True
    
    b_1 = point_to_rad(rx_, rz_)  
    cos_b2 = (config_L2*config_L2 + len_B*len_B - config_L3*config_L3) / (2 * config_L2 * len_B)
    cos_b3 = (config_L2*config_L2 + config_L3*config_L3 - len_B*len_B) / (2 * config_L2 * config_L3)
    
    cos_b2 = max(-1.0, min(1.0, cos_b2))
    cos_b3 = max(-1.0, min(1.0, cos_b3))

    b_2 = math.acos(cos_b2) 
    b_3 = math.acos(cos_b3)  
    
    theta_2 = b_1 - b_2
    theta_3 = math.pi - b_3

    # Offset correctors
    corrected = [theta_1, theta_2 - math.pi, theta_3 - math.pi/2]

    for i in range(3):
        theta = corrected[i]
        if theta > 2 * math.pi:
            theta = theta % (2 * math.pi)
        if theta > math.pi:
            theta = -(2 * math.pi - theta)
        if theta < -math.pi:
            theta = (2 * math.pi + theta)
        corrected[i] = theta

    # Forward kinematics for joint position rendering
    c_inv = math.cos(R1)
    s_inv = math.sin(R1)
    hip_pos_rot = [0.0, config_L1 * math.cos(theta_1), config_L1 * math.sin(theta_1)]
    hip_pos = [
        hip_pos_rot[0],
        c_inv*hip_pos_rot[1] - s_inv*hip_pos_rot[2],
        s_inv*hip_pos_rot[1] + c_inv*hip_pos_rot[2]
    ]
    if is_right:
        hip_pos[1] = -hip_pos[1]

    knee_2d = [config_L2 * math.cos(theta_2), 0.0, config_L2 * math.sin(theta_2)]
    foot_2d = [
        knee_2d[0] + config_L3 * math.cos(theta_2 + theta_3),
        0.0,
        knee_2d[2] + config_L3 * math.sin(theta_2 + theta_3)
    ]

    c2_inv = math.cos(R2)
    s2_inv = math.sin(R2)
    
    knee_pos_rot = [
        knee_2d[0],
        knee_2d[1] + offset[1],
        knee_2d[2] + offset[2]
    ]
    knee_pos = [
        knee_pos_rot[0],
        c2_inv*knee_pos_rot[1] - s2_inv*knee_pos_rot[2],
        s2_inv*knee_pos_rot[1] + c2_inv*knee_pos_rot[2]
    ]
    knee_pos = [
        knee_pos[0],
        c_inv*knee_pos[1] - s_inv*knee_pos[2],
        s_inv*knee_pos[1] + c_inv*knee_pos[2]
    ]
    if is_right:
        knee_pos[1] = -knee_pos[1]

    foot_pos_rot = [
        foot_2d[0],
        foot_2d[1] + offset[1],
        foot_2d[2] + offset[2]
    ]
    foot_pos = [
        foot_pos_rot[0],
        c2_inv*foot_pos_rot[1] - s2_inv*foot_pos_rot[2],
        s2_inv*foot_pos_rot[1] + c2_inv*foot_pos_rot[2]
    ]
    foot_pos = [
        foot_pos[0],
        c_inv*foot_pos[1] - s_inv*foot_pos[2],
        s_inv*foot_pos[1] + c_inv*foot_pos[2]
    ]
    if is_right:
        foot_pos[1] = -foot_pos[1]

    return {
        'angles': [theta_1, theta_2, theta_3],
        'corrected': corrected,
        'is_out_of_bounds': is_out_of_bounds,
        'joints': {
            'shoulder': [0.0, 0.0, 0.0],
            'hip': hip_pos,
            'knee': knee_pos,
            'foot': foot_pos
        },
        'steps': {
            'is_right': is_right,
            'R1': R1,
            'r_foot_rot': [rx, ry, rz],
            'len_A': len_A,
            'a_1': a_1,
            'a_2': a_2,
            'a_3': a_3,
            'theta_1': theta_1,
            'offset': offset,
            'R2': R2,
            'j4_2_vec_': [rx_, ry_, rz_],
            'len_B': len_B,
            'b_1': b_1,
            'b_2': b_2,
            'b_3': b_3,
            'theta_2': theta_2,
            'theta_3': theta_3
        }
    }

def solve_fk_in_python(t1, t2, t3, leg_index):
    is_right = 0 if (leg_index == 1 or leg_index == 3) else 1
    theta_1 = t1
    theta_2 = t2 + math.pi
    theta_3 = t3 + math.pi/2
    
    R1 = math.pi/2 - config_phi
    offset = [0.0, config_L1 * math.cos(theta_1), config_L1 * math.sin(theta_1)]
    R2 = theta_1 + config_phi - math.pi / 2
    
    # 2D local plane
    knee_2d = [config_L2 * math.cos(theta_2), 0.0, config_L2 * math.sin(theta_2)]
    foot_2d = [
        knee_2d[0] + config_L3 * math.cos(theta_2 + theta_3),
        0.0,
        knee_2d[2] + config_L3 * math.sin(theta_2 + theta_3)
    ]
    
    c_inv = math.cos(R1)
    s_inv = math.sin(R1)
    
    hip_pos_rot = [0.0, config_L1 * math.cos(theta_1), config_L1 * math.sin(theta_1)]
    hip_pos = [
        hip_pos_rot[0],
        c_inv*hip_pos_rot[1] - s_inv*hip_pos_rot[2],
        s_inv*hip_pos_rot[1] + c_inv*hip_pos_rot[2]
    ]
    if is_right: hip_pos[1] = -hip_pos[1]
    
    c2_inv = math.cos(R2)
    s2_inv = math.sin(R2)
    
    knee_pos_rot = [
        knee_2d[0],
        knee_2d[1] + offset[1],
        knee_2d[2] + offset[2]
    ]
    knee_pos = [
        knee_pos_rot[0],
        c2_inv*knee_pos_rot[1] - s2_inv*knee_pos_rot[2],
        s2_inv*knee_pos_rot[1] + c2_inv*knee_pos_rot[2]
    ]
    knee_pos = [
        knee_pos[0],
        c_inv*knee_pos[1] - s_inv*knee_pos[2],
        s_inv*knee_pos[1] + c_inv*knee_pos[2]
    ]
    if is_right: knee_pos[1] = -knee_pos[1]

    foot_pos_rot = [
        foot_2d[0],
        foot_2d[1] + offset[1],
        foot_2d[2] + offset[2]
    ]
    foot_pos = [
        foot_pos_rot[0],
        c2_inv*foot_pos_rot[1] - s2_inv*foot_pos_rot[2],
        s2_inv*foot_pos_rot[1] + c2_inv*foot_pos_rot[2]
    ]
    foot_pos = [
        foot_pos[0],
        c_inv*foot_pos[1] - s_inv*foot_pos[2],
        s_inv*foot_pos[1] + c_inv*foot_pos[2]
    ]
    if is_right: foot_pos[1] = -foot_pos[1]
    
    return {
        'joints': {
            'shoulder': [0.0, 0.0, 0.0],
            'hip': hip_pos,
            'knee': knee_pos,
            'foot': foot_pos
        }
    }

def soft_motion_thread_func():
    global current_angles, target_angles, use_soft_motion_active
    while True:
        if kit and use_soft_motion_active:
            for ch in JOINT_TO_CHANNEL:
                if target_angles[ch] is None:
                    if current_angles[ch] is not None:
                        current_angles[ch] = None
                        try:
                            kit.servo[ch].angle = None
                        except Exception:
                            pass
                else:
                    if current_angles[ch] is None:
                        current_angles[ch] = target_angles[ch]
                        try:
                            kit.servo[ch].angle = current_angles[ch]
                        except Exception:
                            pass
                    else:
                        error = target_angles[ch] - current_angles[ch]
                        if abs(error) > 0.05:
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
                                current_angles[ch] = min(target_angles[ch], current_angles[ch] + step)
                            else:
                                current_angles[ch] = max(target_angles[ch], current_angles[ch] - step)
                            
                            try:
                                kit.servo[ch].angle = current_angles[ch]
                            except Exception:
                                pass
        time.sleep(0.02)

def motor_control_thread_func():
    global demo_time, target_x, target_z, use_soft_motion_active
    global legs_target_x, legs_target_z
    while True:
        with state_lock:
            # 1. Update walk demo trajectory if active
            if demo_running:
                demo_time += 0.03
                cx = 0.0
                cz = target_z # default stance height
                radiusX = 0.05
                radiusZ = 0.04
                
                if active_leg == 'all':
                    for i in range(4):
                        phase = demo_time
                        if i == 1 or i == 2: # FR and BL are 180 degrees out of phase (trot gait)
                            phase += math.pi
                        legs_target_x[i] = cx - radiusX * math.cos(phase)
                        t_z = cz + radiusZ * math.sin(phase)
                        if t_z < cz:
                            t_z = cz
                        legs_target_z[i] = t_z
                else:
                    # Single leg walking
                    idx = int(active_leg)
                    legs_target_x[idx] = cx - radiusX * math.cos(demo_time)
                    t_z = cz + radiusZ * math.sin(demo_time)
                    if t_z < cz:
                        t_z = cz
                    legs_target_z[idx] = t_z
                    target_x = legs_target_x[idx]
                    target_z = legs_target_z[idx]
            else:
                # If not walking, update active leg target from target_x, target_z
                if active_leg != 'all':
                    idx = int(active_leg)
                    legs_target_x[idx] = target_x
                    legs_target_z[idx] = target_z

            # 2. Solve kinematics for active leg or all legs
            solved_angles_list = [ [0.0,0.0,0.0] for _ in range(4) ]
            
            if active_leg == 'all':
                for i in range(4):
                    res_i = solve_ik_in_python(legs_target_x[i], target_y, legs_target_z[i], i)
                    solved_angles_list[i] = res_i.get('corrected', [0.0,0.0,0.0])
            else:
                idx = int(active_leg)
                if active_mode == 'IK':
                    res = solve_ik_in_python(target_x, target_y, target_z, idx)
                    solved_angles_list[idx] = res.get('corrected', [0.0, 0.0, 0.0])
                else:
                    solved_angles_list[idx] = target_fk_angles

            # 3. Apply to physical servos if enabled
            if hw_enabled:
                use_soft_motion_active = use_soft_motion
                
                # Identify which leg indices we need to command
                legs_to_write = range(4) if active_leg == 'all' else [int(active_leg)]
                
                raw_to_write = {}
                for l in legs_to_write:
                    ctrl_leg = vis_to_ctrl.get(l, 0)
                    solved_angles = solved_angles_list[l]
                    
                    # 1. Hip (t1)
                    THETA1 = max(math.radians(-22.0), min(math.radians(22.0), solved_angles[0]))
                    hip_ang_deg = math.degrees(THETA1) + 90.0
                    offset_h = offsets[ctrl_leg * 3]
                    inv_h = inversions[ctrl_leg * 3]
                    r_h = (180.0 - (hip_ang_deg + offset_h)) if inv_h else (hip_ang_deg + offset_h)
                    raw_h = max(0.0, min(180.0, r_h))
                    
                    # 2. Thigh (t2)
                    THETA2 = max(math.radians(0.0), min(math.radians(160.0), solved_angles[1]))
                    up_ang_deg = math.degrees(THETA2)
                    offset_u = offsets[ctrl_leg * 3 + 1]
                    inv_u = inversions[ctrl_leg * 3 + 1]
                    r_u = (180.0 - (up_ang_deg + offset_u)) if inv_u else (up_ang_deg + offset_u)
                    raw_u = max(0.0, min(180.0, r_u))
                    
                    # 3. Calf (t3)
                    THETA3 = max(math.radians(-80.0), min(math.radians(80.0), solved_angles[2]))
                    THETA0 = lower_leg_angle_to_servo_angle(linkage, math.pi/2.0 - THETA2, THETA3 + math.pi/2.0)
                    compensated_l = math.degrees(math.pi/2.0 + math.pi - THETA0)
                    offset_l = offsets[ctrl_leg * 3 + 2]
                    inv_l = inversions[ctrl_leg * 3 + 2]
                    r_l = (180.0 - (compensated_l + offset_l)) if inv_l else (compensated_l + offset_l)
                    raw_l = max(0.0, min(180.0, r_l))
                    
                    last_target_raw[ctrl_leg * 3] = raw_h
                    last_target_raw[ctrl_leg * 3 + 1] = raw_u
                    last_target_raw[ctrl_leg * 3 + 2] = raw_l
                    
                    ch_hip = JOINT_TO_CHANNEL[ctrl_leg * 3]
                    ch_upper = JOINT_TO_CHANNEL[ctrl_leg * 3 + 1]
                    ch_lower = JOINT_TO_CHANNEL[ctrl_leg * 3 + 2]
                    
                    raw_to_write[ch_hip] = raw_h
                    raw_to_write[ch_upper] = raw_u
                    raw_to_write[ch_lower] = raw_l
                
                if use_soft_motion_active:
                    for l in range(4):
                        ch_hip = JOINT_TO_CHANNEL[l * 3]
                        ch_upper = JOINT_TO_CHANNEL[l * 3 + 1]
                        ch_lower = JOINT_TO_CHANNEL[l * 3 + 2]
                        
                        ctrl_leg = vis_to_ctrl.get(l, 0)
                        if l in legs_to_write:
                            target_angles[ch_hip] = raw_to_write[ch_hip]
                            target_angles[ch_upper] = raw_to_write[ch_upper]
                            target_angles[ch_lower] = raw_to_write[ch_lower]
                        elif ctrl_only_selected:
                            target_angles[ch_hip] = None
                            target_angles[ch_upper] = None
                            target_angles[ch_lower] = None
                        else:
                            target_angles[ch_hip] = last_target_raw[ctrl_leg * 3]
                            target_angles[ch_upper] = last_target_raw[ctrl_leg * 3 + 1]
                            target_angles[ch_lower] = last_target_raw[ctrl_leg * 3 + 2]
                else:
                    for ch in JOINT_TO_CHANNEL:
                        target_angles[ch] = None
                        current_angles[ch] = None
                        
                    if kit:
                        for l in range(4):
                            ch_hip = JOINT_TO_CHANNEL[l * 3]
                            ch_upper = JOINT_TO_CHANNEL[l * 3 + 1]
                            ch_lower = JOINT_TO_CHANNEL[l * 3 + 2]
                            
                            ctrl_leg = vis_to_ctrl.get(l, 0)
                            if l in legs_to_write:
                                try:
                                    kit.servo[ch_hip].angle = raw_to_write[ch_hip]
                                    kit.servo[ch_upper].angle = raw_to_write[ch_upper]
                                    kit.servo[ch_lower].angle = raw_to_write[ch_lower]
                                except Exception: pass
                            elif ctrl_only_selected:
                                try:
                                    kit.servo[ch_hip].angle = None
                                    kit.servo[ch_upper].angle = None
                                    kit.servo[ch_lower].angle = None
                                except Exception: pass
                            else:
                                try:
                                    kit.servo[ch_hip].angle = last_target_raw[ctrl_leg * 3]
                                    kit.servo[ch_upper].angle = last_target_raw[ctrl_leg * 3 + 1]
                                    kit.servo[ch_lower].angle = last_target_raw[ctrl_leg * 3 + 2]
                                except Exception: pass
            else:
                for ch in JOINT_TO_CHANNEL:
                    target_angles[ch] = None
                    if not use_soft_motion_active and kit:
                        try:
                            kit.servo[ch].angle = None
                        except Exception: pass
                        
        time.sleep(0.02)

class IKHTTPRequestHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        return
        
    def do_GET(self):
        if self.path == "/" or self.path == "/index.html" or self.path == "/ik_visualizer.html":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            with open(os.path.join(base_dir, "ik_visualizer.html"), "rb") as f:
                self.wfile.write(f.read())
        else:
            self.send_error(404, "Not Found")
            
    def do_POST(self):
        if self.path == "/api/update_state":
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length)
            try:
                data = json.loads(body.decode('utf-8'))
                
                global active_mode, active_leg, target_x, target_y, target_z
                global target_fk_angles, demo_running, hw_enabled, ctrl_only_selected
                global use_soft_motion, config_L1, config_L2, config_L3, config_phi
                
                with state_lock:
                    active_mode = data.get("mode", active_mode)
                    
                    # Can be "0".."3" or "all"
                    req_active_leg = data.get("leg_index", active_leg)
                    if req_active_leg == "all":
                        active_leg = "all"
                    else:
                        active_leg = str(req_active_leg)
                    
                    if "target_coords" in data:
                        target_x = float(data["target_coords"][0])
                        target_y = float(data["target_coords"][1])
                        target_z = float(data["target_coords"][2])
                        
                    if "target_angles" in data:
                        target_fk_angles = [float(a) for a in data["target_angles"]]
                        
                    demo_running = bool(data.get("demo_active", demo_running))
                    hw_enabled = bool(data.get("hardware_control_enabled", hw_enabled))
                    ctrl_only_selected = bool(data.get("control_only_this_leg", ctrl_only_selected))
                    use_soft_motion = bool(data.get("use_soft_motion", use_soft_motion))
                    
                    if "config_dims" in data:
                        config_dims = data["config_dims"]
                        config_L1 = float(config_dims.get("L1", config_L1))
                        config_L2 = float(config_dims.get("L2", config_L2))
                        config_L3 = float(config_dims.get("L3", config_L3))
                        config_phi = float(config_dims.get("phi", config_phi))

                    # Solve kinematics depending on whether we want single active leg or all legs
                    if active_leg == 'all':
                        # Solve trot geometry for all 4 legs
                        legs_joints_data = []
                        for i in range(4):
                            res_i = solve_ik_in_python(legs_target_x[i], target_y, legs_target_z[i], i)
                            offset_i = LEG_ORIGINS_3D[i]
                            offset_joints = {}
                            for key, val in res_i['joints'].items():
                                offset_joints[key] = [
                                    val[0] + offset_i[0],
                                    val[1] + offset_i[1],
                                    val[2] + offset_i[2]
                                ]
                            legs_joints_data.append({
                                "joints": offset_joints,
                                "corrected": res_i.get("corrected", [0.0,0.0,0.0]),
                                "angles": res_i.get("angles", [0.0,0.0,0.0])
                            })
                            
                        # Default mock response structure for "all" mode
                        res = {
                            'joints': {},
                            'corrected': [0.0, 0.0, 0.0],
                            'angles': [0.0, 0.0, 0.0],
                            'is_out_of_bounds': False,
                            'steps': {}
                        }
                    else:
                        idx = int(active_leg)
                        if active_mode == 'IK':
                            res = solve_ik_in_python(target_x, target_y, target_z, idx)
                        else:
                            res = solve_fk_in_python(target_fk_angles[0], target_fk_angles[1], target_fk_angles[2], idx)
                            res['corrected'] = target_fk_angles
                            res['angles'] = target_fk_angles
                            res['is_out_of_bounds'] = False
                            res['steps'] = {}
                        legs_joints_data = []

                raw_angles = [0.0, 0.0, 0.0]
                if hw_enabled and active_leg != 'all':
                    ctrl_leg = vis_to_ctrl.get(int(active_leg), 0)
                    raw_angles = [
                        round(last_target_raw[ctrl_leg * 3], 1),
                        round(last_target_raw[ctrl_leg * 3 + 1], 1),
                        round(last_target_raw[ctrl_leg * 3 + 2], 1)
                    ]
                
                response_data = {
                    "status": "ok",
                    "view_mode": "full" if active_leg == "all" else "single",
                    "legs_joints": legs_joints_data,
                    "angles": res.get('angles', [0.0, 0.0, 0.0]),
                    "corrected": res.get('corrected', [0.0, 0.0, 0.0]),
                    "isOutOfBounds": res.get('is_out_of_bounds', False),
                    "joints": res['joints'],
                    "steps": res.get('steps', {}),
                    "target_coords": [target_x, target_y, target_z],
                    "raw_angles": raw_angles
                }
                
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps(response_data).encode('utf-8'))
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"status": "error", "message": str(e)}).encode('utf-8'))

def main():
    # Start soft motion thread
    t = threading.Thread(target=soft_motion_thread_func, daemon=True)
    t.start()
    
    # Start main motor control loop
    t2 = threading.Thread(target=motor_control_thread_func, daemon=True)
    t2.start()
    
    port = 8081
    server_address = ('', port)
    httpd = ThreadingHTTPServer(server_address, IKHTTPRequestHandler)
    print(f"Starting Inverse Kinematics Visualizer Web Server on port {port}...")
    print(f"Open http://localhost:{port} in your browser.")
    webbrowser.open(f"http://localhost:{port}")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down server.")
        sys.exit(0)

if __name__ == "__main__":
    main()
