#!/usr/bin/env python3
import sys
import os
import math
import numpy as np

# Mock rospy/messages so configuration & kinematics imports work
from types import ModuleType
class DummyRospy(ModuleType):
    def loginfo(self, *args, **kwargs): pass
    def logwarn(self, *args, **kwargs): pass
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

# Add robodog source paths
base_dir = "/home/pi/robodog"
sys.path.append(os.path.join(base_dir, "robodog_ws", "src", "robodog_control", "src"))
sys.path.append(os.path.join(base_dir, "robodog_ws", "src", "robodog_utilities", "src"))
sys.path.append(os.path.join(base_dir, "robodog_ws", "src", "robodog_hardware_interfacing", "robodog_servo_interfacing", "src"))
sys.path.append(os.path.join(base_dir, "robodog_ws", "src", "robodog_hardware_interfacing", "robodog_peripheral_interfacing", "src"))
sys.path.append(os.path.join(base_dir, "robodog_ws", "src", "robodog_hardware_interfacing", "robodog_input_interfacing", "src"))

from robodog_control.Config import Configuration, Leg_linkage
from robodog_control.Kinematics import four_legs_inverse_kinematics
from transforms3d.euler import euler2mat
import json

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
    """Calculate the 3D position of the knee relative to the body center (0,0,0)"""
    theta_1, theta_2, _ = angles
    
    # Position of knee relative to the hip origin (before rotation about x-axis by theta_1)
    # L1 offset is along the rotated Y/Z axis.
    # Thigh rotates around Y by theta_2
    # In the leg's local plane:
    x_local = -config.L2 * math.cos(theta_2)
    y_local = 0.0
    z_local = -config.L2 * math.sin(theta_2)
    
    # Rotate by theta_1 + geometry offset to align with leg roll axis
    R1 = math.pi/2 - config.phi
    # Rotate back to global frame:
    # Rotate local point around X-axis by (theta_1 + R1)
    angle_x = theta_1 + R1
    
    # 3D relative to hip mounting joint:
    # Adding the L1 link offset:
    x_hip = x_local
    y_hip = config.L1 * math.cos(theta_1) + y_local * math.cos(angle_x) - z_local * math.sin(angle_x)
    z_hip = config.L1 * math.sin(theta_1) + y_local * math.sin(angle_x) + z_local * math.cos(angle_x)
    
    if is_right:
        y_hip = -y_hip
        
    # Relative to body center
    return origin + np.array([x_hip, y_hip, z_hip])

def main():
    config = Configuration()
    linkage = Leg_linkage(config)
    
    leg_names = ["Front Right (FR)", "Front Left (FL)", "Back Right (BR)", "Back Left (BL)"]
    
    # Load offsets if any
    cal_path = os.path.join(base_dir, "calibration_tool", "robodog_calibration_offsets.json")
    offsets = [0.0] * 12
    inversions = [False] * 12
    if os.path.exists(cal_path):
        try:
            with open(cal_path, "r") as f:
                data = json.load(f)
            offsets = data.get("offsets", [0.0]*12)
            inversions = data.get("inversions", [False]*12)
        except Exception:
            pass

    print("\n" + "="*60)
    print("      ROBODOG SIM-TO-REAL CALIBRATION & KINEMATICS MAPPER")
    print("="*60)
    print("This tool will guide you to manually position your real dog")
    print("to match the simulated model coordinates, then calculate offsets.")
    
    # 1. Ask user for standing height
    try:
        height_cm = float(input("\nEnter desired standing height (e.g. 20 for 20cm): "))
    except ValueError:
        height_cm = 20.0
    target_height = -height_cm / 100.0
    
    # 2. Select leg
    print("\nSelect a leg to calibrate:")
    for idx, name in enumerate(leg_names):
        print(f"  [{idx}] {name}")
    try:
        leg_idx = int(input("Choose leg index (0-3): "))
        if leg_idx < 0 or leg_idx > 3:
            leg_idx = 0
    except ValueError:
        leg_idx = 0
        
    is_right = 1 if leg_idx in [0, 2] else 0
    leg_name = leg_names[leg_idx]
    
    # Calculate foot positions
    foot_locations = config.default_stance + np.array([0, 0, target_height])[:, np.newaxis]
    
    # Run IK
    joint_angles = four_legs_inverse_kinematics(foot_locations, config)
    angles = joint_angles[:, leg_idx]
    
    # Calculate Knee position
    origin = config.LEG_ORIGINS[:, leg_idx]
    paw = foot_locations[:, leg_idx]
    knee = calculate_knee_position(angles, config, origin, is_right)
    
    print("\n" + "-"*50)
    print(f"TARGET COORDINATES (in cm, relative to BODY CENTER [0,0,0]):")
    print(f"  Leg: {leg_name}")
    print(f"  * Body Center:    (0.00,  0.00,  0.00) cm")
    print(f"  * Hip Joint:      ({origin[0]*100:+.2f}, {origin[1]*100:+.2f}, {origin[2]*100:+.2f}) cm")
    print(f"  * Knee Joint:     ({knee[0]*100:+.2f}, {knee[1]*100:+.2f}, {knee[2]*100:+.2f}) cm")
    print(f"  * Paw (Foot Tip): ({paw[0]*100:+.2f}, {paw[1]*100:+.2f}, {paw[2]*100:+.2f}) cm")
    print("-"*50)
    
    # Compute theoretical motor angles
    # Hip
    hip_phys = math.degrees(angles[0])
    hip_servo = hip_phys + 90.0
    # Thigh
    THETA2 = angles[1]
    thigh_servo = math.degrees(THETA2)
    # Calf
    THETA3 = angles[2]
    THETA0 = lower_leg_angle_to_servo_angle(linkage, math.pi/2.0 - THETA2, THETA3 + math.pi/2.0)
    calf_servo = math.degrees(math.pi/2.0 + math.pi - THETA0)
    
    print("\nTHEORETICAL SOFTWARE ANGLES REQUIRED FOR THIS STATE:")
    print(f"  - Hip Joint Angle:   {hip_phys:+.2f}°  -> Target Servo Shaft: {hip_servo:.2f}°")
    print(f"  - Thigh Joint Angle: {math.degrees(THETA2):+.2f}°  -> Target Servo Shaft: {thigh_servo:.2f}°")
    print(f"  - Calf Joint Angle:  {math.degrees(THETA3):+.2f}°  -> Target Servo Shaft: {calf_servo:.2f}°")
    print("-"*50)
    
    print("\nINSTRUCTIONS:")
    print("1. Manually move your physical dog's leg so that:")
    print(f"   - The hip frame is at the origin offset.")
    print(f"   - The Knee is located at: ({knee[0]*100:+.2f}, {knee[1]*100:+.2f}, {knee[2]*100:+.2f}) cm relative to the body center.")
    print(f"   - The Paw is on the ground at: ({paw[0]*100:+.2f}, {paw[1]*100:+.2f}, {paw[2]*100:+.2f}) cm.")
    print("2. Once aligned, read/estimate what the current real servo shaft angles are,")
    print("   or check your servo calibration tool values.")
    
    print("\nWould you like to calculate calibration offsets now?")
    calc = input("Enter 'y' to calculate offsets, or any other key to exit: ").strip().lower()
    if calc == 'y':
        # Prompt for real angles
        print(f"\nEnter the actual commanded servo value that makes the physical leg match this pose:")
        try:
            real_hip = float(input(f"  Real Hip Servo Angle (nominal={hip_servo:.1f}°): "))
            real_thigh = float(input(f"  Real Thigh Servo Angle (nominal={thigh_servo:.1f}°): "))
            real_calf = float(input(f"  Real Calf Servo Angle (nominal={calf_servo:.1f}°): "))
        except ValueError:
            print("Invalid input. Exiting.")
            return
            
        # Offset calculations
        # servo_angle = (180.0 - (calculated_angle + offset)) if inverted else (calculated_angle + offset)
        # So:
        # If not inverted: offset = real_servo - calculated_angle
        # If inverted:     offset = (180.0 - real_servo) - calculated_angle
        
        inv_h = inversions[leg_idx*3]
        inv_t = inversions[leg_idx*3 + 1]
        inv_c = inversions[leg_idx*3 + 2]
        
        offset_h = (180.0 - real_hip) - hip_servo if inv_h else real_hip - hip_servo
        offset_t = (180.0 - real_thigh) - thigh_servo if inv_t else real_thigh - thigh_servo
        offset_c = (180.0 - real_calf) - calf_servo if inv_c else real_calf - calf_servo
        
        print("\n" + "="*50)
        print("RECOMMENDED CALIBRATION OFFSETS FOR JSON FILE:")
        print(f"  Replace values in 'offsets' array for leg {leg_name}:")
        print(f"  * Hip Channel (Index {leg_idx*3}):   {offset_h:+.2f}° (Inversion={inv_h})")
        print(f"  * Thigh Channel (Index {leg_idx*3+1}): {offset_t:+.2f}° (Inversion={inv_t})")
        print(f"  * Calf Channel (Index {leg_idx*3+2}):  {offset_c:+.2f}° (Inversion={inv_c})")
        print("="*50 + "\n")

if __name__ == "__main__":
    main()
