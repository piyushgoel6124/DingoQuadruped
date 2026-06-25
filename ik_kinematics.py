# coding: utf-8
import math

class KinematicsSolver:
    def __init__(self, linkage, adc_calib):
        self.linkage = linkage
        self.adc_calib = adc_calib
        self.L1 = 0.05162
        self.L2 = 0.130
        self.L3 = 0.13
        self.phi = 73.917 * math.pi / 180.0

    def point_to_rad(self, p1, p2):
        theta = math.atan2(p2, p1)
        return (theta + 2 * math.pi) % (2 * math.pi)

    def calculate_4_bar(self, th2, a, b, c, d):
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

    def lower_leg_angle_to_servo_angle(self, THETA2, THETA3):
        GDE, DEF, EFG = self.calculate_4_bar(THETA3 + self.linkage.lower_leg_bend_angle, self.linkage.i, self.linkage.h, self.linkage.f, self.linkage.g)
        CDH = 1.5 * math.pi - THETA2 - GDE - self.linkage.EDC
        CDA = CDH + self.linkage.gamma
        DAB, ABC, BCD = self.calculate_4_bar(CDA, self.linkage.d, self.linkage.a, self.linkage.b, self.linkage.c)
        THETA0 = DAB + self.linkage.gamma
        return THETA0

    def solve_ik_in_python(self, x, y, z, leg_index):
        is_right = 1 if (leg_index == 1 or leg_index == 3) else 0
        
        inputY = y
        if is_right:
            inputY = -inputY

        r_body_foot = [x, inputY, z]
        
        # Step 2: Rotate origin frame
        R1 = math.pi/2 - self.phi
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
            
        ratio = (math.sin(self.phi) * self.L1) / len_A
        if ratio > 1.0:
            ratio = 1.0
            is_out_of_bounds = True
        elif ratio < -1.0:
            ratio = -1.0
            is_out_of_bounds = True

        a_1 = self.point_to_rad(ry, rz)
        a_2 = math.asin(ratio)
        a_3 = math.pi - a_2 - self.phi               

        theta_1 = a_1 + a_3
        if theta_1 >= 2 * math.pi:
            theta_1 = theta_1 % (2 * math.pi)
        
        # Step 4: Translate and rotate to 2D plane
        offset = [0.0, self.L1 * math.cos(theta_1), self.L1 * math.sin(theta_1)]
        translated_frame = [
            rx - offset[0],
            ry - offset[1],
            rz - offset[2]
        ]
        
        R2 = theta_1 + self.phi - math.pi / 2
        c2 = math.cos(-R2)
        s2 = math.sin(-R2)
        
        rx_ = translated_frame[0]
        ry_ = c2*translated_frame[1] - s2*translated_frame[2]
        rz_ = s2*translated_frame[1] + c2*translated_frame[2]
        
        len_B = math.sqrt(rx_ * rx_ + rz_ * rz_)
        
        # Step 5: Solve 2D Knee and Thigh angles
        if len_B >= (self.L2 + self.L3):
            len_B = (self.L2 + self.L3) * 0.999
            is_out_of_bounds = True
        if len_B < abs(self.L2 - self.L3):
            len_B = abs(self.L2 - self.L3) * 1.001
            is_out_of_bounds = True
        
        b_1 = self.point_to_rad(rx_, rz_)  
        cos_b2 = (self.L2*self.L2 + len_B*len_B - self.L3*self.L3) / (2 * self.L2 * len_B)
        cos_b3 = (self.L2*self.L2 + self.L3*self.L3 - len_B*len_B) / (2 * self.L2 * self.L3)
        
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
        hip_pos_rot = [0.0, self.L1 * math.cos(theta_1), self.L1 * math.sin(theta_1)]
        hip_pos = [
            hip_pos_rot[0],
            c_inv*hip_pos_rot[1] - s_inv*hip_pos_rot[2],
            s_inv*hip_pos_rot[1] + c_inv*hip_pos_rot[2]
        ]
        if is_right:
            hip_pos[1] = -hip_pos[1]

        knee_2d = [self.L2 * math.cos(theta_2), 0.0, self.L2 * math.sin(theta_2)]
        foot_2d = [
            knee_2d[0] + self.L3 * math.cos(theta_2 + theta_3),
            0.0,
            knee_2d[2] + self.L3 * math.sin(theta_2 + theta_3)
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

    def solve_fk_in_python(self, t1, t2, t3, leg_index):
        is_right = 1 if (leg_index == 1 or leg_index == 3) else 0
        theta_1 = t1
        theta_2 = t2 + math.pi
        theta_3 = t3 + math.pi/2
        
        R1 = math.pi/2 - self.phi
        offset = [0.0, self.L1 * math.cos(theta_1), self.L1 * math.sin(theta_1)]
        R2 = theta_1 + self.phi - math.pi / 2
        
        # 2D local plane
        knee_2d = [self.L2 * math.cos(theta_2), 0.0, self.L2 * math.sin(theta_2)]
        foot_2d = [
            knee_2d[0] + self.L3 * math.cos(theta_2 + theta_3),
            0.0,
            knee_2d[2] + self.L3 * math.sin(theta_2 + theta_3)
        ]
        
        c_inv = math.cos(R1)
        s_inv = math.sin(R1)
        
        hip_pos_rot = [0.0, self.L1 * math.cos(theta_1), self.L1 * math.sin(theta_1)]
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

    def estimate_adc_from_angle(self, leg_idx, joint_idx, angle):
        angle_min = self.adc_calib.get("angle_min_grid", [[0]*4]*3)[joint_idx][leg_idx]
        angle_max = self.adc_calib.get("angle_max_grid", [[0]*4]*3)[joint_idx][leg_idx]
        adc_min = self.adc_calib.get("adc_min", [[0]*4]*3)[joint_idx][leg_idx]
        adc_max = self.adc_calib.get("adc_max", [[0]*4]*3)[joint_idx][leg_idx]
        
        if angle_max == angle_min: return adc_min
        
        slope = (adc_max - adc_min) / (angle_max - angle_min)
        return int(adc_min + slope * (angle - angle_min))

    def estimate_angle_from_adc(self, leg_idx, joint_idx, adc_val):
        angle_min = self.adc_calib.get("angle_min_grid", [[0]*4]*3)[joint_idx][leg_idx]
        angle_max = self.adc_calib.get("angle_max_grid", [[0]*4]*3)[joint_idx][leg_idx]
        adc_min = self.adc_calib.get("adc_min", [[0]*4]*3)[joint_idx][leg_idx]
        adc_max = self.adc_calib.get("adc_max", [[0]*4]*3)[joint_idx][leg_idx]
        
        if adc_max == adc_min: return angle_min
        
        slope = (angle_max - angle_min) / (adc_max - adc_min)
        return angle_min + slope * (adc_val - adc_min)
