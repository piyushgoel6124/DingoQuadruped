# coding: utf-8
import os
import time
import math
import threading

from nano_controller import JOINT_TO_CHANNEL, PIN_TO_JOINT

# Global control/PID variables
integral_pitch = 0.0
integral_roll = 0.0
last_pitch = 0.0
last_roll = 0.0
last_time_pid = time.time()

# Global RL tracking variables
rl_prev_state = None
rl_prev_action = None
rl_prev_mean_action = None

# Throttled CSV logger
last_csv_write_time = 0.0

def log_angles_to_csv(state, angles_12, heights_4):
    global last_csv_write_time
    now = time.time()
    if now - last_csv_write_time >= 0.1: # 10Hz throttling
        last_csv_write_time = now
        file_exists = os.path.exists(state["csv_log_path"])
        with state["csv_lock"]:
            try:
                with open(state["csv_log_path"], "a") as f:
                    if not file_exists:
                        f.write("timestamp,FR_Hip,FR_Thigh,FR_Calf,FL_Hip,FL_Thigh,FL_Calf,BR_Hip,BR_Thigh,BR_Calf,BL_Hip,BL_Thigh,BL_Calf,FR_Height,FL_Height,BR_Height,BL_Height\n")
                    all_vals = list(angles_12) + list(heights_4)
                    row_str = f"{now:.3f}," + ",".join(f"{val:.2f}" for val in all_vals) + "\n"
                    f.write(row_str)
            except Exception as e:
                print(f"Error writing to CSV log: {e}")

def motor_control_thread_func(state):
    global integral_pitch, integral_roll, last_pitch, last_roll, last_time_pid
    global rl_prev_state, rl_prev_action, rl_prev_mean_action
    
    nano_controller = state["nano_controller"]
    solver = state["solver"]
    rl_balancer = state["rl_balancer"]
    imu_sensor = state["imu_sensor"]
    
    while True:
        with state["state_lock"]:
            # 1. Update walk demo trajectory if active
            if state["demo_running"]:
                state["demo_time"] += state["walk_speed"]
                cx = (state["walk_front"] + state["walk_back"]) / 2.0
                cz = -state["walk_height"] # Use dynamic walk height
                radiusX = abs(state["walk_front"] - state["walk_back"]) / 2.0
                radiusZ = state["walk_lift"]
                
                if state["active_leg"] == 'all':
                    for i in range(4):
                        phase = state["demo_time"]
                        if i == 1 or i == 2: # FR and BL are 180 degrees out of phase (trot gait)
                            phase += math.pi
                        state["legs_target_x"][i] = cx - radiusX * math.cos(phase)
                        t_z = cz + radiusZ * math.sin(phase)
                        if t_z < cz:
                            t_z = cz
                        state["legs_target_z"][i] = t_z
                else:
                    # Single leg walking
                    idx = int(state["active_leg"])
                    state["legs_target_x"][idx] = cx - radiusX * math.cos(state["demo_time"])
                    t_z = cz + radiusZ * math.sin(state["demo_time"])
                    if t_z < cz:
                        t_z = cz
                    state["legs_target_z"][idx] = t_z
                    state["target_x"] = state["legs_target_x"][idx]
                    state["target_z"] = state["legs_target_z"][idx]
            else:
                # If not walking, update active leg target from target_x, target_z
                if state["active_leg"] != 'all':
                    idx = int(state["active_leg"])
                    state["legs_target_x"][idx] = state["target_x"]
                    state["legs_target_z"][idx] = state["target_z"]
                else:
                    for i in range(4):
                        state["legs_target_x"][i] = state["target_x"]
                        if state["diagonal_lift"] > 0.0:
                            if i == 0 or i == 3: # Pair 1: FL & BR
                                state["legs_target_z"][i] = state["target_z"] + state["diagonal_lift"]
                            else: # Pair 2: FR & BL (grounded)
                                state["legs_target_z"][i] = state["target_z"]
                        else:
                            state["legs_target_z"][i] = state["target_z"]

            # 2. Compute IMU Balancing Offsets
            pitch_offset_z = 0.0
            roll_offset_z = 0.0
            if imu_sensor:
                try:
                    euler = imu_sensor.euler
                    if euler and len(euler) == 3:
                        # Extract, offset, and invert if needed based on config
                        pitch_raw = euler[state["imu_pitch_index"]] if euler[state["imu_pitch_index"]] is not None else 0.0
                        roll_raw = euler[state["imu_roll_index"]] if euler[state["imu_roll_index"]] is not None else 0.0
                        yaw_raw = euler[state["imu_yaw_index"]] if euler[state["imu_yaw_index"]] is not None else 0.0
                        
                        if state["imu_inv_pitch"]: pitch_raw = -pitch_raw
                        if state["imu_inv_roll"]: roll_raw = -roll_raw
                        if state["imu_inv_yaw"]: yaw_raw = -yaw_raw

                        pitch_raw -= state["imu_zero_pitch"]
                        roll_raw -= state["imu_zero_roll"]
                        yaw_raw -= state["imu_zero_yaw"]

                        pitch_raw = (pitch_raw + 180.0) % 360.0 - 180.0
                        roll_raw = (roll_raw + 180.0) % 360.0 - 180.0
                        yaw_raw = (yaw_raw + 180.0) % 360.0 - 180.0
                        
                        roll = math.radians(roll_raw)
                        pitch = math.radians(pitch_raw)
                        yaw = math.radians(yaw_raw)
                        
                        now = time.time()
                        dt = max(0.01, now - last_time_pid)
                        last_time_pid = now
                        
                        # Numerical rates for RL state
                        if not hasattr(motor_control_thread_func, "last_raw_pitch"):
                            motor_control_thread_func.last_raw_pitch = pitch
                            motor_control_thread_func.last_raw_roll = roll
                        
                        pitch_rate = (pitch - motor_control_thread_func.last_raw_pitch) / dt
                        roll_rate = (roll - motor_control_thread_func.last_raw_roll) / dt
                        
                        motor_control_thread_func.last_raw_pitch = pitch
                        motor_control_thread_func.last_raw_roll = roll
                        
                        if state["rl_balancing_enabled"]:
                            # RL balance mode
                            reward = -(pitch**2 + roll**2)
                            rl_state = [pitch, roll, pitch_rate, roll_rate]
                            
                            if rl_prev_state is not None and rl_prev_action is not None:
                                rl_balancer.update(rl_prev_state, rl_prev_action, reward, rl_state, rl_prev_mean_action)
                                
                            action, mean_action = rl_balancer.select_action(rl_state, explore=True)
                            
                            rl_prev_state = rl_state
                            rl_prev_action = action
                            rl_prev_mean_action = mean_action
                            state["rl_last_reward"] = reward
                            state["rl_last_action"] = [float(action[0]), float(action[1])]
                            
                            pitch_offset_z = action[0]
                            roll_offset_z = action[1]
                        else:
                            # PID mode
                            rl_prev_state = None
                            rl_prev_action = None
                            rl_prev_mean_action = None
                            
                            Kp_pitch = 0.1
                            Ki_pitch = 0.05
                            Kd_pitch = 0.01
                            
                            Kp_roll = 0.1
                            Ki_roll = 0.05
                            Kd_roll = 0.01
                            
                            # Calculate errors (target is 0)
                            error_pitch = -pitch
                            error_roll = -roll
                            
                            integral_pitch += error_pitch * dt
                            integral_roll += error_roll * dt
                            
                            # Anti-windup
                            integral_pitch = max(-0.5, min(0.5, integral_pitch))
                            integral_roll = max(-0.5, min(0.5, integral_roll))
                            
                            deriv_pitch = (error_pitch - last_pitch) / dt
                            deriv_roll = (error_roll - last_roll) / dt
                            
                            last_pitch = error_pitch
                            last_roll = error_roll
                            
                            pitch_out = (error_pitch * Kp_pitch) + (integral_pitch * Ki_pitch) + (deriv_pitch * Kd_pitch)
                            roll_out = (error_roll * Kp_roll) + (integral_roll * Ki_roll) + (deriv_roll * Kd_roll)
                            
                            pitch_offset_z = max(-0.06, min(0.06, pitch_out))
                            roll_offset_z = max(-0.06, min(0.06, roll_out))
                except Exception:
                    pass
            
            state["pitch_offset_z_active"] = pitch_offset_z
            state["roll_offset_z_active"] = roll_offset_z

            solved_angles_list = [ [0.0,0.0,0.0] for _ in range(4) ]
            
            if state["active_mode"] == 'CALIB':
                for i in range(4):
                    solved_angles_list[i] = [0.0, 0.0, 0.0]
            else:
                for i in range(4):
                    # Z offset is always active based on IMU balancing
                    if i == 0:   # FL
                        z_offset = pitch_offset_z - roll_offset_z
                    elif i == 1: # FR
                        z_offset = pitch_offset_z + roll_offset_z
                    elif i == 2: # BL
                        z_offset = -pitch_offset_z - roll_offset_z
                    elif i == 3: # BR
                        z_offset = -pitch_offset_z + roll_offset_z

                    if state["active_leg"] != 'all' and i != int(state["active_leg"]):
                        y_val = state["target_y"]
                        if state["no_hip_walk"]:
                            is_right_walk = 1 if (i == 1 or i == 3) else 0
                            y_val += -solver.L1 if is_right_walk else solver.L1
                        res_i = solver.solve_ik_in_python(state["legs_target_x"][i], y_val, state["legs_target_z"][i] + z_offset, i)
                        solved_angles_list[i] = res_i.get('corrected', [0.0,0.0,0.0])
                    else:
                        if state["active_mode"] == 'IK' or state["active_leg"] == 'all':
                            y_val = state["target_y"]
                            if state["no_hip_walk"]:
                                is_right_walk = 1 if (i == 1 or i == 3) else 0
                                y_val += -solver.L1 if is_right_walk else solver.L1
                            
                            res_i = solver.solve_ik_in_python(state["legs_target_x"][i], y_val, state["legs_target_z"][i] + z_offset, i)
                            solved_angles_list[i] = res_i.get('corrected', [0.0,0.0,0.0])
                        else:
                            solved_angles_list[i] = state["target_fk_angles"]
                    
                    if state["no_hip_walk"]:
                        solved_angles_list[i][0] = 0.0

            # Calculate raw sending angles for all 4 legs
            current_time = time.time()
            all_raw_angles = [0.0] * 12
            for l in range(4):
                ctrl_leg = state["vis_to_ctrl"].get(l, 0)
                solved_angles = solved_angles_list[l]
                
                # 1. Hip (t1)
                THETA1 = max(math.radians(-22.0), min(math.radians(22.0), solved_angles[0]))
                hip_ang_deg = math.degrees(THETA1) + 90.0
                offset_h = state["offsets"][ctrl_leg * 3]
                inv_h = state["inversions"][ctrl_leg * 3]
                r_h = (180.0 - (hip_ang_deg + offset_h)) if inv_h else (hip_ang_deg + offset_h)
                raw_h = int(round(max(0.0, min(180.0, r_h))))
                
                # 2. Thigh (t2)
                THETA2 = max(math.radians(0.0), min(math.radians(160.0), solved_angles[1]))
                up_ang_deg = math.degrees(THETA2)
                offset_u = state["offsets"][ctrl_leg * 3 + 1]
                inv_u = state["inversions"][ctrl_leg * 3 + 1]
                r_u = (180.0 - (up_ang_deg + offset_u)) if inv_u else (up_ang_deg + offset_u)
                raw_u = int(round(max(0.0, min(180.0, r_u))))
                
                # 3. Calf (t3)
                THETA3 = max(math.radians(-80.0), min(math.radians(80.0), solved_angles[2]))
                if state["active_mode"] == 'CALIB':
                    compensated_l = 90.0
                else:
                    THETA0 = solver.lower_leg_angle_to_servo_angle(math.pi/2.0 - THETA2, THETA3 + math.pi/2.0)
                    compensated_l = math.degrees(math.pi/2.0 + math.pi - THETA0)
                offset_l = state["offsets"][ctrl_leg * 3 + 2]
                inv_l = state["inversions"][ctrl_leg * 3 + 2]
                r_l = (180.0 - (compensated_l + offset_l)) if inv_l else (compensated_l + offset_l)
                raw_l = int(round(max(0.0, min(180.0, r_l))))
                
                all_raw_angles[ctrl_leg * 3] = raw_h
                all_raw_angles[ctrl_leg * 3 + 1] = raw_u
                all_raw_angles[ctrl_leg * 3 + 2] = raw_l
                
                state["last_target_raw"][ctrl_leg * 3] = raw_h
                state["last_target_raw"][ctrl_leg * 3 + 1] = raw_u
                state["last_target_raw"][ctrl_leg * 3 + 2] = raw_l

                # Save software angles (logical angles in degrees, before offsets and inversions)
                state["sw_angles"][0][l] = int(round(math.degrees(solved_angles[0])))
                state["sw_angles"][1][l] = int(round(math.degrees(solved_angles[1])))
                state["sw_angles"][2][l] = int(round(math.degrees(solved_angles[2])))

            # Calculate heights for all 4 legs (Z coordinates in control leg order: FR, FL, BR, BL)
            heights_4 = [0.0] * 4
            for l in [1, 0, 3, 2]:
                h_idx = 0 if l == 1 else (1 if l == 0 else (2 if l == 3 else 3))
                try:
                    fk_res = solver.solve_fk_in_python(solved_angles_list[l][0], solved_angles_list[l][1], solved_angles_list[l][2], l)
                    heights_4[h_idx] = fk_res['joints']['foot'][2]
                except Exception:
                    heights_4[h_idx] = state["legs_target_z"][l]

            # Write to CSV if enabled
            if state["csv_logging_enabled"]:
                log_angles_to_csv(state, all_raw_angles, heights_4)

            # 3. Apply to physical servos if enabled
            if state["hw_enabled"]:
                if nano_controller:
                    nano_controller.enable_soft_motion(state["use_soft_motion"])
                    nano_controller.enable_feedback(state["use_soft_motion"], solver)
                
                legs_to_write = range(4) if state["active_leg"] == 'all' else [int(state["active_leg"])]
                
                for l in legs_to_write:
                    ctrl_leg = state["vis_to_ctrl"].get(l, 0)
                    raw_h = all_raw_angles[ctrl_leg * 3]
                    raw_u = all_raw_angles[ctrl_leg * 3 + 1]
                    raw_l = all_raw_angles[ctrl_leg * 3 + 2]
                    
                    j_idx_h = ctrl_leg * 3 + 0
                    if abs(raw_h - state["last_undithered_targets"][j_idx_h]) > 0.01:
                        state["last_undithered_targets"][j_idx_h] = raw_h
                        state["last_target_change_times"][j_idx_h] = current_time
                    if state["dither_enabled"] and state["active_leg"] == 'all' and (current_time - state["last_target_change_times"][j_idx_h] >= 1.0):
                        dither_val = 1.0 if (int(current_time * 20) % 2 == 0) else 0.0
                        raw_h = max(0.0, min(180.0, raw_h + dither_val))
                    
                    j_idx_u = ctrl_leg * 3 + 1
                    if abs(raw_u - state["last_undithered_targets"][j_idx_u]) > 0.01:
                        state["last_undithered_targets"][j_idx_u] = raw_u
                        state["last_target_change_times"][j_idx_u] = current_time
                    if state["dither_enabled"] and state["active_leg"] == 'all' and (current_time - state["last_target_change_times"][j_idx_u] >= 1.0):
                        dither_val = 1.0 if (int(current_time * 20) % 2 == 0) else 0.0
                        raw_u = max(0.0, min(180.0, raw_u + dither_val))
                    
                    j_idx_l = ctrl_leg * 3 + 2
                    if abs(raw_l - state["last_undithered_targets"][j_idx_l]) > 0.01:
                        state["last_undithered_targets"][j_idx_l] = raw_l
                        state["last_target_change_times"][j_idx_l] = current_time
                    if state["dither_enabled"] and state["active_leg"] == 'all' and (current_time - state["last_target_change_times"][j_idx_l] >= 1.0):
                        dither_val = 1.0 if (int(current_time * 20) % 2 == 0) else 0.0
                        raw_l = max(0.0, min(180.0, raw_l + dither_val))
                    
                    all_raw_angles[ctrl_leg * 3] = raw_h
                    all_raw_angles[ctrl_leg * 3 + 1] = raw_u
                    all_raw_angles[ctrl_leg * 3 + 2] = raw_l
                    
                    state["last_target_raw"][ctrl_leg * 3] = raw_h
                    state["last_target_raw"][ctrl_leg * 3 + 1] = raw_u
                    state["last_target_raw"][ctrl_leg * 3 + 2] = raw_l
                    
                    if nano_controller:
                        nano_controller.set_target_angle(ctrl_leg, 0, raw_h, pre_adjusted=True)
                        nano_controller.set_target_angle(ctrl_leg, 1, raw_u, pre_adjusted=True)
                        nano_controller.set_target_angle(ctrl_leg, 2, raw_l, pre_adjusted=True)
                
                if state["ctrl_only_selected"] and state["active_leg"] != 'all':
                    active_l_idx = int(state["active_leg"])
                    for l in range(4):
                        if l != active_l_idx:
                            ctrl_leg = state["vis_to_ctrl"].get(l, 0)
                            if nano_controller:
                                nano_controller.target_angles[JOINT_TO_CHANNEL[ctrl_leg * 3]] = None
                                nano_controller.target_angles[JOINT_TO_CHANNEL[ctrl_leg * 3 + 1]] = None
                                nano_controller.target_angles[JOINT_TO_CHANNEL[ctrl_leg * 3 + 2]] = None
            else:
                if nano_controller:
                    for ch in JOINT_TO_CHANNEL:
                        nano_controller.target_angles[ch] = None
                        
        time.sleep(0.02)

def start_control_loops(state):
    # Start main motor control loop
    t = threading.Thread(target=motor_control_thread_func, args=(state,), daemon=True)
    t.start()
