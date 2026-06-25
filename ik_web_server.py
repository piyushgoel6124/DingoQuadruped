# coding: utf-8
import os
import json
import math
import webbrowser
import threading
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

class IKHTTPRequestHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        return
        
    def do_GET(self):
        state = self.server.state
        if self.path == "/" or self.path == "/index.html" or self.path == "/ik_visualizer.html":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            with open(os.path.join(state["base_dir"], "ik_visualizer.html"), "rb") as f:
                self.wfile.write(f.read())
        elif self.path == "/api/get_csv_log":
            try:
                lines_data = []
                headers = []
                if os.path.exists(state["csv_log_path"]):
                    with state["csv_lock"]:
                        with open(state["csv_log_path"], "r") as f:
                            lines = f.readlines()
                    if len(lines) > 0:
                        headers = lines[0].strip().split(",")
                        data_lines = lines[1:]
                        last_lines = data_lines[-150:]
                        for line in last_lines:
                            parts = line.strip().split(",")
                            if len(parts) == len(headers):
                                try:
                                    row = {}
                                    for i, part in enumerate(parts):
                                        row[headers[i]] = float(part)
                                    lines_data.append(row)
                                except ValueError:
                                    pass
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"status": "ok", "headers": headers, "data": lines_data}).encode('utf-8'))
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"status": "error", "message": str(e)}).encode('utf-8'))
        else:
            self.send_error(404, "Not Found")

    def do_POST(self):
        state = self.server.state
        if self.path == "/api/clear_csv_log":
            try:
                with state["csv_lock"]:
                    if os.path.exists(state["csv_log_path"]):
                        os.remove(state["csv_log_path"])
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"status": "ok"}).encode('utf-8'))
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"status": "error", "message": str(e)}).encode('utf-8'))
        elif self.path == "/api/update_state":
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length)
            try:
                data = json.loads(body.decode('utf-8'))
                
                with state["state_lock"]:
                    if "rl_balancing_enabled" in data:
                        state["rl_balancing_enabled"] = bool(data["rl_balancing_enabled"])
                    if "mode" in data:
                        state["active_mode"] = data["mode"]
                    if "leg_index" in data:
                        state["active_leg"] = data["leg_index"]
                    if "demo_active" in data:
                        state["demo_running"] = bool(data["demo_active"])
                    
                    if "target_coords" in data:
                        state["target_x"] = float(data["target_coords"][0])
                        state["target_y"] = float(data["target_coords"][1])
                        state["target_z"] = float(data["target_coords"][2])
                        
                    if "target_angles" in data:
                        state["target_fk_angles"] = [float(a) for a in data["target_angles"]]
                        
                    state["hw_enabled"] = bool(data.get("hardware_control_enabled", state["hw_enabled"]))
                    state["ctrl_only_selected"] = bool(data.get("control_only_this_leg", state["ctrl_only_selected"]))
                    state["use_soft_motion"] = bool(data.get("use_soft_motion", state["use_soft_motion"]))
                    state["no_hip_walk"] = bool(data.get("no_hip_walk", state["no_hip_walk"]))
                    state["dither_enabled"] = bool(data.get("dither_enabled", state["dither_enabled"]))
                    state["csv_logging_enabled"] = bool(data.get("csv_logging_enabled", state["csv_logging_enabled"]))

                    if "walk_front" in data:
                        state["walk_front"] = float(data["walk_front"])
                    if "walk_back" in data:
                        state["walk_back"] = float(data["walk_back"])
                    if "walk_lift" in data:
                        state["walk_lift"] = float(data["walk_lift"])
                    if "walk_speed" in data:
                        state["walk_speed"] = float(data["walk_speed"])
                    if "walk_height" in data:
                        state["walk_height"] = float(data["walk_height"])
                    if "diagonal_lift" in data:
                        state["diagonal_lift"] = float(data["diagonal_lift"])
                    
                    solver = state["solver"]
                    if "config_dims" in data:
                        config_dims = data["config_dims"]
                        solver.L1 = float(config_dims.get("L1", solver.L1))
                        solver.L2 = float(config_dims.get("L2", solver.L2))
                        solver.L3 = float(config_dims.get("L3", solver.L3))
                        solver.phi = float(config_dims.get("phi", solver.phi))

                    # Solve kinematics depending on whether we want single active leg or all legs
                    if state["active_mode"] == 'CALIB':
                        legs_joints_data = []
                        for i in range(4):
                            res_i = solver.solve_fk_in_python(0.0, 0.0, 0.0, i)
                            offset_i = state["LEG_ORIGINS_3D"][i]
                            offset_joints = {}
                            for key, val in res_i['joints'].items():
                                offset_joints[key] = [
                                    val[0] + offset_i[0],
                                    val[1] + offset_i[1],
                                    val[2] + offset_i[2]
                                ]
                            legs_joints_data.append({
                                "joints": offset_joints,
                                "corrected": [0.0, 0.0, 0.0],
                                "angles": [0.0, 0.0, 0.0]
                            })
                        res = solver.solve_fk_in_python(0.0, 0.0, 0.0, 0 if state["active_leg"] == 'all' else int(state["active_leg"]))
                        res['corrected'] = [0.0, 0.0, 0.0]
                        res['angles'] = [0.0, 0.0, 0.0]
                        res['is_out_of_bounds'] = False
                        res['steps'] = {}
                    elif state["active_leg"] == 'all':
                        pitch_offset_z = state["pitch_offset_z_active"]
                        roll_offset_z = state["roll_offset_z_active"]

                        legs_joints_data = []
                        for i in range(4):
                            y_val = state["target_y"]
                            if state["no_hip_walk"]:
                                is_right_walk = 1 if (i == 1 or i == 3) else 0
                                y_val += -solver.L1 if is_right_walk else solver.L1
                                
                            if i == 0:   # FL
                                z_offset = pitch_offset_z - roll_offset_z
                            elif i == 1: # FR
                                z_offset = pitch_offset_z + roll_offset_z
                            elif i == 2: # BL
                                z_offset = -pitch_offset_z - roll_offset_z
                            elif i == 3: # BR
                                z_offset = -pitch_offset_z + roll_offset_z
                                
                            res_i = solver.solve_ik_in_python(state["legs_target_x"][i], y_val, state["legs_target_z"][i] + z_offset, i)
                            offset_i = state["LEG_ORIGINS_3D"][i]
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
                            
                        res = {
                            'joints': {},
                            'corrected': [0.0, 0.0, 0.0],
                            'angles': [0.0, 0.0, 0.0],
                            'is_out_of_bounds': False,
                            'steps': {}
                        }
                    else:
                        idx = int(state["active_leg"])
                        if state["active_mode"] == 'IK':
                            y_val = state["target_y"]
                            if state["no_hip_walk"]:
                                is_right_walk = 1 if (idx == 1 or idx == 3) else 0
                                y_val += -solver.L1 if is_right_walk else solver.L1
                            res = solver.solve_ik_in_python(state["target_x"], y_val, state["target_z"], idx)
                        else:
                            res = solver.solve_fk_in_python(state["target_fk_angles"][0], state["target_fk_angles"][1], state["target_fk_angles"][2], idx)
                            res['corrected'] = state["target_fk_angles"]
                            res['angles'] = state["target_fk_angles"]
                            res['is_out_of_bounds'] = False
                            res['steps'] = {}
                        legs_joints_data = []

                raw_angles = [0.0, 0.0, 0.0]
                if state["hw_enabled"] and state["active_leg"] != 'all':
                    ctrl_leg = state["vis_to_ctrl"].get(int(state["active_leg"]), 0)
                    raw_angles = [
                        round(state["last_target_raw"][ctrl_leg * 3], 1),
                        round(state["last_target_raw"][ctrl_leg * 3 + 1], 1),
                        round(state["last_target_raw"][ctrl_leg * 3 + 2], 1)
                    ]
                
                adc_grid = [[0]*4 for _ in range(3)]
                real_angles = [[0.0]*4 for _ in range(3)]
                hw_angles = [[0.0]*4 for _ in range(3)]
                if state["nano_controller"]:
                    adc_grid, _, _ = state["nano_controller"].get_adc_values()
                    for l in range(4):
                        for j in range(3):
                            real_angles[j][l] = int(round(solver.estimate_angle_from_adc(l, j, adc_grid[j][l])))
                            hw_angles[j][l] = int(round(state["last_target_raw"][l * 3 + j]))
                            
                imu_euler = [0.0, 0.0, 0.0]
                if state["imu_sensor"]:
                    try:
                        euler = state["imu_sensor"].euler
                        if euler and len(euler) == 3:
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
                            
                            imu_euler = [
                                0.0,
                                math.radians(roll_raw),
                                math.radians(pitch_raw)
                            ]
                    except Exception:
                        pass
                
                response_data = {
                    "status": "ok",
                    "view_mode": "full" if state["active_leg"] == "all" else "single",
                    "legs_joints": legs_joints_data,
                    "angles": res.get('angles', [0.0, 0.0, 0.0]),
                    "corrected": res.get('corrected', [0.0, 0.0, 0.0]),
                    "isOutOfBounds": res.get('is_out_of_bounds', False),
                    "joints": res['joints'],
                    "steps": res.get('steps', {}),
                    "target_coords": [state["target_x"], state["target_y"], state["target_z"]],
                    "raw_angles": raw_angles,
                    "adc_grid": adc_grid,
                    "real_angles": real_angles,
                    "hw_angles": hw_angles,
                    "sw_angles": state["sw_angles"],
                    "imu_euler": imu_euler,
                    "csv_logging_enabled": state["csv_logging_enabled"],
                    "rl_balancing_enabled": state["rl_balancing_enabled"],
                    "rl_steps": state["rl_balancer"].total_steps,
                    "rl_reward": state["rl_last_reward"],
                    "rl_action": state["rl_last_action"]
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

class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    def __init__(self, server_address, RequestHandlerClass, state):
        self.state = state
        super().__init__(server_address, RequestHandlerClass)

def start_web_server(state, port=8081):
    # Bind strictly to localhost/127.0.0.1 for testing security compliance
    # TODO(security): Allow binding to public interface if remote visualization is requested
    server_address = ('127.0.0.1', port)
    httpd = ReusableThreadingHTTPServer(server_address, IKHTTPRequestHandler, state)
    print(f"Starting Inverse Kinematics Visualizer Web Server on port {port}...")
    print(f"Open http://127.0.0.1:{port} in your browser.")
    try:
        webbrowser.open(f"http://127.0.0.1:{port}")
    except Exception:
        pass
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down server.")
