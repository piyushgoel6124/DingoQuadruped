# coding: utf-8
import os
import json
import time
import serial
import serial.tools.list_ports
import threading
import atexit

# PIN_TO_JOINT mapping PCA9685 channels back to leg_idx, joint_idx
PIN_TO_JOINT = {
    14: (0, 0), 13: (0, 1), 12: (0, 2), # FR
    10: (1, 0), 9: (1, 1), 8: (1, 2),   # FL
    2: (2, 0), 1: (2, 1), 0: (2, 2),    # BR
    6: (3, 0), 5: (3, 1), 4: (3, 2)     # BL
}

JOINT_TO_CHANNEL = [14, 13, 12, 10, 9, 8, 2, 1, 0, 6, 5, 4]

vis_to_ctrl = {
    0: 1,  # FL
    1: 0,  # FR
    2: 3,  # BL
    3: 2   # BR
}

class NanoController:
    def __init__(self, inversions):
        self.inversions = inversions
        self.ports = []
        self.connections = {}
        self.latest_analogs = {}  # {name: [0,0,0,0,0,0]}
        self.lock = threading.Lock()
        self.running = True
        self.threads = []
        self.servo_angles = {
            "x1": [90, 90, 90, 90, 90, 90],
            "x2": [90, 90, 90, 90, 90, 90]
        }
        
        self.target_angles = [None] * 16
        self.current_angles = [None] * 16
        self.use_soft_motion = False
        self.use_feedback = False
        self.solver = None
        
        self.config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "nano_calibration_config.json")
        self.mapping = {
            "x1 D3": 1, # FL
            "x1 D6": 3, # BL
            "x2 D3": 0, # FR
            "x2 D6": 2  # BR
        }
        self.load_config()
        self.detect_and_connect()
        
        # Start background soft motion thread
        t_motion = threading.Thread(target=self._soft_motion_loop, daemon=True)
        t_motion.start()

    def get_adjusted_angle(self, leg_idx, joint_idx, angle):
        user_inverted = False
        try:
            user_inverted = bool(self.inversions[joint_idx, leg_idx])
        except Exception:
            pass
        
        if user_inverted:
            return 180 - angle
        return angle

    def load_config(self):
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, "r") as f:
                    config = json.load(f)
                    self.mapping = config.get("mapping", self.mapping)
            except Exception as e:
                print(f"Error loading config: {e}")

    def save_config(self):
        try:
            with open(self.config_file, "w") as f:
                json.dump({"mapping": self.mapping}, f)
        except Exception as e:
            print(f"Error saving config: {e}")

    def detect_and_connect(self):
        atexit.register(self.shutdown)

        ports = [p.device for p in serial.tools.list_ports.comports() 
                 if "usb" in p.device.lower() or "ttyusb" in p.device.lower() or "ttyacm" in p.device.lower()]
        ports.sort()
        self.ports = ports
        print(f"Detected serial ports: {self.ports}")
        
        for idx, port in enumerate(self.ports[:2]):
            try:
                # Open with slightly longer timeout for initial handshake query
                ser = serial.Serial(port, 115200, timeout=0.8)
                
                # Wait for Arduino bootloader to complete execution after serial DTR reset
                time.sleep(1.8)
                
                # Kickstart and request board ID
                ser.write(b"START\n")
                ser.flush()
                time.sleep(0.15)
                ser.write(b"ID\n")
                ser.flush()
                
                # Try reading up to 5 lines to find the ID header
                name = f"Nano {idx+1}"
                for _ in range(5):
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("ID:"):
                        board_id = line.split("ID:")[1]
                        if board_id in ["x1", "x2"]:
                            name = board_id
                        break
                
                # Re-set timeout to 0.1 for normal non-blocking loop reads
                ser.timeout = 0.1
                
                self.connections[name] = ser
                self.latest_analogs[name] = [0,0,0,0,0,0]
                print(f"Connected to {name} at {port}")
                
                t = threading.Thread(target=self.read_serial_loop, args=(name, ser), daemon=True)
                t.start()
                self.threads.append(t)
            except Exception as e:
                print(f"Failed to connect at {port}: {e}")

    def read_serial_loop(self, name, ser):
        last_heartbeat = 0
        while self.running:
            try:
                # Send a heartbeat every 2 seconds to keep connection active
                now = time.time()
                if now - last_heartbeat > 2.0:
                    try:
                        ser.write(b"START\n")
                        ser.flush()
                    except Exception:
                        pass
                    last_heartbeat = now

                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    parts = line.split(',')
                    if len(parts) == 6:
                        vals = [int(p) for p in parts]
                        with self.lock:
                            self.latest_analogs[name] = vals
            except Exception as e:
                time.sleep(0.01)

    def shutdown(self):
        self.running = False
        for name, ser in list(self.connections.items()):
            try:
                ser.write(b"STOP\n")
                ser.flush()
                ser.close()
                print(f"Sent STOP and closed connection to {name}")
            except Exception:
                pass

    def write_angles(self, name, angles):
        if name in self.connections:
            cmd = ",".join(str(int(a)) for a in angles) + "\n"
            try:
                self.connections[name].write(cmd.encode('utf-8'))
            except Exception as e:
                pass

    def get_joint_routing(self, leg_idx, joint_idx):
        for nano in ["x1", "x2"]:
            for pin_key, pin_offset in [("D3", 0), ("D6", 3)]:
                key = f"{nano} {pin_key}"
                if key in self.mapping and self.mapping[key] == leg_idx:
                    return nano, joint_idx + pin_offset
        return None

    def set_servo_angle(self, leg_idx, joint_idx, angle):
        routing = self.get_joint_routing(leg_idx, joint_idx)
        if routing:
            nano_name, pin_index = routing
            adjusted = self.get_adjusted_angle(leg_idx, joint_idx, angle)
            adjusted = max(0, min(180, adjusted))
            self.servo_angles[nano_name][pin_index] = adjusted
            self.write_angles(nano_name, self.servo_angles[nano_name])

    def get_adc_values(self):
        adc_grid = [[0]*4 for _ in range(3)]
        with self.lock:
            for leg_idx in range(4):
                for joint_idx in range(3):
                    routing = self.get_joint_routing(leg_idx, joint_idx)
                    if routing:
                        nano_name, pin_index = routing
                        if nano_name in self.latest_analogs:
                            adc_grid[joint_idx][leg_idx] = self.latest_analogs[nano_name][pin_index]
            nano1_d3 = self.latest_analogs.get("x1", [0]*6)[0]
            nano2_d3 = self.latest_analogs.get("x2", [0]*6)[0]
        return adc_grid, nano1_d3, nano2_d3

    def get_connection_status(self):
        return {
            "ports": self.ports,
            "connected": list(self.connections.keys()),
            "mapping": self.mapping
        }

    def wiggle_d3(self, nano_name):
        def run_wiggle():
            conn_name = nano_name
            if conn_name not in self.connections:
                if conn_name == "x1" and "Nano 1" in self.connections:
                    conn_name = "Nano 1"
                elif conn_name == "x2" and "Nano 2" in self.connections:
                    conn_name = "Nano 2"
                else:
                    return
            orig_val = self.servo_angles[conn_name][0]
            for _ in range(3):
                self.servo_angles[conn_name][0] = 80
                self.write_angles(conn_name, self.servo_angles[conn_name])
                time.sleep(0.3)
                self.servo_angles[conn_name][0] = 100
                self.write_angles(conn_name, self.servo_angles[conn_name])
                time.sleep(0.3)
            self.servo_angles[conn_name][0] = orig_val
            self.write_angles(conn_name, self.servo_angles[conn_name])
        threading.Thread(target=run_wiggle, daemon=True).start()

    def enable_soft_motion(self, enabled):
        self.use_soft_motion = enabled

    def enable_feedback(self, enabled, solver=None):
        self.use_feedback = enabled
        self.solver = solver

    def set_target_angle(self, leg_idx, joint_idx, angle, pre_adjusted=False):
        # Maps joint to channel
        ch_list = [14, 13, 12] if leg_idx == 0 else \
                  [10, 9, 8] if leg_idx == 1 else \
                  [2, 1, 0] if leg_idx == 2 else \
                  [6, 5, 4]
        ch = ch_list[joint_idx]
        if not pre_adjusted:
            angle = self.get_adjusted_angle(leg_idx, joint_idx, angle)
        self.target_angles[ch] = float(angle)

    def set_target_angles(self, angles_matrix, pre_adjusted=False):
        for leg_idx in range(4):
            for joint_idx in range(3):
                self.set_target_angle(leg_idx, joint_idx, angles_matrix[joint_idx][leg_idx], pre_adjusted=pre_adjusted)

    def _soft_motion_loop(self):
        last_time = time.time()
        while self.running:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            # Clamp dt to reasonable bounds to handle thread pause/jumps
            dt = min(0.1, max(0.001, dt))
            
            if any(t is not None for t in self.target_angles):
                dirty_nanos = set()
                
                for ch in JOINT_TO_CHANNEL:
                    if self.target_angles[ch] is None:
                        if self.current_angles[ch] is not None:
                            self.current_angles[ch] = None
                            if ch in PIN_TO_JOINT:
                                ctrl_leg, joint_idx = PIN_TO_JOINT[ch]
                                routing = self.get_joint_routing(ctrl_leg, joint_idx)
                                if routing:
                                    nano_name, pin_index = routing
                                    self.servo_angles[nano_name][pin_index] = 404
                                    dirty_nanos.add(nano_name)
                    else:
                        if self.current_angles[ch] is None:
                            self.current_angles[ch] = self.target_angles[ch]
                            if ch in PIN_TO_JOINT:
                                ctrl_leg, joint_idx = PIN_TO_JOINT[ch]
                                routing = self.get_joint_routing(ctrl_leg, joint_idx)
                                if routing:
                                    nano_name, pin_index = routing
                                    self.servo_angles[nano_name][pin_index] = int(self.current_angles[ch])
                                    dirty_nanos.add(nano_name)
                        else:
                            error = self.target_angles[ch] - self.current_angles[ch]
                            if abs(error) > 0.05:
                                # Max speed: 180 degrees in 2.5 seconds = 72 degrees / second
                                max_step = 72.0 * dt
                                
                                if self.use_soft_motion:
                                    # Smooth ease-out proportional step capped by max_step
                                    step = min(max_step, max(0.2, abs(error) * 0.15))
                                else:
                                    # Hard rate-limiting safety limit
                                    step = max_step
                                    
                                if error > 0:
                                    self.current_angles[ch] = min(self.target_angles[ch], self.current_angles[ch] + step)
                                else:
                                    self.current_angles[ch] = max(self.target_angles[ch], self.current_angles[ch] - step)
                                    
                                if ch in PIN_TO_JOINT:
                                    ctrl_leg, joint_idx = PIN_TO_JOINT[ch]
                                    routing = self.get_joint_routing(ctrl_leg, joint_idx)
                                    if routing:
                                        nano_name, pin_index = routing
                                        self.servo_angles[nano_name][pin_index] = int(self.current_angles[ch])
                                        dirty_nanos.add(nano_name)
                            elif self.use_feedback and self.solver and ch in PIN_TO_JOINT:
                                # Error is small. Comp for friction using ADC feedback.
                                ctrl_leg, joint_idx = PIN_TO_JOINT[ch]
                                routing = self.get_joint_routing(ctrl_leg, joint_idx)
                                if routing:
                                    adc_grid, _, _ = self.get_adc_values()
                                    current_adc = adc_grid[joint_idx][ctrl_leg] if adc_grid else 0
                                    expected_adc = self.solver.estimate_adc_from_angle(ctrl_leg, joint_idx, self.current_angles[ch])
                                    
                                    if abs(current_adc - expected_adc) > 3:
                                        actual_angle = self.solver.estimate_angle_from_adc(ctrl_leg, joint_idx, current_adc)
                                        angle_error = self.current_angles[ch] - actual_angle
                                        Kp = 1.0
                                        adjusted_angle = self.current_angles[ch] + Kp * angle_error
                                        adjusted_angle = max(0.0, min(180.0, adjusted_angle))
                                        
                                        self.servo_angles[nano_name][pin_index] = int(adjusted_angle)
                                        dirty_nanos.add(nano_name)
                                        
                for nano_name in dirty_nanos:
                    try:
                        self.write_angles(nano_name, self.servo_angles[nano_name])
                    except Exception:
                        pass
            time.sleep(0.02)
