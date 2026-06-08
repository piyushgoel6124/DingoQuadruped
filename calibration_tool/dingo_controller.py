#!/usr/bin/env python3
"""
Dingo Quadruped - Persistent Calibration & Over-the-Wire Controller Client

This utility allows you to establish a serial communication session with the Arduino Uno
to perform real-time joint calibration and control the Dingo Quadruped over a wired USB connection.
It retrieves, edits, and commits calibration offsets to the Arduino Uno's persistent EEPROM.
"""

import sys
import time
import serial
import serial.tools.list_ports

# Joint descriptions and hardware mapping
JOINT_NAMES = [
    "FR Hip (Ch 14)", "FR Upper (Ch 13)", "FR Lower (Ch 12)",
    "FL Hip (Ch 10)", "FL Upper (Ch  9)", "FL Lower (Ch  8)",
    "BR Hip (Ch  2)", "BR Upper (Ch  1)", "BR Lower (Ch  0)",
    "BL Hip (Ch  6)", "BL Upper (Ch  5)", "BL Lower (Ch  4)"
]

# Predefined poses (12 angles, joint 0 to 11)
POSES = {
    "Neutral":  [90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90],
    "Stand":    [90, 60, 45, 90, 60, 45, 90, 60, 45, 90, 60, 45],
    "Sit":      [90, 90, 90, 90, 90, 90, 90, 30, 20, 90, 30, 20],
    "Lay Down": [90, 15, 10, 90, 15, 10, 90, 15, 10, 90, 15, 10]
}

class DingoController:
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.offsets = [0] * 12
        self.angles = [90.0] * 12

    def connect(self):
        try:
            print(f"Connecting to Arduino on {self.port} at {self.baudrate} baud...")
            # Open serial port with 2 seconds timeout
            self.ser = serial.Serial(self.port, self.baudrate, timeout=2.0)
            # Arduino Uno auto-resets on connection; wait for it to boot up
            time.sleep(2.0)
            self.ser.reset_input_buffer()
            self.query_state()
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial connection closed.")

    def send_cmd(self, cmd_str):
        if not self.ser or not self.ser.is_open:
            print("Error: Serial port is not open.")
            return None
        try:
            full_cmd = f"{cmd_str}\n"
            self.ser.write(full_cmd.encode('ascii'))
            self.ser.flush()
            # Read responses
            response = self.ser.readline().decode('ascii').strip()
            return response
        except Exception as e:
            print(f"Write error: {e}")
            return None

    def query_state(self):
        # Query persistent offsets from Arduino
        try:
            self.ser.write(b"P\n")
            self.ser.flush()
            time.sleep(0.1)
            # Find state response in buffer
            for _ in range(5):
                line = self.ser.readline().decode('ascii').strip()
                if line.startswith("STATE"):
                    parts = line.split()
                    if len(parts) == 13:
                        self.offsets = [int(p) for p in parts[1:]]
                        print("Successfully synchronized persistent offsets from Arduino EEPROM.")
                        return
        except Exception as e:
            print(f"Could not query calibration state: {e}")

    def display_status(self):
        print("\n================ DINGO JOINT STATUS ================")
        print(f"Port: {self.port} | Connection: Connected")
        print("----------------------------------------------------")
        print("Idx | Joint Description   | Offset (deg) | Cur Angle")
        print("----------------------------------------------------")
        for i in range(12):
            print(f"{i:2d}  | {JOINT_NAMES[i]:19s} | {self.offsets[i]:12d} | {self.angles[i]:9.1f}")
        print("====================================================")

    def calibrate_joint(self, joint_idx):
        if joint_idx < 0 or joint_idx >= 12:
            print("Invalid joint index.")
            return

        print(f"\n--- Calibrating: {JOINT_NAMES[joint_idx]} ---")
        print("Commands:")
        print("  [+] Increase offset by 1 deg")
        print("  [-] Decrease offset by 1 deg")
        print("  [value] Set exact numeric offset (e.g. 5, -12)")
        print("  [q] Finish calibration and return")
        
        while True:
            cur_offset = self.offsets[joint_idx]
            print(f"Current Offset: {cur_offset:d} deg | Target Angle: {self.angles[joint_idx]:.1f} deg")
            user_input = input("Calibration command > ").strip()
            
            if not user_input:
                continue
            
            if user_input.lower() == 'q':
                break
            
            new_offset = cur_offset
            if user_input == '+':
                new_offset += 1
            elif user_input == '-':
                new_offset -= 1
            else:
                try:
                    new_offset = int(user_input)
                except ValueError:
                    print("Invalid input. Use '+', '-', an integer offset, or 'q'.")
                    continue
            
            # Send the new offset command
            resp = self.send_cmd(f"C {joint_idx} {new_offset}")
            if resp and resp.startswith("ACK C"):
                self.offsets[joint_idx] = new_offset
                print(f"Offset updated: {new_offset:d} deg")
            else:
                print(f"Arduino error: {resp}")

    def move_joint(self, joint_idx, angle):
        if joint_idx < 0 or joint_idx >= 12:
            print("Invalid joint index.")
            return
        if angle < 0.0 or angle > 180.0:
            print("Target angle must be between 0 and 180 degrees.")
            return
        
        resp = self.send_cmd(f"M {joint_idx} {angle}")
        if resp and resp.startswith("ACK M"):
            self.angles[joint_idx] = angle
            print(f"Actuated {JOINT_NAMES[joint_idx]} to {angle:.1f} degrees.")
        else:
            print(f"Arduino error: {resp}")

    def set_pose(self, pose_name):
        if pose_name not in POSES:
            print("Unknown pose.")
            return
        
        pose_angles = POSES[pose_name]
        cmd_str = "A" + "".join([f" {a}" for a in pose_angles])
        resp = self.send_cmd(cmd_str)
        if resp and resp.startswith("ACK A"):
            self.angles = list(pose_angles)
            print(f"Actuated to pose: {pose_name}")
        else:
            print(f"Arduino error: {resp}")

    def save_to_eeprom(self):
        resp = self.send_cmd("S")
        if resp and "ACK S" in resp:
            print("Success: Calibration offsets saved persistently to Arduino EEPROM.")
        else:
            print(f"Failed to save calibration: {resp}")

    def reload_from_eeprom(self):
        resp = self.send_cmd("L")
        if resp and "ACK L" in resp:
            self.query_state()
            print("Success: Reloaded calibration offsets from Arduino EEPROM.")
        else:
            print(f"Failed to load calibration: {resp}")


def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]


def main():
    print("====================================================")
    print(" Dingo Quadruped Wire Calibration & Control Utility ")
    print("====================================================")
    
    ports = list_serial_ports()
    if not ports:
        print("No serial ports found. Connect your Arduino Uno and try again.")
        sys.exit(1)
        
    print("Available serial ports:")
    for idx, port in enumerate(ports):
        print(f"  {idx}: {port}")
        
    try:
        port_sel = int(input("Select port index > "))
        selected_port = ports[port_sel]
    except (ValueError, IndexError):
        print("Invalid selection. Exiting.")
        sys.exit(1)

    controller = DingoController(selected_port)
    if not controller.connect():
        sys.exit(1)

    try:
        while True:
            controller.display_status()
            print("\nOptions:")
            print("  1: Calibrate a specific joint (persistent calibration)")
            print("  2: Move a specific joint")
            print("  3: Trigger a predefined pose")
            print("  4: Save current calibration permanently to EEPROM")
            print("  5: Reload calibration from EEPROM")
            print("  6: Exit")
            
            try:
                choice = input("\nSelect choice (1-6) > ").strip()
                if choice == '1':
                    idx = int(input("Enter joint index to calibrate (0-11) > "))
                    controller.calibrate_joint(idx)
                elif choice == '2':
                    idx = int(input("Enter joint index to move (0-11) > "))
                    angle = float(input("Enter target angle in degrees (0-180) > "))
                    controller.move_joint(idx, angle)
                elif choice == '3':
                    print("Predefined poses:")
                    pose_keys = list(POSES.keys())
                    for p_idx, pose in enumerate(pose_keys):
                        print(f"  {p_idx}: {pose}")
                    p_sel = int(input("Select pose index > "))
                    controller.set_pose(pose_keys[p_sel])
                elif choice == '4':
                    controller.save_to_eeprom()
                elif choice == '5':
                    controller.reload_from_eeprom()
                elif choice == '6':
                    print("Exiting controller utility.")
                    break
                else:
                    print("Invalid choice.")
            except ValueError:
                print("Invalid input. Please enter a valid number.")
            except KeyboardInterrupt:
                print("\nInterrupted by user.")
                break
            
            input("\nPress Enter to continue...")
    finally:
        controller.close()


if __name__ == "__main__":
    main()
