#!/usr/bin/env python3
import time
import json
import os
import board
import adafruit_bno055
import sys
import select

def get_latest_euler(sensor):
    # Try a few times to get a valid reading, sometimes BNO055 returns None
    for _ in range(10):
        euler = sensor.euler
        if euler and all(x is not None for x in euler):
            return euler
        time.sleep(0.01)
    return (0.0, 0.0, 0.0)

def wait_for_enter():
    # Flushes stdin and waits for user to press enter
    while select.select([sys.stdin], [], [], 0)[0]:
        sys.stdin.read(1)
    input("Press Enter when ready...")

def main():
    print("========================================")
    print("      RoboDog IMU Calibration Wizard      ")
    print("========================================\n")
    
    try:
        i2c = board.I2C()
        sensor = adafruit_bno055.BNO055_I2C(i2c)
    except Exception as e:
        print(f"Failed to find BNO055 IMU on I2C: {e}")
        return

    print("Connected to IMU successfully!\n")
    
    config = {
        "axis_mapping": {"pitch_index": 2, "roll_index": 1, "yaw_index": 0},
        "inversions": {"pitch": False, "roll": False, "yaw": False},
        "zero_offsets_deg": {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}
    }
    
    axes = ["pitch", "roll", "yaw"]
    used_indices = []

    # 1. Axis Mapping
    for axis_name in axes:
        print(f"\n--- CALIBRATING {axis_name.upper()} ---")
        print(f"I will now stream the raw indices [0, 1, 2].")
        
        if axis_name == "pitch":
            print("Please pitch the dog (tilt NOSE UP and NOSE DOWN).")
        elif axis_name == "roll":
            print("Please roll the dog (tilt LEFT to RIGHT).")
        else:
            print("Please yaw the dog (spin LEFT and RIGHT like a compass).")
            
        print("Watch which index number changes the most.")
        wait_for_enter()
        
        print("\nStreaming values for 5 seconds...")
        for _ in range(50):
            e = get_latest_euler(sensor)
            print(f"Index 0: {e[0]:>6.1f} | Index 1: {e[1]:>6.1f} | Index 2: {e[2]:>6.1f}", end='\r')
            time.sleep(0.1)
        print("\n")
        
        while True:
            try:
                idx = int(input(f"Which index (0, 1, or 2) corresponds to {axis_name}? "))
                if idx in [0, 1, 2]:
                    if idx in used_indices:
                        print(f"Warning: Index {idx} is already used, but proceeding...")
                    used_indices.append(idx)
                    config["axis_mapping"][f"{axis_name}_index"] = idx
                    break
                else:
                    print("Please enter 0, 1, or 2.")
            except ValueError:
                print("Invalid input.")

        # Inversion
        while True:
            if axis_name == "pitch":
                prompt_text = "Does the pitch value go POSITIVE when the NOSE is moved DOWN? (y/n) "
            elif axis_name == "roll":
                prompt_text = "Does the roll value go POSITIVE when the RIGHT SIDE is FALLEN (tilted right)? (y/n) "
            else:
                prompt_text = "Does the yaw value go POSITIVE when spun to the RIGHT? (y/n) "
                
            inv = input(prompt_text).lower()
            if inv in ['y', 'n']:
                # The visualizer uses positive pitch = nose down, positive roll = right down
                if inv == 'n':
                    config["inversions"][axis_name] = True
                    print(f"-> Marked {axis_name} as inverted.")
                else:
                    config["inversions"][axis_name] = False
                break
    
    # 2. Zero Calibration
    print("\n--- ZERO OFFSET CALIBRATION ---")
    print("Please place the dog on a perfectly flat and level surface.")
    print("Ensure all 4 legs are in their natural stance.")
    wait_for_enter()
    
    print("Taking average over 2 seconds...")
    sums = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}
    count = 0
    for _ in range(20):
        e = get_latest_euler(sensor)
        
        for ax in axes:
            raw_val = e[config["axis_mapping"][f"{ax}_index"]]
            if config["inversions"][ax]:
                raw_val = -raw_val
            sums[ax] += raw_val
            
        count += 1
        time.sleep(0.1)
        
    for ax in axes:
        config["zero_offsets_deg"][ax] = sums[ax] / count
        
    print(f"\nZero Offsets Calculated:")
    print(f"Pitch: {config['zero_offsets_deg']['pitch']:.2f} deg")
    print(f"Roll:  {config['zero_offsets_deg']['roll']:.2f} deg")
    print(f"Yaw:   {config['zero_offsets_deg']['yaw']:.2f} deg")

    # 3. Save Config
    base_dir = "/home/pi/robodog/calibration_tool"
    os.makedirs(base_dir, exist_ok=True)
    out_path = os.path.join(base_dir, "imu_calibration.json")
    with open(out_path, "w") as f:
        json.dump(config, f, indent=4)
        
    print(f"\nCalibration saved successfully to {out_path}!")
    print("Restart your IK Visualizer Server for the changes to take effect.")

if __name__ == '__main__':
    main()
