#!/usr/bin/env python3
import time
import sys
try:
    from adafruit_servokit import ServoKit
except ImportError:
    print("Error: adafruit-circuitpython-servokit is not installed.")
    print("Please install it using: pip install --break-system-packages adafruit-circuitpython-servokit")
    sys.exit(1)

def main():
    print("Initializing PCA9685 Servo Kit (16 channels)...")
    kit = ServoKit(channels=16)
    
    # Configure channel 5
    kit.servo[5].actuation_range = 180
    kit.servo[5].set_pulse_width_range(370, 2400)
    
    print("Starting servo loop on Channel 5 (60 to 90 degrees)...")
    print("Press Ctrl+C to stop.")
    
    try:
        while True:
            # Sweep from 60 to 90 degrees
            print("Sweeping 60 -> 90 degrees")
            for angle in range(60, 91):
                kit.servo[5].angle = angle
                time.sleep(0.02) # Adjust speed of sweep here
            
            time.sleep(0.5) # Pause at 90 degrees
            
            # Sweep from 90 to 60 degrees
            print("Sweeping 90 -> 60 degrees")
            for angle in range(90, 59, -1):
                kit.servo[5].angle = angle
                time.sleep(0.02)
                
            time.sleep(0.5) # Pause at 60 degrees
            
    except KeyboardInterrupt:
        print("\nStopping and setting channel 5 to neutral (90 degrees)...")
        kit.servo[5].angle = 90
        print("Done.")

if __name__ == "__main__":
    main()
