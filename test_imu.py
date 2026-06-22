#!/usr/bin/env python3
import time
import board
import adafruit_bno055

def test_imu():
    print("Initializing I2C bus...")
    try:
        i2c = board.I2C()
    except Exception as e:
        print(f"Failed to initialize I2C bus: {e}")
        return

    print("Connecting to BNO055 IMU...")
    try:
        sensor = adafruit_bno055.BNO055_I2C(i2c)
    except Exception as e:
        print(f"Failed to find BNO055 IMU on I2C: {e}")
        print("Please check wiring (SDA/SCL pins) and power.")
        return

    print("\nBNO055 IMU Detected Successfully!")
    print("Reading sensor values (Press Ctrl+C to stop)...\n")
    print(f"{'Temperature (°C)':<18} | {'Euler Angles (Yaw, Roll, Pitch)':<32} | {'Calibration (Sys, Gyro, Accel, Mag)'}")
    print("-" * 90)

    try:
        while True:
            temp = sensor.temperature
            euler = sensor.euler
            calib = sensor.calibration_status
            
            # Format outputs safely (handles None values from sensor during initialization/glitches)
            temp_str = f"{temp}°C" if temp is not None else "N/A"
            
            if euler and all(x is not None for x in euler):
                euler_str = f"Y:{euler[0]:6.1f} R:{euler[1]:6.1f} P:{euler[2]:6.1f}"
            else:
                euler_str = "N/A"
                
            if calib:
                calib_str = f"S:{calib[0]} G:{calib[1]} A:{calib[2]} M:{calib[3]}"
            else:
                calib_str = "N/A"

            print(f"\r{temp_str:<18} | {euler_str:<32} | {calib_str}", end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping IMU test. Done.")

if __name__ == "__main__":
    test_imu()
