#!/usr/bin/env python3
import rospy
import numpy as np
import time
import board
import adafruit_bno055
import math as m
import json
import os

class IMU:
    def __init__(self):
        self.i2c = board.I2C()  # uses board.SCL and board.SDA
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
        self.last_euler = np.array([ 0, 0, 0])
        self.start_time = time.time()

        # Load IMU calibration
        self.imu_pitch_index = 2
        self.imu_roll_index = 1
        self.imu_yaw_index = 0
        self.imu_inv_pitch = False
        self.imu_inv_roll = False
        self.imu_inv_yaw = False
        self.imu_zero_pitch = 0.0
        self.imu_zero_roll = 0.0
        self.imu_zero_yaw = 0.0

        # Try to find the file from the common robodog location
        base_dir = "/home/pi/robodog"
        imu_calib_path = os.path.join(base_dir, "calibration_tool", "imu_calibration.json")
        if os.path.exists(imu_calib_path):
            try:
                with open(imu_calib_path, "r") as f:
                    imu_data = json.load(f)
                if "axis_mapping" in imu_data:
                    self.imu_pitch_index = imu_data["axis_mapping"].get("pitch_index", self.imu_pitch_index)
                    self.imu_roll_index = imu_data["axis_mapping"].get("roll_index", self.imu_roll_index)
                    self.imu_yaw_index = imu_data["axis_mapping"].get("yaw_index", self.imu_yaw_index)
                if "inversions" in imu_data:
                    self.imu_inv_pitch = imu_data["inversions"].get("pitch", self.imu_inv_pitch)
                    self.imu_inv_roll = imu_data["inversions"].get("roll", self.imu_inv_roll)
                    self.imu_inv_yaw = imu_data["inversions"].get("yaw", self.imu_inv_yaw)
                if "zero_offsets_deg" in imu_data:
                    self.imu_zero_pitch = imu_data["zero_offsets_deg"].get("pitch", self.imu_zero_pitch)
                    self.imu_zero_roll = imu_data["zero_offsets_deg"].get("roll", self.imu_zero_roll)
                    self.imu_zero_yaw = imu_data["zero_offsets_deg"].get("yaw", self.imu_zero_yaw)
            except Exception as e:
                pass

    def read_orientation(self):
        """Reads quaternion measurements from the IMU until . Returns the last read Euler angle.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        np array (3,)
            If there was quaternion data to read on the serial port returns the quaternion as a numpy array, otherwise returns the last read quaternion.
        """
        try: 
            euler = self.sensor.euler
            if euler and len(euler) == 3:
                pitch_raw = euler[self.imu_pitch_index] if euler[self.imu_pitch_index] is not None else 0.0
                roll_raw = euler[self.imu_roll_index] if euler[self.imu_roll_index] is not None else 0.0
                yaw_raw = euler[self.imu_yaw_index] if euler[self.imu_yaw_index] is not None else 0.0
                
                if self.imu_inv_pitch: pitch_raw = -pitch_raw
                if self.imu_inv_roll: roll_raw = -roll_raw
                if self.imu_inv_yaw: yaw_raw = -yaw_raw

                pitch_raw -= self.imu_zero_pitch
                roll_raw -= self.imu_zero_roll
                yaw_raw -= self.imu_zero_yaw
                
                pitch_raw = (pitch_raw + 180.0) % 360.0 - 180.0
                roll_raw = (roll_raw + 180.0) % 360.0 - 180.0
                yaw_raw = (yaw_raw + 180.0) % 360.0 - 180.0
                
                yaw = m.radians(yaw_raw)
                pitch = m.radians(pitch_raw)
                roll = m.radians(roll_raw)
                
                self.last_euler = [yaw, pitch, roll]
        except:
            pass
            
        return self.last_euler
