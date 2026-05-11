#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import time
import csv
from datetime import datetime

# Import the Waveshare SDK
sys.path.append("..")
from scservo_sdk import *

class ST3215Core:
    def __init__(self, port='/dev/ttyACM0', baudrate=1000000):
        self.portHandler = PortHandler(port)
        self.packetHandler = sms_sts(self.portHandler)
        
        if not self.portHandler.openPort():
            raise ConnectionError(f"Failed to open the port: {port}")
            
        if not self.portHandler.setBaudRate(baudrate):
            raise ConnectionError(f"Failed to change the baudrate to {baudrate}")
            
        # Setup Logging
        os.makedirs("datas", exist_ok=True)
        self.log_file = f"datas/servo_telemetry_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self._init_logger()

    def _init_logger(self):
        with open(self.log_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Timestamp", "ID", "Position", "Speed", "Load", "Voltage", "Temperature", "Current"])

    def set_wheel_mode(self, servo_id):
        """Sets the servo to continuous rotation mode."""
        self.packetHandler.WheelMode(servo_id)

    def write_speed(self, servo_id, speed, accel=50):
        """Commands the servo to move at a specific speed."""
        self.packetHandler.WriteSpec(servo_id, speed, accel)

    def stop(self, servo_id, accel=50):
        """Halts the servo."""
        self.packetHandler.WriteSpec(servo_id, 0, accel)

    def read_telemetry(self, servo_id):
        """Reads all feedback from the servo and logs it to CSV."""
        pos, res_p, err = self.packetHandler.ReadPos(servo_id)
        spd, res_s, err = self.packetHandler.ReadSpeed(servo_id)
        load, res_l, err = self.packetHandler.ReadLoad(servo_id)
        volt, res_v, err = self.packetHandler.ReadVoltage(servo_id)
        temp, res_t, err = self.packetHandler.ReadTemper(servo_id)
        curr, res_c, err = self.packetHandler.ReadCurrent(servo_id)

        if res_p == COMM_SUCCESS and res_l == COMM_SUCCESS:
            load_mag = abs(load) 
            with open(self.log_file, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([time.time(), servo_id, pos, spd, load_mag, volt, temp, curr])
            return pos, spd, load_mag
            
        return None, None, None

    def close(self):
        """Closes the serial port safely."""
        self.portHandler.closePort()