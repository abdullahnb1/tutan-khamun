#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import time
import select
import csv
from datetime import datetime
from evdev import InputDevice, ecodes, list_devices, AbsInfo

# Import the Waveshare SDK
sys.path.append("..")
from scservo_sdk import *

# -------------------------------------------------
# HARDWARE SETTINGS
# -------------------------------------------------
DEVICENAME  = '/dev/ttyACM0'    # Check your port (ttyACM0 or ttyACM1)
BAUDRATE    = 1000000

ID_RIGHT    = 1                 # Right Gripper Servo
ID_LEFT     = 2                 # Left Gripper Servo

# -------------------------------------------------
# DRIVE & LIMIT SETTINGS
# -------------------------------------------------
MAX_SPEED   = 2400
ACCEL       = 50
DEADZONE    = 0.15
LEVEL       = 2
DEBOUNCE_MS = 250

# --- HOMING SETTINGS ---
HOMING_SPEED          = 800     # Slower speed for gentle collision with hard stops
HOMING_LOAD_THRESHOLD = 280     # ST3215 load value indicating a hard stop (Tune this! 0-1000 range)
OFFSET_DEGREES        = 3.0     # Degrees to back away from the absolute hard stop
OFFSET_STEPS          = int((OFFSET_DEGREES / 360.0) * 4096)  # Convert degrees to steps (~34 steps)

# --- ORIENTATION SETTINGS ---
# Since servos are mounted in the same orientation, they move together.
# If Phase 1 opens the gripper instead of closing it, simply swap these two values (1 and -1).
CLOSE_DIR = -1   # 1 = CW, -1 = CCW
OPEN_DIR  = 1    # 1 = CW, -1 = CCW

# Dictionary to store our safe operational limits (Updated by the homing sequence)
servo_limits = {
    ID_LEFT:  {'min': 0, 'max': 4095},
    ID_RIGHT: {'min': 0, 'max': 4095}
}

is_homed = False

# -------------------------------------------------
# GAMEPAD DEFINITIONS (Linux evdev standards)
# -------------------------------------------------
AX_LEFT_X  = ecodes.ABS_X
AX_RIGHT_X = ecodes.ABS_RX
AX_RIGHT_Z = ecodes.ABS_Z

BTN_SQUARE   = ecodes.BTN_WEST    # The Left face button (Quit)
BTN_CROSS    = ecodes.BTN_SOUTH   # The Bottom face button (Emergency Brake)
BTN_TRIANGLE = ecodes.BTN_NORTH   # The Top face button (Toggle 1-Stick/2-Stick)
BTN_CIRCLE   = ecodes.BTN_EAST    # The Right face button (Unused currently)

BTN_L1       = ecodes.BTN_TL      # Left Bumper (Decrease Speed)
BTN_R1       = ecodes.BTN_TR      # Right Bumper (Increase Speed)
BTN_SELECT   = ecodes.BTN_SELECT  # Select / Back Button (Manual Homing)

JOY = None
JOY_STATE = {'axes': {}, 'btn': set()}
PREV_BTN = set()
last_toggle = {'quit': 0, 'r1': 0, 'l1': 0, 'mode': 0, 'home': 0}

# -------------------------------------------------
# TELEMETRY LOGGER SETUP
# -------------------------------------------------
os.makedirs("datas", exist_ok=True)
LOG_FILE = f"datas/servo_telemetry_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

def init_logger():
    with open(LOG_FILE, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp", "ID", "Position", "Speed", "Load", "Voltage", "Temperature", "Current"])

def log_telemetry(packetHandler, servo_id):
    """Reads all feedback from the servo and writes to CSV."""
    pos, res_p, err = packetHandler.ReadPos(servo_id)
    spd, res_s, err = packetHandler.ReadSpeed(servo_id)
    load, res_l, err = packetHandler.ReadLoad(servo_id)
    volt, res_v, err = packetHandler.ReadVoltage(servo_id)
    temp, res_t, err = packetHandler.ReadTemper(servo_id)
    curr, res_c, err = packetHandler.ReadCurrent(servo_id)

    if res_p == COMM_SUCCESS and res_l == COMM_SUCCESS:
        load_mag = abs(load) 
        with open(LOG_FILE, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([time.time(), servo_id, pos, spd, load_mag, volt, temp, curr])
        return pos, spd, load_mag
    return None, None, None

# -------------------------------------------------
# SENSORLESS HOMING LOGIC 
# -------------------------------------------------
def drive_single_until_stop(packetHandler, servo_id, direction):
    """Drives a single servo until it hits a hard stop (current spike)."""
    packetHandler.WriteSpec(servo_id, HOMING_SPEED * direction, ACCEL)
    time.sleep(0.3) # Wait for initial starting inertia to settle
    
    spike_count = 0
    pos_stop = None
    
    while True:
        pos, spd, load = log_telemetry(packetHandler, servo_id)
        if load is not None and load > HOMING_LOAD_THRESHOLD:
            spike_count += 1
            if spike_count >= 3:
                packetHandler.WriteSpec(servo_id, 0, ACCEL) # Stop Servo
                pos_stop = pos
                dir_str = "CW" if direction > 0 else "CCW"
                print(f"  > Servo ID {servo_id} hit {dir_str} hard stop at: {pos_stop}")
                break
        else:
            spike_count = 0
            
        time.sleep(0.01)
        
    return pos_stop

def drive_until_stop(packetHandler, dir_L, dir_R):
    """Drives both servos simultaneously until they both hit a hard stop."""
    packetHandler.WriteSpec(ID_LEFT, HOMING_SPEED * dir_L, ACCEL)
    packetHandler.WriteSpec(ID_RIGHT, HOMING_SPEED * dir_R, ACCEL)
    time.sleep(0.3)
    
    stop_L, stop_R = False, False
    spike_L, spike_R = 0, 0
    pos_L_stop, pos_R_stop = None, None
    
    while not (stop_L and stop_R):
        if not stop_L:
            pos, spd, load = log_telemetry(packetHandler, ID_LEFT)
            if load is not None and load > HOMING_LOAD_THRESHOLD:
                spike_L += 1
                if spike_L >= 3:
                    packetHandler.WriteSpec(ID_LEFT, 0, ACCEL)
                    pos_L_stop = pos
                    stop_L = True
                    print(f"  > Left  Servo hit hard stop at: {pos_L_stop}")
            else:
                spike_L = 0
                
        if not stop_R:
            pos, spd, load = log_telemetry(packetHandler, ID_RIGHT)
            if load is not None and load > HOMING_LOAD_THRESHOLD:
                spike_R += 1
                if spike_R >= 3:
                    packetHandler.WriteSpec(ID_RIGHT, 0, ACCEL)
                    pos_R_stop = pos
                    stop_R = True
                    print(f"  > Right Servo hit hard stop at: {pos_R_stop}")
            else:
                spike_R = 0
                
        time.sleep(0.01)
        
    return pos_L_stop, pos_R_stop

def execute_homing(packetHandler):
    global is_homed, servo_limits
    print("\n\n[WARNING] Starting Sensorless Homing Sequence...")
    print("Keep hands clear! Servos will seek hard stops.")
    
    # Phase 1: Sequential Closing (Both moving in the CLOSE_DIR)
    print("\n--- Phase 1: Seeking Inner Limits (Sequential Closing) ---")
    print("Moving Right Servo (ID 1) to close position...")
    limit1_R = drive_single_until_stop(packetHandler, ID_RIGHT, OPEN_DIR) 
    time.sleep(0.5) # Let mechanical tension release
    
    print("Moving Left Servo (ID 2) to close position...")
    limit1_L = drive_single_until_stop(packetHandler, ID_LEFT, CLOSE_DIR) 
    time.sleep(0.5)
    
    # Phase 2: Simultaneous Opening (Both moving in the OPEN_DIR)
    print("\n--- Phase 2: Seeking Outer Limits (Simultaneous Opening) ---")
    limit2_L, limit2_R = drive_until_stop(packetHandler, OPEN_DIR, CLOSE_DIR) 
    time.sleep(0.5)
    
    # Calculate absolute Min and Max ranges for each servo based on the two walls they hit
    raw_min_L = min(limit1_L, limit2_L)
    raw_max_L = max(limit1_L, limit2_L)
    
    raw_min_R = min(limit1_R, limit2_R)
    raw_max_R = max(limit1_R, limit2_R)
    
    # Apply the safety offset (stay 3 degrees away from the absolute walls)
    servo_limits[ID_LEFT]['min'] = raw_min_L + OFFSET_STEPS
    servo_limits[ID_LEFT]['max'] = raw_max_L - OFFSET_STEPS
    
    servo_limits[ID_RIGHT]['min'] = raw_min_R + OFFSET_STEPS
    servo_limits[ID_RIGHT]['max'] = raw_max_R - OFFSET_STEPS
    
    print("\n--- Calibration Complete ---")
    print(f"Left  Safe Range: [{servo_limits[ID_LEFT]['min']} to {servo_limits[ID_LEFT]['max']}]")
    print(f"Right Safe Range: [{servo_limits[ID_RIGHT]['min']} to {servo_limits[ID_RIGHT]['max']}]")

    is_homed = True
    print("[SUCCESS] Homing Complete. Returning to manual control.")

# -------------------------------------------------
# JOYSTICK LOGIC
# -------------------------------------------------
def find_joystick():
    for p in list_devices():
        d = InputDevice(p)
        name = (d.name or "").lower()
        if any(k in name for k in ["logitech", "gamepad", "joystick", "controller", "xbox", "dualshock"]):
            return d
    return None

def norm_axis(dev, code, val):
    info = dev.absinfo(code) if hasattr(dev, "absinfo") else None
    if isinstance(info, AbsInfo) and info.max != info.min:
        x = (val - (info.min + info.max) / 2) / ((info.max - info.min) / 2)
    else:
        x = (val - 32767) / 32767.0
    if abs(x) < DEADZONE: x = 0.0
    return max(-1.0, min(1.0, x))

def poll_joystick(timeout=0.0):
    if not JOY: return
    r, _, _ = select.select([JOY.fd], [], [], timeout)
    if not r: return
    for ev in JOY.read():
        if ev.type == ecodes.EV_ABS and ev.code in (AX_LEFT_X, AX_RIGHT_X, AX_RIGHT_Z):
            JOY_STATE['axes'][ev.code] = norm_axis(JOY, ev.code, ev.value)
        elif ev.type == ecodes.EV_KEY:
            if ev.value == 1: JOY_STATE['btn'].add(ev.code)
            elif ev.value == 0: JOY_STATE['btn'].discard(ev.code)

def button_just_pressed(code, now_ms, name):
    if (code in JOY_STATE['btn']) and (code not in PREV_BTN) and (now_ms - last_toggle[name] > DEBOUNCE_MS):
        last_toggle[name] = now_ms
        return True
    return False

# -------------------------------------------------
# MAIN LOOP
# -------------------------------------------------
def main():
    global JOY, LEVEL, PREV_BTN, is_homed
    single_stick_mode = False

    portHandler = PortHandler(DEVICENAME)
    packetHandler = sms_sts(portHandler)
        
    if not portHandler.openPort():
        print("Failed to open the port")
        quit()
    if not portHandler.setBaudRate(BAUDRATE):
        print("Failed to change the baudrate")
        quit()

    packetHandler.WheelMode(ID_LEFT)
    packetHandler.WheelMode(ID_RIGHT)
    init_logger()

    JOY = find_joystick()
    if JOY:
        print(f"[JOY] Connected: {JOY.name}")
        try: JOY.grab()
        except: pass
    else:
        print("[ERROR] No gamepad found. Exiting.")
        portHandler.closePort()
        return

    # --- AUTO-HOME ON STARTUP ---
    execute_homing(packetHandler)

    print(f"""
    ---------------------------------------------------------
      Logitech Gripper Control Active (Logging to {LOG_FILE})
    ---------------------------------------------------------
      [Select/Back]     -> Trigger Manual Re-Homing
      [Triangle/Y]      -> Toggle 1-Stick vs 2-Stick Mode
      [Square/X]        -> Quit
      [Cross/A]         -> Emergency Brake
    ---------------------------------------------------------
    """)

    try:
        while True:
            now_ms = time.time() * 1000.0
            poll_joystick(0.0)

            pos_L, _, _ = log_telemetry(packetHandler, ID_LEFT)
            pos_R, _, _ = log_telemetry(packetHandler, ID_RIGHT)
            
            if pos_L is None or pos_R is None:
                continue

            # --- Check Buttons ---
            if button_just_pressed(BTN_SELECT, now_ms, 'home'):
                execute_homing(packetHandler)
                continue
                
            if button_just_pressed(BTN_TRIANGLE, now_ms, 'mode'):
                single_stick_mode = not single_stick_mode
                mode_str = "SINGLE-STICK (Synchronized)" if single_stick_mode else "DUAL-STICK (Independent)"
                print(f"\n[MODE] Switched to {mode_str}")

            if button_just_pressed(BTN_SQUARE, now_ms, 'quit'):
                break

            # --- Calculate Speeds ---
            current_max = int(MAX_SPEED * (LEVEL / 4.0))

            if BTN_CROSS in JOY_STATE['btn']:
                speed_L, speed_R = 0, 0
            else:
                left_x = JOY_STATE['axes'].get(AX_LEFT_X, 0.0)
                
                if single_stick_mode:
                    # Both servos move in the same direction because of identical mounting orientation
                    speed_L = int(left_x * current_max)
                    speed_R = int(left_x * current_max)
                else:
                    right_x = JOY_STATE['axes'].get(AX_RIGHT_X, 0.0)
                    if abs(JOY_STATE['axes'].get(AX_RIGHT_Z, 0.0)) > abs(right_x):
                        right_x = JOY_STATE['axes'].get(AX_RIGHT_Z, 0.0)
                    speed_L = int(left_x * current_max)
                    speed_R = int(right_x * current_max)

            # --- SOFTWARE LIMIT ENFORCEMENT ---
            if pos_L <= servo_limits[ID_LEFT]['max'] and speed_L > 0: speed_L = 0
            if pos_L >= servo_limits[ID_LEFT]['min'] and speed_L < 0: speed_L = 0
            
            if pos_R >= servo_limits[ID_RIGHT]['max'] and speed_R > 0: speed_R = 0
            if pos_R <= servo_limits[ID_RIGHT]['min'] and speed_R < 0: speed_R = 0

            # --- Write to Servos ---
            packetHandler.WriteSpec(ID_LEFT, speed_L, ACCEL)
            packetHandler.WriteSpec(ID_RIGHT, speed_R, ACCEL)
            
            PREV_BTN = set(JOY_STATE['btn'])
            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        print("\nShutting down safely. Stopping motors...")
        packetHandler.WriteSpec(ID_LEFT, 0, ACCEL)
        packetHandler.WriteSpec(ID_RIGHT, 0, ACCEL)
        portHandler.closePort()
        if JOY: 
            try: JOY.ungrab()
            except: pass

if __name__ == "__main__":
    main()
