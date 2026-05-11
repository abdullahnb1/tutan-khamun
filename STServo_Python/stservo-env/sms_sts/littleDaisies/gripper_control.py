#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import select
from evdev import InputDevice, ecodes, list_devices, AbsInfo

# Import Custom Modules
from st3215_core import ST3215Core
from gripper_homing import execute_homing, OFFSET_STEPS

# -------------------------------------------------
# OPERATIONAL SETTINGS
# -------------------------------------------------
ID_RIGHT    = 1                 
ID_LEFT     = 2                 
MAX_SPEED   = 2400
ACCEL       = 50
DEADZONE    = 0.15
LEVEL       = 2
DEBOUNCE_MS = 250

# -------------------------------------------------
# GAMEPAD DEFINITIONS
# -------------------------------------------------
AX_LEFT_X  = ecodes.ABS_X
AX_RIGHT_X = ecodes.ABS_RX
AX_RIGHT_Z = ecodes.ABS_Z

BTN_SQUARE   = ecodes.BTN_WEST    
BTN_CROSS    = ecodes.BTN_SOUTH   
BTN_TRIANGLE = ecodes.BTN_NORTH   
BTN_L1       = ecodes.BTN_TL      
BTN_R1       = ecodes.BTN_TR      
BTN_SELECT   = ecodes.BTN_SELECT  

JOY = None
JOY_STATE = {'axes': {}, 'btn': set()}
PREV_BTN = set()
last_toggle = {'quit': 0, 'r1': 0, 'l1': 0, 'mode': 0, 'home': 0}

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
# MODULAR MATH LIMIT ENFORCEMENT
# -------------------------------------------------
def get_distance_to_wall(current_pos, wall_pos, direction):
    """Calculates safe travel distance remaining before hitting the wall, handling 4095->0 wrap."""
    if direction > 0: # Moving CW
        return (wall_pos - current_pos) % 4096
    else:             # Moving CCW
        return (current_pos - wall_pos) % 4096

# -------------------------------------------------
# MAIN LOOP
# -------------------------------------------------
def main():
    global JOY, LEVEL, PREV_BTN
    single_stick_mode = False
    is_homed = False
    servo_limits = {}

    try:
        core = ST3215Core(port='/dev/ttyACM0', baudrate=1000000)
    except ConnectionError as e:
        print(e)
        return

    core.set_wheel_mode(ID_LEFT)
    core.set_wheel_mode(ID_RIGHT)

    JOY = find_joystick()
    if JOY:
        print(f"[JOY] Connected: {JOY.name}")
        try: JOY.grab()
        except: pass
    else:
        print("[ERROR] No gamepad found. Exiting.")
        core.close()
        return

    # --- AUTO-HOME ON STARTUP ---
    servo_limits = execute_homing(core, ID_LEFT, ID_RIGHT)
    is_homed = True

    print(f"""
    ---------------------------------------------------------
      Logitech Gripper Control Active (Logging to {core.log_file})
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

            pos_L, _, _ = core.read_telemetry(ID_LEFT)
            pos_R, _, _ = core.read_telemetry(ID_RIGHT)
            
            if pos_L is None or pos_R is None:
                continue

            # --- Check Buttons ---
            if button_just_pressed(BTN_SELECT, now_ms, 'home'):
                servo_limits = execute_homing(core, ID_LEFT, ID_RIGHT)
                is_homed = True
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
                    speed_L = int(left_x * current_max)
                    speed_R = int(left_x * current_max)
                else:
                    right_x = JOY_STATE['axes'].get(AX_RIGHT_X, 0.0)
                    if abs(JOY_STATE['axes'].get(AX_RIGHT_Z, 0.0)) > abs(right_x):
                        right_x = JOY_STATE['axes'].get(AX_RIGHT_Z, 0.0)
                    speed_L = int(left_x * current_max)
                    speed_R = int(right_x * current_max)

            # --- SOFTWARE LIMIT ENFORCEMENT (Zero-Crossing Safe) ---
            if is_homed:
                # Left Servo Limits
                if speed_L > 0: 
                    dist = get_distance_to_wall(pos_L, servo_limits[ID_LEFT]['cw'], 1)
                    if dist <= OFFSET_STEPS or dist > 2048: speed_L = 0
                elif speed_L < 0: 
                    dist = get_distance_to_wall(pos_L, servo_limits[ID_LEFT]['ccw'], -1)
                    if dist <= OFFSET_STEPS or dist > 2048: speed_L = 0

                # Right Servo Limits
                if speed_R > 0: 
                    dist = get_distance_to_wall(pos_R, servo_limits[ID_RIGHT]['cw'], 1)
                    if dist <= OFFSET_STEPS or dist > 2048: speed_R = 0
                elif speed_R < 0: 
                    dist = get_distance_to_wall(pos_R, servo_limits[ID_RIGHT]['ccw'], -1)
                    if dist <= OFFSET_STEPS or dist > 2048: speed_R = 0

            # --- Write to Servos ---
            core.write_speed(ID_LEFT, speed_L, ACCEL)
            core.write_speed(ID_RIGHT, speed_R, ACCEL)
            
            PREV_BTN = set(JOY_STATE['btn'])
            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        print("\nShutting down safely. Stopping motors...")
        core.stop(ID_LEFT, ACCEL)
        core.stop(ID_RIGHT, ACCEL)
        core.close()
        if JOY: 
            try: JOY.ungrab()
            except: pass

if __name__ == "__main__":
    main()