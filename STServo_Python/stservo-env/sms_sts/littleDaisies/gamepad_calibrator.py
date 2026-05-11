#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import select
from evdev import InputDevice, ecodes, list_devices

def find_joystick():
    for p in list_devices():
        d = InputDevice(p)
        name = (d.name or "").lower()
        if any(k in name for k in ["logitech", "gamepad", "joystick", "controller", "xbox", "dualshock"]):
            return d
    return None

def flush_events(joy):
    """Clears out any old inputs before we ask the user for a new one."""
    while True:
        r, _, _ = select.select([joy.fd], [], [], 0.1)
        if r:
            for _ in joy.read(): pass
        else:
            break

def get_button(joy, action_name):
    print(f"\n[?] PRESS AND HOLD the button for: {action_name}")
    flush_events(joy)
    while True:
        r, _, _ = select.select([joy.fd], [], [], 0.1)
        if r:
            for ev in joy.read():
                if ev.type == ecodes.EV_KEY and ev.value == 1:
                    print(f"    -> Recorded Code: {ev.code}")
                    time.sleep(0.5) # Debounce pause
                    return ev.code

def get_axis(joy, action_name):
    print(f"\n[?] MOVE the {action_name} ALL THE WAY to the left or right, then let go.")
    flush_events(joy)
    while True:
        r, _, _ = select.select([joy.fd], [], [], 0.1)
        if r:
            for ev in joy.read():
                if ev.type == ecodes.EV_ABS:
                    # Ignore tiny jitters/stick drift. Only trigger if moved significantly.
                    # Standard evdev axes usually go -32768 to 32767 or 0 to 255
                    val = ev.value
                    if abs(val) > 15000 or (val < 20 or val > 230): 
                        print(f"    -> Recorded Axis Code: {ev.code}")
                        time.sleep(1.0) # Pause so we don't double-read the release
                        return ev.code

def measure_deadzone(joy):
    print("\n[?] Measuring Stick Drift (Deadzone)...")
    print("    -> DO NOT TOUCH THE JOYSTICKS for 3 seconds.")
    time.sleep(1)
    flush_events(joy)
    
    max_drift = 0
    start_time = time.time()
    
    while time.time() - start_time < 3.0:
        r, _, _ = select.select([joy.fd], [], [], 0.1)
        if r:
            for ev in joy.read():
                if ev.type == ecodes.EV_ABS:
                    # Calculate how far from center it is resting
                    info = joy.absinfo(ev.code)
                    if info.max != info.min:
                        center = (info.max + info.min) / 2
                        range_val = (info.max - info.min) / 2
                        drift_percent = abs(ev.value - center) / range_val
                        if drift_percent > max_drift:
                            max_drift = drift_percent

    # Add a 5% safety buffer to the maximum drift observed
    recommended_deadzone = round(max_drift + 0.05, 3)
    # Ensure a minimum deadzone of 0.08
    if recommended_deadzone < 0.08:
        recommended_deadzone = 0.08
        
    print(f"    -> Recommended Deadzone: {recommended_deadzone}")
    return recommended_deadzone

def main():
    joy = find_joystick()
    if not joy:
        print("[ERROR] No gamepad found. Is it plugged in?")
        return
        
    print(f"--- Connected to: {joy.name} ---")
    print("Let's calibrate your specific controller layout.\n")
    
    # 1. Map Buttons
    btn_quit   = get_button(joy, "QUIT (Usually X or Square)")
    btn_brake  = get_button(joy, "EMERGENCY BRAKE (Usually A or Cross)")
    btn_mode   = get_button(joy, "TOGGLE 1-STICK/2-STICK (Usually Y or Triangle)")
    btn_circle = get_button(joy, "UNUSED / EXTRA BUTTON (Usually B or Circle)")
    btn_l1     = get_button(joy, "DECREASE SPEED (Left Bumper / L1)")
    btn_r1     = get_button(joy, "INCREASE SPEED (Right Bumper / R1)")
    btn_home   = get_button(joy, "MANUAL HOMING (Select or Back button)")
    
    # 2. Map Axes
    axis_left  = get_axis(joy, "LEFT ANALOG STICK (Horizontal)")
    axis_right = get_axis(joy, "RIGHT ANALOG STICK (Horizontal)")
    
    # 3. Measure Deadzone
    deadzone   = measure_deadzone(joy)
    
    print("\n" + "="*60)
    print("CALIBRATION COMPLETE!")
    print("Copy the code block below and replace the 'GAMEPAD DEFINITIONS'")
    print("section in your gripper_control.py file.")
    print("="*60 + "\n")
    
    print(f"DEADZONE    = {deadzone}")
    print("")
    print("# -------------------------------------------------")
    print("# GAMEPAD DEFINITIONS (Custom Calibrated)")
    print("# -------------------------------------------------")
    print(f"AX_LEFT_X  = {axis_left}")
    print(f"AX_RIGHT_X = {axis_right}")
    print(f"AX_RIGHT_Z = {axis_right}       # Logitech fallback")
    print("")
    print(f"BTN_SQUARE   = {btn_quit}       # Quit")
    print(f"BTN_CROSS    = {btn_brake}       # Emergency Brake")
    print(f"BTN_TRIANGLE = {btn_mode}       # Toggle 1-Stick/2-Stick")
    print(f"BTN_CIRCLE   = {btn_circle}       # Unused currently")
    print(f"BTN_L1       = {btn_l1}       # Decrease Speed")
    print(f"BTN_R1       = {btn_r1}       # Increase Speed")
    print(f"BTN_SELECT   = {btn_home}       # Manual Homing")
    print("")
    print("="*60 + "\n")

if __name__ == "__main__":
    main()
