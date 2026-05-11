#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

# --- HOMING SETTINGS ---
HOMING_SPEED           = 800     
HOMING_BASE_LOAD       = 150     # Ignore slope noise during free-spin (must be under some tension)
HOMING_SLOPE_THRESHOLD = 200     # Minimum dLoad/dt to trigger stop
OFFSET_DEGREES         = 3.0     
OFFSET_STEPS           = int((OFFSET_DEGREES / 360.0) * 4096)  

# --- PHASE 2 TARGET (45 Degrees) ---
OPEN_TARGET_DEGREES    = 45.0
OPEN_TARGET_STEPS      = int((OPEN_TARGET_DEGREES / 360.0) * 4096) # 512 steps

# --- ORIENTATION SETTINGS ---
CLOSE_DIR = -1   # -1 = CCW, 1 = CW
OPEN_DIR  = 1    # 1 = CW, -1 = CCW

def get_distance_to_target(current_pos, target_pos, direction):
    """Calculates travel distance remaining to a target, safely handling the 4095->0 wrap."""
    if direction > 0: # Moving CW
        return (target_pos - current_pos) % 4096
    else:             # Moving CCW
        return (current_pos - target_pos) % 4096

def drive_single_until_stop(core, servo_id, direction, accel=50):
    """Phase 1: Drives a single servo until it hits a hard stop (detected by Load Slope)."""
    core.write_speed(servo_id, HOMING_SPEED * direction, accel)
    time.sleep(0.3) # Wait for initial starting inertia to settle
    
    spike_count = 0
    pos_stop = None
    last_load, last_time = None, time.time()
    
    while True:
        pos, spd, load = core.read_telemetry(servo_id)
        current_time = time.time()
        
        if load is not None:
            if last_load is not None:
                dt = current_time - last_time
                if dt > 0:
                    load_slope = (load - last_load) / dt
                    
                    if load > HOMING_BASE_LOAD and load_slope > HOMING_SLOPE_THRESHOLD:
                        spike_count += 1
                        if spike_count >= 2:
                            core.stop(servo_id, accel)
                            pos_stop = pos
                            dir_str = "CW" if direction > 0 else "CCW"
                            print(f"  > Servo ID {servo_id} hit {dir_str} hard stop at: {pos_stop} (Slope: {load_slope:.1f})")
                            break
                    else:
                        spike_count = 0
            
            last_load = load
            last_time = current_time
            
        time.sleep(0.01)
        
    return pos_stop

def execute_homing(core, id_left, id_right):
    """Executes the full Tutan-Khamun sequential close / 45-degree open sequence."""
    print("\n\n[WARNING] Starting Homing Sequence...")
    print("Keep hands clear! Servos will seek inner hard stops, then open 45 degrees.")
    
    # =======================================================
    # PHASE 1: Sequential Closing (Using Load Slope Detection)
    # =======================================================
    print("\n--- Phase 1: Seeking Inner Limits (Sequential Closing) ---")
    print(f"Moving Right Servo (ID {id_right}) to close position...")
    limit_close_R = drive_single_until_stop(core, id_right, OPEN_DIR) 
    time.sleep(0.5) 
    
    print(f"Moving Left Servo (ID {id_left}) to close position...")
    limit_close_L = drive_single_until_stop(core, id_left, CLOSE_DIR) 
    time.sleep(0.5)
    
    # =======================================================
    # PHASE 2: Simultaneous Opening (Exact 45 Degree Math)
    # =======================================================
    print(f"\n--- Phase 2: Opening exactly {OPEN_TARGET_DEGREES} degrees ---")
    
    # Calculate the exact mathematical open limits based on the closed walls
    limit_open_L = int((limit_close_L + (OPEN_DIR * OPEN_TARGET_STEPS)) % 4096)
    limit_open_R = int((limit_close_R + (CLOSE_DIR * OPEN_TARGET_STEPS)) % 4096)
    
    # Start driving both servos in their respective opening directions
    core.write_speed(id_left, HOMING_SPEED * OPEN_DIR, 50)
    core.write_speed(id_right, HOMING_SPEED * CLOSE_DIR, 50)
    
    stop_L, stop_R = False, False
    
    while not (stop_L and stop_R):
        # Check Left Servo Position
        if not stop_L:
            pos_L, _, _ = core.read_telemetry(id_left)
            if pos_L is not None:
                dist_L = get_distance_to_target(pos_L, limit_open_L, OPEN_DIR)
                # If we are within 10 steps of the target, OR we overshot it (> 2048), stop.
                if dist_L <= 10 or dist_L > 2048:
                    core.stop(id_left, 50)
                    stop_L = True
                    print(f"  > Left  Servo reached 45-deg open limit at: {pos_L}")

        # Check Right Servo Position
        if not stop_R:
            pos_R, _, _ = core.read_telemetry(id_right)
            if pos_R is not None:
                dist_R = get_distance_to_target(pos_R, limit_open_R, CLOSE_DIR)
                if dist_R <= 10 or dist_R > 2048:
                    core.stop(id_right, 50)
                    stop_R = True
                    print(f"  > Right Servo reached 45-deg open limit at: {pos_R}")

        time.sleep(0.01)

    time.sleep(0.5)
    
    # =======================================================
    # MAP LIMITS FOR MAIN CONTROL SCRIPT
    # =======================================================
    servo_limits = {id_left: {}, id_right: {}}
    
    # Map Left Limits (Closed using CLOSE_DIR, Opened using OPEN_DIR)
    if CLOSE_DIR == 1: 
        servo_limits[id_left]['cw'] = limit_close_L
        servo_limits[id_left]['ccw'] = limit_open_L
    else:              
        servo_limits[id_left]['ccw'] = limit_close_L
        servo_limits[id_left]['cw'] = limit_open_L

    # Map Right Limits (Closed using OPEN_DIR, Opened using CLOSE_DIR)
    if OPEN_DIR == 1: 
        servo_limits[id_right]['cw'] = limit_close_R
        servo_limits[id_right]['ccw'] = limit_open_R
    else:              
        servo_limits[id_right]['ccw'] = limit_close_R
        servo_limits[id_right]['cw'] = limit_open_R
            
    # Apply the 3-degree safety offset to the closed limits so we don't bind against the physical wall
    servo_limits[id_left]['cw'] = (servo_limits[id_left]['cw'] - (1 * CLOSE_DIR * OFFSET_STEPS)) % 4096
    servo_limits[id_left]['ccw'] = (servo_limits[id_left]['ccw'] - (1 * OPEN_DIR * OFFSET_STEPS)) % 4096
    
    servo_limits[id_right]['cw'] = (servo_limits[id_right]['cw'] - (1 * OPEN_DIR * OFFSET_STEPS)) % 4096
    servo_limits[id_right]['ccw'] = (servo_limits[id_right]['ccw'] - (1 * CLOSE_DIR * OFFSET_STEPS)) % 4096
    
    print("\n--- Calibration Complete ---")
    print(f"Left  Walls -> CW: {int(servo_limits[id_left]['cw'])} | CCW: {int(servo_limits[id_left]['ccw'])}")
    print(f"Right Walls -> CW: {int(servo_limits[id_right]['cw'])} | CCW: {int(servo_limits[id_right]['ccw'])}")

    return servo_limits


# -------------------------------------------------
# STANDALONE EXECUTION BLOCK
# -------------------------------------------------
if __name__ == "__main__":
    from st3215_core import ST3215Core
    
    ID_RIGHT = 1
    ID_LEFT  = 2
    
    print("--- Tutan-Khamun Derivative & Positional Homing Routine Test ---")
    try:
        core = ST3215Core(port='/dev/ttyACM0', baudrate=1000000)
        core.set_wheel_mode(ID_LEFT)
        core.set_wheel_mode(ID_RIGHT)
        
        final_limits = execute_homing(core, ID_LEFT, ID_RIGHT)
        
    except Exception as e:
        print(f"\n[ERROR] An issue occurred during homing: {e}")
        
    finally:
        print("\nShutting down safely and closing port...")
        try:
            core.stop(ID_LEFT)
            core.stop(ID_RIGHT)
            core.close()
        except:
            pass