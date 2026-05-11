#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

# --- HOMING SETTINGS ---
HOMING_SPEED          = 800     
HOMING_LOAD_THRESHOLD = 370     
OFFSET_DEGREES        = 3.0     
OFFSET_STEPS          = int((OFFSET_DEGREES / 360.0) * 4096)  

# --- ORIENTATION SETTINGS ---
CLOSE_DIR = -1   # -1 = CCW, 1 = CW
OPEN_DIR  = 1    # 1 = CW, -1 = CCW

def drive_single_until_stop(core, servo_id, direction, accel=50):
    """Drives a single servo until it hits a hard stop (current spike)."""
    core.write_speed(servo_id, HOMING_SPEED * direction, accel)
    time.sleep(0.3) # Wait for initial starting inertia to settle
    
    spike_count = 0
    pos_stop = None
    
    while True:
        pos, spd, load = core.read_telemetry(servo_id)
        if load is not None and load > HOMING_LOAD_THRESHOLD:
            spike_count += 1
            if spike_count >= 3:
                core.stop(servo_id, accel)
                pos_stop = pos
                dir_str = "CW" if direction > 0 else "CCW"
                print(f"  > Servo ID {servo_id} hit {dir_str} hard stop at: {pos_stop}")
                break
        else:
            spike_count = 0
        time.sleep(0.01)
        
    return pos_stop

def drive_until_stop(core, id_left, id_right, dir_L, dir_R, accel=50):
    """Drives both servos simultaneously until they both hit a hard stop."""
    core.write_speed(id_left, HOMING_SPEED * dir_L, accel)
    core.write_speed(id_right, HOMING_SPEED * dir_R, accel)
    time.sleep(0.3)
    
    stop_L, stop_R = False, False
    spike_L, spike_R = 0, 0
    pos_L_stop, pos_R_stop = None, None
    
    while not (stop_L and stop_R):
        if not stop_L:
            pos, spd, load = core.read_telemetry(id_left)
            if load is not None and load > HOMING_LOAD_THRESHOLD:
                spike_L += 1
                if spike_L >= 3:
                    core.stop(id_left, accel)
                    pos_L_stop = pos
                    stop_L = True
                    print(f"  > Left  Servo hit hard stop at: {pos_L_stop}")
            else:
                spike_L = 0
                
        if not stop_R:
            pos, spd, load = core.read_telemetry(id_right)
            if load is not None and load > HOMING_LOAD_THRESHOLD:
                spike_R += 1
                if spike_R >= 3:
                    core.stop(id_right, accel)
                    pos_R_stop = pos
                    stop_R = True
                    print(f"  > Right Servo hit hard stop at: {pos_R_stop}")
            else:
                spike_R = 0
        time.sleep(0.01)
        
    return pos_L_stop, pos_R_stop

def execute_homing(core, id_left, id_right):
    """Executes the full Tutan-Khamun sequential close / simultaneous open sequence."""
    print("\n\n[WARNING] Starting Sensorless Homing Sequence...")
    print("Keep hands clear! Servos will seek hard stops.")
    
    # Phase 1: Sequential Closing
    print("\n--- Phase 1: Seeking Inner Limits (Sequential Closing) ---")
    print(f"Moving Right Servo (ID {id_right}) to close position...")
    limit_close_R = drive_single_until_stop(core, id_right, OPEN_DIR) 
    time.sleep(0.5) 
    
    print(f"Moving Left Servo (ID {id_left}) to close position...")
    limit_close_L = drive_single_until_stop(core, id_left, CLOSE_DIR) 
    time.sleep(0.5)
    
    # Phase 2: Simultaneous Opening
    print("\n--- Phase 2: Seeking Outer Limits (Simultaneous Opening) ---")
    limit_open_L, limit_open_R = drive_until_stop(core, id_left, id_right, OPEN_DIR, CLOSE_DIR) 
    time.sleep(0.5)
    
    # Map limits to physical CW and CCW walls based on orientation
    servo_limits = {id_left: {}, id_right: {}}
    
    for servo_id, lim_close, lim_open in [(id_right, limit_close_R, limit_open_R), (id_left, limit_close_L, limit_open_L)]:
        if CLOSE_DIR == 1: 
            servo_limits[servo_id]['cw'] = lim_close
            servo_limits[servo_id]['ccw'] = lim_open
        else:              
            servo_limits[servo_id]['ccw'] = lim_close
            servo_limits[servo_id]['cw'] = lim_open
            
    # Apply safety offset
    servo_limits[id_left]['cw'] = (servo_limits[id_left]['cw'] - (1 * CLOSE_DIR * OFFSET_STEPS)) % 4096
    servo_limits[id_left]['ccw'] = (servo_limits[id_left]['ccw'] - (1 * OPEN_DIR * OFFSET_STEPS)) % 4096
    
    servo_limits[id_right]['cw'] = (servo_limits[id_right]['cw'] - (1 * CLOSE_DIR * OFFSET_STEPS)) % 4096
    servo_limits[id_right]['ccw'] = (servo_limits[id_right]['ccw'] - (1 * OPEN_DIR * OFFSET_STEPS)) % 4096
    
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
    
    print("--- Tutan-Khamun Homing Routine Test ---")
    try:
        # Initialize the core hardware layer
        core = ST3215Core(port='/dev/ttyACM0', baudrate=1000000)
        
        # Servos must be in wheel mode to drive using velocity commands
        core.set_wheel_mode(ID_LEFT)
        core.set_wheel_mode(ID_RIGHT)
        
        # Run the sequence
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
