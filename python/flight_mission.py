import time
import math
import sys
import apm
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from apm import TelemetryData

# Mission State Constants
MS_IDLE = "IDLE"
MS_ARMING = "ARMING"
MS_TAKEOFF = "TAKEOFF"
MS_FLY_FORWARD = "FLY_FORWARD"
MS_HOVER_1 = "HOVER_1"
MS_FLY_RIGHT = "FLY_RIGHT"
MS_HOVER_2 = "HOVER_2"
MS_LANDING_1 = "LANDING_1"
MS_GROUND_WAIT = "GROUND_WAIT"
MS_REARMING = "REARMING"
MS_RETAKEOFF = "RETAKEOFF"
MS_FLY_BACK_RIGHT = "FLY_BACK_RIGHT"
MS_HOVER_3 = "HOVER_3"
MS_FLY_BACK_FORWARD = "FLY_BACK_FORWARD"
MS_HOVER_4 = "HOVER_4"
MS_FINAL_LANDING = "FINAL_LANDING"
MS_DONE = "DONE"

MISSION_STATE = MS_IDLE
STATE_START_TIME = 0.0
STABILIZE_START_TIME = None
START_YAW = 0.0
TARGET_YAW = 0.0
LAST_ARM_TIME = 0.0
MISSION_LOGS = []

def set_mission_state(new_state):
    global MISSION_STATE, STATE_START_TIME, STABILIZE_START_TIME, MISSION_LOGS, LAST_ARM_TIME
    log_msg = f"[MISSION] Transition: {MISSION_STATE} -> {new_state}"
    sys.stdout.write(f"\n{log_msg}\n")
    sys.stdout.flush()
    MISSION_LOGS.append(log_msg)
    if len(MISSION_LOGS) > 3:
        MISSION_LOGS.pop(0)
    MISSION_STATE = new_state
    STATE_START_TIME = time.perf_counter()
    STABILIZE_START_TIME = None
    LAST_ARM_TIME = 0.0

def normalize_yaw(yaw):
    while yaw > 180.0:
        yaw -= 360.0
    while yaw < -180.0:
        yaw += 360.0
    return yaw

def compute_yaw_rate(target_yaw: float, current_yaw: float, max_rate: float = 30.0, kp: float = 1.0) -> float:
    """
    Computes desired yaw angular velocity (deg/s) to track target heading.
    When heading error is small (< 2 deg), returns 0.0 to prevent oscillation/drift.
    """
    err = normalize_yaw(target_yaw - current_yaw)
    if abs(err) < 2.0:
        return 0.0
    rate = kp * err
    return max(-max_rate, min(max_rate, rate))

ARM_CONSECUTIVE_COUNT = 0
LAST_RAW_ARM_STATE = None
STABLE_ARM_STATE = False

def get_stable_armed(telemetry: 'TelemetryData', required_frames: int = 5) -> bool:
    global ARM_CONSECUTIVE_COUNT, LAST_RAW_ARM_STATE, STABLE_ARM_STATE
    if not telemetry:
        return STABLE_ARM_STATE
    # sys_disarm_flag: False is ARMED, True is DISARMED
    raw_armed = (telemetry.get('sys_disarm_flag') is False)
    if raw_armed == LAST_RAW_ARM_STATE:
        ARM_CONSECUTIVE_COUNT += 1
        if ARM_CONSECUTIVE_COUNT >= required_frames:
            STABLE_ARM_STATE = raw_armed
    else:
        LAST_RAW_ARM_STATE = raw_armed
        ARM_CONSECUTIVE_COUNT = 1
    return STABLE_ARM_STATE

def start_mission(telemetry: 'TelemetryData', trigger_source: str = "B3 01"):
    global MISSION_STATE, MISSION_LOGS
    if MISSION_STATE == MS_IDLE:
        log_msg = f"[MISSION] Starting flight mission (Trigger: {trigger_source})..."
        sys.stdout.write(f"\n========================================\n{log_msg}\n========================================\n")
        sys.stdout.flush()
        MISSION_LOGS.append(log_msg)
        if len(MISSION_LOGS) > 3:
            MISSION_LOGS.pop(0)
        is_armed = get_stable_armed(telemetry)
        if is_armed:
            set_mission_state(MS_TAKEOFF)
        else:
            set_mission_state(MS_ARMING)

def run_mission_state_machine(telemetry: 'TelemetryData'):
    global MISSION_STATE, STATE_START_TIME, STABILIZE_START_TIME, START_YAW, TARGET_YAW, LAST_ARM_TIME
    
    if MISSION_STATE == MS_IDLE:
        return
        
    now = time.perf_counter()
    elapsed = now - STATE_START_TIME
    
    # Get altitude
    alt = 0.0
    nav_rel_pos = telemetry.get('nav_relative_pos')
    if nav_rel_pos and len(nav_rel_pos) >= 3 and nav_rel_pos[2] is not None:
        alt = nav_rel_pos[2]
        
    current_yaw = telemetry.get('att_euler_angle_yaw_v', 0.0)
    if current_yaw is None:
        current_yaw = 0.0

    if MISSION_STATE == MS_ARMING:
        # Check arm flag with debounce filter
        is_armed = get_stable_armed(telemetry)
        if is_armed:
            sys.stdout.write("\n[MISSION] Arm confirmed! Transitioning to TAKEOFF...\n")
            sys.stdout.flush()
            set_mission_state(MS_TAKEOFF)
        else:
            # Continuously pulse arm command
            apm.arm()
            if now - LAST_ARM_TIME >= 1.0:
                disarm_flag = telemetry.get('sys_arm_flag') if telemetry else None
                sys.stdout.write(f"\n[MISSION] MS_ARMING: Pulsing apm.arm() (sys_arm_flag={disarm_flag})...\n")
                sys.stdout.flush()
                LAST_ARM_TIME = now
                
    elif MISSION_STATE == MS_TAKEOFF:
        # Command takeoff to Z=50cm, keeping current heading
        apm.set_position(0, 0, 50, current_yaw, True)
        
        # Check stabilization around 50cm
        if 42.0 <= alt <= 58.0:
            if STABILIZE_START_TIME is None:
                STABILIZE_START_TIME = now
            elif now - STABILIZE_START_TIME >= 1.5:
                START_YAW = current_yaw
                TARGET_YAW = normalize_yaw(START_YAW - 90.0)
                set_mission_state(MS_FLY_FORWARD)
        else:
            STABILIZE_START_TIME = None
            
    elif MISSION_STATE == MS_FLY_FORWARD:
        # Fly forward in body frame at 20 cm/s for 2.0s
        if elapsed >= 2.0:
            set_mission_state(MS_HOVER_1)
        else:
            apm.set_speed(20, 0, 0, 0.0)
            
    elif MISSION_STATE == MS_HOVER_1:
        # Turn to TARGET_YAW and reset home at current point (X=0, Y=0)
        target_alt = int(alt) if alt >= 30.0 else 50
        apm.set_position(0, 0, target_alt, TARGET_YAW, True)
        
        yaw_err = abs(normalize_yaw(current_yaw - TARGET_YAW))
        if yaw_err < 5.0 or elapsed >= 2.0:
            set_mission_state(MS_FLY_RIGHT)
            
    elif MISSION_STATE == MS_FLY_RIGHT:
        # Body has turned: fly forward along current heading at 20 cm/s for 2.0s
        if elapsed >= 2.0:
            set_mission_state(MS_HOVER_2)
        else:
            apm.set_speed(20, 0, 0, 0.0)
            
    elif MISSION_STATE == MS_HOVER_2:
        # Hover for 5.0s at current position (reset home)
        target_alt = int(alt) if alt >= 30.0 else 50
        apm.set_position(0, 0, target_alt, TARGET_YAW, True)
        if elapsed >= 5.0:
            set_mission_state(MS_LANDING_1)
            
    elif MISSION_STATE == MS_LANDING_1:
        # Land in place at 50 cm/s descent
        if alt <= 3.0:
            sys.stdout.write(f"\n[MISSION] Touchdown confirmed (alt={alt:.1f}cm <= 3cm). Disarming, setting speed to 0 and resetting Home...\n")
            sys.stdout.flush()
            apm.disarm()
            apm.set_speed(0, 0, 0, 0.0)
            apm.set_position(0, 0, 0, TARGET_YAW, True)
            set_mission_state(MS_GROUND_WAIT)
        else:
            apm.set_speed(0, 0, 50, 0.0)
            
    elif MISSION_STATE == MS_GROUND_WAIT:
        # Wait on ground for 5.0s
        apm.set_speed(0, 0, 0, 0.0)
        if elapsed >= 5.0:
            set_mission_state(MS_REARMING)
            
    elif MISSION_STATE == MS_REARMING:
        # Arm again with debounce filter
        is_armed = get_stable_armed(telemetry)
        if is_armed:
            sys.stdout.write("\n[MISSION] Re-arm confirmed! Transitioning to RETAKEOFF...\n")
            sys.stdout.flush()
            set_mission_state(MS_RETAKEOFF)
        else:
            # Continuously pulse arm command on ground
            apm.arm()
            if now - LAST_ARM_TIME >= 1.0:
                disarm_flag = telemetry.get('sys_disarm_flag') if telemetry else None
                sys.stdout.write(f"\n[MISSION] MS_REARMING: Pulsing apm.arm() (sys_disarm_flag={disarm_flag}, alt={alt:.1f}cm)...\n")
                sys.stdout.flush()
                LAST_ARM_TIME = now
                
    elif MISSION_STATE == MS_RETAKEOFF:
        # Takeoff again to 50cm
        apm.set_position(0, 0, 50, TARGET_YAW, True)
        
        # Check stabilization around 50cm
        if 42.0 <= alt <= 58.0:
            if STABILIZE_START_TIME is None:
                STABILIZE_START_TIME = now
            elif now - STABILIZE_START_TIME >= 1.5:
                set_mission_state(MS_FLY_BACK_RIGHT)
        else:
            STABILIZE_START_TIME = None
            
    elif MISSION_STATE == MS_FLY_BACK_RIGHT:
        # Fly backward along current body axis at -20 cm/s for 2.0s
        if elapsed >= 2.0:
            set_mission_state(MS_HOVER_3)
        else:
            apm.set_speed(-20, 0, 0, 0.0)
            
    elif MISSION_STATE == MS_HOVER_3:
        # Turn back to START_YAW and reset home at current point (X=0, Y=0)
        target_alt = int(alt) if alt >= 30.0 else 50
        apm.set_position(0, 0, target_alt, START_YAW, True)
        
        yaw_err = abs(normalize_yaw(current_yaw - START_YAW))
        if yaw_err < 5.0 or elapsed >= 2.0:
            set_mission_state(MS_FLY_BACK_FORWARD)
            
    elif MISSION_STATE == MS_FLY_BACK_FORWARD:
        # Fly backward along initial body axis at -20 cm/s for 2.0s
        if elapsed >= 2.0:
            set_mission_state(MS_HOVER_4)
        else:
            apm.set_speed(-20, 0, 0, 0.0)
            
    elif MISSION_STATE == MS_HOVER_4:
        # Hover for 1.0s at current position (reset home)
        target_alt = int(alt) if alt >= 30.0 else 50
        apm.set_position(0, 0, target_alt, START_YAW, True)
        if elapsed >= 1.0:
            set_mission_state(MS_FINAL_LANDING)
            
    elif MISSION_STATE == MS_FINAL_LANDING:
        # Land in place at 50 cm/s
        if alt <= 3.0:
            sys.stdout.write(f"\n[MISSION] Final Touchdown confirmed (alt={alt:.1f}cm <= 3cm). Disarming and setting speed to 0...\n")
            sys.stdout.flush()
            apm.disarm()
            apm.set_speed(0, 0, 0, 0.0)
            set_mission_state(MS_DONE)
        else:
            apm.set_speed(0, 0, 50, 0.0)
            
    elif MISSION_STATE == MS_DONE:
        set_mission_state(MS_IDLE)
