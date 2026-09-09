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
MS_TURN_180 = "TURN_180"
MS_FLY_RETURN_1 = "FLY_RETURN_1"
MS_HOVER_3 = "HOVER_3"
MS_FLY_RETURN_2 = "FLY_RETURN_2"
MS_HOVER_4 = "HOVER_4"
MS_FINAL_LANDING = "FINAL_LANDING"
MS_DONE = "DONE"

MISSION_STATE = MS_IDLE
STATE_START_TIME = 0.0
STABILIZE_START_TIME = None
TOUCHDOWN_START_TIME = None
TAKEOFF_HEADING = None
IS_STATE_FIRST_FRAME = True
START_YAW = 0.0
TARGET_YAW = 0.0
YAW_RETURN_1 = 0.0
YAW_RETURN_2 = 0.0
LAST_ARM_TIME = 0.0
LAST_LOG_TIME = 0.0
MISSION_LOGS = []

def set_mission_state(new_state):
    global MISSION_STATE, STATE_START_TIME, STABILIZE_START_TIME, TOUCHDOWN_START_TIME, TAKEOFF_HEADING
    global IS_STATE_FIRST_FRAME, MISSION_LOGS, LAST_ARM_TIME, LAST_LOG_TIME
    log_msg = f"[MISSION] Transition: {MISSION_STATE} -> {new_state}"
    sys.stdout.write(f"\n{log_msg}\n")
    sys.stdout.flush()
    MISSION_LOGS.append(log_msg)
    if len(MISSION_LOGS) > 3:
        MISSION_LOGS.pop(0)
    MISSION_STATE = new_state
    STATE_START_TIME = time.perf_counter()
    STABILIZE_START_TIME = None
    TOUCHDOWN_START_TIME = None
    TAKEOFF_HEADING = None
    IS_STATE_FIRST_FRAME = True
    LAST_ARM_TIME = 0.0
    LAST_LOG_TIME = 0.0

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
        return False
    # sys_disarm_flag: False is ARMED, True is DISARMED
    disarm_flag = telemetry.get('sys_disarm_flag')
    if disarm_flag is None:
        disarm_flag = telemetry.get('sys_arm_flag')
    if disarm_flag is None:
        return False
    raw_armed = (disarm_flag is False)
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
    global MISSION_STATE, STATE_START_TIME, STABILIZE_START_TIME, TOUCHDOWN_START_TIME, TAKEOFF_HEADING
    global IS_STATE_FIRST_FRAME, START_YAW, TARGET_YAW, YAW_RETURN_1, YAW_RETURN_2, LAST_ARM_TIME, LAST_LOG_TIME
    global STABLE_ARM_STATE, LAST_RAW_ARM_STATE, ARM_CONSECUTIVE_COUNT
    
    if MISSION_STATE == MS_IDLE:
        return
        
    now = time.perf_counter()
    elapsed = now - STATE_START_TIME
    is_first_frame = IS_STATE_FIRST_FRAME
    IS_STATE_FIRST_FRAME = False
    
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
            set_mission_state(MS_TAKEOFF)
        else:
            # Continuously pulse arm command
            apm.arm()
                
    elif MISSION_STATE == MS_TAKEOFF:
        # Lock ground heading so drone climbs straight without turning on ground
        if TAKEOFF_HEADING is None:
            TAKEOFF_HEADING = current_yaw
        apm.set_position(0, 0, 50, TAKEOFF_HEADING, is_first_frame)
        
        # Check stabilization around 50cm
        if 42.0 <= alt <= 58.0:
            if STABILIZE_START_TIME is None:
                STABILIZE_START_TIME = now
            elif now - STABILIZE_START_TIME >= 1.5:
                START_YAW = current_yaw
                TARGET_YAW = normalize_yaw(START_YAW - 90.0)
                YAW_RETURN_1 = normalize_yaw(TARGET_YAW + 180.0)
                YAW_RETURN_2 = normalize_yaw(START_YAW + 180.0)
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
        # Turn to TARGET_YAW in the air: lock current point as origin (0,0,50) and rotate
        apm.set_position(0, 0, 50, TARGET_YAW, is_first_frame)
        
        yaw_err = abs(normalize_yaw(current_yaw - TARGET_YAW))
        if yaw_err < 3.0:
            if STABILIZE_START_TIME is None:
                STABILIZE_START_TIME = now
            elif now - STABILIZE_START_TIME >= 1.0:
                set_mission_state(MS_FLY_RIGHT)
        else:
            STABILIZE_START_TIME = None
            
    elif MISSION_STATE == MS_FLY_RIGHT:
        # Body has turned: fly forward along current heading at 20 cm/s for 2.0s
        if elapsed >= 2.0:
            set_mission_state(MS_HOVER_2)
        else:
            apm.set_speed(20, 0, 0, 0.0)
            
    elif MISSION_STATE == MS_HOVER_2:
        # Hover for 5.0s at current position in the air: lock current point as origin (0,0,50)
        apm.set_position(0, 0, 50, TARGET_YAW, is_first_frame)
        if elapsed >= 5.0:
            set_mission_state(MS_LANDING_1)
            
    elif MISSION_STATE == MS_LANDING_1:
        # Land in place at 50 cm/s descent, wait 2 seconds after reaching <= 3cm before locking
        if alt <= 3.0:
            if TOUCHDOWN_START_TIME is None:
                TOUCHDOWN_START_TIME = now
            elif now - TOUCHDOWN_START_TIME >= 2.0:
                apm.disarm()
                apm.set_speed(0, 0, 0, 0.0)
                STABLE_ARM_STATE = False
                LAST_RAW_ARM_STATE = False
                ARM_CONSECUTIVE_COUNT = 0
                set_mission_state(MS_GROUND_WAIT)
        elif elapsed >= 3.0 and alt <= 15.0:
            # Fallback timeout if ground sensor stays between 3-15cm
            if TOUCHDOWN_START_TIME is None:
                TOUCHDOWN_START_TIME = now
            elif now - TOUCHDOWN_START_TIME >= 2.0:
                apm.disarm()
                apm.set_speed(0, 0, 0, 0.0)
                STABLE_ARM_STATE = False
                LAST_RAW_ARM_STATE = False
                ARM_CONSECUTIVE_COUNT = 0
                set_mission_state(MS_GROUND_WAIT)
        else:
            TOUCHDOWN_START_TIME = None

        if MISSION_STATE == MS_LANDING_1:
            apm.set_speed(0, 0, 50, 0.0)
            
    elif MISSION_STATE == MS_GROUND_WAIT:
        # Wait on ground for 5.0s
        if elapsed >= 5.0:
            set_mission_state(MS_REARMING)
            
    elif MISSION_STATE == MS_REARMING:
        # Check actual arm flag from telemetry
        is_armed = get_stable_armed(telemetry)
        if is_armed:
            set_mission_state(MS_RETAKEOFF)
        else:
            # Pulse arm command on ground to trigger takeoff
            apm.arm()
                
    elif MISSION_STATE == MS_RETAKEOFF:
        # Lock ground heading during retakeoff so it climbs straight without rotating on ground
        if TAKEOFF_HEADING is None:
            TAKEOFF_HEADING = current_yaw
        apm.arm()
        apm.set_position(0, 0, 50, TAKEOFF_HEADING, is_first_frame)
        
        # Check stabilization around 50cm
        if 42.0 <= alt <= 58.0:
            if STABILIZE_START_TIME is None:
                STABILIZE_START_TIME = now
            elif now - STABILIZE_START_TIME >= 1.5:
                set_mission_state(MS_TURN_180)
        else:
            STABILIZE_START_TIME = None
            
    elif MISSION_STATE == MS_TURN_180:
        # Turn 180 degrees in the air to face back towards Corner 1
        apm.set_position(0, 0, 50, YAW_RETURN_1, is_first_frame)
        
        yaw_err = abs(normalize_yaw(current_yaw - YAW_RETURN_1))
        if yaw_err < 3.0:
            if STABILIZE_START_TIME is None:
                STABILIZE_START_TIME = now
            elif now - STABILIZE_START_TIME >= 1.0:
                set_mission_state(MS_FLY_RETURN_1)
        else:
            STABILIZE_START_TIME = None
            
    elif MISSION_STATE == MS_FLY_RETURN_1:
        # Fly FORWARD in body frame at 20 cm/s for 2.0s back to Corner 1
        if elapsed >= 2.0:
            set_mission_state(MS_HOVER_3)
        else:
            apm.set_speed(20, 0, 0, 0.0)
            
    elif MISSION_STATE == MS_HOVER_3:
        # Turn at Corner 1 to face Home (YAW_RETURN_2)
        apm.set_position(0, 0, 50, YAW_RETURN_2, is_first_frame)
        
        yaw_err = abs(normalize_yaw(current_yaw - YAW_RETURN_2))
        if yaw_err < 3.0:
            if STABILIZE_START_TIME is None:
                STABILIZE_START_TIME = now
            elif now - STABILIZE_START_TIME >= 1.0:
                set_mission_state(MS_FLY_RETURN_2)
        else:
            STABILIZE_START_TIME = None
            
    elif MISSION_STATE == MS_FLY_RETURN_2:
        # Fly FORWARD in body frame at 20 cm/s for 2.0s back to Home
        if elapsed >= 2.0:
            set_mission_state(MS_HOVER_4)
        else:
            apm.set_speed(20, 0, 0, 0.0)
            
    elif MISSION_STATE == MS_HOVER_4:
        # Hover for 1.5s at current position in the air before final landing
        apm.set_position(0, 0, 50, YAW_RETURN_2, is_first_frame)
        if elapsed >= 1.5:
            set_mission_state(MS_FINAL_LANDING)
            
    elif MISSION_STATE == MS_FINAL_LANDING:
        # Land in place at 50 cm/s, wait 2 seconds after reaching <= 3cm before locking
        if alt <= 3.0:
            if TOUCHDOWN_START_TIME is None:
                TOUCHDOWN_START_TIME = now
            elif now - TOUCHDOWN_START_TIME >= 2.0:
                apm.disarm()
                apm.set_speed(0, 0, 0, 0.0)
                set_mission_state(MS_DONE)
        elif elapsed >= 3.0 and alt <= 15.0:
            if TOUCHDOWN_START_TIME is None:
                TOUCHDOWN_START_TIME = now
            elif now - TOUCHDOWN_START_TIME >= 2.0:
                apm.disarm()
                apm.set_speed(0, 0, 0, 0.0)
                set_mission_state(MS_DONE)
        else:
            TOUCHDOWN_START_TIME = None

        if MISSION_STATE == MS_FINAL_LANDING:
            apm.set_speed(0, 0, 50, 0.0)
            
    elif MISSION_STATE == MS_DONE:
        set_mission_state(MS_IDLE)
