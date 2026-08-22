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
MISSION_LOGS = []

def set_mission_state(new_state):
    global MISSION_STATE, STATE_START_TIME, STABILIZE_START_TIME, MISSION_LOGS
    log_msg = f"[MISSION] Transition: {MISSION_STATE} -> {new_state}"
    MISSION_LOGS.append(log_msg)
    if len(MISSION_LOGS) > 3:
        MISSION_LOGS.pop(0)
    MISSION_STATE = new_state
    STATE_START_TIME = time.perf_counter()
    STABILIZE_START_TIME = None

def normalize_yaw(yaw):
    while yaw > 180.0:
        yaw -= 360.0
    while yaw < -180.0:
        yaw += 360.0
    return yaw

def start_mission(telemetry: 'TelemetryData'):
    global MISSION_STATE, MISSION_LOGS
    if MISSION_STATE == MS_IDLE:
        log_msg = "[MISSION] Recv B3 01: Starting mission..."
        MISSION_LOGS.append(log_msg)
        if len(MISSION_LOGS) > 3:
            MISSION_LOGS.pop(0)
        # sys_arm_flag: False = Armed (unlocked), True = Disarmed (locked)
        armed = (telemetry.get('sys_arm_flag') is False) if telemetry else False
        if armed:
            set_mission_state(MS_TAKEOFF)
        else:
            set_mission_state(MS_ARMING)

def run_mission_state_machine(telemetry: 'TelemetryData'):
    global MISSION_STATE, STATE_START_TIME, STABILIZE_START_TIME, START_YAW, TARGET_YAW
    
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
        # Check arm flag (False = Armed / unlocked)
        armed = (telemetry.get('sys_arm_flag') is False)
        if armed:
            set_mission_state(MS_TAKEOFF)
        else:
            # Keep sending arm command every 1 second
            if elapsed >= 1.0:
                apm.arm()
                STATE_START_TIME = now
                
    elif MISSION_STATE == MS_TAKEOFF:
        # Command takeoff to Z=100cm, keeping current heading
        apm.set_position(0, 0, 100, current_yaw, True)
        
        # Check stabilization
        if 90.0 <= alt <= 110.0:
            if STABILIZE_START_TIME is None:
                STABILIZE_START_TIME = now
            elif now - STABILIZE_START_TIME >= 1.5:
                START_YAW = current_yaw
                TARGET_YAW = normalize_yaw(START_YAW + 90.0)
                set_mission_state(MS_FLY_FORWARD)
        else:
            STABILIZE_START_TIME = None
            
    elif MISSION_STATE == MS_FLY_FORWARD:
        # Fly forward at 20 cm/s for 2.0s
        if elapsed >= 2.0:
            set_mission_state(MS_HOVER_1)
        else:
            yaw_rad = math.radians(START_YAW)
            vx = 20.0 * math.cos(yaw_rad)
            vy = 20.0 * math.sin(yaw_rad)
            apm.set_speed(int(vx), int(vy), 0, START_YAW)
            
    elif MISSION_STATE == MS_HOVER_1:
        # Hover for 1.0s
        if elapsed >= 1.0:
            set_mission_state(MS_FLY_RIGHT)
        else:
            apm.set_speed(0, 0, 0, START_YAW)
            
    elif MISSION_STATE == MS_FLY_RIGHT:
        # Turn right 90 deg and fly forward at 20 cm/s for 2.0s
        if elapsed >= 2.0:
            set_mission_state(MS_HOVER_2)
        else:
            yaw_rad = math.radians(TARGET_YAW)
            vx = 20.0 * math.cos(yaw_rad)
            vy = 20.0 * math.sin(yaw_rad)
            apm.set_speed(int(vx), int(vy), 0, TARGET_YAW)
            
    elif MISSION_STATE == MS_HOVER_2:
        # Hover for 5.0s
        if elapsed >= 5.0:
            set_mission_state(MS_LANDING_1)
        else:
            apm.set_speed(0, 0, 0, TARGET_YAW)
            
    elif MISSION_STATE == MS_LANDING_1:
        # Land in place at 50 cm/s
        if alt <= 5.0:
            apm.disarm()
            set_mission_state(MS_GROUND_WAIT)
        else:
            apm.set_speed(0, 0, 50, TARGET_YAW)
            
    elif MISSION_STATE == MS_GROUND_WAIT:
        # Wait on ground for 5.0s
        if elapsed >= 5.0:
            set_mission_state(MS_REARMING)
            
    elif MISSION_STATE == MS_REARMING:
        # Arm again (False = Armed / unlocked)
        armed = (telemetry.get('sys_arm_flag') is False)
        if armed:
            set_mission_state(MS_RETAKEOFF)
        else:
            if elapsed >= 1.0:
                apm.arm()
                STATE_START_TIME = now
                
    elif MISSION_STATE == MS_RETAKEOFF:
        # Takeoff again to 100cm
        apm.set_position(0, 0, 100, TARGET_YAW, True)
        
        # Check stabilization
        if 90.0 <= alt <= 110.0:
            if STABILIZE_START_TIME is None:
                STABILIZE_START_TIME = now
            elif now - STABILIZE_START_TIME >= 1.5:
                set_mission_state(MS_FLY_BACK_RIGHT)
        else:
            STABILIZE_START_TIME = None
            
    elif MISSION_STATE == MS_FLY_BACK_RIGHT:
        # Fly backward along TARGET_YAW at -20 cm/s for 2.0s
        if elapsed >= 2.0:
            set_mission_state(MS_HOVER_3)
        else:
            yaw_rad = math.radians(TARGET_YAW)
            vx = -20.0 * math.cos(yaw_rad)
            vy = -20.0 * math.sin(yaw_rad)
            apm.set_speed(int(vx), int(vy), 0, TARGET_YAW)
            
    elif MISSION_STATE == MS_HOVER_3:
        # Hover for 1.0s
        if elapsed >= 1.0:
            set_mission_state(MS_FLY_BACK_FORWARD)
        else:
            apm.set_speed(0, 0, 0, TARGET_YAW)
            
    elif MISSION_STATE == MS_FLY_BACK_FORWARD:
        # Fly backward along START_YAW at -20 cm/s for 2.0s
        if elapsed >= 2.0:
            set_mission_state(MS_HOVER_4)
        else:
            yaw_rad = math.radians(START_YAW)
            vx = -20.0 * math.cos(yaw_rad)
            vy = -20.0 * math.sin(yaw_rad)
            apm.set_speed(int(vx), int(vy), 0, START_YAW)
            
    elif MISSION_STATE == MS_HOVER_4:
        # Hover for 1.0s
        if elapsed >= 1.0:
            set_mission_state(MS_FINAL_LANDING)
        else:
            apm.set_speed(0, 0, 0, START_YAW)
            
    elif MISSION_STATE == MS_FINAL_LANDING:
        # Land in place at 50 cm/s
        if alt <= 5.0:
            apm.disarm()
            set_mission_state(MS_DONE)
        else:
            apm.set_speed(0, 0, 50, START_YAW)
            
    elif MISSION_STATE == MS_DONE:
        set_mission_state(MS_IDLE)
