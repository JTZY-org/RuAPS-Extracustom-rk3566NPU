# /etc/rknn/user_app.py
import numpy as np
import cv2
import sys
import apm
import time
import math
from typing import TYPE_CHECKING
import flight_mission
import color_detector
import human_tracker

if TYPE_CHECKING:
    from apm import TelemetryData

INITIAL_STATUS_CHECKED = False

def init(width: int, height: int, pixfmt: int) -> int:
    """
    Called once during UserAppInit
    :param width: Image width
    :param height: Image height
    :param pixfmt: Image pixel format (V4L2 format code)
    """
    print(f"[Python] init() called: width={width}, height={height}, pixfmt={pixfmt}")
    # Return 0 or initialization status
    return 0

# Global frame counter and timer for FPS and Latency measurement
FRAME_COUNTER = 0
LAST_CALL_TIME = None
SUM_LATENCY = 0.0
SUM_INTERVAL = 0.0

# Python landing logic states
PYTHON_LANDING = False
PYTHON_LANDING_START_TIME = None

def debug_print_telemetry(telemetry: 'TelemetryData'):
    if not telemetry:
        return
    
    items = []
    
    # 1. Scalars
    scalars = [
        "sys_disarm_flag", "sys_pre_arm_flag", "sys_failsafe_flag", "sys_apm_status",
        "cpu_temp", "battery_voltage", "battery_voltage_single", "gyro_cycle_time",
        "baro_temp", "baro_pressure_hpa", "baro_agl_altitude_cm", "rangefinder_agl_alt_cm",
        "accel_clipped_times", "accel_gforce", "att_euler_angle_yaw_v",
        "nav_relative_head", "nav_global_head", "nav_global_sat_count", "nav_gps_hdop"
    ]
    for s in scalars:
        val = telemetry.get(s)
        val_str = f"{val:.2f}" if isinstance(val, float) else str(val)
        items.append(f"{s}:{val_str}")
        
    # 2. Arrays
    arrays = {
        "accel_acc": "accel_acceleration",
        "accel_vibe": "accel_vibe",
        "accel_raw_g": "accel_raw_g",
        "att_quat": "att_quaternion",
        "att_euler": "att_euler_angle",
        "gyro_rate": "gyro_angle_rate",
        "mag_raw": "mag_raw_l",
        "nav_speed": "nav_speed",
        "nav_g_speed": "nav_global_speed",
        "nav_g_pos": "nav_global_pos",
        "nav_g_home": "nav_global_home",
        "nav_rel_pos": "nav_relative_pos"
    }
    for prefix, key in arrays.items():
        arr = telemetry.get(key)
        if isinstance(arr, list):
            for idx, val in enumerate(arr):
                val_str = f"{val:.2f}" if isinstance(val, float) else str(val)
                items.append(f"{prefix}[{idx}]:{val_str}")
                
    # 3. RC / EF channels
    for chan_type in ["rc_channel_raw", "ef_channel_raw"]:
        arr = telemetry.get(chan_type)
        prefix = "rc" if chan_type == "rc_channel_raw" else "ef"
        if isinstance(arr, list):
            for idx, val in enumerate(arr):
                if val is not None:
                    items.append(f"{prefix}[{idx}]:{val}")

    # Format output: 4 items per line, fixed-width columns
    lines = ["\033[H\033[2J", "=== TELEMETRY MONITOR (4 ITEMS/LINE) ==="]
    
    # Live Mission Dashboard at top of monitor
    elapsed_mission = time.perf_counter() - flight_mission.STATE_START_TIME if flight_mission.MISSION_STATE != flight_mission.MS_IDLE else 0.0
    lines.append(f"MISSION STATE: {flight_mission.MISSION_STATE:<12} | Elapsed: {elapsed_mission:.1f}s | Start Yaw: {flight_mission.START_YAW:.1f} | Target Yaw: {flight_mission.TARGET_YAW:.1f}")
    lines.append(f"HUMAN TRACKER: {human_tracker.g_human_tracker.get_status_str()}")
    
    recv_packets = telemetry.get('BroadcastRecv', [])
    recv_hex = ", ".join(p.hex().upper() for p in recv_packets) if recv_packets else "None"
    lines.append(f"LAST RECV UDP: {recv_hex:<25}")
    lines.append("---------------------------------------------------------------------------------")
    
    for i in range(0, len(items), 4):
        chunk = items[i:i+4]
        line_str = " | ".join(f"{item:<22}" for item in chunk)
        lines.append(line_str)
        
    lines.append("---------------------------------------------------------------------------------")
    lines.append("Recent Mission / Tracker Logs:")
    for log in flight_mission.MISSION_LOGS:
        lines.append(f"  {log}")
    for log in human_tracker.g_human_tracker.logs:
        lines.append(f"  {log}")
    lines.append("=================================================================================")
    
    sys.stdout.write("\n".join(lines) + "\n")
    sys.stdout.flush()

def exchange(frame_bytes: bytes, width: int, height: int, pixfmt: int, telemetry: 'TelemetryData') -> int:
    global FRAME_COUNTER, LAST_CALL_TIME, SUM_LATENCY, SUM_INTERVAL, PYTHON_LANDING, PYTHON_LANDING_START_TIME, INITIAL_STATUS_CHECKED
    start_time = time.perf_counter()
    
    # if telemetry and FRAME_COUNTER % 5 == 0:
    #     debug_print_telemetry(telemetry)
        
    if telemetry and not INITIAL_STATUS_CHECKED:
        is_disarmed = telemetry.get('sys_disarm_flag')
        status = "LOCKED (DISARMED)" if is_disarmed is True else "UNLOCKED (ARMED)"
        log_msg = f"[MISSION] Initial lock status check: {status}"
        flight_mission.MISSION_LOGS.append(log_msg)
        INITIAL_STATUS_CHECKED = True
        
    if telemetry:
        flight_mission.run_mission_state_machine(telemetry)
    
    # Measure call interval (actual calling FPS)
    current_time = start_time
    if LAST_CALL_TIME is not None:
        interval = current_time - LAST_CALL_TIME
        SUM_INTERVAL += interval
    LAST_CALL_TIME = current_time

    # Print BroadcastRecv packets if any are received
    if telemetry and 'BroadcastRecv' in telemetry:
        broadcast_packets = telemetry['BroadcastRecv']
        if broadcast_packets:
            sys.stdout.write(f"\n[Python] BroadcastRecv Packets: {[p.hex() for p in broadcast_packets]}\n")
            sys.stdout.flush()
            for p in broadcast_packets:
                if len(p) >= 2 and p[0] == 0xCB and p[1] == 0x01:
                    sys.stdout.write("\n[Python] Received CB 01: Arming flight controller\n")
                    sys.stdout.flush()
                    apm.arm()
                elif len(p) >= 2 and p[0] == 0xCB and p[1] == 0x00:
                    sys.stdout.write("\n[Python] Received CB 00: Disarming flight controller\n")
                    sys.stdout.flush()
                    apm.disarm()
                elif len(p) >= 2 and p[0] == 0xB1 and p[1] == 0x01:
                    if not PYTHON_LANDING:
                        sys.stdout.write("\n[Python] Received B1 01: Starting speed landing (50cm/s descent)\n")
                        sys.stdout.flush()
                        PYTHON_LANDING = True
                elif len(p) >= 2 and p[0] == 0xB3 and p[1] == 0x01:
                    sys.stdout.write("\n[Python] Received B3 01: Starting flight mission...\n")
                    sys.stdout.flush()
                    flight_mission.start_mission(telemetry)
                elif len(p) >= 2 and p[0] == 0xB4 and p[1] == 0x01:
                    sys.stdout.write("\n[Python] Received B4 01: Starting human yaw tracking...\n")
                    sys.stdout.flush()
                    human_tracker.g_human_tracker.start_tracking(telemetry)
                elif len(p) >= 2 and p[0] == 0xB4 and p[1] == 0x00:
                    sys.stdout.write("\n[Python] Received B4 00: Stopping human yaw tracking...\n")
                    sys.stdout.flush()
                    human_tracker.g_human_tracker.stop_tracking()

    # Process Python Landing Telemetry Verification
    if PYTHON_LANDING:
        apm.set_speed(0, 0, 50, 0.0)
        
        nav_relative_pos = telemetry.get('nav_relative_pos') if telemetry else None
        if nav_relative_pos and len(nav_relative_pos) >= 3 and nav_relative_pos[2] is not None:
            current_alt = nav_relative_pos[2]
            if current_alt <= 5.0:
                sys.stdout.write(f"\n[Python Landing] Alt: {current_alt:.2f} cm < 5cm. Disarming...\n")
                sys.stdout.flush()
                apm.disarm()
                PYTHON_LANDING = False

    # Process Human Target Yaw Tracking (when not in landing or mission state)
    if telemetry and not PYTHON_LANDING and flight_mission.MISSION_STATE == flight_mission.MS_IDLE:
        human_tracker.g_human_tracker.update(telemetry, width, height)

    try:
        FRAME_COUNTER += 1
        
        # Call color detector module to get target info
        target_x, target_y, target_area = color_detector.detect_red_target(frame_bytes, width, height)
        
        # Accumulate Python execution latency
        latency = time.perf_counter() - start_time
        SUM_LATENCY += latency

        # Only print detection result once every 30 frames
        if FRAME_COUNTER % 30 == 0:
            avg_latency_ms = (SUM_LATENCY / 30.0) * 1000.0
            avg_fps = 30.0 / SUM_INTERVAL if SUM_INTERVAL > 0 else 0.0
            
            # Reset accumulators
            SUM_LATENCY = 0.0
            SUM_INTERVAL = 0.0

            target_str = f"Found: X={target_x}, Y={target_y}, Area={target_area:.1f}" if target_x != -1 else "No target"
            temp = telemetry.get('cpu_temp') if telemetry else None
            volt = telemetry.get('battery_voltage') if telemetry else None
            arm = telemetry.get('sys_arm_flag') if telemetry else None

            # Demonstration of reading detections from C++ NPU
            # detections = telemetry.get('detections') if telemetry else None
            # if detections:
            #     sys.stdout.write(f"\n[Python] Active Tracks: {[{'id': d['track_id'], 'cls': d['class_id'], 'box': d['box']} for d in detections]}\n")
            #     sys.stdout.flush()


    except Exception as e:
        import traceback
        sys.stdout.write(f"\n[Python Error] Exception in exchange: {e}\n")
        traceback.print_exc()
        
    return 0
