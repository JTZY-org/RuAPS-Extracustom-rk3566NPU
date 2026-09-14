# /etc/rknn/user_app.py
from __future__ import annotations
import math
import struct
import sys
import time
from typing import TYPE_CHECKING, Optional, List, Tuple

import apm
import color_detector
import servo_controller
from protocol import CmdID, FMT_ARM, FMT_LAND, FMT_POS_REL_HORIZ, FMT_POS_VERT

if TYPE_CHECKING:
    from apm import TelemetryData

# ==============================================================================
# Configuration & Global States
# ==============================================================================
ENABLE_TELEMETRY_MONITOR = False  # Set to True for full-terminal telemetry display

FRAME_COUNTER = 0
LAST_CALL_TIME: Optional[float] = None
SUM_LATENCY = 0.0
SUM_INTERVAL = 0.0

INITIAL_STATUS_CHECKED = False

# Landing logic states
PYTHON_LANDING = False
PYTHON_LANDING_START_TIME: Optional[float] = None

# Position control states
HORIZ_TARGET_ACTIVE = False
TARGET_X: int = 0
TARGET_Y: int = 0
TARGET_HORIZ_ALT: Optional[int] = None

VERT_TARGET_ACTIVE = False
TARGET_ALT: Optional[int] = None


# ==============================================================================
# Lifecycle Functions
# ==============================================================================
def init(width: int, height: int, pixfmt: int) -> int:
    """
    Called once during UserAppInit.
    :param width: Image width
    :param height: Image height
    :param pixfmt: Image pixel format (V4L2 format code)
    """
    print(f"[Python] init() called: width={width}, height={height}, pixfmt={pixfmt}")
    return 0


# ==============================================================================
# Telemetry Helpers & Monitor
# ==============================================================================
def get_current_nav(telemetry: Optional['TelemetryData']) -> Tuple[int, int, int, float]:
    """
    Extracts current relative position (X, Y, Altitude) strictly from _NAV_Relative_Pos
    and Yaw angle from att_euler_angle_yaw_v.
    """
    cur_x, cur_y, cur_alt = 0, 0, 0
    if telemetry:
        nav_pos = telemetry.get('nav_relative_pos')
        if nav_pos and len(nav_pos) >= 3:
            cur_x = int(nav_pos[0]) if nav_pos[0] is not None else 0
            cur_y = int(nav_pos[1]) if nav_pos[1] is not None else 0
            cur_alt = int(nav_pos[2]) if nav_pos[2] is not None else 0

    cur_yaw = float(telemetry.get('att_euler_angle_yaw_v') or 0.0) if telemetry else 0.0
    return cur_x, cur_y, cur_alt, cur_yaw


def debug_print_telemetry(telemetry: 'TelemetryData') -> None:
    """
    Prints a formatted, full-terminal telemetry status screen.
    """
    if not telemetry:
        return

    items: List[str] = []

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

    landing_status = "ACTIVE" if PYTHON_LANDING else "IDLE"
    lines.append(f"LANDING STATE: {landing_status:<10} | HORIZ_CTRL: {'ACTIVE' if HORIZ_TARGET_ACTIVE else 'IDLE'} | VERT_CTRL: {'ACTIVE' if VERT_TARGET_ACTIVE else 'IDLE'}")

    recv_packets = telemetry.get('BroadcastRecv', [])
    recv_hex = ", ".join(p.hex().upper() for p in recv_packets) if recv_packets else "None"
    lines.append(f"LAST RECV UDP: {recv_hex:<25}")
    lines.append("---------------------------------------------------------------------------------")

    for i in range(0, len(items), 4):
        chunk = items[i:i+4]
        line_str = " | ".join(f"{item:<22}" for item in chunk)
        lines.append(line_str)

    lines.append("=================================================================================")

    sys.stdout.write("\n".join(lines) + "\n")
    sys.stdout.flush()


# ==============================================================================
# Command & Flight Processing Helpers
# ==============================================================================
def handle_broadcast_packets(packets: List[bytes], telemetry: Optional['TelemetryData']) -> None:
    """
    Parses received UDP broadcast packets using struct.unpack and executes actions.
    """
    global PYTHON_LANDING, HORIZ_TARGET_ACTIVE, TARGET_X, TARGET_Y, TARGET_HORIZ_ALT, VERT_TARGET_ACTIVE, TARGET_ALT

    sys.stdout.write(f"\n[Python] BroadcastRecv Packets: {[p.hex() for p in packets]}\n")
    sys.stdout.flush()

    cur_x, cur_y, cur_alt, cur_yaw = get_current_nav(telemetry)

    for p in packets:
        if not p:
            continue
        cmd_id = p[0]

        # 1. ARM / DISARM (CB / CC)
        if cmd_id in (CmdID.ARM, CmdID.ARM_ALT):
            size = struct.calcsize(FMT_ARM)
            if len(p) >= size:
                _, action = struct.unpack(FMT_ARM, p[:size])
                if action == 1:
                    sys.stdout.write(f"\n[Python] Received {cmd_id:02X} 01: Arming flight controller\n")
                    sys.stdout.flush()
                    apm.arm()
                elif action == 0:
                    sys.stdout.write(f"\n[Python] Received {cmd_id:02X} 00: Disarming flight controller and setting speed to 0\n")
                    sys.stdout.flush()
                    apm.disarm()
                    apm.set_speed(0, 0, 0, 0.0)
                    HORIZ_TARGET_ACTIVE = False
                    VERT_TARGET_ACTIVE = False

        # 2. Standalone Speed Landing (B0 / B1)
        elif cmd_id in (CmdID.LAND, CmdID.LAND_ALT):
            size = struct.calcsize(FMT_LAND)
            if len(p) >= size:
                _, action = struct.unpack(FMT_LAND, p[:size])
                if action == 1:
                    if not PYTHON_LANDING:
                        sys.stdout.write(f"\n[Python] Received {cmd_id:02X} 01: Starting speed landing (50cm/s descent)\n")
                        sys.stdout.flush()
                        PYTHON_LANDING = True
                        HORIZ_TARGET_ACTIVE = False
                        VERT_TARGET_ACTIVE = False
                elif action == 0:
                    sys.stdout.write(f"\n[Python] Received {cmd_id:02X} 00: Canceling speed landing\n")
                    sys.stdout.flush()
                    PYTHON_LANDING = False

        # 3. Relative horizontal position control (0xD1): cmdid(uint8) + x(int16) + y(int16) [cm]
        elif cmd_id == CmdID.POS_REL_HORIZ:
            size = struct.calcsize(FMT_POS_REL_HORIZ)
            if len(p) >= size:
                _, target_x, target_y = struct.unpack(FMT_POS_REL_HORIZ, p[:size])

                TARGET_X = target_x
                TARGET_Y = target_y
                TARGET_HORIZ_ALT = cur_alt  # Latch current _NAV_Relative_Pos[2] altitude
                HORIZ_TARGET_ACTIVE = True

                sys.stdout.write(f"\n[Python] Received D1: Relative horizontal target ({target_x}, {target_y}) cm, holding altitude {TARGET_HORIZ_ALT} cm\n")
                sys.stdout.flush()

                # Set reset_home=False to start moving towards target
                apm.set_position(TARGET_X, TARGET_Y, TARGET_HORIZ_ALT, cur_yaw, False)

        # 4. Vertical position control (0xD2): cmdid(uint8) + alt(int16) [cm]
        elif cmd_id == CmdID.POS_VERT:
            size = struct.calcsize(FMT_POS_VERT)
            if len(p) >= size:
                _, target_alt = struct.unpack(FMT_POS_VERT, p[:size])

                sys.stdout.write(f"\n[Python] Received D2: Target vertical altitude {target_alt} cm\n")
                sys.stdout.flush()

                TARGET_ALT = target_alt
                VERT_TARGET_ACTIVE = True

                # Set: current X, current Y, target altitude, reset_home=False
                apm.set_position(cur_x, cur_y, TARGET_ALT, cur_yaw, False)


def process_position_control(telemetry: Optional['TelemetryData']) -> None:
    """
    Continuous position tracking state machine for horizontal and vertical control.
    """
    global HORIZ_TARGET_ACTIVE, TARGET_X, TARGET_Y, TARGET_HORIZ_ALT, VERT_TARGET_ACTIVE, TARGET_ALT

    if not telemetry or PYTHON_LANDING:
        return

    cur_x, cur_y, cur_alt, cur_yaw = get_current_nav(telemetry)

    # 1. Relative horizontal position tracking
    if HORIZ_TARGET_ACTIVE:
        dist = math.hypot(cur_x - TARGET_X, cur_y - TARGET_Y)
        hold_alt = TARGET_HORIZ_ALT if TARGET_HORIZ_ALT is not None else cur_alt
        if dist <= 10.0:
            # Destination reached: set reset_home=True to reset origin
            sys.stdout.write(f"\n[Python] Destination reached (dist={dist:.1f}cm <= 10cm). Resetting home origin.\n")
            sys.stdout.flush()
            apm.set_position(0, 0, hold_alt, cur_yaw, True)
            HORIZ_TARGET_ACTIVE = False

            # Lock position at (0, 0) with reset_home=False
            apm.set_position(0, 0, hold_alt, cur_yaw, False)
        else:
            # Keep moving towards target with reset_home=False and held altitude
            apm.set_position(TARGET_X, TARGET_Y, hold_alt, cur_yaw, False)

    # 2. Vertical position hold (when not in horizontal control)
    elif VERT_TARGET_ACTIVE and TARGET_ALT is not None:
        apm.set_position(cur_x, cur_y, TARGET_ALT, cur_yaw, False)
        if abs(cur_alt - TARGET_ALT) <= 5.0:
            sys.stdout.write(f"\n[Python] Vertical altitude reached: {cur_alt}cm\n")
            sys.stdout.flush()
            VERT_TARGET_ACTIVE = False


def process_landing(telemetry: Optional['TelemetryData'], current_time: float) -> None:
    """
    Executes descent landing at 50 cm/s until touch-down is detected, then disarms.
    """
    global PYTHON_LANDING, PYTHON_LANDING_START_TIME

    if not PYTHON_LANDING:
        return

    # Descend at 50 cm/s (positive Z in body speed)
    apm.set_speed(0, 0, 50, 0.0)

    nav_relative_pos = telemetry.get('nav_relative_pos') if telemetry else None
    if nav_relative_pos and len(nav_relative_pos) >= 3 and nav_relative_pos[2] is not None:
        current_alt = nav_relative_pos[2]
        # Altitude <= 3cm indicates touch-down on ground
        if current_alt <= 3.0:
            if PYTHON_LANDING_START_TIME is None:
                PYTHON_LANDING_START_TIME = current_time
            elif current_time - PYTHON_LANDING_START_TIME >= 2.0:
                sys.stdout.write("\n[Python] Landing completed. Disarming flight controller.\n")
                sys.stdout.flush()
                apm.disarm()
                apm.set_speed(0, 0, 0, 0.0)
                PYTHON_LANDING = False
                PYTHON_LANDING_START_TIME = None
        else:
            PYTHON_LANDING_START_TIME = None


# ==============================================================================
# Main Exchange Entry Point
# ==============================================================================
def exchange(frame_bytes: bytes, width: int, height: int, pixfmt: int, telemetry: 'TelemetryData') -> int:
    global FRAME_COUNTER, LAST_CALL_TIME, SUM_LATENCY, SUM_INTERVAL, INITIAL_STATUS_CHECKED
    start_time = time.perf_counter()

    # 1. Calling interval & FPS measurement
    if LAST_CALL_TIME is not None:
        SUM_INTERVAL += (start_time - LAST_CALL_TIME)
    LAST_CALL_TIME = start_time

    # 2. Initial lock status check (once)
    if telemetry and not INITIAL_STATUS_CHECKED:
        is_disarmed = telemetry.get('sys_disarm_flag')
        status = "LOCKED (DISARMED)" if is_disarmed is True else "UNLOCKED (ARMED)"
        sys.stdout.write(f"\n[Python] Initial lock status: {status}\n")
        sys.stdout.flush()
        INITIAL_STATUS_CHECKED = True

    # 3. Optional live telemetry monitor
    if ENABLE_TELEMETRY_MONITOR and telemetry and FRAME_COUNTER % 5 == 0:
        debug_print_telemetry(telemetry)

    # 4. RC Servo update
    if telemetry:
        servo_controller.g_servo_controller.update(telemetry)

    # 5. Broadcast command handling
    if telemetry and 'BroadcastRecv' in telemetry:
        broadcast_packets = telemetry['BroadcastRecv']
        if broadcast_packets:
            handle_broadcast_packets(broadcast_packets, telemetry)

    # 6. Continuous position control (horizontal & vertical tracking)
    process_position_control(telemetry)

    # 7. Landing state machine
    process_landing(telemetry, start_time)

    # 8. Computer Vision & Frame Processing
    try:
        FRAME_COUNTER += 1

        target_x, target_y, target_area = -1, -1, 0.0
        if frame_bytes and width > 0 and height > 0:
            target_x, target_y, target_area = color_detector.detect_red_target(frame_bytes, width, height)

        latency = time.perf_counter() - start_time
        SUM_LATENCY += latency

        # Print statistics every 30 frames
        if FRAME_COUNTER % 30 == 0:
            avg_latency_ms = (SUM_LATENCY / 30.0) * 1000.0
            avg_fps = 30.0 / SUM_INTERVAL if SUM_INTERVAL > 0 else 0.0

            # Reset accumulators
            SUM_LATENCY = 0.0
            SUM_INTERVAL = 0.0

            if target_x != -1:
                sys.stdout.write(f"[Python] Red target: X={target_x}, Y={target_y}, Area={target_area:.1f} | Latency={avg_latency_ms:.2f}ms, FPS={avg_fps:.1f}\n")
                sys.stdout.flush()

    except Exception as e:
        import traceback
        sys.stdout.write(f"\n[Python Error] Exception in exchange: {e}\n")
        traceback.print_exc()

    return 0
