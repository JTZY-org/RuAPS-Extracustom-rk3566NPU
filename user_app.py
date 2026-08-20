# /etc/rknn/user_app.py
import numpy as np
import cv2
import sys
import apm
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from apm import TelemetryData

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

# 全局帧计数器与时间计量，用于统计 FPS 与 Latency
FRAME_COUNTER = 0
LAST_CALL_TIME = None
SUM_LATENCY = 0.0
SUM_INTERVAL = 0.0

# Python landing logic states
PYTHON_LANDING = False
PYTHON_LANDING_START_TIME = None

def exchange(frame_bytes: bytes, width: int, height: int, pixfmt: int, telemetry: 'TelemetryData') -> int:
    global FRAME_COUNTER, LAST_CALL_TIME, SUM_LATENCY, SUM_INTERVAL, PYTHON_LANDING, PYTHON_LANDING_START_TIME
    start_time = time.perf_counter()
    
    # 统计调用帧间隔（实际呼叫 FPS）
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
                        sys.stdout.write("\n[Python] Received B1 01: Replicating Landing Logic (Position to 0,0,0)\n")
                        sys.stdout.flush()
                        apm.set_position(0, 0, 0, True)
                        PYTHON_LANDING = True
                        PYTHON_LANDING_START_TIME = time.perf_counter()

    # Process Python Landing Telemetry Verification
    if PYTHON_LANDING:
        nav_relative_pos = telemetry.get('nav_relative_pos') if telemetry else None
        if nav_relative_pos and len(nav_relative_pos) >= 3 and nav_relative_pos[2] is not None:
            current_alt = nav_relative_pos[2]
            if abs(current_alt) <= 8.0:
                elapsed_ms = (time.perf_counter() - PYTHON_LANDING_START_TIME) * 1000.0
                sys.stdout.write(f"\r[Python Landing] Alt: {current_alt:.2f} cm | stable: {elapsed_ms:.1f}ms / 800ms")
                sys.stdout.flush()
                if elapsed_ms >= 800.0:
                    sys.stdout.write("\n[Python Landing] Touchdown stable for 800ms. Disarming...\n")
                    sys.stdout.flush()
                    apm.disarm()
                    PYTHON_LANDING = False
                    PYTHON_LANDING_START_TIME = None
            else:
                # Reset landing start time to current time if altitude is not close to ground
                PYTHON_LANDING_START_TIME = time.perf_counter()

    try:
        FRAME_COUNTER += 1
        
        # 1. Convert raw bytes to a NumPy array (cv2 image format)
        frame = np.frombuffer(frame_bytes, dtype=np.uint8)
        
        # 2. Reshape frame based on format (already converted to BGR24 in C++ using RGA)
        img = None
        if len(frame) == width * height * 3:
            img = frame.reshape((height, width, 3))
            
            # 1. Gaussian Blur for denoising (5x5 kernel)
            blurred = cv2.GaussianBlur(img, (5, 5), 0)
            
            # 2. Convert to HSV color space
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            
            # 3. Red color thresholding (Red wraps around Hue 0 and 180)
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 100, 100])
            upper_red2 = np.array([180, 255, 255])
            
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = mask1 | mask2
            
            # 4. Find external contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            target_x, target_y, target_area = -1, -1, 0.0
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                target_area = cv2.contourArea(largest_contour)
                if target_area > 100:  # Filter out tiny noise
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        target_x = int(M["m10"] / M["m00"])
                        target_y = int(M["m01"] / M["m00"])
            
            # 计算 Python 执行耗时并累加
            latency = time.perf_counter() - start_time
            SUM_LATENCY += latency

            # Only print detection result once every 30 frames
            if FRAME_COUNTER % 30 == 0:
                avg_latency_ms = (SUM_LATENCY / 30.0) * 1000.0
                avg_fps = 30.0 / SUM_INTERVAL if SUM_INTERVAL > 0 else 0.0
                
                # 重置累加器
                SUM_LATENCY = 0.0
                SUM_INTERVAL = 0.0

                target_str = f"Found: X={target_x}, Y={target_y}, Area={target_area:.1f}" if target_x != -1 else "No target"
                temp = telemetry.get('cpu_temp') if telemetry else None
                volt = telemetry.get('battery_voltage') if telemetry else None
                arm = telemetry.get('sys_arm_flag') if telemetry else None
                
                # 示範讀取 C++ NPU 傳過來的追蹤結果
                # detections = telemetry.get('detections') if telemetry else None
                # if detections:
                #     sys.stdout.write(f"\n[Python] Active Tracks: {[{'id': d['track_id'], 'cls': d['class_id'], 'box': d['box']} for d in detections]}\n")
                #     sys.stdout.flush()
                

    except Exception as e:
        import traceback
        sys.stdout.write(f"\n[Python Error] Exception in exchange: {e}\n")
        traceback.print_exc()
        
    return 0
