# /etc/rknn/user_app.py
import numpy as np
import cv2
import sys
import apm

def init(width, height, pixfmt):
    """
    Called once during UserAppInit
    :param width: Image width
    :param height: Image height
    :param pixfmt: Image pixel format (V4L2 format code)
    """
    print(f"[Python] init() called: width={width}, height={height}, pixfmt={pixfmt}")
    # Return 0 or initialization status
    return 0

# 全局帧计数器，用于降频打印以节省 CPU
FRAME_COUNTER = 0

def exchange(frame_bytes, width, height, pixfmt, telemetry):
    global FRAME_COUNTER
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
            
            # Only print detection result once every 30 frames
            if FRAME_COUNTER % 30 == 0:
                target_str = f"Found: X={target_x}, Y={target_y}, Area={target_area:.1f}" if target_x != -1 else "No target"
                
                temp = telemetry.get('cpu_temp') if telemetry else None
                volt = telemetry.get('battery_voltage') if telemetry else None
                arm = telemetry.get('sys_arm_flag') if telemetry else None
                
                # Use carriage return (\r) to overwrite the line and prevent scrolling
                sys.stdout.write(f"\r[Vision] {target_str:<35} | [Telemetry] Temp={temp}C, Bat={volt}V, Armed={arm}")
                sys.stdout.flush()
                
    except Exception as e:
        import traceback
        sys.stdout.write(f"\n[Python Error] Exception in exchange: {e}\n")
        traceback.print_exc()
        
    return 0
