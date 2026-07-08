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

def exchange(frame_bytes, width, height, pixfmt, telemetry):
    try:
        # 1. Convert raw bytes to a NumPy array (cv2 image format)
        frame = np.frombuffer(frame_bytes, dtype=np.uint8)
        
        # 2. Reshape frame based on format.
        img = None
        if len(frame) == width * height * 3:
            img = frame.reshape((height, width, 3))
        elif len(frame) == int(width * height * 1.5):
            # Convert NV12 to BGR
            img = cv2.cvtColor(frame.reshape((height * 3 // 2, width)), cv2.COLOR_YUV2BGR_NV12)
            
        # -------------------------------------------------------------
        # Telemetry Dictionary Usage Example
        # -------------------------------------------------------------
        print("\n--- [Python] Telemetry Update ---")
        if telemetry is None:
            print("  Telemetry dict is None!")
            return 0
            
        print(f"  CPU Temp:         {telemetry.get('cpu_temp')} C")
        print(f"  Battery Voltage:  {telemetry.get('battery_voltage')} V")
        print(f"  SYS ARM Flag:     {telemetry.get('sys_arm_flag')}")
        print(f"  APM Status:       {telemetry.get('sys_apm_status')}")
        
        # Array/List metrics
        euler = telemetry.get('att_euler_angle')
        if euler:
            euler_val = [x if x is not None else 0.0 for x in euler]
            print(f"  Euler Angle (R/P/Y): Roll={euler_val[0]:.2f}, Pitch={euler_val[1]:.2f}, Yaw={euler_val[2]:.2f}")
            
        pos = telemetry.get('nav_global_pos')
        if pos:
            pos_val = [x if x is not None else 0 for x in pos]
            print(f"  Global Pos:       Lat={pos_val[0]}, Lon={pos_val[1]}, Alt={pos_val[2]}")
            
        speed = telemetry.get('nav_speed')
        if speed:
            speed_val = [x if x is not None else 0.0 for x in speed]
            print(f"  Speed (X/Y/Z):    Vx={speed_val[0]:.2f}, Vy={speed_val[1]:.2f}, Vz={speed_val[2]:.2f}")
    except Exception as e:
        import traceback
        print(f"[Python Error] Exception in exchange: {e}")
        traceback.print_exc()
        
    return 0
        
    return 0
