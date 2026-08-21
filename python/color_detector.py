import numpy as np
import cv2

def detect_red_target(frame_bytes: bytes, width: int, height: int) -> tuple:
    """
    Detects the largest red color block in the BGR frame.
    Returns (target_x, target_y, target_area).
    If no target is found, returns (-1, -1, 0.0).
    """
    frame = np.frombuffer(frame_bytes, dtype=np.uint8)
    if len(frame) != width * height * 3:
        return -1, -1, 0.0
        
    img = frame.reshape((height, width, 3))
    
    # Downscale image by 4x to speed up Python CPU processing
    img_small = cv2.resize(img, (0, 0), fx=0.25, fy=0.25)
    
    # 1. Gaussian Blur for denoising
    blurred = cv2.GaussianBlur(img_small, (5, 5), 0)
    
    # 2. Convert to HSV color space
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    # 3. Red color thresholding
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
        area = cv2.contourArea(largest_contour)
        if area > 6.25:  # Scaled threshold
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                target_x = int((M["m10"] / M["m00"]) * 4.0)
                target_y = int((M["m01"] / M["m00"]) * 4.0)
                target_area = area * 16.0
                
    return target_x, target_y, target_area
