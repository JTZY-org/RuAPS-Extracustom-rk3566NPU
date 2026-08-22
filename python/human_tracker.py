"""
human_tracker.py - Human Target Yaw Tracking Module for APM Drone
Tracks the newest valid human track (YOLO COCO person, class_id=0).
Controls heading (yaw) only, maintaining hover (0 speed in x, y, z).
"""

import time
import math
import sys
import apm
from typing import TYPE_CHECKING, Optional, Tuple, Dict, Any, List

if TYPE_CHECKING:
    from apm import TelemetryData

# State Constants
STATE_INACTIVE = "INACTIVE"
STATE_WAITING_ARM = "WAITING_ARM"
STATE_SEARCHING = "SEARCHING"
STATE_TRACKING = "TRACKING"
STATE_TARGET_LOST = "LOST_SEARCHING"

# Tracking & Control Hyperparameters
PERSON_CLASS_ID = 0          # COCO 80 class index for person
MIN_CONFIDENCE = 0.40        # Minimum detection confidence
MAX_LOST_FRAMES = 6          # Max consecutive frames without detection before declaring target lost (~200ms)
DEADBAND_RATIO = 0.05        # Normalized deadband around image center (+/- 5%)
KP_YAW = 25.0                # Proportional gain for yaw heading control (degrees per full normalized error)
MAX_YAW_STEP = 15.0          # Max degree correction per update step to prevent jitter


class HumanTracker:
    def __init__(self):
        self.is_active: bool = False
        self.state: str = STATE_INACTIVE
        self.current_target_id: Optional[int] = None
        self.current_box: Optional[List[int]] = None  # [x1, y1, x2, y2]
        self.current_conf: float = 0.0
        self.lost_frames: int = 0
        self.start_time: float = 0.0
        
        self.last_error_norm: float = 0.0
        self.last_target_yaw: float = 0.0
        self.last_current_yaw: float = 0.0
        
        self.logs: List[str] = []

    def log(self, message: str) -> None:
        formatted = f"[Tracker] {message}"
        sys.stdout.write(f"\n{formatted}\n")
        sys.stdout.flush()
        self.logs.append(formatted)
        if len(self.logs) > 5:
            self.logs.pop(0)

    def normalize_yaw(self, yaw: float) -> float:
        """Normalizes yaw angle to [-180.0, 180.0] range."""
        while yaw > 180.0:
            yaw -= 360.0
        while yaw < -180.0:
            yaw += 360.0
        return yaw

    def start_tracking(self, telemetry: Optional['TelemetryData'] = None) -> None:
        """Enables tracking mode."""
        self.is_active = True
        self.current_target_id = None
        self.current_box = None
        self.current_conf = 0.0
        self.lost_frames = 0
        self.start_time = time.perf_counter()
        
        # sys_arm_flag: False = Armed (unlocked), True = Disarmed (locked)
        armed = (telemetry.get('sys_arm_flag') is False) if telemetry else False
        if armed:
            self.state = STATE_SEARCHING
            self.log("Tracking STARTED. Drone is ARMED. Searching for newest human track...")
        else:
            self.state = STATE_WAITING_ARM
            self.log("Tracking STARTED (Standby: Waiting for drone to ARM/unlock)...")

    def stop_tracking(self) -> None:
        """Disables tracking mode."""
        self.is_active = False
        self.state = STATE_INACTIVE
        self.current_target_id = None
        self.current_box = None
        self.lost_frames = 0
        self.log("Tracking STOPPED.")

    def find_newest_human_track(self, detections: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
        """
        Finds the newest valid human track (highest track_id with class_id == PERSON_CLASS_ID
        and confidence >= MIN_CONFIDENCE).
        """
        valid_human_tracks = []
        for det in detections:
            if det.get('class_id') == PERSON_CLASS_ID and det.get('confidence', 0.0) >= MIN_CONFIDENCE:
                valid_human_tracks.append(det)

        if not valid_human_tracks:
            return None

        # Sort by track_id descending (newest track has highest ID)
        valid_human_tracks.sort(key=lambda d: d.get('track_id', -1), reverse=True)
        return valid_human_tracks[0]

    def update(self, telemetry: 'TelemetryData', frame_width: int, frame_height: int) -> None:
        """
        Main update step called on each exchange frame.
        - Checks armed status
        - Manages target tracking & lost switching
        - Controls yaw heading while maintaining hover (0 speed in x, y, z)
        """
        if not self.is_active:
            return

        if not telemetry:
            return

        # sys_arm_flag: False = Armed (unlocked), True = Disarmed (locked)
        armed = (telemetry.get('sys_arm_flag') is False)
        current_yaw = telemetry.get('att_euler_angle_yaw_v', 0.0)
        if current_yaw is None:
            current_yaw = 0.0
        self.last_current_yaw = current_yaw

        # 1. Safety check: Check if vehicle is armed
        if not armed:
            if self.state != STATE_WAITING_ARM:
                self.state = STATE_WAITING_ARM
                self.log("Vehicle disarmed. Pausing tracking until ARMED.")
            return

        # Vehicle is armed
        if self.state == STATE_WAITING_ARM:
            self.state = STATE_SEARCHING
            self.log("Vehicle ARMED detected! Resuming human search.")

        # 2. Extract YOLO detections
        detections = telemetry.get('detections', [])
        
        # 3. Target maintenance & state transitions
        matched_det = None
        if self.current_target_id is not None:
            for det in detections:
                if det.get('class_id') == PERSON_CLASS_ID and det.get('track_id') == self.current_target_id:
                    matched_det = det
                    break

        if matched_det is not None:
            # Target is still visible and tracked
            self.current_box = matched_det.get('box')
            self.current_conf = matched_det.get('confidence', 0.0)
            self.lost_frames = 0
            self.state = STATE_TRACKING
        else:
            # Target not found in current frame
            if self.current_target_id is not None:
                self.lost_frames += 1
                if self.lost_frames <= MAX_LOST_FRAMES:
                    # Still in grace period
                    self.state = STATE_TARGET_LOST
                else:
                    # Target lost beyond threshold -> Auto-switch to newest valid track
                    self.log(f"Track #{self.current_target_id} lost (> {MAX_LOST_FRAMES} frames). Auto-switching to newest track...")
                    new_target = self.find_newest_human_track(detections)
                    if new_target is not None:
                        self.current_target_id = new_target.get('track_id')
                        self.current_box = new_target.get('box')
                        self.current_conf = new_target.get('confidence', 0.0)
                        self.lost_frames = 0
                        self.state = STATE_TRACKING
                        self.log(f"Auto-switched and locked onto new Track #{self.current_target_id} (conf={self.current_conf:.2f})")
                    else:
                        self.current_target_id = None
                        self.current_box = None
                        self.state = STATE_SEARCHING
            else:
                # Currently no target -> search for newest valid human track
                new_target = self.find_newest_human_track(detections)
                if new_target is not None:
                    self.current_target_id = new_target.get('track_id')
                    self.current_box = new_target.get('box')
                    self.current_conf = new_target.get('confidence', 0.0)
                    self.lost_frames = 0
                    self.state = STATE_TRACKING
                    self.log(f"Found and locked onto newest Track #{self.current_target_id} (conf={self.current_conf:.2f})")
                else:
                    self.state = STATE_SEARCHING

        # 4. Heading (Yaw) Controller
        if self.state == STATE_TRACKING and self.current_box and len(self.current_box) == 4 and frame_width > 0:
            x1, y1, x2, y2 = self.current_box
            center_x = (x1 + x2) / 2.0
            frame_center_x = frame_width / 2.0
            
            # Normalized horizontal error: -1.0 (far left) to +1.0 (far right)
            error_norm = (center_x - frame_center_x) / frame_center_x
            self.last_error_norm = error_norm

            # Deadband check: if within center 5%, no yaw change needed
            if abs(error_norm) < DEADBAND_RATIO:
                delta_yaw = 0.0
            else:
                # Calculate yaw adjustment with proportional gain & clamp to max step
                raw_delta = KP_YAW * error_norm
                delta_yaw = max(-MAX_YAW_STEP, min(MAX_YAW_STEP, raw_delta))

            target_yaw = self.normalize_yaw(current_yaw + delta_yaw)
            self.last_target_yaw = target_yaw

            # Send command: only control yaw, keep linear speed at (0, 0, 0) for hovering
            apm.set_speed(0, 0, 0, target_yaw)

        elif self.state in (STATE_SEARCHING, STATE_TARGET_LOST):
            # No valid target or searching: maintain current heading and hover
            self.last_error_norm = 0.0
            self.last_target_yaw = current_yaw
            apm.set_speed(0, 0, 0, current_yaw)

    def get_status_str(self) -> str:
        """Returns a compact formatted string of the current tracking status."""
        if not self.is_active:
            return "INACTIVE (Command 0xB4 0x01 to start)"
        
        target_info = f"Track #{self.current_target_id}" if self.current_target_id is not None else "None"
        box_str = f"[{self.current_box[0]},{self.current_box[1]},{self.current_box[2]},{self.current_box[3]}]" if self.current_box else "N/A"
        return (
            f"State: {self.state:<12} | Target: {target_info:<9} | "
            f"Err: {self.last_error_norm:+.2f} | Yaw: {self.last_current_yaw:.1f}->{self.last_target_yaw:.1f} | "
            f"Lost: {self.lost_frames}"
        )


# Global singleton instance
g_human_tracker = HumanTracker()

