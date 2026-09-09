"""
servo_controller.py - Gimbal / Servo Control Module
Filters RC Channel 10 (rc_channel_raw[9]) with a 1.0 Hz low-pass filter
and controls Servo Pin 4 via apm.set_servo.
"""
import time
import math
import apm
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from apm import TelemetryData


class ServoController:
    def __init__(self, pin: int = 4, rc_channel_idx: int = 9, cutoff_hz: float = 1.0):
        self.pin = pin
        self.rc_channel_idx = rc_channel_idx
        self.cutoff_hz = cutoff_hz
        self.filtered_val: float = 1500.0
        self.first_run: bool = True
        self.last_time: Optional[float] = None
        self.last_sent_pwm: int = -1

    def update(self, telemetry: Optional['TelemetryData']) -> None:
        """
        Update servo position based on RC channel raw input.
        Applies a low-pass filter (default 1.0 Hz) to smooth input fluctuations.
        """
        if not telemetry:
            return

        rc_raw = telemetry.get('rc_channel_raw')
        if not rc_raw or len(rc_raw) <= self.rc_channel_idx:
            return

        raw_val = rc_raw[self.rc_channel_idx]
        if raw_val is None or not (800 <= raw_val <= 2200):
            return

        now = time.perf_counter()
        if self.first_run or self.last_time is None:
            self.filtered_val = float(raw_val)
            self.first_run = False
            self.last_time = now
        else:
            dt = now - self.last_time
            self.last_time = now
            if dt > 0:
                # Cutoff frequency fc = 1.0 Hz -> tau = 1 / (2 * pi * fc)
                tau = 1.0 / (2.0 * math.pi * self.cutoff_hz)
                alpha = dt / (tau + dt)
                self.filtered_val = self.filtered_val + alpha * (raw_val - self.filtered_val)

        rounded_pwm = int(self.filtered_val + 0.5)
        if rounded_pwm != self.last_sent_pwm:
            try:
                apm.set_servo(self.pin, rounded_pwm)
                self.last_sent_pwm = rounded_pwm
            except Exception as e:
                # In case APMControllerServo pointer is temporarily not ready
                pass


g_servo_controller = ServoController()
