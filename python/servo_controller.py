"""
servo_controller.py - Gimbal / Servo Control Module

Input is ground-station CONTROLDATA (Flutter WebSocket 0xFB, PWM 1000–2000),
not RC CH10. Output is still Servo Pin 4 via apm.set_servo.
A 1.0 Hz low-pass filter smooths the command.
"""
import time
import math
import apm
from typing import Optional


class ServoController:
    def __init__(self, pin: int = 4, cutoff_hz: float = 1.0):
        self.pin = pin
        self.cutoff_hz = cutoff_hz
        self.target_val: float = 1500.0
        self.filtered_val: float = 1500.0
        self.has_target: bool = False
        self.first_run: bool = True
        self.last_time: Optional[float] = None
        self.last_sent_pwm: int = -1

    def set_target(self, pwm: int) -> None:
        """Accept a ground-station servo PWM (same range as Flutter _servoValue)."""
        if pwm is None or not (800 <= pwm <= 2200):
            return
        self.target_val = float(pwm)
        self.has_target = True

    def update(self) -> None:
        """
        Drive servo toward the last ground-station target.
        Applies a low-pass filter (default 1.0 Hz) to smooth input fluctuations.
        """
        if not self.has_target:
            return

        now = time.perf_counter()
        if self.first_run or self.last_time is None:
            self.filtered_val = self.target_val
            self.first_run = False
            self.last_time = now
        else:
            dt = now - self.last_time
            self.last_time = now
            if dt > 0:
                # Cutoff frequency fc = 1.0 Hz -> tau = 1 / (2 * pi * fc)
                tau = 1.0 / (2.0 * math.pi * self.cutoff_hz)
                alpha = dt / (tau + dt)
                self.filtered_val = self.filtered_val + alpha * (self.target_val - self.filtered_val)

        rounded_pwm = int(self.filtered_val + 0.5)
        if rounded_pwm != self.last_sent_pwm:
            try:
                apm.set_servo(self.pin, rounded_pwm)
                self.last_sent_pwm = rounded_pwm
            except Exception:
                # In case APMControllerServo pointer is temporarily not ready
                pass


g_servo_controller = ServoController()
