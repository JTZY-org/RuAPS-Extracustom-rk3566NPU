# /etc/rknn/vcm_controller.py
"""
OV5695 companion DW9714 voice-coil focus (VCM).
Bus /dev/i2c-4, 7-bit address 0x0C, 10-bit DAC (0~1023).
Protocol matches VCM_CONTROL_GUIDE under linshi/package.

Do not write DAC 0 for infinity: 0 is the spring rest / coil-off position and
the lens loses electromagnetic stiffness. Infinity needs a bias current in the
200~350 band to clamp the lens (electromagnetic clamp).
"""
from __future__ import annotations

import fcntl
import os
import sys
import time
from typing import Optional

# Linux i2c-dev: force claim the slave address (a kernel driver may already bind it).
I2C_SLAVE_FORCE = 0x0706

DEFAULT_I2C_DEV = "/dev/i2c-4"
DEFAULT_SLAVE_ADDR = 0x0C
DAC_MIN = 0
DAC_MAX = 1023
# Infinity + force-lock: guide-calibrated band 200~350. Do not use 0.
DEFAULT_INFINITY_DAC = 280
# Keep old name so other call sites do not break.
DEFAULT_CRUISE_DAC = DEFAULT_INFINITY_DAC
# Boot scan: far 0, near 1023, then lock infinity bias (DEFAULT_INFINITY_DAC).
# Whole sequence ~3s: far -> near -> infinity lock; do not linger at DAC 0 in flight.
DEFAULT_BOOT_FAR_DAC = 0
DEFAULT_BOOT_NEAR_DAC = 1023
DEFAULT_BOOT_SCAN_S = 3.0
DEFAULT_BOOT_SCAN_STEP = 80
DEFAULT_BOOT_STEP_INTERVAL_S = 0.02
# Hold ~0.9s at each end; measured total near 3s including motion.
DEFAULT_BOOT_HOLD_S = 0.9
DEFAULT_BOOT_START_DELAY_S = 0.2


class VcmController:
    def __init__(
        self,
        i2c_dev: str = DEFAULT_I2C_DEV,
        slave_addr: int = DEFAULT_SLAVE_ADDR,
        cruise_dac: int = DEFAULT_INFINITY_DAC,
    ) -> None:
        self.i2c_dev = i2c_dev
        self.slave_addr = int(slave_addr) & 0x7F
        # cruise_dac: infinity force-lock setpoint (not 0).
        self.cruise_dac = self._clamp(cruise_dac)
        self._fd: int = -1
        self._last_dac: Optional[int] = None
        self._opened = False
        self._open_failed = False

    @staticmethod
    def _clamp(dac: int) -> int:
        value = int(dac)
        if value < DAC_MIN:
            return DAC_MIN
        if value > DAC_MAX:
            return DAC_MAX
        return value

    @staticmethod
    def _encode(dac: int) -> bytes:
        dac = VcmController._clamp(dac)
        byte0 = (dac >> 4) & 0xFF
        byte1 = (dac & 0x0F) << 4
        return bytes((byte0, byte1))

    @staticmethod
    def _decode(buf: bytes) -> int:
        return ((buf[0] << 4) | (buf[1] >> 4)) & 0x3FF

    def open(self) -> bool:
        """Open I2C. On failure record once; do not raise into exchange."""
        if self._opened and self._fd >= 0:
            return True
        if self._open_failed:
            return False
        try:
            fd = os.open(self.i2c_dev, os.O_RDWR)
            fcntl.ioctl(fd, I2C_SLAVE_FORCE, self.slave_addr)
        except OSError as exc:
            self._open_failed = True
            sys.stdout.write(
                f"\n[Python] VCM open failed {self.i2c_dev}@{self.slave_addr:#04x}: {exc}\n"
            )
            sys.stdout.flush()
            return False
        self._fd = fd
        self._opened = True
        return True

    def close(self) -> None:
        if self._fd >= 0:
            try:
                os.close(self._fd)
            except OSError:
                pass
        self._fd = -1
        self._opened = False

    def set_focus(self, dac: int) -> bool:
        """Write target DAC directly. Returns True on success."""
        if not self.open():
            return False
        payload = self._encode(dac)
        try:
            wrote = os.write(self._fd, payload)
        except OSError as exc:
            sys.stdout.write(f"\n[Python] VCM write failed: {exc}\n")
            sys.stdout.flush()
            return False
        if wrote != 2:
            return False
        self._last_dac = self._clamp(dac)
        return True

    def get_focus(self) -> Optional[int]:
        """Read back current DAC. Returns None on failure."""
        if not self.open():
            return None
        try:
            buf = os.read(self._fd, 2)
        except OSError as exc:
            sys.stdout.write(f"\n[Python] VCM read failed: {exc}\n")
            sys.stdout.flush()
            return None
        if not buf or len(buf) < 2:
            return None
        return self._decode(buf)

    def set_focus_smooth(
        self,
        dac: int,
        step: int = 16,
        delay_s: float = 0.008,
    ) -> bool:
        """Step toward a large jump to reduce ringing. Used for init / manual jumps."""
        target = self._clamp(dac)
        current = self._last_dac
        if current is None:
            rb = self.get_focus()
            current = rb if rb is not None else DAC_MIN
        if step <= 0:
            step = 16
        ok = True
        while current != target:
            if current < target:
                current = min(target, current + step)
            else:
                current = max(target, current - step)
            if not self.set_focus(current):
                ok = False
                break
            if current != target and delay_s > 0:
                time.sleep(delay_s)
        return ok

    def _smooth_to_timed(
        self,
        dac: int,
        duration_s: float,
        step: int = DEFAULT_BOOT_SCAN_STEP,
    ) -> bool:
        """Smooth to target DAC over about duration_s (sleep split by travel)."""
        target = self._clamp(dac)
        current = self._last_dac
        if current is None:
            rb = self.get_focus()
            current = rb if rb is not None else DAC_MIN
        if step <= 0:
            step = DEFAULT_BOOT_SCAN_STEP
        delta = abs(target - int(current))
        n_steps = max(1, (delta + step - 1) // step)
        delay_s = max(0.0, float(duration_s) / float(n_steps))
        return self.set_focus_smooth(target, step=step, delay_s=delay_s)

    def apply_cruise(self, smooth: bool = True) -> bool:
        """Hold infinity bias current and force-lock the lens."""
        if smooth:
            return self.set_focus_smooth(self.cruise_dac)
        return self.set_focus(self.cruise_dac)

    def apply_infinity_lock(self, smooth: bool = True) -> bool:
        """Same as apply_cruise: infinity + electromagnetic clamp."""
        return self.apply_cruise(smooth=smooth)

    def release_power(self, smooth: bool = True) -> bool:
        """Power the coil down to DAC 0. Loses electromagnetic stiffness; user_app avoids this by default."""
        if smooth:
            return self.set_focus_smooth(DAC_MIN)
        return self.set_focus(DAC_MIN)

    @property
    def last_dac(self) -> Optional[int]:
        return self._last_dac


# Airborne singleton. user_app writes cruise DAC on init / arm edge.
g_vcm_controller = VcmController()
