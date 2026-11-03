# /etc/rknn/vcm_controller.py
"""
OV5695 配套 DW9714 音圈对焦（VCM）。
总线 /dev/i2c-4，7 位地址 0x0C，10-bit DAC（0~1023）。
协议与 linshi/package 里的 VCM_CONTROL_GUIDE 一致。

无限远不能写 DAC 0：0 是弹簧自然位、线圈断电，镜组失去电磁刚度。
无限远要用 200~350 一带的偏置电流把镜头锁住（electromagnetic clamp）。
"""
from __future__ import annotations

import fcntl
import os
import sys
import time
from typing import Optional

# Linux i2c-dev：强制占用从机地址（内核驱动可能已绑这个地址）。
I2C_SLAVE_FORCE = 0x0706

DEFAULT_I2C_DEV = "/dev/i2c-4"
DEFAULT_SLAVE_ADDR = 0x0C
DAC_MIN = 0
DAC_MAX = 1023
# 无限远 + 加力锁镜：指南标定区间 200~350。不要用 0。
DEFAULT_INFINITY_DAC = 280
# 旧名保留，避免别处引用断开。
DEFAULT_CRUISE_DAC = DEFAULT_INFINITY_DAC
# 开机扫描：远端 0、近端 1023，结束后锁无限远偏置（DEFAULT_INFINITY_DAC）。
# 整段约 3s：拉远→拉近→锁无限远；飞行中不要长期停在 DAC 0。
DEFAULT_BOOT_FAR_DAC = 0
DEFAULT_BOOT_NEAR_DAC = 1023
DEFAULT_BOOT_SCAN_S = 3.0
DEFAULT_BOOT_SCAN_STEP = 80
DEFAULT_BOOT_STEP_INTERVAL_S = 0.02
# 端点各停约 0.9s，整段实测接近 3s（含移动）。
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
        # cruise_dac：无限远加力锁位点（非 0）。
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
        """打开 I2C。失败只记一次，不抛到 exchange。"""
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
        """直接写目标 DAC。成功返回 True。"""
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
        """回读当前 DAC。失败返回 None。"""
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
        """大跨度按步进逼近，减轻振铃。init / 手动跳变时用。"""
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
        """在约 duration_s 内平滑走到目标 DAC（按路程均分 sleep）。"""
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
        """保持无限远偏置电流，加力锁住镜头。"""
        if smooth:
            return self.set_focus_smooth(self.cruise_dac)
        return self.set_focus(self.cruise_dac)

    def apply_infinity_lock(self, smooth: bool = True) -> bool:
        """同 apply_cruise：无限远 + 电磁锁定。"""
        return self.apply_cruise(smooth=smooth)

    def release_power(self, smooth: bool = True) -> bool:
        """线圈断电到 DAC 0。会失去电磁刚度，user_app 默认不走这里。"""
        if smooth:
            return self.set_focus_smooth(DAC_MIN)
        return self.set_focus(DAC_MIN)

    @property
    def last_dac(self) -> Optional[int]:
        return self._last_dac


# 机端单例。user_app 在 init / 解锁上沿写入巡航 DAC。
g_vcm_controller = VcmController()
