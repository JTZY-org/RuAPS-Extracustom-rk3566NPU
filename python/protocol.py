# /etc/rknn/protocol.py
"""
Communication command protocol definitions and struct formats.
Uses Python standard library `struct` for lightweight binary packing/unpacking.
"""
import struct
from enum import IntEnum

# ==============================================================================
# Command ID Enum
# ==============================================================================
class CmdID(IntEnum):
    ARM = 0xCB
    ARM_ALT = 0xCC
    LAND = 0xB0
    LAND_ALT = 0xB1
    POS_REL_HORIZ = 0xD1  # len>=5: relative horizontal position (X, Y). len==2: person-track switch, handled in user_app.py
    POS_VERT = 0xD2       # Vertical position control (Altitude)
    # len==2：飞行平移跟踪。01 开始，00 停止。和 2 字节 D1 的偏航跟踪互斥。
    FLIGHT_FOLLOW = 0xD3
    # len==2：后退模式。01 开启，00 关闭。偏航跟踪和平移跟踪共用，看行人框大小写速度 x。
    RANGE_HOLD = 0xD4
    # len==3：VCM 对焦。int16 dac 0~1023；负数表示回到巡航 DAC。
    VCM_FOCUS = 0xD5
    GIMBAL_CONTROL = 0xFB  # Ground-station servo/laser CONTROLDATA (Flutter)
    FIXED_BOX = 0xE0       # len==2 kept for old packets. Flight follow ignores it; mode stays off.

# ==============================================================================
# Struct Formats (Standard little-endian '<')
# ==============================================================================
# ARM: uint8 cmd, uint8 action (1: arm, 0: disarm)
FMT_ARM = "<BB"

# LAND: uint8 cmd, uint8 action (1: start, 0: stop)
FMT_LAND = "<BB"

# POS_REL_HORIZ: uint8 cmd, int16 x (cm), int16 y (cm)
FMT_POS_REL_HORIZ = "<Bhh"

# POS_VERT: uint8 cmd, int16 alt (cm)
FMT_POS_VERT = "<Bh"

# Flutter / COM CONTROLDATA_t: reverse(u8) + Mark(0xFB) + servo(u16) + laser(u16)
FMT_GIMBAL_CONTROL = "<BBHH"

# VCM：uint8 cmd(0xD5), int16 dac（0~1023；负数=巡航）
FMT_VCM_FOCUS = "<Bh"
