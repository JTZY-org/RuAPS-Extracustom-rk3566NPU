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
    # len==2: flight pan follow. 01 start, 00 stop. Mutually exclusive with 2-byte D1 yaw track.
    FLIGHT_FOLLOW = 0xD3
    # len==2: range-hold / retreat mode. 01 on, 00 off. Shared by yaw and pan track; drives body X from box size.
    RANGE_HOLD = 0xD4
    # len==3: VCM focus. int16 dac 0~1023; negative returns to cruise DAC.
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

# VCM: uint8 cmd(0xD5), int16 dac (0~1023; negative = cruise)
FMT_VCM_FOCUS = "<Bh"
