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
    POS_REL_HORIZ = 0xD1  # Relative horizontal position control (X, Y)
    POS_VERT = 0xD2       # Vertical position control (Altitude)

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
