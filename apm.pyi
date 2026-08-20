# apm.pyi - Type stub definition for C++ CPython 'apm' module and TelemetryData
from typing import TypedDict, List, Optional

class TelemetryData(TypedDict, total=False):
    accel_clipped_times: Optional[int]
    accel_gforce: Optional[float]
    baro_temp: Optional[float]
    baro_pressure_hpa: Optional[float]
    baro_agl_altitude_cm: Optional[float]
    rangefinder_agl_alt_cm: Optional[float]
    sys_arm_flag: Optional[bool]
    sys_pre_arm_flag: Optional[int]
    sys_failsafe_flag: Optional[int]
    sys_apm_status: Optional[int]
    nav_relative_head: Optional[float]
    nav_global_head: Optional[float]
    nav_global_sat_count: Optional[int]
    nav_gps_hdop: Optional[int]
    gyro_cycle_time: Optional[float]
    battery_voltage: Optional[float]
    battery_voltage_single: Optional[float]
    cpu_temp: Optional[float]
    accel_acceleration: List[Optional[float]]
    accel_vibe: List[Optional[float]]
    accel_raw_g: List[Optional[int]]
    att_quaternion: List[Optional[float]]
    att_euler_angle: List[Optional[float]]
    gyro_angle_rate: List[Optional[float]]
    mag_raw_l: List[Optional[int]]
    sys_time_info: List[Optional[int]]
    nav_speed: List[Optional[float]]
    nav_global_speed: List[Optional[float]]
    nav_global_pos: List[Optional[int]]
    nav_global_home: List[Optional[int]]
    nav_relative_pos: List[Optional[float]]
    rc_channel_raw: List[Optional[int]]
    ef_channel_raw: List[Optional[int]]
    BroadcastRecv: List[bytes]

def arm() -> None:
    """Arm the flight controller."""
    ...

def disarm() -> None:
    """Disarm the flight controller."""
    ...

def set_position(x: int, y: int, z: int, reset_home: bool = True) -> None:
    """Set flight controller target position."""
    ...

def set_gps_position(lat: int, lng: int, alt: int) -> None:
    """Set flight controller target GPS position."""
    ...

def set_speed(x: int, y: int, z: int) -> None:
    """Set flight controller speed."""
    ...

def set_servo(pin: int, pwm_in_us: int) -> None:
    """Set servo output PWM."""
    ...

def push_broadcast(data: bytes) -> None:
    """Push custom broadcast data bytes to output queue."""
    ...
