# apm.pyi - Type stub definition for C++ CPython 'apm' module and TelemetryData
from typing import TypedDict, List, Optional

class DetectionItem(TypedDict):
    track_id: int
    class_id: int
    confidence: float
    box: List[int]

class TelemetryData(TypedDict, total=False):
    accel_clipped_times: Optional[int]
    accel_gforce: Optional[float]
    baro_temp: Optional[float]
    baro_pressure_hpa: Optional[float]
    baro_agl_altitude_cm: Optional[float]
    rangefinder_agl_alt_cm: Optional[float]
    sys_disarm_flag: Optional[bool]  # True: Disarmed (locked), False: Armed (unlocked)
    sys_pre_arm_flag: Optional[int]
    sys_failsafe_flag: Optional[int]
    sys_apm_status: Optional[int]
    att_euler_angle_yaw_v: Optional[float]  # Virtual yaw in +-180 deg
    nav_relative_head: Optional[float]
    nav_global_head: Optional[float]
    nav_global_sat_count: Optional[int]
    nav_gps_hdop: Optional[int]
    gyro_cycle_time: Optional[float]
    battery_voltage: Optional[float]
    battery_voltage_single: Optional[float]
    cpu_temp: Optional[float]
    accel_acceleration: List[Optional[float]]  # Body frame in cm/s^2
    accel_vibe: List[Optional[float]]
    accel_raw_g: List[Optional[int]]
    att_quaternion: List[Optional[float]]
    att_euler_angle: List[Optional[float]]     # deg, yaw 0-360 deg
    gyro_angle_rate: List[Optional[float]]     # deg/s
    mag_raw_l: List[Optional[int]]
    sys_time_info: List[Optional[int]]
    nav_speed: List[Optional[float]]           # Body frame in cm/s
    nav_global_speed: List[Optional[float]]
    nav_global_pos: List[Optional[int]]        # lat and lon in 1e-7 int
    nav_global_home: List[Optional[int]]
    nav_relative_pos: List[Optional[float]]    # Body frame in cm
    rc_channel_raw: List[Optional[int]]        # 1000 - 2000 us
    ef_channel_raw: List[Optional[int]]        # 1000 - 2000 us
    BroadcastRecv: List[bytes]
    detections: List[DetectionItem]

def arm() -> None:
    """Arm the flight controller. In UserAuto mode, takes off to 50cm or configured altitude."""
    ...

def disarm() -> None:
    """Disarm the flight controller immediately (lock down ESC)."""
    ...

def set_position(x: int, y: int, z: int, yawdeg: float, reset_home: bool = True) -> None:
    """
    Set body-frame target position in cm and target yaw in +-180 deg.
    :param x: Body frame X in cm
    :param y: Body frame Y in cm
    :param z: Altitude in cm
    :param yawdeg: Target yaw using att_euler_angle_yaw_v (+-180 deg)
    :param reset_home: If True, resets _NAV_Relative_Pos to 0
    """
    ...

def set_gps_position(lat: int, lng: int, yawdeg: float, alt: int) -> None:
    """
    Set flight controller target GPS position.
    :param lat: Latitude in 1e-7 int
    :param lng: Longitude in 1e-7 int
    :param yawdeg: Target yaw in +-180 deg
    :param alt: Altitude in cm
    """
    ...

def set_speed(x: int, y: int, z: int, yaw_rate_deg_s: float) -> None:
    """
    Set flight controller body-frame speed and yaw rate.
    :param x: Body frame forward speed in cm/s
    :param y: Body frame rightward speed in cm/s
    :param z: Body frame downward speed in cm/s (positive = descent)
    :param yaw_rate_deg_s: Yaw rate angular velocity in deg/s (NOT angle)
    """
    ...

def set_servo(pin: int, pwm_in_us: int) -> None:
    """Set servo / motor output PWM (1000 - 2000 us)."""
    ...

def push_broadcast(data: bytes) -> None:
    """Push custom broadcast data bytes to output queue."""
    ...
