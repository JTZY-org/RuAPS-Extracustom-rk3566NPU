# /etc/rknn/user_app.py
from __future__ import annotations
import math
import struct
import sys
import time
from typing import TYPE_CHECKING, Optional, List, Tuple

import apm
import color_detector
import servo_controller
from protocol import CmdID, FMT_ARM, FMT_LAND, FMT_POS_REL_HORIZ, FMT_POS_VERT, FMT_GIMBAL_CONTROL

if TYPE_CHECKING:
    from apm import TelemetryData

# ==============================================================================
# Configuration & Global States
# ==============================================================================
ENABLE_TELEMETRY_MONITOR = False  # Set to True for full-terminal telemetry display

FRAME_COUNTER = 0
LAST_CALL_TIME: Optional[float] = None
SUM_LATENCY = 0.0
SUM_INTERVAL = 0.0

INITIAL_STATUS_CHECKED = False

# Landing logic states
PYTHON_LANDING = False
PYTHON_LANDING_START_TIME: Optional[float] = None

# Position control states
HORIZ_TARGET_ACTIVE = False
TARGET_X: int = 0
TARGET_Y: int = 0
TARGET_HORIZ_ALT: Optional[int] = None

VERT_TARGET_ACTIVE = False
TARGET_ALT: Optional[int] = None
# Latch XY while climbing/descending so VERT is not overridden by noisy cur_x/cur_y
TARGET_VERT_X: int = 0
TARGET_VERT_Y: int = 0

# ------------------------------------------------------------------------------
# 人物跟踪只让机头转。x/y/z 固定 0，偏航速率单位 deg/s。
# 框坐标和 exchange(width, height) 是同一套旋转后的画面，不要按地面站视频 180° 再翻。
# 增益和限幅是台架初值，飞测再改。
# PLGUserDefine.hpp 的 APMControllerSpeed 没写正偏航是左转还是右转，不要用 apm.pyi 改符号。
# ------------------------------------------------------------------------------
TRACK_DEADZONE_PX = 24
# 固定台架上原先 0.05 °/s 每像素、限幅 10，人稍偏一点几乎看不出在转。
TRACK_YAW_GAIN_DEG_S_PER_PX = 0.6
TRACK_YAW_LIMIT_DEG_S = 60
TRACK_YAW_SIGN = 1                   # 只能是 1 或 -1。画面右侧的 error_px 乘这个符号得到偏航速率

# 飞行平移跟踪：不转头，用机体系速度把人拉回画面正中。偏航保持 0。
# 前后不在这里写，由共用的后退模式决定。
# 死区沿用 TRACK_DEADZONE_PX。增益是 0.5 cm/s 每像素，限幅仍是 25 cm/s。
# Y 正方向头文件没写。地面站现有按钮约定 +Y 左、−Y 右。人在画面右侧时给负 Y。
# 飞测若左右反了，只把 TRACK_PAN_Y_SIGN 改成 +1，不要改误差的正负。
# Z 与本文件降落一致：set_speed 的第三个参数正值是下降。人在画面下方时给正 Z。
# 飞测若上下反了，只把 TRACK_PAN_Z_SIGN 改成 -1。
TRACK_PAN_GAIN_CM_S_PER_PX = 0.5
TRACK_PAN_LIMIT_CM_S = 25
TRACK_PAN_Y_SIGN = -1
TRACK_PAN_Z_SIGN = 1
FLIGHT_FEEDBACK_FMT = "<BBBbbhhhhHH"  # 17 字节，小端。0xFD 0x02

# 后退模式，偏航跟踪和平移跟踪共用。按下时记住行人框等效边长 sqrt(宽*高)。
# 框比记住的更大就后退，更小就前进。头文件没写 X 谁是机头。
# 地面站按钮约定 +X 前进、−X 后退。框变大时 size_err>0，乘 -1 得到负 X。
# 飞测若前后反了，只改 TRACK_RANGE_X_SIGN。
TRACK_RANGE_GAIN_CM_S_PER_PX = 0.5
TRACK_RANGE_LIMIT_CM_S = 25
TRACK_RANGE_DEADZONE_PX = 16
TRACK_RANGE_X_SIGN = -1
RANGE_FEEDBACK_FMT = "<BBBhhHH"  # 11 字节，小端。0xFD 0x03

# 固定自动化：万向节实验箱。位移/高度/降落/云台在这个模式下丢掉。
# 油门是 PWM μs。头文件只提供 APMControllerServo(pin, PWM)，没有写电机在哪些 pin。
# 云台占用 pin 4。FIXED_MOTOR_PINS 为空时只记状态，不调用 set_servo。
FIXED_THROTTLE_FROM = 1000
FIXED_THROTTLE_INIT = 1200
FIXED_THROTTLE_CAP = 1250
FIXED_THROTTLE_RAMP_S = 3.0
FIXED_YAW_TRAVEL_DEG = 178.0         # 中心两侧合计 356°
FIXED_MOTOR_PINS = ()                # 例如 (0, 1, 2, 3)。不要填 4。

PERSON_LABELS_PATH = "/etc/rknn/coco_80_labels_list.txt"
TRACK_FEEDBACK_FMT = "<BBBbhbhH"  # 11 字节，小端，前面不加长度

PERSON_TRACK_ENABLED = False
FLIGHT_FOLLOW_ENABLED = False        # D3 01。左右 + 上下平移，和偏航跟踪互斥
RANGE_HOLD_ENABLED = False           # D4 01。前后距离，偏航和平移都能带着走
RANGE_HOLD_REF_PX = None             # 按下后退模式时锁定的等效边长，像素
PERSON_TRACK_LOCK_ID = None          # 上一帧跟上的 track_id；消失后再改选
LAST_FRAME_WIDTH = 0
LAST_FRAME_HEIGHT = 0
LAST_DISARM_FLAG = None              # None：还没见过上锁标志，不能当成“刚上锁”
_PERSON_CLASS_ID = None
_PERSON_CLASS_RESOLVED = False
_TRACK_LOG_DIVIDER = 0

FIXED_MODE = False
FIXED_YAW_ACCUM_DEG = 0.0
FIXED_YAW_LAST_T = None
FIXED_RAMP_START = None
FIXED_THROTTLE_PWM = FIXED_THROTTLE_FROM
_FIXED_PIN_WARNED = False
_FIXED_RAMP_DONE = False


# ==============================================================================
# Lifecycle Functions
# ==============================================================================
def init(width: int, height: int, pixfmt: int) -> int:
    """
    Called once during UserAppInit.
    :param width: Image width
    :param height: Image height
    :param pixfmt: Image pixel format (V4L2 format code)
    """
    print(f"[Python] init() called: width={width}, height={height}, pixfmt={pixfmt}")
    return 0


# ==============================================================================
# Telemetry Helpers & Monitor
# ==============================================================================
def get_current_nav(telemetry: Optional['TelemetryData']) -> Tuple[int, int, int, float]:
    """
    Extracts current relative position (X, Y, Altitude) strictly from _NAV_Relative_Pos
    and Yaw angle from att_euler_angle_yaw_v.
    """
    cur_x, cur_y, cur_alt = 0, 0, 0
    if telemetry:
        nav_pos = telemetry.get('nav_relative_pos')
        if nav_pos and len(nav_pos) >= 3:
            cur_x = int(nav_pos[0]) if nav_pos[0] is not None else 0
            cur_y = int(nav_pos[1]) if nav_pos[1] is not None else 0
            cur_alt = int(nav_pos[2]) if nav_pos[2] is not None else 0

    cur_yaw = float(telemetry.get('att_euler_angle_yaw_v') or 0.0) if telemetry else 0.0
    return cur_x, cur_y, cur_alt, cur_yaw


def debug_print_telemetry(telemetry: 'TelemetryData') -> None:
    """
    Prints a formatted, full-terminal telemetry status screen.
    """
    if not telemetry:
        return

    items: List[str] = []

    # 1. Scalars
    scalars = [
        "sys_disarm_flag", "sys_pre_arm_flag", "sys_failsafe_flag", "sys_apm_status",
        "cpu_temp", "battery_voltage", "battery_voltage_single", "gyro_cycle_time",
        "baro_temp", "baro_pressure_hpa", "baro_agl_altitude_cm", "rangefinder_agl_alt_cm",
        "accel_clipped_times", "accel_gforce", "att_euler_angle_yaw_v",
        "nav_relative_head", "nav_global_head", "nav_global_sat_count", "nav_gps_hdop"
    ]
    for s in scalars:
        val = telemetry.get(s)
        val_str = f"{val:.2f}" if isinstance(val, float) else str(val)
        items.append(f"{s}:{val_str}")

    # 2. Arrays
    arrays = {
        "accel_acc": "accel_acceleration",
        "accel_vibe": "accel_vibe",
        "accel_raw_g": "accel_raw_g",
        "att_quat": "att_quaternion",
        "att_euler": "att_euler_angle",
        "gyro_rate": "gyro_angle_rate",
        "mag_raw": "mag_raw_l",
        "nav_speed": "nav_speed",
        "nav_g_speed": "nav_global_speed",
        "nav_g_pos": "nav_global_pos",
        "nav_g_home": "nav_global_home",
        "nav_rel_pos": "nav_relative_pos"
    }
    for prefix, key in arrays.items():
        arr = telemetry.get(key)
        if isinstance(arr, list):
            for idx, val in enumerate(arr):
                val_str = f"{val:.2f}" if isinstance(val, float) else str(val)
                items.append(f"{prefix}[{idx}]:{val_str}")

    # 3. RC / EF channels
    for chan_type in ["rc_channel_raw", "ef_channel_raw"]:
        arr = telemetry.get(chan_type)
        prefix = "rc" if chan_type == "rc_channel_raw" else "ef"
        if isinstance(arr, list):
            for idx, val in enumerate(arr):
                if val is not None:
                    items.append(f"{prefix}[{idx}]:{val}")

    # Format output: 4 items per line, fixed-width columns
    lines = ["\033[H\033[2J", "=== TELEMETRY MONITOR (4 ITEMS/LINE) ==="]

    landing_status = "ACTIVE" if PYTHON_LANDING else "IDLE"
    lines.append(
        f"LANDING STATE: {landing_status:<10} | HORIZ_CTRL: {'ACTIVE' if HORIZ_TARGET_ACTIVE else 'IDLE'} | "
        f"VERT_CTRL: {'ACTIVE' if VERT_TARGET_ACTIVE else 'IDLE'} | PERSON_TRACK: {'ON' if PERSON_TRACK_ENABLED else 'OFF'}"
    )

    recv_packets = telemetry.get('BroadcastRecv', [])
    recv_hex = ", ".join(p.hex().upper() for p in recv_packets) if recv_packets else "None"
    lines.append(f"LAST RECV UDP: {recv_hex:<25}")
    lines.append("---------------------------------------------------------------------------------")

    for i in range(0, len(items), 4):
        chunk = items[i:i+4]
        line_str = " | ".join(f"{item:<22}" for item in chunk)
        lines.append(line_str)

    lines.append("=================================================================================")

    sys.stdout.write("\n".join(lines) + "\n")
    sys.stdout.flush()


# ==============================================================================
# Command & Flight Processing Helpers
# ==============================================================================
def _try_apply_gimbal_control(packet: bytes) -> bool:
    """
    Parse Flutter/COM CONTROLDATA (00 FB servo_le laser_le) and set servo target.
    Returns True if this packet was a gimbal frame (caller should not treat it as a cmd).
    """
    size = struct.calcsize(FMT_GIMBAL_CONTROL)
    if len(packet) < size:
        return False
    reverse, mark, servo_pwm, _laser_pwm = struct.unpack(FMT_GIMBAL_CONTROL, packet[:size])
    if reverse != 0 or mark != CmdID.GIMBAL_CONTROL:
        return False
    servo_controller.g_servo_controller.set_target(int(servo_pwm))
    return True


def _yaw_sign_ready() -> Tuple[bool, int]:
    """TRACK_YAW_SIGN 必须是整数 1 或 -1。bool 不算。不是这个值就只回传、不调用 set_speed。"""
    sign = TRACK_YAW_SIGN
    if type(sign) is not int or sign not in (1, -1):
        return False, 0
    return True, sign


def _resolve_person_class_id() -> Optional[int]:
    """
    按标签文件找人。第一行是 person 时 class_id 为 0；
    第一行不是 person 就按名字找，不把 0 写死。
    """
    global _PERSON_CLASS_ID, _PERSON_CLASS_RESOLVED
    if _PERSON_CLASS_RESOLVED:
        return _PERSON_CLASS_ID

    _PERSON_CLASS_RESOLVED = True
    try:
        with open(PERSON_LABELS_PATH, "r", encoding="utf-8") as handle:
            labels = []
            for line in handle:
                name = line.strip().lower()
                if name:
                    labels.append(name)
    except OSError as exc:
        sys.stdout.write(f"\n[Python] person track: cannot read labels {PERSON_LABELS_PATH}: {exc}\n")
        sys.stdout.flush()
        _PERSON_CLASS_ID = None
        return None

    if not labels:
        sys.stdout.write(f"\n[Python] person track: labels file empty: {PERSON_LABELS_PATH}\n")
        sys.stdout.flush()
        _PERSON_CLASS_ID = None
        return None

    if labels[0] == "person":
        _PERSON_CLASS_ID = 0
    else:
        _PERSON_CLASS_ID = next((idx for idx, name in enumerate(labels) if name == "person"), None)

    if _PERSON_CLASS_ID is None:
        sys.stdout.write(f"\n[Python] person track: 'person' not in {PERSON_LABELS_PATH}\n")
    else:
        sys.stdout.write(
            f"\n[Python] person track: class_id={_PERSON_CLASS_ID} "
            f"label='{labels[_PERSON_CLASS_ID]}' from {PERSON_LABELS_PATH}\n"
        )
    sys.stdout.flush()
    return _PERSON_CLASS_ID


def _push_track_feedback(flags: int, turn: int, speed: int, axis: int, error_px: int, width: int) -> None:
    """固定 11 字节，小端，前面不加长度。地面站靠 0xFD 0x01 刷新跟踪框。"""
    error_px = int(error_px)
    if error_px > 32767:
        error_px = 32767
    elif error_px < -32768:
        error_px = -32768
    width = int(width)
    if width < 0:
        width = 0
    elif width > 65535:
        width = 65535
    packet = struct.pack(
        TRACK_FEEDBACK_FMT,
        0xFD,
        0x01,
        int(flags) & 0xFF,
        int(turn),
        int(speed),
        int(axis),
        error_px,
        width,
    )
    apm.push_broadcast(packet)


def _set_fixed_mode(enabled: bool) -> None:
    """E0 01 / E0 00。打开时清掉位置环和降落，偏航行程从 0 重新计。"""
    global FIXED_MODE, FIXED_YAW_ACCUM_DEG, FIXED_YAW_LAST_T
    global FIXED_RAMP_START, FIXED_THROTTLE_PWM, _FIXED_RAMP_DONE
    global HORIZ_TARGET_ACTIVE, VERT_TARGET_ACTIVE, PYTHON_LANDING
    FIXED_MODE = enabled
    FIXED_YAW_ACCUM_DEG = 0.0
    FIXED_YAW_LAST_T = None
    FIXED_RAMP_START = None
    FIXED_THROTTLE_PWM = FIXED_THROTTLE_FROM
    _FIXED_RAMP_DONE = False
    if enabled:
        HORIZ_TARGET_ACTIVE = False
        VERT_TARGET_ACTIVE = False
        PYTHON_LANDING = False
        # 实验箱只留偏航跟踪。平移速度环在这里关掉，避免和油门斜坡抢飞控。
        _stop_flight_follow("fixed box", always_zero_speed=False)
        _stop_range_hold("fixed box", always_zero_speed=False)
        sys.stdout.write(
            "\n[Python] Fixed-box mode ON. horiz/vert/land/gimbal ignored. "
            f"throttle ramp {FIXED_THROTTLE_FROM}->{FIXED_THROTTLE_INIT} in {FIXED_THROTTLE_RAMP_S:.0f}s, "
            f"cap {FIXED_THROTTLE_CAP}. motor pins={FIXED_MOTOR_PINS or 'unset'}\n"
        )
    else:
        sys.stdout.write("\n[Python] Fixed-box mode OFF\n")
    sys.stdout.flush()


def _release_fixed_throttle() -> None:
    """上锁时停掉斜坡。已经写过电机的话把 PWM 收回 1000。"""
    global FIXED_RAMP_START, FIXED_THROTTLE_PWM, _FIXED_RAMP_DONE
    FIXED_RAMP_START = None
    FIXED_THROTTLE_PWM = FIXED_THROTTLE_FROM
    _FIXED_RAMP_DONE = False
    _write_fixed_throttle(FIXED_THROTTLE_FROM)


def _begin_fixed_throttle_ramp() -> None:
    """解锁后开始 3 秒油门斜坡。模式没开时不动。实机平移跟踪不走这里。"""
    global FIXED_RAMP_START, FIXED_THROTTLE_PWM, _FIXED_RAMP_DONE
    if not FIXED_MODE:
        return
    FIXED_RAMP_START = time.perf_counter()
    FIXED_THROTTLE_PWM = FIXED_THROTTLE_FROM
    _FIXED_RAMP_DONE = False
    sys.stdout.write(
        f"\n[Python] Fixed throttle ramp start -> {FIXED_THROTTLE_INIT} in {FIXED_THROTTLE_RAMP_S:.0f}s, "
        f"cap {FIXED_THROTTLE_CAP}\n"
    )
    sys.stdout.flush()


def _write_fixed_throttle(pwm: int) -> None:
    """把油门 PWM 写到已填写的电机 pin。没填 pin 就只打印一次。"""
    global _FIXED_PIN_WARNED
    pwm = int(max(FIXED_THROTTLE_FROM, min(FIXED_THROTTLE_CAP, pwm)))
    pins = tuple(int(pin) for pin in FIXED_MOTOR_PINS)
    if not pins:
        if not _FIXED_PIN_WARNED:
            _FIXED_PIN_WARNED = True
            sys.stdout.write(
                "\n[Python] Fixed throttle not written: FIXED_MOTOR_PINS is empty. "
                "PLGUserDefine.hpp does not name motor pins. Gimbal uses pin 4.\n"
            )
            sys.stdout.flush()
        return
    for pin in pins:
        if pin == 4:
            continue
        apm.set_servo(pin, pwm)


def process_fixed_throttle(now: float) -> None:
    """只在固定自动化且斜坡已开始时跑。平移跟踪不开这个模式，不会进这里。"""
    global FIXED_THROTTLE_PWM, _FIXED_RAMP_DONE
    if not FIXED_MODE or FIXED_RAMP_START is None:
        return
    elapsed = now - FIXED_RAMP_START
    span = max(FIXED_THROTTLE_INIT - FIXED_THROTTLE_FROM, 0)
    if elapsed >= FIXED_THROTTLE_RAMP_S:
        pwm = FIXED_THROTTLE_INIT
        done_edge = not _FIXED_RAMP_DONE
        _FIXED_RAMP_DONE = True
    else:
        pwm = int(round(FIXED_THROTTLE_FROM + span * (elapsed / FIXED_THROTTLE_RAMP_S)))
        done_edge = False
    pwm = min(pwm, FIXED_THROTTLE_CAP)
    if pwm != FIXED_THROTTLE_PWM or done_edge:
        FIXED_THROTTLE_PWM = pwm
        _write_fixed_throttle(pwm)
        if done_edge:
            sys.stdout.write(f"\n[Python] Fixed throttle hold {pwm} us (cap {FIXED_THROTTLE_CAP})\n")
            sys.stdout.flush()


def _limit_fixed_yaw(yaw_rate: int, now: float) -> int:
    """
    按已经发出去的偏航速率积分，离开进入模式时的方向累计超过 ±178° 就停。
    不靠偏航角的正负方向，头文件没写正转速对应左转还是右转。
    """
    global FIXED_YAW_ACCUM_DEG, FIXED_YAW_LAST_T
    if not FIXED_MODE:
        return yaw_rate
    dt = 0.0
    if FIXED_YAW_LAST_T is not None:
        raw_dt = now - FIXED_YAW_LAST_T
        if 0.0 < raw_dt < 1.0:
            dt = raw_dt
    FIXED_YAW_LAST_T = now
    if FIXED_YAW_ACCUM_DEG >= FIXED_YAW_TRAVEL_DEG and yaw_rate > 0:
        yaw_rate = 0
    elif FIXED_YAW_ACCUM_DEG <= -FIXED_YAW_TRAVEL_DEG and yaw_rate < 0:
        yaw_rate = 0
    FIXED_YAW_ACCUM_DEG += yaw_rate * dt
    return yaw_rate


def _apply_track_yaw(yaw_rate_deg_s: int, x_cm_s: int = 0) -> None:
    """写偏航速率。后退模式开着时 x 用框大小算出来的前后速度，否则 x 为 0。"""
    apm.set_speed(int(x_cm_s), 0, 0, int(yaw_rate_deg_s))


def _start_person_track() -> None:
    """D1 01。清掉位置环锁存，避免和偏航速度环一起写。"""
    global PERSON_TRACK_ENABLED, PERSON_TRACK_LOCK_ID
    global HORIZ_TARGET_ACTIVE, VERT_TARGET_ACTIVE
    # 偏航和平移不能同时写速度。先把平移停掉。
    _stop_flight_follow("yaw track", always_zero_speed=False)
    PERSON_TRACK_ENABLED = True
    PERSON_TRACK_LOCK_ID = None
    HORIZ_TARGET_ACTIVE = False
    VERT_TARGET_ACTIVE = False
    ready, sign = _yaw_sign_ready()
    if ready:
        msg = f"yaw sign={sign:+d}, set_speed(0, 0, 0, yaw_rate_deg_s)"
    else:
        msg = "TRACK_YAW_SIGN is not 1 or -1, feedback only, set_speed blocked"
    sys.stdout.write(f"\n[Python] Person track ON. {msg}\n")
    sys.stdout.flush()


def _stop_person_track(reason: str, always_zero_speed: bool) -> None:
    """
    关掉跟踪。
    always_zero_speed：D1 00 / 降落 / 上锁即使本来没在跟踪也要把速度清零。
    跟踪原本开着时，无论哪种原因都清零，并立刻再推一帧停止回传（bit2=1）。
    """
    global PERSON_TRACK_ENABLED, PERSON_TRACK_LOCK_ID
    global HORIZ_TARGET_ACTIVE, VERT_TARGET_ACTIVE
    was_on = PERSON_TRACK_ENABLED
    PERSON_TRACK_ENABLED = False
    PERSON_TRACK_LOCK_ID = None
    # 水平位移包会在本函数返回后自己重新打开 HORIZ，那种情况不要在这里清目标。
    if always_zero_speed:
        HORIZ_TARGET_ACTIVE = False
        VERT_TARGET_ACTIVE = False

    did_speed = False
    # 平移跟踪还占着速度环，或后退模式还要写 x 时，不要把速度清成全 0。
    if (was_on or always_zero_speed) and not FLIGHT_FOLLOW_ENABLED and not RANGE_HOLD_ENABLED:
        apm.set_speed(0, 0, 0, 0)
        did_speed = True

    if was_on:
        # bit0=0 跟踪关，bit2=1 因为上面已经 set_speed(0,0,0,0)。axis=2 表示 speed 是偏航速率。
        flags = 0x04 if did_speed else 0x00
        _push_track_feedback(flags, 0, 0, 2, 0, LAST_FRAME_WIDTH)
        sys.stdout.write(f"\n[Python] Person track OFF ({reason}), speed zeroed\n")
        sys.stdout.flush()


def _stop_track_if_just_locked(telemetry: Optional['TelemetryData']) -> None:
    """RC 上锁不一定有 CB/CC 包。只在解锁变成上锁的这一拍停跟踪。"""
    global LAST_DISARM_FLAG
    if not telemetry:
        return
    flag = telemetry.get("sys_disarm_flag")
    if flag is None:
        return
    locked = bool(flag)
    if LAST_DISARM_FLAG is False and locked:
        _stop_flight_follow("disarm flag", always_zero_speed=True)
        _stop_person_track("disarm flag", always_zero_speed=True)
        _stop_range_hold("disarm flag", always_zero_speed=True)
        if FIXED_MODE:
            _release_fixed_throttle()
    LAST_DISARM_FLAG = locked


def _select_person(detections, person_class_id: int):
    """多人时沿用上一帧的 track_id；这个 id 消失后再选 confidence 最高的人。"""
    global PERSON_TRACK_LOCK_ID
    people = []
    for det in detections or []:
        if not isinstance(det, dict):
            continue
        if det.get("class_id") != person_class_id:
            continue
        box = det.get("box")
        if not isinstance(box, (list, tuple)) or len(box) < 4:
            continue
        people.append(det)
    if not people:
        return None

    if PERSON_TRACK_LOCK_ID is not None:
        for det in people:
            if det.get("track_id") == PERSON_TRACK_LOCK_ID:
                return det

    best = max(people, key=lambda det: float(det.get("confidence") or 0.0))
    PERSON_TRACK_LOCK_ID = best.get("track_id")
    return best


def process_person_track(width: int, telemetry: Optional['TelemetryData']) -> None:
    """
    跟踪开着时每帧都跑，有没有人都回传。
    符号是 1 或 -1 时每帧都 set_speed(0, 0, 0, yaw)。不是这个值时只回传。
    """
    global _TRACK_LOG_DIVIDER
    if not PERSON_TRACK_ENABLED or PYTHON_LANDING:
        return

    frame_w = int(width) if width and width > 0 else 0
    person = None
    if frame_w > 0 and telemetry:
        person_class_id = _resolve_person_class_id()
        if person_class_id is not None:
            person = _select_person(telemetry.get("detections"), person_class_id)

    error_px = 0
    turn = 0
    used_box = person is not None and frame_w > 0
    if used_box:
        box = person["box"]
        center_x = (float(box[0]) + float(box[2])) / 2.0
        # 人在画面右侧为正，不按地面站视频的 180° 再翻
        error_px = int(round(center_x - (frame_w / 2.0)))
        if error_px > TRACK_DEADZONE_PX:
            turn = 1
        elif error_px < -TRACK_DEADZONE_PX:
            turn = -1
        else:
            turn = 0

    ready, sign = _yaw_sign_ready()
    yaw_rate = 0
    commanded = False
    box = person["box"] if used_box else None
    x_cm_s, size_err, side_px, has_ref, range_ready = (0, 0, 0, False, False)
    if RANGE_HOLD_ENABLED:
        x_cm_s, size_err, side_px, has_ref, range_ready = _range_command(box, used_box)
        if not range_ready:
            x_cm_s = 0
    # axis=2：这一拍的 speed 是偏航速率 deg/s，地面站据此显示「偏航 ±N °/s」
    axis_code = 2
    if ready:
        if used_box and turn != 0:
            raw = error_px * sign * TRACK_YAW_GAIN_DEG_S_PER_PX
            limit = float(TRACK_YAW_LIMIT_DEG_S)
            if raw > limit:
                raw = limit
            elif raw < -limit:
                raw = -limit
            yaw_rate = int(round(raw))
        # 固定模式累计转过 ±178° 后，继续往外的转速写成 0。
        yaw_rate = _limit_fixed_yaw(yaw_rate, time.perf_counter())
        # 死区内或这一帧没有人：上一拍的转速会留着，必须写成 0。x 是共用的后退模式。
        _apply_track_yaw(yaw_rate, x_cm_s)
        commanded = True
    elif RANGE_HOLD_ENABLED and range_ready:
        apm.set_speed(int(x_cm_s), 0, 0, 0)
        commanded = True

    flags = 0x01  # bit0 跟踪开
    if used_box:
        flags |= 0x02
    if commanded:
        flags |= 0x04  # bit2：本帧确实调用了 set_speed
    _push_track_feedback(flags, turn, yaw_rate if commanded else 0, axis_code, error_px if used_box else 0, frame_w)
    if RANGE_HOLD_ENABLED:
        range_flags = 0x01
        if used_box:
            range_flags |= 0x02
        if commanded and range_ready:
            range_flags |= 0x04
        if has_ref:
            range_flags |= 0x08
        ref_px = int(round(RANGE_HOLD_REF_PX)) if RANGE_HOLD_REF_PX else 0
        _push_range_feedback(range_flags, x_cm_s if range_ready else 0, size_err, ref_px, side_px)

    _TRACK_LOG_DIVIDER += 1
    if _TRACK_LOG_DIVIDER >= 30:
        _TRACK_LOG_DIVIDER = 0
        who = "none" if not used_box else f"id={person.get('track_id')}"
        sys.stdout.write(
            f"[Python] track {who} turn={turn} err={error_px} "
            f"yaw={yaw_rate if commanded else 0} deg/s x={x_cm_s} cm/s commanded={int(commanded)}\n"
        )
        sys.stdout.flush()


def _pan_signs_ready() -> Tuple[bool, int, int]:
    """两个平移符号都必须是整数 1 或 -1。bool 不算。否则只回传、不调用 set_speed。"""
    y_sign = TRACK_PAN_Y_SIGN
    z_sign = TRACK_PAN_Z_SIGN
    if type(y_sign) is not int or y_sign not in (1, -1):
        return False, 0, 0
    if type(z_sign) is not int or z_sign not in (1, -1):
        return False, 0, 0
    return True, y_sign, z_sign


def _clamp_pan_cm_s(raw: float, limit: float = None) -> int:
    """速度限幅。不传 limit 时用平移上限。"""
    cap = float(TRACK_PAN_LIMIT_CM_S if limit is None else limit)
    if raw > cap:
        raw = cap
    elif raw < -cap:
        raw = -cap
    return int(round(raw))


def _box_side_px(box) -> float:
    """行人框的等效边长，像素。宽或高为 0 时返回 0。"""
    width = abs(float(box[2]) - float(box[0]))
    height = abs(float(box[3]) - float(box[1]))
    if width <= 0.0 or height <= 0.0:
        return 0.0
    return math.sqrt(width * height)


def _range_sign_ready() -> Tuple[bool, int]:
    """前后符号必须是整数 1 或 -1。"""
    sign = TRACK_RANGE_X_SIGN
    if type(sign) is not int or sign not in (1, -1):
        return False, 0
    return True, sign


def _range_command(box, used_box: bool):
    """
    后退模式开着时，按当前框相对记住的边长给出 x。
    返回 x_cm_s、边长差、当前边长、是否已记住、符号是否可下发。
    """
    global RANGE_HOLD_REF_PX
    ready, sign = _range_sign_ready()
    side = _box_side_px(box) if used_box and box is not None else 0.0
    if side > 0.0 and RANGE_HOLD_REF_PX is None:
        RANGE_HOLD_REF_PX = side
        sys.stdout.write(f"\n[Python] Range hold reference {side:.0f} px\n")
        sys.stdout.flush()
    has_ref = RANGE_HOLD_REF_PX is not None and RANGE_HOLD_REF_PX > 0.0
    size_err = 0
    x_cm_s = 0
    if has_ref and side > 0.0:
        size_err = int(round(side - RANGE_HOLD_REF_PX))
        if ready and abs(size_err) > TRACK_RANGE_DEADZONE_PX:
            x_cm_s = _clamp_pan_cm_s(
                size_err * sign * TRACK_RANGE_GAIN_CM_S_PER_PX,
                TRACK_RANGE_LIMIT_CM_S,
            )
    return x_cm_s, size_err, int(round(side)), has_ref, ready


def _push_range_feedback(flags: int, x_cm_s: int, size_err: int, ref_px: int, cur_px: int) -> None:
    """固定 11 字节。0xFD 0x03。地面站用来显示后退模式。"""

    def clamp_i16(value: int) -> int:
        value = int(value)
        if value > 32767:
            return 32767
        if value < -32768:
            return -32768
        return value

    def clamp_u16(value: int) -> int:
        value = int(value)
        if value < 0:
            return 0
        if value > 65535:
            return 65535
        return value

    packet = struct.pack(
        RANGE_FEEDBACK_FMT,
        0xFD,
        0x03,
        int(flags) & 0xFF,
        clamp_i16(x_cm_s),
        clamp_i16(size_err),
        clamp_u16(ref_px),
        clamp_u16(cur_px),
    )
    apm.push_broadcast(packet)


def _start_range_hold() -> None:
    """D4 01。记住的边长等下一帧有人再锁。偏航和平移都不用关。"""
    global RANGE_HOLD_ENABLED, RANGE_HOLD_REF_PX
    global HORIZ_TARGET_ACTIVE, VERT_TARGET_ACTIVE
    if FIXED_MODE:
        sys.stdout.write("\n[Python] Range hold ignored: fixed-box mode is on\n")
        sys.stdout.flush()
        return
    if PYTHON_LANDING:
        sys.stdout.write("\n[Python] Range hold ignored: landing\n")
        sys.stdout.flush()
        return
    RANGE_HOLD_ENABLED = True
    RANGE_HOLD_REF_PX = None
    HORIZ_TARGET_ACTIVE = False
    VERT_TARGET_ACTIVE = False
    ready, sign = _range_sign_ready()
    if ready:
        msg = (
            f"x_sign={sign:+d}, bigger box -> back. "
            f"gain={TRACK_RANGE_GAIN_CM_S_PER_PX} cm/s/px limit={TRACK_RANGE_LIMIT_CM_S}"
        )
    else:
        msg = "TRACK_RANGE_X_SIGN is not 1 or -1, feedback only, set_speed blocked"
    sys.stdout.write(f"\n[Python] Range hold ON. {msg}\n")
    sys.stdout.flush()


def _stop_range_hold(reason: str, always_zero_speed: bool) -> None:
    """关掉后退模式。偏航或平移还开着时不清速度，下一拍由那一套自己写。"""
    global RANGE_HOLD_ENABLED, RANGE_HOLD_REF_PX
    global HORIZ_TARGET_ACTIVE, VERT_TARGET_ACTIVE
    was_on = RANGE_HOLD_ENABLED
    RANGE_HOLD_ENABLED = False
    RANGE_HOLD_REF_PX = None
    if always_zero_speed:
        HORIZ_TARGET_ACTIVE = False
        VERT_TARGET_ACTIVE = False
    did_speed = False
    if (was_on or always_zero_speed) and not PERSON_TRACK_ENABLED and not FLIGHT_FOLLOW_ENABLED:
        apm.set_speed(0, 0, 0, 0)
        did_speed = True
    if was_on:
        flags = 0x04 if did_speed else 0x00
        _push_range_feedback(flags, 0, 0, 0, 0)
        sys.stdout.write(f"\n[Python] Range hold OFF ({reason})\n")
        sys.stdout.flush()


def _apply_flight_speed(x_cm_s: int, y_cm_s: int, z_cm_s: int) -> None:
    """写前后、左右、上下。偏航保持 0，不调用 set_position。"""
    apm.set_speed(int(x_cm_s), int(y_cm_s), int(z_cm_s), 0)


def _push_flight_feedback(
    flags: int,
    pan: int,
    vert: int,
    y_cm_s: int,
    z_cm_s: int,
    err_x: int,
    err_y: int,
    width: int,
    height: int,
) -> None:
    """固定 17 字节。地面站靠 0xFD 0x02 刷新平移跟踪框。"""

    def clamp_i16(value: int) -> int:
        value = int(value)
        if value > 32767:
            return 32767
        if value < -32768:
            return -32768
        return value

    def clamp_u16(value: int) -> int:
        value = int(value)
        if value < 0:
            return 0
        if value > 65535:
            return 65535
        return value

    packet = struct.pack(
        FLIGHT_FEEDBACK_FMT,
        0xFD,
        0x02,
        int(flags) & 0xFF,
        int(pan),
        int(vert),
        clamp_i16(y_cm_s),
        clamp_i16(z_cm_s),
        clamp_i16(err_x),
        clamp_i16(err_y),
        clamp_u16(width),
        clamp_u16(height),
    )
    apm.push_broadcast(packet)


def _start_flight_follow() -> None:
    """D3 01。退出位置环，并停掉偏航跟踪，避免两套速度环一起写。"""
    global FLIGHT_FOLLOW_ENABLED, PERSON_TRACK_LOCK_ID
    global HORIZ_TARGET_ACTIVE, VERT_TARGET_ACTIVE
    if FIXED_MODE:
        sys.stdout.write("\n[Python] Flight follow ignored: fixed-box mode is on\n")
        sys.stdout.flush()
        return
    if PYTHON_LANDING:
        sys.stdout.write("\n[Python] Flight follow ignored: landing\n")
        sys.stdout.flush()
        return
    _stop_person_track("flight follow", always_zero_speed=False)
    FLIGHT_FOLLOW_ENABLED = True
    PERSON_TRACK_LOCK_ID = None
    HORIZ_TARGET_ACTIVE = False
    VERT_TARGET_ACTIVE = False
    ready, y_sign, z_sign = _pan_signs_ready()
    if ready:
        msg = (
            f"y_sign={y_sign:+d} z_sign={z_sign:+d}, set_speed(0, y, z, 0) "
            f"gain={TRACK_PAN_GAIN_CM_S_PER_PX} cm/s/px limit={TRACK_PAN_LIMIT_CM_S}"
        )
    else:
        msg = "TRACK_PAN_Y_SIGN / TRACK_PAN_Z_SIGN is not 1 or -1, feedback only, set_speed blocked"
    sys.stdout.write(f"\n[Python] Flight follow ON. {msg}\n")
    sys.stdout.flush()


def _stop_flight_follow(reason: str, always_zero_speed: bool) -> None:
    """
    关掉平移跟踪。
    偏航跟踪还开着时不清速度，避免 D3 00 把偏航转速掐掉。
    原本开着时推一帧 0xFD 0x02，bit0=0。
    """
    global FLIGHT_FOLLOW_ENABLED, PERSON_TRACK_LOCK_ID
    global HORIZ_TARGET_ACTIVE, VERT_TARGET_ACTIVE
    was_on = FLIGHT_FOLLOW_ENABLED
    FLIGHT_FOLLOW_ENABLED = False
    if was_on:
        PERSON_TRACK_LOCK_ID = None
    if always_zero_speed:
        HORIZ_TARGET_ACTIVE = False
        VERT_TARGET_ACTIVE = False

    did_speed = False
    # 偏航跟踪或后退模式还占着速度环时，不要在这里清成全 0。
    if (was_on or always_zero_speed) and not PERSON_TRACK_ENABLED and not RANGE_HOLD_ENABLED:
        apm.set_speed(0, 0, 0, 0)
        did_speed = True

    if was_on:
        flags = 0x04 if did_speed else 0x00
        _push_flight_feedback(flags, 0, 0, 0, 0, 0, 0, LAST_FRAME_WIDTH, LAST_FRAME_HEIGHT)
        sys.stdout.write(f"\n[Python] Flight follow OFF ({reason}), speed zeroed\n")
        sys.stdout.flush()


def process_flight_follow(width: int, height: int, telemetry: Optional['TelemetryData']) -> None:
    """
    平移跟踪或单独的后退模式每帧都跑。
    平移开着时写 y、z。后退模式开着时再写 x。两套都关着的偏航不走这里。
    """
    global _TRACK_LOG_DIVIDER
    follow_on = FLIGHT_FOLLOW_ENABLED and not PYTHON_LANDING and not FIXED_MODE
    range_on = RANGE_HOLD_ENABLED and not PYTHON_LANDING and not FIXED_MODE
    if not follow_on and not range_on:
        return

    frame_w = int(width) if width and width > 0 else 0
    frame_h = int(height) if height and height > 0 else 0
    person = None
    if frame_w > 0 and frame_h > 0 and telemetry:
        person_class_id = _resolve_person_class_id()
        if person_class_id is not None:
            person = _select_person(telemetry.get("detections"), person_class_id)

    err_x = 0
    err_y = 0
    pan = 0
    vert = 0
    used_box = person is not None and frame_w > 0 and frame_h > 0
    box = person["box"] if used_box else None
    if used_box:
        center_x = (float(box[0]) + float(box[2])) / 2.0
        center_y = (float(box[1]) + float(box[3])) / 2.0
        # 右为正、下为正。框和 exchange 的宽高是同一套画面，不按地面站 180° 再翻。
        err_x = int(round(center_x - (frame_w / 2.0)))
        err_y = int(round(center_y - (frame_h / 2.0)))
        if err_x > TRACK_DEADZONE_PX:
            pan = 1
        elif err_x < -TRACK_DEADZONE_PX:
            pan = -1
        if err_y > TRACK_DEADZONE_PX:
            vert = 1
        elif err_y < -TRACK_DEADZONE_PX:
            vert = -1

    ready, y_sign, z_sign = _pan_signs_ready()
    y_cm_s = 0
    z_cm_s = 0
    commanded = False
    if follow_on and ready:
        if used_box and pan != 0:
            # 人在右侧（err_x>0）乘 Y 符号。默认 -1，对应地面站「−Y = 右移」。
            y_cm_s = _clamp_pan_cm_s(err_x * y_sign * TRACK_PAN_GAIN_CM_S_PER_PX)
        if used_box and vert != 0:
            # 人在下方（err_y>0）乘 Z 符号。默认 +1，与降落的正 Z 下降一致。
            z_cm_s = _clamp_pan_cm_s(err_y * z_sign * TRACK_PAN_GAIN_CM_S_PER_PX)
        commanded = True

    x_cm_s, size_err, side_px, has_ref, range_ready = (0, 0, 0, False, False)
    if range_on:
        x_cm_s, size_err, side_px, has_ref, range_ready = _range_command(box, used_box)
        if not range_ready:
            x_cm_s = 0
    if commanded or (range_on and range_ready):
        _apply_flight_speed(x_cm_s if range_on and range_ready else 0, y_cm_s, z_cm_s)
        commanded = True

    if follow_on:
        flags = 0x01
        if used_box:
            flags |= 0x02
        if commanded and ready:
            flags |= 0x04
        _push_flight_feedback(
            flags,
            pan,
            vert,
            y_cm_s if ready else 0,
            z_cm_s if ready else 0,
            err_x if used_box else 0,
            err_y if used_box else 0,
            frame_w,
            frame_h,
        )
    if range_on:
        range_flags = 0x01
        if used_box:
            range_flags |= 0x02
        if range_ready:
            range_flags |= 0x04
        if has_ref:
            range_flags |= 0x08
        ref_px = int(round(RANGE_HOLD_REF_PX)) if RANGE_HOLD_REF_PX else 0
        _push_range_feedback(range_flags, x_cm_s if range_ready else 0, size_err, ref_px, side_px)

    _TRACK_LOG_DIVIDER += 1
    if _TRACK_LOG_DIVIDER >= 30:
        _TRACK_LOG_DIVIDER = 0
        who = "none" if not used_box else f"id={person.get('track_id')}"
        sys.stdout.write(
            f"[Python] follow {who} pan={pan} vert={vert} "
            f"err=({err_x},{err_y}) x={x_cm_s} y={y_cm_s} z={z_cm_s} cm/s "
            f"commanded={int(commanded)}\n"
        )
        sys.stdout.flush()


def handle_broadcast_packets(packets: List[bytes], telemetry: Optional['TelemetryData']) -> None:
    """
    Parses received UDP broadcast packets using struct.unpack and executes actions.
    """
    global PYTHON_LANDING, HORIZ_TARGET_ACTIVE, TARGET_X, TARGET_Y, TARGET_HORIZ_ALT
    global VERT_TARGET_ACTIVE, TARGET_ALT, TARGET_VERT_X, TARGET_VERT_Y

    other_packets = []
    for p in packets:
        if FIXED_MODE and len(p) >= 2 and p[0] == 0x00 and p[1] == CmdID.GIMBAL_CONTROL:
            sys.stdout.write(f"\n[Python] Gimbal ignored in fixed-box mode raw={p.hex()}\n")
            sys.stdout.flush()
            continue
        if _try_apply_gimbal_control(p):
            continue
        other_packets.append(p)

    if other_packets:
        sys.stdout.write(f"\n[Python] BroadcastRecv Packets: {[p.hex() for p in other_packets]}\n")
        sys.stdout.flush()

    cur_x, cur_y, cur_alt, cur_yaw = get_current_nav(telemetry)

    for p in other_packets:
        if not p:
            continue
        cmd_id = p[0]

        # 1. ARM / DISARM (CB / CC)
        if cmd_id in (CmdID.ARM, CmdID.ARM_ALT):
            size = struct.calcsize(FMT_ARM)
            if len(p) >= size:
                _, action = struct.unpack(FMT_ARM, p[:size])
                if action == 1:
                    sys.stdout.write(f"\n[Python] Received {cmd_id:02X} 01: Arming flight controller\n")
                    sys.stdout.flush()
                    apm.arm()
                    _begin_fixed_throttle_ramp()
                elif action == 0:
                    sys.stdout.write(f"\n[Python] Received {cmd_id:02X} 00: Disarming flight controller and setting speed to 0\n")
                    sys.stdout.flush()
                    apm.disarm()
                    # 上锁：两套跟踪都停，速度清零。停止帧在各自的 stop 里推。
                    _stop_flight_follow("disarm cmd", always_zero_speed=True)
                    _stop_person_track("disarm cmd", always_zero_speed=True)
                    _stop_range_hold("disarm cmd", always_zero_speed=True)
                    if FIXED_MODE:
                        _release_fixed_throttle()

        # 2. Standalone Speed Landing (B0 / B1)
        elif cmd_id in (CmdID.LAND, CmdID.LAND_ALT):
            if FIXED_MODE:
                sys.stdout.write(f"\n[Python] Land ignored in fixed-box mode raw={p.hex()}\n")
                sys.stdout.flush()
                continue
            size = struct.calcsize(FMT_LAND)
            if len(p) >= size:
                _, action = struct.unpack(FMT_LAND, p[:size])
                if action == 1:
                    # 降落先停两套跟踪并清速度，后面的 process_landing 再写成下降速度。
                    _stop_flight_follow("land", always_zero_speed=True)
                    _stop_person_track("land", always_zero_speed=True)
                    _stop_range_hold("land", always_zero_speed=True)
                    if not PYTHON_LANDING:
                        sys.stdout.write(f"\n[Python] Received {cmd_id:02X} 01: Starting speed landing (50cm/s descent)\n")
                        sys.stdout.flush()
                        PYTHON_LANDING = True
                        HORIZ_TARGET_ACTIVE = False
                        VERT_TARGET_ACTIVE = False
                elif action == 0:
                    sys.stdout.write(f"\n[Python] Received {cmd_id:02X} 00: Canceling speed landing\n")
                    sys.stdout.flush()
                    PYTHON_LANDING = False

        # 3. 0xD1 用包长区分：2 字节是跟踪开关，5 字节及以上是水平位移
        elif cmd_id == CmdID.POS_REL_HORIZ:
            if len(p) == 2:
                action = p[1]
                if action == 0x01:
                    sys.stdout.write(f"\n[Python] Received D1 01: start person track raw={p.hex()}\n")
                    sys.stdout.flush()
                    _start_person_track()
                elif action == 0x00:
                    sys.stdout.write(f"\n[Python] Received D1 00: stop person track raw={p.hex()}\n")
                    sys.stdout.flush()
                    _stop_person_track("D1 00", always_zero_speed=True)
                else:
                    sys.stdout.write(f"\n[Python] D1 toggle ignored: raw={p.hex()}\n")
                    sys.stdout.flush()
                continue

            if FIXED_MODE:
                sys.stdout.write(f"\n[Python] Horiz move ignored in fixed-box mode raw={p.hex()}\n")
                sys.stdout.flush()
                continue

            size = struct.calcsize(FMT_POS_REL_HORIZ)
            if len(p) >= size:
                # 位置环和速度环不能同时写。先停两套跟踪并清掉上一拍速度，再走原来的水平位移。
                _stop_flight_follow("D1 horiz", always_zero_speed=False)
                _stop_person_track("D1 horiz", always_zero_speed=False)
                _stop_range_hold("D1 horiz", always_zero_speed=False)
                _, target_x, target_y = struct.unpack(FMT_POS_REL_HORIZ, p[:size])

                TARGET_X = target_x
                TARGET_Y = target_y
                TARGET_HORIZ_ALT = cur_alt  # Latch current _NAV_Relative_Pos[2] altitude
                HORIZ_TARGET_ACTIVE = True
                # D1 owns the loop — do not let a previous D2 keep fighting altitude
                VERT_TARGET_ACTIVE = False

                sys.stdout.write(
                    f"\n[Python] Received D1 raw={p.hex()} unpack=({target_x},{target_y}) cm | "
                    f"nav=({cur_x},{cur_y},{cur_alt}) hold_alt={TARGET_HORIZ_ALT} yaw={cur_yaw:.1f} | "
                    f"set_position({TARGET_X},{TARGET_Y},{TARGET_HORIZ_ALT},{cur_yaw:.1f},reset=False) VERT cleared\n"
                )
                sys.stdout.flush()

                # Set reset_home=False to start moving towards target
                apm.set_position(TARGET_X, TARGET_Y, TARGET_HORIZ_ALT, cur_yaw, False)
            else:
                sys.stdout.write(f"\n[Python] D1 packet too short: len={len(p)} raw={p.hex()} need={size}\n")
                sys.stdout.flush()

        # 4. Vertical position control (0xD2): cmdid(uint8) + alt(int16) [cm]
        elif cmd_id == CmdID.POS_VERT:
            if FIXED_MODE:
                sys.stdout.write(f"\n[Python] D2 ignored in fixed-box mode raw={p.hex()}\n")
                sys.stdout.flush()
                continue
            if PERSON_TRACK_ENABLED or FLIGHT_FOLLOW_ENABLED or RANGE_HOLD_ENABLED:
                sys.stdout.write(
                    f"\n[Python] D2 ignored while tracking raw={p.hex()} (speed loop owns the aircraft)\n"
                )
                sys.stdout.flush()
                continue
            size = struct.calcsize(FMT_POS_VERT)
            if len(p) >= size:
                raw_cmd, target_alt = struct.unpack(FMT_POS_VERT, p[:size])

                # D2 must preempt HORIZ — otherwise process_position_control keeps
                # rewriting altitude back to TARGET_HORIZ_ALT and vertical looks "dead".
                HORIZ_TARGET_ACTIVE = False
                TARGET_ALT = int(target_alt)
                TARGET_VERT_X = cur_x
                TARGET_VERT_Y = cur_y
                VERT_TARGET_ACTIVE = True

                sys.stdout.write(
                    f"\n[Python] Received D2 raw={p.hex()} bytes={list(p[:size])} "
                    f"unpack=(cmd=0x{raw_cmd:02X}, alt={target_alt}) | "
                    f"nav=({cur_x},{cur_y},alt={cur_alt}) yaw={cur_yaw:.1f} | "
                    f"HORIZ cleared → set_position({TARGET_VERT_X},{TARGET_VERT_Y},{TARGET_ALT},{cur_yaw:.1f},reset=False)\n"
                )
                sys.stdout.flush()

                apm.set_position(TARGET_VERT_X, TARGET_VERT_Y, TARGET_ALT, cur_yaw, False)
            else:
                sys.stdout.write(
                    f"\n[Python] D2 packet too short / misaligned: len={len(p)} raw={p.hex()} "
                    f"need={size} (expect D2 + int16_le alt_cm)\n"
                )
                sys.stdout.flush()
        elif cmd_id == CmdID.FLIGHT_FOLLOW and len(p) == 2:
            action = p[1]
            if action == 0x01:
                sys.stdout.write(f"\n[Python] Received D3 01: start flight follow raw={p.hex()}\n")
                sys.stdout.flush()
                _start_flight_follow()
            elif action == 0x00:
                sys.stdout.write(f"\n[Python] Received D3 00: stop flight follow raw={p.hex()}\n")
                sys.stdout.flush()
                _stop_flight_follow("D3 00", always_zero_speed=True)
            else:
                sys.stdout.write(f"\n[Python] D3 toggle ignored: raw={p.hex()}\n")
                sys.stdout.flush()
        elif cmd_id == CmdID.RANGE_HOLD and len(p) == 2:
            action = p[1]
            if action == 0x01:
                sys.stdout.write(f"\n[Python] Received D4 01: start range hold raw={p.hex()}\n")
                sys.stdout.flush()
                _start_range_hold()
            elif action == 0x00:
                sys.stdout.write(f"\n[Python] Received D4 00: stop range hold raw={p.hex()}\n")
                sys.stdout.flush()
                _stop_range_hold("D4 00", always_zero_speed=True)
            else:
                sys.stdout.write(f"\n[Python] D4 toggle ignored: raw={p.hex()}\n")
                sys.stdout.flush()
        elif cmd_id == CmdID.FIXED_BOX and len(p) == 2:
            # 实机飞行跟踪没有固定自动化。E0 不进模式，1000→1200 油门斜坡也不会启动。
            sys.stdout.write(
                f"\n[Python] E0 ignored, fixed-box mode is not used raw={p.hex()}\n"
            )
            sys.stdout.flush()
        else:
            sys.stdout.write(f"\n[Python] Unhandled cmd=0x{cmd_id:02X} raw={p.hex()} len={len(p)}\n")
            sys.stdout.flush()


def process_position_control(telemetry: Optional['TelemetryData']) -> None:
    """
    Continuous position tracking state machine for horizontal and vertical control.
    """
    global HORIZ_TARGET_ACTIVE, TARGET_X, TARGET_Y, TARGET_HORIZ_ALT
    global VERT_TARGET_ACTIVE, TARGET_ALT, TARGET_VERT_X, TARGET_VERT_Y

    # 跟踪开着时只允许速度环写飞控，这里不能再 set_position。
    if PERSON_TRACK_ENABLED or FLIGHT_FOLLOW_ENABLED or RANGE_HOLD_ENABLED or not telemetry or PYTHON_LANDING:
        return

    cur_x, cur_y, cur_alt, cur_yaw = get_current_nav(telemetry)

    # 1. Relative horizontal position tracking (mutually exclusive with VERT)
    if HORIZ_TARGET_ACTIVE:
        dist = math.hypot(cur_x - TARGET_X, cur_y - TARGET_Y)
        hold_alt = TARGET_HORIZ_ALT if TARGET_HORIZ_ALT is not None else cur_alt
        if dist <= 10.0:
            # Destination reached: set reset_home=True to reset origin
            sys.stdout.write(f"\n[Python] Destination reached (dist={dist:.1f}cm <= 10cm). Resetting home origin.\n")
            sys.stdout.flush()
            apm.set_position(0, 0, hold_alt, cur_yaw, True)
            HORIZ_TARGET_ACTIVE = False

            # Lock position at (0, 0) with reset_home=False
            apm.set_position(0, 0, hold_alt, cur_yaw, False)
        else:
            # Keep moving towards target with reset_home=False and held altitude
            apm.set_position(TARGET_X, TARGET_Y, hold_alt, cur_yaw, False)

    # 2. Vertical position hold (only when HORIZ is idle)
    elif VERT_TARGET_ACTIVE and TARGET_ALT is not None:
        apm.set_position(TARGET_VERT_X, TARGET_VERT_Y, TARGET_ALT, cur_yaw, False)
        if abs(cur_alt - TARGET_ALT) <= 5.0:
            sys.stdout.write(
                f"\n[Python] Vertical altitude reached: cur={cur_alt}cm target={TARGET_ALT}cm "
                f"hold_xy=({TARGET_VERT_X},{TARGET_VERT_Y})\n"
            )
            sys.stdout.flush()
            VERT_TARGET_ACTIVE = False


def process_landing(telemetry: Optional['TelemetryData'], current_time: float) -> None:
    """
    Executes descent landing at 50 cm/s until touch-down is detected, then disarms.
    """
    global PYTHON_LANDING, PYTHON_LANDING_START_TIME

    if not PYTHON_LANDING:
        return

    # Descend at 50 cm/s (positive Z in body speed)
    apm.set_speed(0, 0, 50, 0.0)

    nav_relative_pos = telemetry.get('nav_relative_pos') if telemetry else None
    if nav_relative_pos and len(nav_relative_pos) >= 3 and nav_relative_pos[2] is not None:
        current_alt = nav_relative_pos[2]
        # Altitude <= 3cm indicates touch-down on ground
        if current_alt <= 3.0:
            if PYTHON_LANDING_START_TIME is None:
                PYTHON_LANDING_START_TIME = current_time
            elif current_time - PYTHON_LANDING_START_TIME >= 2.0:
                sys.stdout.write("\n[Python] Landing completed. Disarming flight controller.\n")
                sys.stdout.flush()
                apm.disarm()
                apm.set_speed(0, 0, 0, 0.0)
                PYTHON_LANDING = False
                PYTHON_LANDING_START_TIME = None
        else:
            PYTHON_LANDING_START_TIME = None


# ==============================================================================
# Main Exchange Entry Point
# ==============================================================================
def exchange(frame_bytes: bytes, width: int, height: int, pixfmt: int, telemetry: 'TelemetryData') -> int:
    global FRAME_COUNTER, LAST_CALL_TIME, SUM_LATENCY, SUM_INTERVAL, INITIAL_STATUS_CHECKED
    global LAST_FRAME_WIDTH, LAST_FRAME_HEIGHT
    start_time = time.perf_counter()
    if width and width > 0:
        LAST_FRAME_WIDTH = int(width)
    if height and height > 0:
        LAST_FRAME_HEIGHT = int(height)

    # 1. Calling interval & FPS measurement
    if LAST_CALL_TIME is not None:
        SUM_INTERVAL += (start_time - LAST_CALL_TIME)
    LAST_CALL_TIME = start_time

    # 2. Initial lock status check (once)
    if telemetry and not INITIAL_STATUS_CHECKED:
        is_disarmed = telemetry.get('sys_disarm_flag')
        status = "LOCKED (DISARMED)" if is_disarmed is True else "UNLOCKED (ARMED)"
        sys.stdout.write(f"\n[Python] Initial lock status: {status}\n")
        sys.stdout.flush()
        INITIAL_STATUS_CHECKED = True

    # 3. Optional live telemetry monitor
    if ENABLE_TELEMETRY_MONITOR and telemetry and FRAME_COUNTER % 5 == 0:
        debug_print_telemetry(telemetry)

    # 4. Ground-station gimbal (0xFB) then drive Servo Pin 4
    if telemetry and 'BroadcastRecv' in telemetry:
        broadcast_packets = telemetry['BroadcastRecv']
        if broadcast_packets:
            handle_broadcast_packets(broadcast_packets, telemetry)
    if not FIXED_MODE:
        servo_controller.g_servo_controller.update()

    # 5. 上锁沿（没有广播包的 RC 上锁）也要停跟踪
    _stop_track_if_just_locked(telemetry)

    if FIXED_MODE:
        process_fixed_throttle(start_time)
    else:
        # 6. Continuous position control (horizontal & vertical tracking)
        process_position_control(telemetry)
        # 7. Landing state machine
        process_landing(telemetry, start_time)

    # 7b. 人物跟踪。降落已经在上面把跟踪关掉，避免下降速度被这一拍盖掉。
    # 框和这里的 width/height 是同一套旋转后的画面。偏航和平移二选一。
    if PERSON_TRACK_ENABLED:
        process_person_track(width, telemetry)
    elif FLIGHT_FOLLOW_ENABLED or RANGE_HOLD_ENABLED:
        process_flight_follow(width, height, telemetry)

    # 8. Computer Vision & Frame Processing
    try:
        FRAME_COUNTER += 1

        target_x, target_y, target_area = -1, -1, 0.0
        if frame_bytes and width > 0 and height > 0:
            target_x, target_y, target_area = color_detector.detect_red_target(frame_bytes, width, height)

        latency = time.perf_counter() - start_time
        SUM_LATENCY += latency

        # Print statistics every 30 frames
        if FRAME_COUNTER % 30 == 0:
            avg_latency_ms = (SUM_LATENCY / 30.0) * 1000.0
            avg_fps = 30.0 / SUM_INTERVAL if SUM_INTERVAL > 0 else 0.0

            # Reset accumulators
            SUM_LATENCY = 0.0
            SUM_INTERVAL = 0.0

            if target_x != -1:
                sys.stdout.write(f"[Python] Red target: X={target_x}, Y={target_y}, Area={target_area:.1f} | Latency={avg_latency_ms:.2f}ms, FPS={avg_fps:.1f}\n")
                sys.stdout.flush()

    except Exception as e:
        import traceback
        sys.stdout.write(f"\n[Python Error] Exception in exchange: {e}\n")
        traceback.print_exc()

    return 0
