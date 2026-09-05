from machine import Pin, PWM
import time


# ==============================
# 云台舵机 PWM 初始化
# ==============================

# 水平舵机
# 信号线 -> OpenMV P7
servo_pan = PWM(
    Pin("P7"),
    freq=50,
    duty_ns=1500000
)

# 俯仰舵机
# 信号线 -> OpenMV P8
servo_tilt = PWM(
    Pin("P8"),
    freq=50,
    duty_ns=1500000
)


# ==============================
# 舵机参数
# ==============================

SERVO_MIN_US = 1000
SERVO_MAX_US = 2000

PAN_MIN_ANGLE = 0
PAN_MAX_ANGLE = 180

TILT_MIN_ANGLE = 0
TILT_MAX_ANGLE = 180


# ==============================
# PWM角度转换
# ==============================

def angle_to_pulse(angle):
    """
    角度转 PWM 脉宽
    0~180° -> 1000~2000us
    """

    pulse_us = SERVO_MIN_US + \
               (SERVO_MAX_US - SERVO_MIN_US) * angle // 180

    return pulse_us * 1000


# ==============================
# 水平舵机控制
# ==============================

def set_pan(angle):
    """
    设置水平云台角度

    angle: 0~180°
    """

    angle = max(PAN_MIN_ANGLE,
                min(PAN_MAX_ANGLE, angle))

    servo_pan.duty_ns(
        angle_to_pulse(angle)
    )


# ==============================
# 俯仰舵机控制
# ==============================

def set_tilt(angle):
    """
    设置俯仰云台角度

    angle: 0~180°
    """

    angle = max(TILT_MIN_ANGLE,
                min(TILT_MAX_ANGLE, angle))

    servo_tilt.duty_ns(
        angle_to_pulse(angle)
    )


# ==============================
# 两个舵机同时控制
# ==============================

def set_gimbal(pan_angle, tilt_angle):
    """
    同时设置云台两个舵机

    pan_angle:
        水平角度 0~180°

    tilt_angle:
        俯仰角度 0~180°
    """

    set_pan(pan_angle)
    set_tilt(tilt_angle)


# ==============================
# 云台回中
# ==============================

def gimbal_center():
    """
    云台回到中心位置
    """

    set_gimbal(90, 90)
