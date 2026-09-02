# pyright: reportMissingImports=false

from machine import Pin, PWM


# ==============================
# 云台舵机初始化
# ==============================

# 水平舵机 -> P7
servo_pan = PWM(
    Pin("P7"),
    freq=50,
    duty_ns=1500000
)

# 俯仰舵机 -> P8
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


def angle_to_pulse(angle):
    """0~180度转换成PWM脉宽"""

    angle = max(0, min(180, angle))

    pulse_us = (
        SERVO_MIN_US
        + (SERVO_MAX_US - SERVO_MIN_US) * angle // 180
    )

    return pulse_us * 1000


def set_pan(angle):
    """控制水平舵机"""
    servo_pan.duty_ns(angle_to_pulse(angle))


def set_tilt(angle):
    """控制俯仰舵机"""
    servo_tilt.duty_ns(angle_to_pulse(angle))


def set_gimbal(pan_angle, tilt_angle):
    """同时控制两个舵机"""
    set_pan(pan_angle)
    set_tilt(tilt_angle)


def gimbal_center():
    """云台回中"""
    set_gimbal(90, 90)
