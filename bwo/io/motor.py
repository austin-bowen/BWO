from enum import Enum
from typing import Union

import numpy as np

from maestro import Maestro
from servo_motor.model import ParallaxHighSpeedContinuousServoMotor
from .. import settings

Number = Union[float, int]


class DriveMotor:
    def __init__(self, channel: int, dead_zone):
        self.channel = channel
        self.model = ParallaxHighSpeedContinuousServoMotor(settings.MOTOR_VOLTAGE, dead_zone)


class DriveMotorEnum(Enum):
    """Represents the three drive motors and their corresponding PWM channel."""

    LEFT = DriveMotor(settings.LEFT_MOTOR_CHANNEL, (1497, 1506))
    RIGHT = DriveMotor(settings.RIGHT_MOTOR_CHANNEL, (1499, 1508))
    BACK = DriveMotor(settings.BACK_MOTOR_CHANNEL, (1500, 1509))

    def get_motor(self) -> DriveMotor:
        return self.value


class DriveMotorController:
    SPEED = 0
    ACCELERATION = 0

    _BODY_TO_WHEEL_SPEED = np.array((
        # v_x, v_y, omega
        (-1 / np.sqrt(3), -1, -1),  # Left motor
        (-1 / np.sqrt(3), +1, -1),  # Right motor
        (+1.75 / np.sqrt(3), +0, -1)  # Back motor
    ))

    def __init__(self, maestro: Maestro):
        self.maestro = maestro

        for motor in DriveMotorEnum:
            channel = motor.get_motor().channel
            maestro.set_speed(channel, self.SPEED)
            maestro.set_acceleration(channel, self.ACCELERATION)

        self.stop()

    def set_motor_speed(self, motor: DriveMotorEnum, speed: Number):
        motor = motor.get_motor()
        self.maestro.set_target(
            motor.channel,
            motor.model.get_pwm_us_for_speed(speed)
        )

    def set_motor_speeds(self, left_speed: Number, right_speed: Number, back_speed: Number):
        self.set_motor_speed(DriveMotorEnum.LEFT, left_speed)
        self.set_motor_speed(DriveMotorEnum.RIGHT, right_speed)
        self.set_motor_speed(DriveMotorEnum.BACK, back_speed)

    def set_body_velocity(self, v_x: Number, v_y: Number, omega: Number):
        """
        :param v_x: X velocity [-1, 1].
        :param v_y: Y velocity [-1, 1].
        :param omega: Rotational velocity [-1, 1]. Positive value corresponds to clockwise rotation.
        """
        print('set body velocity:    v_x={} m/s    v_y={} m/s    omega={} deg/s'.format(v_x, v_y, omega))

        body_velocity = np.array((
            (v_x,),
            (v_y,),
            (omega,)
        ))

        # Shape: [left-motor, right-motor, back-motor]
        motor_speeds = np.matmul(self._BODY_TO_WHEEL_SPEED, body_velocity)[:, 0]
        print('Motor speeds [L, R, B]:', motor_speeds)

        max_motor_speed = np.abs(motor_speeds).max()
        if max_motor_speed > 1:
            print('WARNING: Commanded to move motor(s) faster than their max speed; scaling down.')
            motor_speeds = motor_speeds / max_motor_speed
            print('Scaled motor speeds [L, R, B]:', motor_speeds)

        self.set_motor_speeds(*motor_speeds)

    def stop(self):
        for motor in DriveMotorEnum:
            self.stop_motor(motor)

    def stop_motor(self, motor: DriveMotorEnum):
        self.set_motor_speed(motor, 0)

    @staticmethod
    def wheel_velocity_to_motor_rpm(v_wheel: Number):
        return (v_wheel * 60) / settings.WHEEL_CIRCUMFERENCE.meters
