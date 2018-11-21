from copy import copy
from enum import Enum
from numbers import Real
from typing import NamedTuple

import numpy as np

from maestro import Maestro
from servo_motor.model import ParallaxHighSpeedContinuousServoMotor
from time import time
from .. import settings
from ..manager import ManagerThread


class BodyVelocity:
    x: Real
    y: Real
    omega: Real

    def __init__(self, x: Real, y: Real, omega: Real):
        self.x = x
        self.y = y
        self.omega = omega


class BodyVelocityTimestamp(NamedTuple):
    velocity: BodyVelocity
    time: float


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


class DriveMotorController(ManagerThread):
    SPEED = 0
    ACCELERATION = 0

    _BODY_TO_WHEEL_SPEED = np.array((
        # v_x, v_y, omega
        (-1 / np.sqrt(3), -1, -1),  # Left motor
        (-1 / np.sqrt(3), +1, -1),  # Right motor
        (+1.75 / np.sqrt(3), +0, -1)  # Back motor
    ))

    LOOP_PERIOD_S: Real = 1 / 15
    _FILTER_FREQ_HZ: Real = 0.75

    def __init__(self, maestro: Maestro):
        assert self.LOOP_PERIOD_S <= 1 / self._FILTER_FREQ_HZ

        ManagerThread.__init__(self, name='Drive Motor Controller')

        # Setup the Maestro
        self.maestro = maestro
        for motor in DriveMotorEnum:
            channel = motor.get_motor().channel
            maestro.set_speed(channel, self.SPEED)
            maestro.set_acceleration(channel, self.ACCELERATION)

        self._target_body_velocity: BodyVelocity = BodyVelocity(0, 0, 0)
        self._previous_body_velocity_timestamp: BodyVelocityTimestamp = None
        self._set_body_velocity_immediately = True
        self.stop_motors()

    def _log(self, message: str):
        print('[{}] {}'.format(self.name, message))

    def get_target_body_velocity(self) -> BodyVelocity:
        """Returns a copy of the target body velocity."""
        return copy(self._target_body_velocity)

    def set_motor_speed(self, motor: DriveMotorEnum, speed: Real):
        motor = motor.get_motor()
        self.maestro.set_target(
            motor.channel,
            motor.model.get_pwm_us_for_speed(speed)
        )

    def set_motor_speeds(self, left_speed: Real, right_speed: Real, back_speed: Real):
        self.set_motor_speed(DriveMotorEnum.LEFT, left_speed)
        self.set_motor_speed(DriveMotorEnum.RIGHT, right_speed)
        self.set_motor_speed(DriveMotorEnum.BACK, back_speed)

    def set_body_velocity(self, v_x: Real, v_y: Real, omega: Real, immediate: bool = False):
        """
        :param v_x: X velocity [-1, 1].
        :param v_y: Y velocity [-1, 1].
        :param omega: Rotational velocity [-1, 1]. Positive value corresponds to clockwise rotation.
        :param immediate: Set the body velocity as soon as possible, without low-pass filtering.
        """
        self._target_body_velocity = BodyVelocity(*np.clip([v_x, v_y, omega], -1, 1))

        if immediate:
            self._set_body_velocity_immediately = True

    def stop_motors(self, immediate=True):
        self.set_body_velocity(0, 0, 0, immediate=immediate)

    def main(self):
        target_body_velocity = self._target_body_velocity
        target_body_velocity = np.array(((target_body_velocity.x, target_body_velocity.y, target_body_velocity.omega),))

        if self._set_body_velocity_immediately:
            self._set_body_velocity_immediately = False
            body_velocity = target_body_velocity
        else:
            previous_body_velocity, previous_time = self._previous_body_velocity_timestamp
            previous_body_velocity = np.array((
                previous_body_velocity.x,
                previous_body_velocity.y,
                previous_body_velocity.omega
            ))

            # Apply low-pass filter to the target body velocity
            dt = time() - previous_time
            a0 = 2 * np.pi * dt * self._FILTER_FREQ_HZ
            alpha = a0 / (a0 + 1)
            body_velocity = alpha * target_body_velocity + (1 - alpha) * previous_body_velocity

        # Shape: [left-motor, right-motor, back-motor]
        motor_speeds = np.matmul(self._BODY_TO_WHEEL_SPEED, body_velocity.T)[:, 0]

        max_motor_speed = np.abs(motor_speeds).max()
        if max_motor_speed > 1:
            print('WARNING: Commanded to move motor(s) faster than their max speed; scaling down.')
            motor_speeds = motor_speeds / max_motor_speed

        self.set_motor_speeds(*motor_speeds)
        self._previous_body_velocity_timestamp = BodyVelocityTimestamp(BodyVelocity(*body_velocity[0]), time())
