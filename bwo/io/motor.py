from enum import Enum
from typing import Union

import numpy as np

from maestro import Maestro
from servo_motor.model import ParallaxHighSpeedContinuousServoMotor
from .. import settings

Number = Union[float, int]


class DriveMotorController:
    SPEED = 8
    ACCELERATION = 0

    _BODY_TO_WHEEL_VELOCITY = np.array((
        (1, 0, -settings.BODY_RADIUS.meters),
        (-0.5, np.sqrt(3) / 2, -settings.BODY_RADIUS.meters),
        (-0.5, -np.sqrt(3) / 2, -settings.BODY_RADIUS.meters)
    ))

    class Motor(Enum):
        """Represents the three drive motors and their corresponding PWM channel."""

        LEFT = settings.LEFT_MOTOR_CHANNEL
        RIGHT = settings.RIGHT_MOTOR_CHANNEL
        BACK = settings.BACK_MOTOR_CHANNEL

        def get_channel(self):
            return self.value

    def __init__(self, maestro: Maestro):
        self.maestro = maestro
        channels = (
            self.Motor.LEFT.get_channel(),
            self.Motor.RIGHT.get_channel(),
            self.Motor.BACK.get_channel()
        )
        for channel in channels:
            maestro.set_speed(channel, self.SPEED)
            maestro.set_acceleration(channel, self.ACCELERATION)

        self.motor_model = ParallaxHighSpeedContinuousServoMotor(settings.MOTOR_VOLTAGE)
        self.stop()

        self.max_wheel_speed_mps = settings.WHEEL_CIRCUMFERENCE.meters * self.motor_model.hard_max_rpm / 60
        print('Max wheel speed [m/s]:', round(self.max_wheel_speed_mps, 2))

    def set_motor_rpm(self, motor: Motor, rpm: Number):
        self.maestro.set_target(
            motor.get_channel(),
            self.motor_model.get_pwm_us_for_rpm(rpm)
        )

    def set_motor_rpms(self, left_rpm: Number, right_rpm: Number, back_rpm: Number):
        self.set_motor_rpm(self.Motor.LEFT, left_rpm)
        self.set_motor_rpm(self.Motor.RIGHT, right_rpm)
        self.set_motor_rpm(self.Motor.BACK, back_rpm)

    def set_motor_speed(self, motor: Motor, speed: Number):
        self.maestro.set_target(
            motor.get_channel(),
            self.motor_model.get_pwm_us_for_speed(speed)
        )

    def set_motor_speeds(self, left_speed: Number, right_speed: Number, back_speed: Number):
        self.set_motor_speed(self.Motor.LEFT, left_speed)
        self.set_motor_speed(self.Motor.RIGHT, right_speed)
        self.set_motor_speed(self.Motor.BACK, back_speed)

    def set_body_velocity(self, v_x: Number, v_y: Number, omega: Number):
        """

        :param v_x: X velocity [m/s].
        :param v_y: Y velocity [m/s].
        :param omega: Rotational velocity [deg/s]. Positive angle corresponds to clockwise rotation.
        :return:
        """
        print('set body velocity:    v_x={} m/s    v_y={} m/s    omega={} deg/s'.format(v_x, v_y, omega))

        body_velocity = np.array((
            (v_x,),
            (v_y,),
            (np.deg2rad(omega),)
        ))

        wheel_velocity = np.matmul(self._BODY_TO_WHEEL_VELOCITY, body_velocity)
        back_wheel_velocity, right_wheel_velocity, left_wheel_velocity = wheel_velocity[:, 0]
        print('left :', left_wheel_velocity)
        print('right:', right_wheel_velocity)
        print('back :', back_wheel_velocity)

        motor_rpms = np.array((
            self.wheel_velocity_to_motor_rpm(left_wheel_velocity),
            self.wheel_velocity_to_motor_rpm(right_wheel_velocity),
            self.wheel_velocity_to_motor_rpm(back_wheel_velocity)
        ))
        print(motor_rpms)

        max_motor_rpm = np.abs(motor_rpms).max()
        if max_motor_rpm > self.motor_model.hard_max_rpm:
            print('WARNING: Commanded to move motor(s) faster than their max RPM; scaling down.')
            motor_rpms = motor_rpms * self.motor_model.hard_max_rpm / max_motor_rpm
            print(motor_rpms)

        self.set_motor_rpms(*motor_rpms)

    def stop(self):
        for motor in DriveMotorController.Motor:
            self.stop_motor(motor)

    def stop_motor(self, motor: Motor):
        self.set_motor_rpm(motor, 0)

    @staticmethod
    def wheel_velocity_to_motor_rpm(v_wheel: Number):
        return (v_wheel * 60) / settings.WHEEL_CIRCUMFERENCE.meters
