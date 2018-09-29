from enum import Enum
from typing import Union

import numpy as np

from maestro import Maestro
from servo_motor.model import ParallaxHighSpeedContinuousServoMotor
from .. import settings

Number = Union[float, int]


class DriveMotorController:
    _body_to_wheel_velocity = np.array((
        (1, 0, -settings.BODY_RADIUS.meters),
        (-0.5, np.sqrt(3), -settings.BODY_RADIUS.meters),
        (-0.5, -np.sqrt(3), -settings.BODY_RADIUS.meters)
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
        self.motor_model = ParallaxHighSpeedContinuousServoMotor(settings.MOTOR_VOLTAGE)
        # self.stop()

        self.max_wheel_speed_mps = settings.WHEEL_CIRCUMFERENCE.meters * self.motor_model.hard_max_rpm / 60
        print('Max wheel speed [m/s]:', round(self.max_wheel_speed_mps, 2))

    def set_motor_rpm(self, motor: Motor, rpm: Number):
        self.maestro.set_target(
            motor.get_channel(),
            self.motor_model.get_pwm_us_for_rpm(rpm)
        )

    def set_motor_speed(self, motor: Motor, speed: Number):
        self.maestro.set_target(
            motor.get_channel(),
            self.motor_model.get_pwm_us_for_speed(speed)
        )

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

        wheel_velocity = np.matmul(self._body_to_wheel_velocity, body_velocity)
        back_wheel_velocity, right_wheel_velocity, left_wheel_velocity = wheel_velocity[:, 0]
        print('left :', left_wheel_velocity)
        print('right:', right_wheel_velocity)
        print('back :', back_wheel_velocity)

        motor_to_wheel_velocity = {
            self.Motor.LEFT: left_wheel_velocity,
            self.Motor.RIGHT: right_wheel_velocity,
            self.Motor.BACK: back_wheel_velocity
        }

        if True:
            for motor, wheel_velocity in motor_to_wheel_velocity.items():
                self.set_motor_rpm(motor, self.wheel_velocity_to_motor_rpm(wheel_velocity))

    def stop(self):
        for motor in DriveMotorController.Motor:
            self.stop_motor(motor)

    def stop_motor(self, motor: Motor):
        self.maestro.stop_channel(motor.get_channel())

    @staticmethod
    def wheel_velocity_to_motor_rpm(v_wheel: Number):
        return (v_wheel * 60) / settings.WHEEL_CIRCUMFERENCE.meters
