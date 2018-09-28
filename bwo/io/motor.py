from enum import Enum
from typing import Union

from maestro import Maestro
from servo_motor.model import ParallaxHighSpeedContinuousServoMotor

IntOrFloat = Union[int, float]


class DriveMotorController:
    LEFT_DRIVE_MOTOR_CHANNEL = 2
    RIGHT_DRIVE_MOTOR_CHANNEL = 3
    BACK_DRIVE_MOTOR_CHANNEL = 4

    class Motor(Enum):
        LEFT = 0
        RIGHT = 1
        BACK = 2

        def get_channel(self):
            if self is DriveMotorController.Motor.LEFT:
                return DriveMotorController.LEFT_DRIVE_MOTOR_CHANNEL
            elif self is DriveMotorController.Motor.RIGHT:
                return DriveMotorController.RIGHT_DRIVE_MOTOR_CHANNEL
            elif self is DriveMotorController.Motor.BACK:
                return DriveMotorController.BACK_DRIVE_MOTOR_CHANNEL

    def __init__(self, maestro: Maestro):
        self.maestro = maestro
        self.motor_model = ParallaxHighSpeedContinuousServoMotor(7.4)
        self.stop()

    def set_motor_rpm(self, motor: Motor, rpm: IntOrFloat):
        self.maestro.set_target(
            motor.get_channel(),
            self.motor_model.get_pwm_us_for_rpm(rpm)
        )

    def set_motor_speed(self, motor: Motor, speed: IntOrFloat):
        self.maestro.set_target(
            motor.get_channel(),
            self.motor_model.get_pwm_us_for_speed(speed)
        )

    def set_velocity(self, x: IntOrFloat, y: IntOrFloat, theta: IntOrFloat):
        print('set velocity:  x={}  y={}  theta={}'.format(x, y, theta))

    def stop(self):
        for motor in DriveMotorController.Motor:
            self.stop_motor(motor)

    def stop_motor(self, motor: Motor):
        self.maestro.stop_channel(motor.get_channel())
