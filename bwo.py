"""
BWO the robot.
"""

from maestro import Maestro

import servo_motor


class ServoMotorController(servo_motor.abstract.ServoMotorController):
    __slots__ = (
        'maestro',
        'servo_channel'
    )

    def __init__(self, maestro: Maestro, servo_channel: int):
        self.maestro = maestro
        self.servo_channel = servo_channel

    def set_pwm_us(self, pwm_us):
        self.maestro.set_target(self.servo_channel, pwm_us)


def main():
    with Maestro(is_micro=True) as maestro:
        left_motor = servo_motor.model.ParallaxHighSpeedContinuousServoMotor(
            ServoMotorController(maestro, 5),
            5
        )

        print('Max RPM: {}\n'.format(left_motor.hard_max_rpm))

        while True:
            rpm = input('Enter RPM: ')
            try:
                rpm = int(rpm)
            except ValueError:
                break

            left_motor.set_rpm(rpm)

        left_motor.stop()


if __name__ == '__main__':
    main()
