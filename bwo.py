"""
BWO the robot.
"""

import maestro
import servo_motor


class ServoMotorController(servo_motor.abstract.ServoMotorController):
    __slots__ = (
        'maestro_conn',
        'servo_channel'
    )

    def __init__(self, maestro_conn: maestro.Controller, servo_channel: int):
        self.maestro_conn = maestro_conn
        self.servo_channel = servo_channel

    def set_pwm_us(self, pwm_us):
        # Target is in 1/4 us units
        target = int(round(4 * pwm_us))
        self.maestro_conn.set_target(self.servo_channel, target)


def main():
    with maestro.Controller() as maestro_conn:
        left_motor = servo_motor.model.ParallaxHighSpeedContinuousServoMotor(
            ServoMotorController(maestro_conn, 5),
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
