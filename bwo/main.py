"""
BWO the robot.
"""

from maestro import Maestro
from .io.motor import DriveMotorController


def main():
    motor = DriveMotorController.Motor.LEFT

    with Maestro(is_micro=True) as maestro:
        drive_motors = DriveMotorController(maestro)

        print('Max RPM: {}\n'.format(drive_motors.motor_model.hard_max_rpm))

        while True:
            rpm = input('Enter RPM: ')
            try:
                rpm = int(rpm)
            except ValueError:
                break

            drive_motors.set_motor_rpm(motor, rpm)

        drive_motors.stop()


if __name__ == '__main__':
    main()
