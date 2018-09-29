"""
BWO the robot.
"""

from maestro import Maestro
from bwo.io.motor import DriveMotorController


def main():
    with Maestro(is_micro=True) as maestro:
        drive_motors = DriveMotorController(maestro)

        print('Max RPM: {}\n'.format(drive_motors.motor_model.hard_max_rpm))

        while True:
            try:
                v_x, v_y, omega = input('v_x, v_y, omega = ').split(',')
                v_x = float(v_x.strip())
                v_y = float(v_y.strip())
                omega = float(omega.strip())
            except:
                break

            drive_motors.set_body_velocity(v_x, v_y, omega)

        drive_motors.stop()


if __name__ == '__main__':
    main()
