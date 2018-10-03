"""
BWO the robot.
"""

import gamesir

from bwo.io.motor import DriveMotorController
from maestro import Maestro


def main(controller: gamesir.GameSirController, maestro: Maestro):
    drive_motors = DriveMotorController(maestro)

    class Velocity:
        def __init__(self, x, y, omega):
            self.x = x
            self.y = y
            self.omega = omega

    velocity = Velocity(0, 0, 0)

    for event in controller.read_loop():
        if event.type not in controller.EVENT_TYPES:
            continue

        print(event)

        event_code = gamesir.GameSirController.EventCode(event.code)

        if event_code == controller.EventCode.LEFT_JOYSTICK_X:
            velocity.x = event.value / 128 - 1
        elif event_code == controller.EventCode.LEFT_JOYSTICK_Y:
            velocity.y = -(event.value / 128 - 1)
        elif event_code == controller.EventCode.RIGHT_JOYSTICK_X:
            velocity.omega = event.value / 128 - 1
        elif event_code == controller.EventCode.B_BUTTON and event.value == 1:
            drive_motors.stop()
            break
        else:
            continue

        drive_motors.set_body_velocity(velocity.x, velocity.y, velocity.omega)


if __name__ == '__main__':
    with gamesir.get_controllers()[0] as _controller, Maestro(is_micro=True) as _maestro:
        main(_controller, _maestro)
