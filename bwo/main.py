"""
BWO the robot.
"""

import gamesir

from bwo.io.head import Head
from bwo.io.motor import DriveMotorController
from maestro import Maestro


def main(controller: gamesir.GameSirController, maestro: Maestro):
    drive_motor_events = {
        controller.EventCode.LEFT_JOYSTICK_X,
        controller.EventCode.LEFT_JOYSTICK_Y,
        controller.EventCode.RIGHT_JOYSTICK_X,
        controller.EventCode.B_BUTTON
    }

    head_events = {
        controller.EventCode.RIGHT_JOYSTICK_Y
    }

    drive_motors = DriveMotorController(maestro)
    head = Head(maestro)

    class Velocity:
        def __init__(self, x, y, omega):
            self.x = x
            self.y = y
            self.omega = omega

    velocity = Velocity(0, 0, 0)

    for event in controller.read_loop():
        if event.type not in controller.EVENT_TYPES:
            continue

        event_code = controller.EventCode(event.code)

        if event_code in drive_motor_events:
            if event_code == controller.EventCode.LEFT_JOYSTICK_X:
                velocity.x = event.value / 128 - 1
            elif event_code == controller.EventCode.LEFT_JOYSTICK_Y:
                velocity.y = -(event.value / 128 - 1)
            elif event_code == controller.EventCode.RIGHT_JOYSTICK_X:
                velocity.omega = event.value / 128 - 1
            elif event_code == controller.EventCode.B_BUTTON and event.value == 1:
                drive_motors.stop()
                break

            drive_motors.set_body_velocity(velocity.x, velocity.y, velocity.omega)

        elif event_code in head_events:
            if event_code == controller.EventCode.RIGHT_JOYSTICK_Y:
                head.set_head_position(-(event.value / 128 - 1))


if __name__ == '__main__':
    with gamesir.get_controllers()[0] as _controller, Maestro(is_micro=True) as _maestro:
        main(_controller, _maestro)
