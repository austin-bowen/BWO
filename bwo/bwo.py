from enum import Enum
from threading import Event
from typing import Optional

from gamesir import GameSirController as Controller
from maestro import Maestro
from . import events
from .io.controller import ControllerManager
from .io.head import Head
from .io.motor import DriveMotorController


class BwoMode(Enum):
    REMOTE_CONTROL = 0
    CAT_SEEKER = 1


class Bwo:
    DRIVE_MOTOR_CONTROLLER_EVENT_CODES = {
        Controller.EventCode.LEFT_JOYSTICK_X,
        Controller.EventCode.LEFT_JOYSTICK_Y,
        Controller.EventCode.RIGHT_JOYSTICK_X,
        Controller.EventCode.B_BUTTON
    }

    HEAD_CONTROLLER_EVENT_CODES = {
        Controller.EventCode.RIGHT_JOYSTICK_Y
    }

    def __init__(self, mode: BwoMode):
        self.mode = mode

        self.controller_manager = ControllerManager()
        self.maestro = Maestro(is_micro=True)

        self.head = None
        self.drive_motors: DriveMotorController = None

        self._shutdown_event = Event()

        events.receive_controller_events(self._controller_event_handler)
        events.receive_shutdown_command(self._shutdown_handler)

    def __enter__(self):
        self.controller_manager.__enter__()
        self.maestro.__enter__()

        self.head = Head(self.maestro)
        self.drive_motors = DriveMotorController(self.maestro)
        self.drive_motors.__enter__()

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.drive_motors.__exit__(exc_type, exc_val, exc_tb)
        self.maestro.__exit__(exc_type, exc_val, exc_tb)
        self.controller_manager.__exit__(exc_type, exc_val, exc_tb)

    def _controller_event_handler(self, event):
        event_code = Controller.EventCode(event.code)

        # Shutdown button B pressed?
        if event_code == Controller.EventCode.B_BUTTON and event.value == 1:
            self.drive_motors.stop_motors()
            events.send_shutdown_command()

        # Drive motor control event?
        elif event_code in self.DRIVE_MOTOR_CONTROLLER_EVENT_CODES:
            velocity = self.drive_motors.get_target_body_velocity()

            if event_code == Controller.EventCode.LEFT_JOYSTICK_X:
                velocity.x = event.value / 128 - 1
            elif event_code == Controller.EventCode.LEFT_JOYSTICK_Y:
                velocity.y = -(event.value / 128 - 1)
            elif event_code == Controller.EventCode.RIGHT_JOYSTICK_X:
                velocity.omega = event.value / 128 - 1
            else:
                raise RuntimeError(f'Controller event code {event_code} is NOT a drive motor controller event code.')

            self.drive_motors.set_body_velocity(velocity.x, velocity.y, velocity.omega)

        # Head control event?
        elif event_code in self.HEAD_CONTROLLER_EVENT_CODES:
            if event_code == Controller.EventCode.RIGHT_JOYSTICK_Y:
                self.head.set_head_position(-(event.value / 128 - 1))
            else:
                raise RuntimeError(f'Controller event code {event_code} is NOT a head controller event code.')

    def _shutdown_handler(self):
        print('BWO received shutdown event!')
        self._shutdown_event.set()

    def wait_for_shutdown(self, timeout: Optional[float] = None) -> bool:
        return self._shutdown_event.wait(timeout=timeout)
