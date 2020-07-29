from enum import Enum
from threading import Event
from typing import Optional

import cv2

from gamesir import GameSirController as Controller
from maestro import MicroMaestro
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
        self._shutdown_event = Event()

        self.mode = mode

        self.controller_manager = ControllerManager()
        self.maestro = MicroMaestro()
        self.head = Head(self.maestro)
        self.drive_motors = DriveMotorController(self.maestro)

        # Resources are entered in order, and exited in reverse order
        self.resources = (
            self.controller_manager,
            self.maestro,
            self.head,
            self.drive_motors
        )

        if self.mode == BwoMode.CAT_SEEKER:
            self.cat_detector = cv2.CascadeClassifier('haarcascade_frontalcatface.xml')
            # self.cat_detector = cv2.CascadeClassifier('haarcascade_frontalcatface_extended.xml')
            events.receive_new_camera_image(self._new_camera_image_handler)

        events.receive_controller_events(self._controller_event_handler)
        events.receive_shutdown_command(self._shutdown_handler)

    def __enter__(self):
        for resource in self.resources:
            resource.__enter__()

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        result = False
        for resource in reversed(self.resources):
            if resource.__exit__(exc_type, exc_val, exc_tb):
                result = True

        return result

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

    def _new_camera_image_handler(self, image):
        cat_detector = self.cat_detector

        # Detect cats
        cat_rects = cat_detector.detectMultiScale(
            image,
            scaleFactor=1.1,
            minNeighbors=1,
            minSize=(10, 10)
        )
        if not len(cat_rects):
            self.drive_motors.stop_motors()
            return

        # Pick the largest detected cat
        cat_rect = max(cat_rects, key=lambda rect: rect[2] * rect[3])
        del cat_rects

        # Get the cat's center point
        cat_center = (
            cat_rect[0] + (cat_rect[2] / 2),
            cat_rect[1] + (cat_rect[3] / 2)
        )
        del cat_rect

        # Get the cat's offset from center
        frame_height, frame_width, _ = image.shape
        cat_offset = (
            2 * (cat_center[0] / frame_width) - 1,
            2 * (cat_center[1] / frame_height) - 1
        )
        del cat_center, frame_height, frame_width

        # Set the body velocity and head position
        self.drive_motors.set_body_velocity(0, 0, cat_offset[0] * 0.25)
        self.head.set_head_position(0)

    def _shutdown_handler(self):
        print('BWO received shutdown event!')
        self._shutdown_event.set()

    def wait_for_shutdown(self, timeout: Optional[float] = None) -> bool:
        return self._shutdown_event.wait(timeout=timeout)
