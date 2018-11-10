"""
BWO the robot.
"""

from enum import Enum
from numbers import Real
from time import sleep, time

import cv2
import numpy as np

import gamesir
from bwo.io.head import Head
from bwo.io.motor import DriveMotorController
from maestro import Maestro


class BwoMode(Enum):
    REMOTE_CONTROL = 1
    CAT_SEEKER = 2


MODE = BwoMode.CAT_SEEKER


class Bwo:
    def __enter__(self):
        self.maestro = Maestro(is_micro=True)
        self.head = Head(self.maestro)
        self.drive_motors = DriveMotorController(self.maestro)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        with self.drive_motors, self.maestro:
            pass


class VideoCapture(cv2.VideoCapture):
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release()


def cat_seeker(bwo: Bwo, controller: gamesir.GameSirController):
    cat_detector = cv2.CascadeClassifier('haarcascade_frontalcatface_extended.xml')

    blackout_start_time = None
    with VideoCapture(0) as camera:
        while True:
            # Get frame
            ret, frame = camera.read()

            # Blackout frame?
            if np.max(frame) <= 20:
                now = time()
                if blackout_start_time is None:
                    blackout_start_time = now
                    bwo.drive_motors.stop()
                    continue
                elif (now - blackout_start_time) >= 10:
                    print('Blackout timeout reached!')
                    break
            else:
                blackout_start_time = None

            # Detect cats
            cat_rects = cat_detector.detectMultiScale(
                frame,
                scaleFactor=1.1,
                minNeighbors=1,
                minSize=(10, 10)
            )
            if not len(cat_rects):
                continue

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
            frame_height, frame_width, _ = frame.shape
            cat_offset = (
                2 * (cat_center[0] / frame_width) - 1,
                2 * (cat_center[1] / frame_height) - 1
            )
            del cat_center, frame_height, frame_width

            # Set the body velocity and head position
            bwo.drive_motors.set_body_velocity(0, 0.5, cat_offset[0] * 0.5)
            bwo.head.set_head_position(0)


def get_controller(timeout: Real = 60) -> gamesir.GameSirController:
    t0 = time()

    while True:
        try:
            return gamesir.get_controllers()[0]
        except IndexError:
            print('No controller found.')

        if (time() - t0) > timeout:
            raise TimeoutError('Took too long for controller to connect; exiting BWO.')

        sleep(1)


def remote_control(bwo: Bwo, controller: gamesir.GameSirController):
    drive_motor_events = {
        controller.EventCode.LEFT_JOYSTICK_X,
        controller.EventCode.LEFT_JOYSTICK_Y,
        controller.EventCode.RIGHT_JOYSTICK_X,
        controller.EventCode.B_BUTTON
    }

    head_events = {
        controller.EventCode.RIGHT_JOYSTICK_Y
    }

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
                print('Stopping BWO!')
                bwo.drive_motors.stop()
                break

            bwo.drive_motors.set_body_velocity(velocity.x, velocity.y, velocity.omega)

        elif event_code in head_events:
            if event_code == controller.EventCode.RIGHT_JOYSTICK_Y:
                bwo.head.set_head_position(-(event.value / 128 - 1))


def main():
    with Bwo as bwo, get_controller() as controller:
        if MODE == BwoMode.REMOTE_CONTROL:
            remote_control(bwo, controller)
        elif MODE == BwoMode.CAT_SEEKER:
            cat_seeker(bwo, controller)


if __name__ == '__main__':
    main()
