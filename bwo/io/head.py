from numbers import Real
from time import sleep, time

import cv2
import numpy as np

from maestro import Maestro
from servo_motor.abstract import Point, scale
from ..events import send_new_camera_image_event
from ..manager import ManagerThread


class VideoCapture(cv2.VideoCapture):
    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.release()


class Head(ManagerThread):
    CAMERA_BLACKOUT_THRESHOLD = 20
    CAMERA_BLACKOUT_TIMEOUT_S = 10

    LOOP_PERIOD_S = 1

    NECK_CHANNEL = 0
    NECK_SPEED = 50
    NECK_ACCELERATION = 0

    def __init__(self, maestro: Maestro):
        ManagerThread.__init__(self, name='Head')

        self.maestro = maestro
        self.maestro.set_speed(self.NECK_CHANNEL, self.NECK_SPEED)
        self.maestro.set_acceleration(self.NECK_CHANNEL, self.NECK_ACCELERATION)
        self.set_head_position(0)

    def main(self):
        with VideoCapture(0) as camera:
            blackout_start_time = None
            prev_logged_error = False
            t0 = time()
            while not self._stop_event.is_set():
                # Capture image from the camera
                return_code, image = camera.read()
                if not image:
                    if not prev_logged_error:
                        self.log_error(f'Failed to read camera image; return code {return_code}.')
                        prev_logged_error = True
                    continue
                prev_logged_error = False

                # Send image
                send_new_camera_image_event(image)

                # Blackout frame?
                if np.max(image) <= self.CAMERA_BLACKOUT_THRESHOLD:
                    now = time()
                    if blackout_start_time is None:
                        blackout_start_time = now
                    elif (now - blackout_start_time) >= self.CAMERA_BLACKOUT_TIMEOUT_S:
                        print('Blackout timeout reached!')
                        sleep(10)
                else:
                    blackout_start_time = None

                # Print FPS
                t1 = time()
                print(f'{1 / (t1 - t0)} FPS')
                t0 = t1

    def set_head_position(self, position: Real):
        position = scale(position, Point(-1, 1000), Point(1, 2000))
        position = np.clip(position, 1200, 2000)
        self.maestro.set_target(self.NECK_CHANNEL, position)
