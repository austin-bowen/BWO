from numbers import Real
from time import sleep, time

import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray

from maestro import Maestro
from servo_motor.abstract import Point, scale
from ..events import send_new_camera_image_event
from ..manager import ManagerThread


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
        with PiCamera() as camera, PiRGBArray(camera) as camera_output:
            camera.framerate = 10

            blackout_start_time = None
            t0 = time()
            for image in camera.capture_continuous(camera_output, format='bgr', use_video_port=True):
                if self._stop_event.is_set():
                    break

                # Send image from the camera in BGR order (rather than RGB) for OpenCV
                image = image.array
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
