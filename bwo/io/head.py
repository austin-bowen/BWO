from numbers import Real

import numpy as np

from maestro import Maestro
from servo_motor.abstract import Point, scale


class Head:
    NECK_CHANNEL = 0

    def __init__(self, maestro: Maestro):
        self.maestro = maestro
        self.maestro.set_acceleration(self.NECK_CHANNEL, 15)
        self.maestro.set_speed(self.NECK_CHANNEL, 30)

        self.set_head_position(0)

    def set_head_position(self, position: Real):
        position = scale(position, Point(-1, 1000), Point(1, 2000))
        position = np.clip(position, 1200, 2000)
        self.maestro.set_target(self.NECK_CHANNEL, position)
