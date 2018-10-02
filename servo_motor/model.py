from typing import Tuple

from .abstract import *


class ParallaxHighSpeedContinuousServoMotor(ContinuousServoMotor):
    def __init__(self, voltage: Number, dead_zone: Tuple[Number, Number] = None):
        if dead_zone is None:
            dead_zone = (1500, 1500)
        self.dead_zone = dead_zone

        left_dead_zone, right_dead_zone = dead_zone
        assert right_dead_zone >= left_dead_zone
        center_pwm_us = left_dead_zone + (right_dead_zone - left_dead_zone) / 2

        hard_max_rpm = scale(voltage, (6, 150), (7.4, 180))
        ContinuousServoMotor.__init__(
            self,
            hard_max_rpm,
            center_pwm_us=center_pwm_us,
            full_cw_pwm_us=1300,
            full_ccw_pwm_us=1700
        )

    def map_speed_to_pwm_us(self, speed: Number) -> Number:
        """
        The map of PWM signal to speed for this servo looks like a sigmoid function.  See page 3 of the documentation:
        https://www.parallax.com/sites/default/files/downloads/900-00025-High-Speed-CR-Servo-Guide-v1.1.pdf
        """
        full_cw_point = (1, self.full_cw_pwm_us)
        near_full_cw_point = (0.88, 1420)
        high_dead_point = (0, self.dead_zone[1])
        low_dead_point = (0, self.dead_zone[0])
        near_full_ccw_point = (-0.88, 1580)
        full_ccw_point = (-1, self.full_ccw_pwm_us)

        if speed >= 1:
            return self.full_cw_pwm_us
        elif speed >= near_full_cw_point[0]:
            return scale(speed, near_full_cw_point, full_cw_point)
        elif speed > 0:
            return scale(speed, high_dead_point, near_full_cw_point)
        elif speed == 0:
            return self.center_pwm_us
        elif speed >= near_full_ccw_point[0]:
            return scale(speed, low_dead_point, near_full_ccw_point)
        elif speed > -1:
            return scale(speed, near_full_ccw_point, full_ccw_point)
        else:
            return self.full_ccw_pwm_us
