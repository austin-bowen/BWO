from numbers import Real
from typing import NamedTuple


class Point(NamedTuple):
    x: Real
    y: Real


def scale(x: Real, point0: Point, point1: Point) -> Real:
    x0, y0 = point0
    x1, y1 = point1
    return (x - x0) * (y1 - y0) / (x1 - x0) + y0


class ServoMotor:
    __slots__ = (
        'full_cw_pwm_us',
        'full_ccw_pwm_us'
    )

    def __init__(self, full_cw_pwm_us: Real, full_ccw_pwm_us: Real):
        self.full_cw_pwm_us = full_cw_pwm_us
        self.full_ccw_pwm_us = full_ccw_pwm_us


class LimitedServoMotor(ServoMotor):
    __slots__ = (
        'full_cw_deg',
        'full_ccw_deg',
    )

    def __init__(self, full_cw_deg: Real, full_cw_pwm_us: Real, full_ccw_deg: Real, full_ccw_pwm_us: Real):
        ServoMotor.__init__(self, full_cw_pwm_us, full_ccw_pwm_us)
        self.full_cw_deg = full_cw_deg
        self.full_ccw_deg = full_ccw_deg

    def get_pwm_us_for_angle(self, deg: Real) -> Real:
        return scale(
            deg,
            Point(self.full_cw_deg, self.full_cw_pwm_us),
            Point(self.full_ccw_deg, self.full_ccw_pwm_us)
        )


class ContinuousServoMotor(ServoMotor):
    __slots__ = (
        'center_pwm_us',
        'hard_max_rpm'
    )

    def __init__(self, hard_max_rpm: Real, center_pwm_us: Real, full_cw_pwm_us: Real, full_ccw_pwm_us: Real):
        ServoMotor.__init__(self, full_cw_pwm_us, full_ccw_pwm_us)
        self.hard_max_rpm = hard_max_rpm
        self.center_pwm_us = center_pwm_us

    def map_speed_to_pwm_us(self, speed: Real) -> Real:
        """
        Continuous rotation servos may have a non-linear speed response to the PWM signal.
        This method should convert the speed into PWM values such that the servo speed
        varies linearly as the speed varies linearly.

        By default, this method simply scales speeds from (0, 1] to (center_pwm_us, full_cw_pwm_us],
        and speeds from [-1, 0) to [full_ccw_pwm_us, center_pwm_us).

        :param speed: Range [-1, 1], where -1 corresponds to full CCW, and +1 corresponds to full CW.
        """
        if speed > 0:
            return scale(speed, Point(0, self.center_pwm_us), Point(1, self.full_cw_pwm_us))
        elif speed < 0:
            return scale(speed, Point(0, self.center_pwm_us), Point(-1, self.full_ccw_pwm_us))
        else:
            return self.center_pwm_us

    def get_pwm_us_for_rpm(self, rpm: Real) -> Real:
        speed = scale(rpm, Point(0, 0), Point(self.hard_max_rpm, 1))
        return self.get_pwm_us_for_speed(speed)

    def get_pwm_us_for_speed(self, speed: Real) -> Real:
        return self.map_speed_to_pwm_us(speed)
