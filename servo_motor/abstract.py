def scale(x, point0, point1):
    x0, y0 = point0
    x1, y1 = point1
    return (x - x0) * (y1 - y0) / (x1 - x0) + y0


class ServoMotor:
    __slots__ = (
        'controller',
        'full_cw_pwm_us',
        'full_ccw_pwm_us'
    )

    def __init__(self, full_cw_pwm_us, full_ccw_pwm_us):
        self.full_cw_pwm_us = full_cw_pwm_us
        self.full_ccw_pwm_us = full_ccw_pwm_us


class LimitedServoMotor(ServoMotor):
    __slots__ = (
        'full_cw_deg',
        'full_ccw_deg',
    )

    def __init__(self, full_cw_deg, full_cw_pwm_us, full_ccw_deg, full_ccw_pwm_us):
        ServoMotor.__init__(self, full_cw_pwm_us, full_ccw_pwm_us)
        self.full_cw_deg = full_cw_deg
        self.full_ccw_deg = full_ccw_deg

    def set_angle(self, deg):
        pwm_us = scale(
            deg,
            (self.full_cw_deg, self.full_cw_pwm_us),
            (self.full_ccw_deg, self.full_ccw_pwm_us)
        )
        self.controller.set_pwm_us(pwm_us)


class ContinuousServoMotor(ServoMotor):
    __slots__ = (
        'center_pwm_us',
        'hard_max_rpm'
    )

    def __init__(self, hard_max_rpm, center_pwm_us, full_cw_pwm_us, full_ccw_pwm_us):
        ServoMotor.__init__(self, full_cw_pwm_us, full_ccw_pwm_us)
        self.hard_max_rpm = hard_max_rpm
        self.center_pwm_us = center_pwm_us

    def map_speed_to_pwm_us(self, speed):
        """
        Continuous rotation servos may have a non-linear speed response to the PWM signal.
        This method should convert the speed into PWM values such that the servo speed
        varies linearly as the speed varies linearly.

        By default, this method simply scales speeds from (0, 1] to (center_pwm_us, full_cw_pwm_us],
        and speeds from [-1, 0) to [full_ccw_pwm_us, center_pwm_us).

        :param speed: Range [-1, 1], where -1 corresponds to full CCW, and +1 corresponds to full CW.
        """
        if speed > 0:
            return scale(speed, (0, self.center_pwm_us), (1, self.full_cw_pwm_us))
        elif speed < 0:
            return scale(speed, (0, self.center_pwm_us), (-1, self.full_ccw_pwm_us))
        else:
            return self.center_pwm_us

    def get_pwm_us_for_rpm(self, rpm):
        speed = scale(rpm, (0, 0), (self.hard_max_rpm, 1))
        self.get_pwm_us_for_speed(speed)

    def get_pwm_us_for_speed(self, speed):
        self.controller.set_pwm_us(self.map_speed_to_pwm_us(speed))
