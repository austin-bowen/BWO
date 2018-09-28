from .abstract import *


class ParallaxHighSpeedContinuousServoMotor(ContinuousServoMotor):
    def __init__(self, voltage: float):
        hard_max_rpm = scale(voltage, (6, 150), (7.4, 180))
        ContinuousServoMotor.__init__(
            self,
            hard_max_rpm,
            center_pwm_us=1500,
            full_cw_pwm_us=1300,
            full_ccw_pwm_us=1700
        )

    def map_speed_to_pwm_us(self, speed):
        """
        The map of PWM signal to speed for this servo looks like a sigmoid function.  See page 3 of the documentation:
        https://www.parallax.com/sites/default/files/downloads/900-00025-High-Speed-CR-Servo-Guide-v1.1.pdf
        """
        # TODO: Account for dead zone?
        full_cw_point = (1, self.full_cw_pwm_us)
        near_full_cw_point = (0.88, 1420)
        center_point = (0, self.center_pwm_us)
        near_full_ccw_point = (-0.88, 1580)
        full_ccw_point = (-1, self.full_ccw_pwm_us)

        if speed >= near_full_cw_point[0]:
            return scale(speed, near_full_cw_point, full_cw_point)
        elif speed > 0:
            return scale(speed, center_point, near_full_cw_point)
        elif speed == 0:
            return self.center_pwm_us
        elif speed >= near_full_ccw_point[0]:
            return scale(speed, center_point, near_full_ccw_point)
        else:
            return scale(speed, near_full_ccw_point, full_ccw_point)
