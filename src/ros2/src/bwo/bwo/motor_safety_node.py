import random
import rclpy
import time

from bwo_interfaces.msg import BumperState
from geometry_msgs.msg import Twist
from rclpy.logging import LoggingSeverity
from .drive_motor_node import DriveMotorsControllerMixin
from .node import Node

random.seed()


class MotorSafetyNode(Node, DriveMotorsControllerMixin):
    """
    Passes motor commands through to the appropriate controller,
    unless an unsafe state is detected, in which case this node
    will ignore higher-level motor commands, and will take actions
    to get back to a safe state. e.g. if one of the bumpers is
    triggered, this will reverse until safe to continue.
    """

    LOGGER_LEVEL = LoggingSeverity.DEBUG
    NODE_NAME = 'motor_safety'

    def __init__(self) -> None:
        super().__init__(self.NODE_NAME, logger_level=self.LOGGER_LEVEL)

        # Setup listeners
        self.create_subscription(
            BumperState,
            '/bumpers_changed',
            self._handle_bumpers_changed,
            1
        )
        self.create_subscription(
            Twist,
            '/drive_motors/set_velocity/safe',
            self._handle_set_velocity,
            1
        )

    def _handle_bumpers_changed(self, bumpers: BumperState) -> None:
        if not self.is_enabled or not bumpers.any:
            return

        self._reverse()
        self._turn(bumpers)
        self._stop()

    def _handle_set_velocity(self, velocity: Twist) -> None:
        if self.is_enabled:
            self.set_drive_motors_velocity(velocity=velocity)

    def _reverse(self) -> None:
        for _ in range(1):
            self.set_drive_motors_velocity(-10, 0)
            time.sleep(0.5)

    def _turn(self, bumpers: BumperState) -> None:
        LEFT = 1
        RIGHT = -1

        if bumpers.left and not bumpers.right:
            direction = RIGHT
        elif not bumpers.left and bumpers.right:
            direction = LEFT
        else:
            direction = random.choice([LEFT, RIGHT])

        for _ in range(1):
            self.set_drive_motors_velocity(0, direction * 45)
            time.sleep(0.5)

    def _stop(self) -> None:
        self.set_drive_motors_velocity(0, 0)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = MotorSafetyNode()

    node.logger.info('Spinning...')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.logger.info('Kill signal received.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

