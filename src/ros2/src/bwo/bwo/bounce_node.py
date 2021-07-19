import random
import rclpy
import time

from bwo_interfaces.msg import BumperState
from geometry_msgs.msg import Twist
from rclpy.logging import LoggingSeverity
#from rclpy.node import Node
from .node import Node

FORWARD = 'FORWARD'
REVERSE = 'REVERSE'
TURN = 'TURN'

random.seed()


class BounceNode(Node):
    LOGGER_LEVEL = LoggingSeverity.DEBUG
    NODE_NAME = 'bounce'

    def __init__(self) -> None:
        super().__init__(self.NODE_NAME, logger_level=self.LOGGER_LEVEL)

        # Initialize instance variables
        self._prev_velocity = None
        self._state = None

        # Setup publishers
        self._drive_motors_set_velocity_publisher = self.create_publisher(
            Twist,
            '/drive_motors/set_velocity',
            1
        )

        # Setup timers
        self.create_timer(0.5, self._send_velocity_command)

        # Start listening for the bumpers_changed topic
        self.create_subscription(
            BumperState, '/drive_motors/bumpers_changed', self._handle_bumpers_changed, 1)

        self._forward()

    def _handle_bumpers_changed(self, bumpers: BumperState) -> None:
        if not self.is_enabled or not bumpers.any:
            return

        self._reverse()
        self._turn(bumpers)
        self._forward()

    def _forward(self) -> None:
        self.logger.debug(FORWARD)
        self._state = FORWARD
        self._set_velocity(10, 0)

    def _reverse(self) -> None:
        self.logger.debug(REVERSE)
        self._state = REVERSE

        for _ in range(2):
            self._set_velocity(-10, 0)
            time.sleep(0.5)

    def _turn(self, bumpers: BumperState) -> None:
        self.logger.debug(TURN)
        self._state = TURN

        LEFT = 1
        RIGHT = -1

        if bumpers.left and not bumpers.right:
            direction = RIGHT
        elif not bumpers.left and bumpers.right:
            direction = LEFT
        else:
            direction = random.choice([LEFT, RIGHT])

        for _ in range(random.randrange(2, 8 + 1)):
            self._set_velocity(0, direction * 45)
            time.sleep(0.5)

    def _set_velocity(self, linear: float, angular: float) -> None:
        velocity = Twist()

        # Determine linear velocity
        linear = float(linear)
        velocity.linear.x = linear

        # Determine angular velocity
        angular = float(angular)
        velocity.angular.z = angular

        # Set the velocity
        self._send_velocity_command(velocity)

    def _send_velocity_command(self, velocity=None) -> None:
        if not self.is_enabled:
            return

        if velocity is None:
            velocity = self._prev_velocity

        if velocity is None:
            return

        self._drive_motors_set_velocity_publisher.publish(velocity)

        self._prev_velocity = velocity


def main(args=None) -> None:
    rclpy.init(args=args)

    node = BounceNode()

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
