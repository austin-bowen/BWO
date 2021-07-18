import random
import rclpy
import time

from bwo_interfaces.msg import BumperState
from geometry_msgs.msg import Twist
from rclpy.logging import LoggingSeverity
from rclpy.node import Node

FORWARD = 'FORWARD'
REVERSE = 'REVERSE'
TURN = 'TURN'

random.seed()


class BounceNode(Node):
    LOGGER_LEVEL = LoggingSeverity.DEBUG
    NODE_NAME = 'bounce'

    def __init__(self) -> None:
        super().__init__(self.NODE_NAME, namespace=self.NODE_NAME)

        # Initialize instance variables
        self._prev_velocity = None
        self._state = None

        # Setup logger
        logger = self.get_logger()
        logger.set_level(self.LOGGER_LEVEL)

        # Setup publishers
        self._drive_motors_set_velocity_publisher = self.create_publisher(
            Twist,
            '/drive_motors/set_velocity',
            1
        )

        self.create_timer(0.5, self._send_velocity_command)

        # Start listening for the bumpers_changed topic
        self.create_subscription(
            BumperState, '/drive_motors/bumpers_changed', self._handle_bumpers_changed, 1)

        self._forward()

    def destroy_node(self) -> bool:
        self.get_logger().info('Destroying...')
        return super().destroy_node()

    def _handle_bumpers_changed(self, bumpers: BumperState) -> None:
        if not bumpers.any:
            return

        self._reverse()
        self._turn(bumpers)
        self._forward()

    def _forward(self) -> None:
        self.get_logger().debug(FORWARD)
        self._state = FORWARD
        self._set_velocity(10, 0)

    def _reverse(self) -> None:
        self.get_logger().debug(REVERSE)
        self._state = REVERSE

        for _ in range(4):
            self._set_velocity(-10, 0)
            time.sleep(0.5)

    def _turn(self, bumpers: BumperState) -> None:
        self.get_logger().debug(TURN)
        self._state = TURN

        LEFT = 1
        RIGHT = -1

        if bumpers.left and not bumpers.right:
            direction = RIGHT
        elif not bumpers.left and bumpers.right:
            direction = LEFT
        else:
            direction = random.choice([LEFT, RIGHT])

        for _ in range(4):
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
        if velocity is None:
            velocity = self._prev_velocity
            print('prev:', velocity)

        if velocity is None:
            return

        self._drive_motors_set_velocity_publisher.publish(velocity)

        self._prev_velocity = velocity


def main(args=None) -> None:
    rclpy.init(args=args)

    node = BounceNode()

    print(f'[{node.get_name()}] Spinning...')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f'[{node.get_name()}] Kill signal received.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

