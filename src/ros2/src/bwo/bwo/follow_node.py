import numpy as np
import random
import rclpy

from .neck_node import PAN_CENTER, PAN_MAX
from bwo_interfaces.msg import ObjectDetection, ObjectDetections
from geometry_msgs.msg import Twist
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from std_msgs.msg import Float32
from typing import List

random.seed()


class FollowNode(Node):
    LOGGER_LEVEL = LoggingSeverity.DEBUG

    MAX_ANGULAR_SPEED = 90  # deg/s
    MAX_LINEAR_SPEED = 30   # cm/s

    def __init__(self) -> None:
        super().__init__('follow')

        # Setup logger
        logger = self.get_logger()
        logger.set_level(self.LOGGER_LEVEL)

        # Setup search variables
        self._search_direction = None

        # Setup publishers
        self._drive_motors_set_velocity_publisher = self.create_publisher(
            Twist,
            '/drive_motors/set_velocity',
            1
        )

        # Setup subscribers
        self.create_subscription(
            ObjectDetections,
            '/object_detect',
            self._handle_object_detect,
            1
        )

        self.create_subscription(
            Float32,
            '/neck/pan/pos',
            self._handle_neck_pan_pos,
            1
        )

    def _handle_object_detect(self, detections: ObjectDetections) -> None:
        detections = detections.detections
        logger = self.get_logger()

        # Pick a target to follow
        target = self._pick_target(detections)
        logger.debug(f'Following: {target.class_name if target else "[None]"}')

        # Stop if no target to follow
        if target:
            self._follow_target(target)
        else:
            self._search()

    def _pick_target(self, detections: List[ObjectDetection]) -> ObjectDetection:
        # Return "closest" person
        detections = filter(lambda d: d.class_name == 'person', detections)
        detections = sorted(detections, key=lambda d: d.area)
        return detections[-1] if detections else None

    def _search(self) -> None:
        if not self._search_direction:
            self._search_direction = random.choice([-1, 1])

        angular_velocity = 0.33 * self._search_direction * self.MAX_ANGULAR_SPEED

        self._set_velocity(0, angular_velocity)

    def _follow_target(self, target: ObjectDetection) -> None:
        # Clear search stuff
        self._search_direction = None

        # Get position of target from center in range [-1, 1],
        # where -1 is bottom/left, and 1 is top/right
        center_dx = 2 * (target.center[0] / target.image_width) - 1
        center_dy = -2 * (target.center[1] / target.image_height) + 1

        # Drive towards the target if it takes up less than a certain
        # percentage of the total image width
        error = (target.image_width - (1 / 0.5) * target.width) / target.image_width
        if error > 0:
            linear_velocity = min(max(0, 100 * error), 40)
        elif error < -0.2:
            linear_velocity = min(max(-15, 50 * error), 0)
        else:
            linear_velocity = 0

        # Turn to center the target
        angular_velocity = -0.4 * center_dx * self.MAX_ANGULAR_SPEED

        self._set_velocity(linear_velocity, angular_velocity)

    def _set_velocity(self, linear: float, angular: float) -> None:
        velocity = Twist()
        velocity.linear.x = float(linear)
        velocity.angular.z = float(angular)

        self._drive_motors_set_velocity_publisher.publish(velocity)

    def _handle_neck_pan_pos(self, pos: Float32) -> None:
        # Range: [-1, 1], where 0 is center, -1 is PAN_MIN, and 1 is PAN_MAX
        d_center = (pos.data - PAN_CENTER) / (PAN_MAX - PAN_CENTER)
        print(f'd_center = {d_center}')


def main(args=None) -> None:
    rclpy.init(args=args)

    node = FollowNode()

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

