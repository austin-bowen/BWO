import numpy as np
import rclpy

from bwo_interfaces.msg import ObjectDetection, ObjectDetections
from geometry_msgs.msg import Twist
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from typing import List


class FollowNode(Node):
    LOGGER_LEVEL = LoggingSeverity.DEBUG

    MAX_ANGULAR_SPEED = 90  # deg/s
    MAX_LINEAR_SPEED = 30   # cm/s

    def __init__(self) -> None:
        super().__init__('follow')

        # Setup logger
        logger = self.get_logger()
        logger.set_level(self.LOGGER_LEVEL)

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

    def _handle_object_detect(self, detections: ObjectDetections) -> None:
        detections = detections.detections
        logger = self.get_logger()

        # Pick an object to follow
        obj = self._pick_object(detections)
        logger.debug(f'Following: {obj.class_name if obj else "[None]"}')

        # Stop if no object to follow
        if not obj:
            self._set_velocity(0, 0)
            return

        # Get position from center in range [-1, 1],
        # where -1 is bottom/left, and 1 is top/right
        center_dx = 2 * (obj.center[0] / obj.image_width) - 1
        center_dy = -2 * (obj.center[1] / obj.image_height) + 1

        self._set_velocity(0, -center_dx * self.MAX_ANGULAR_SPEED)

    def _pick_object(self, detections: List[ObjectDetection]) -> ObjectDetection:
        # Return "closest" person
        detections = filter(lambda d: d.class_name == 'person', detections)
        detections = sorted(detections, key=lambda d: d.area)
        return detections[-1] if detections else None

    def _set_velocity(self, linear: float, angular: float) -> None:
        velocity = Twist()
        velocity.linear.x = float(linear)
        velocity.angular.z = float(angular)

        self._drive_motors_set_velocity_publisher.publish(velocity)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = FollowNode()

    print(f'Spinning node: {node.get_name()}')
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

