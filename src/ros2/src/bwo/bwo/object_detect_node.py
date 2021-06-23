import jetson
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class ObjectDetectNode(Node):
    def __init__(self) -> None:
        super().__init__('ObjectDetect')


def main(args=None) -> None:
    rclpy.init(args=args)

    node = ObjectDetectNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
