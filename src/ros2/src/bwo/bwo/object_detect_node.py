import jetson.inference
import jetson.utils
import numpy as np
import rclpy

from bwo_interfaces.msg import ObjectDetection, ObjectDetections
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from sensor_msgs.msg import Image
from typing import List


class ObjectDetectNode(Node):
    LOGGER_LEVEL = LoggingSeverity.DEBUG

    def __init__(self) -> None:
        super().__init__('object_detect')
        
        self.network_name = 'ssd-mobilenet-v2'
        # One of: 'silent', 'error', 'warning', 'success', 'info', 'verbose', 'debug'
        log_level = 'success'
        self.logging_args = [f'--log-level={log_level}']
        self.network_threshold = 0.5

        # Setup logger
        logger = self.get_logger()
        logger.set_level(self.LOGGER_LEVEL)

        # Initialize the detection network
        logger.info(f'Initializing detection network {self.network_name!r}...')
        self.net = jetson.inference.detectNet(
            self.network_name,
            self.logging_args,
            self.network_threshold
        )

        self._detection_publisher = self.create_publisher(
                ObjectDetections, 'object_detect', 10)

        # Detect objects in camera images
        self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self._detect_objects,
            1
        )

    def destroy_node(self) -> bool:
        self.get_logger().info('Destroying...')
        return super().destroy_node()

    def _detect_objects(self, image: Image) -> None:
        logger = self.get_logger()

        image = np.reshape(image.data, (image.height, image.width, 3))
        image = jetson.utils.cudaFromNumpy(image)

        # Detect objects in the image and iterate over each one
        detections = []
        for jetson_detection in self.net.Detect(image, overlay='none'):
            # Get the class name, e.g. 'person', 'cat'
            class_name = self.net.GetClassDesc(jetson_detection.ClassID)

            # Convert from jetson Detection to our ObjectDetection
            detection = ObjectDetection()
            detection.class_name = class_name
            detection.confidence = jetson_detection.Confidence
            detection.center = jetson_detection.Center
            detection.top = jetson_detection.Top
            detection.bottom = jetson_detection.Bottom
            detection.left = jetson_detection.Left
            detection.right = jetson_detection.Right
            detection.width = jetson_detection.Width
            detection.height = jetson_detection.Height
            detection.area = jetson_detection.Area
            detection.image_width = image.width
            detection.image_height = image.height

            detections.append(detection)

        # Publish detections
        detections_msg = ObjectDetections()
        detections_msg.detections = detections
        self._detection_publisher.publish(detections_msg)

        if logger.is_enabled_for(LoggingSeverity.DEBUG):
            # Log detections
            if detections:
                class_descs = sorted(set(d.class_name for d in detections))
                class_descs = ', '.join(class_descs)
            else:
                class_descs = '[None]'
            logger.debug(f'Detected: {class_descs}')

            # Log FPS
            fps = round(self.net.GetNetworkFPS())
            logger.debug(f'Network FPS: {fps}')


def main(args=None) -> None:
    rclpy.init(args=args)

    node = ObjectDetectNode()

    print(f'Spinning node: {node.get_name()}')
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

