"""
Topics related to the realsense2_camera.realsense2_camera_node node:
/accel/imu_info
/color/camera_info
/color/image_raw
/depth/camera_info
/depth/image_rect_raw
/extrinsics/depth_to_color
/extrinsics/depth_to_infra1
/extrinsics/depth_to_infra2
/gyro/imu_info
/imu
/infra1/camera_info
/infra1/image_rect_raw
/infra2/camera_info
/infra2/image_rect_raw
"""

import jetson.inference
import jetson.utils
import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from typing import List, NamedTuple, Tuple



class ObjectDetectorNode(Node):
    IMAGE_TOPIC = '/color/image_raw'

    def __init__(self, network_name: str = 'ssd-mobilenet-v2', threshold: float = 0.5):
        super().__init__('object_detector')

        self.cuda_image = None

        self.net = jetson.inference.detectNet(
            network_name,
            [f'--log-level=success'],
            threshold
        )

        # TODO: Do I need to assign this to a variable tho?
        self._sub = self.create_subscription(
            Image,
            self.IMAGE_TOPIC,
            self.handle_image,
            10
        )

    def handle_image(self, image) -> None:
        log = self.get_logger()

        #if not self.cuda_image:
        #    self.cuda_image = jetson.utils.cudaImage(
        #        width=image.width, height=image.height, format=image.encoding)
        #cuda_image = self.cuda_image

        # Set the cuda image to the image data
        #np_image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        # TODO: Is there a more efficient way to do this?
        np_image = np.array(image.data, dtype=np.uint8, copy=False).reshape(image.height, image.width, -1)
        cuda_image = jetson.utils.cudaFromNumpy(np_image)

        # Detect stuff
        detections = self.net.Detect(cuda_image)
        log.info(f'Detected {len(detections)} objects @ {round(self.net.GetNetworkFPS())} FPS')
        if detections:
            d = detections[0]
            log.info(str(d.Center))


def main(args=None) -> None:
    rclpy.init(args=args)

    object_detector = ObjectDetectorNode()

    try:
        rclpy.spin(object_detector)
    finally:
        object_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

