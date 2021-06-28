import jetson.inference
import jetson.utils
import numpy as np
import rclpy

from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from threading import Event, RLock, Thread
from typing import List


class ObjectDetectNode(Node):
    LOGGER_LEVEL = LoggingSeverity.DEBUG

    def __init__(self) -> None:
        super().__init__('object_detect')

        # Setup logger
        logger = self.get_logger()
        logger.set_level(self.LOGGER_LEVEL)

        # Setup and start object detect thread
        self._object_detect_thread = ObjectDetectThread(
            logger,
            None,   # TODO: Not this
            video_source_uri='/dev/video2',
            video_source_args=[
                '--input-width=848',
                '--input-height=480',
                #'--input-width=1280',
                #'--input-height=720',
                '--input-rate=15'
            ],
        )
        self._object_detect_thread.start()

        self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self._object_detect_thread.set_image,
            1
        )

    def destroy_node(self) -> bool:
        self.get_logger().info('Destroying...')

        self._object_detect_thread.stop()
        self._object_detect_thread.join()

        return super().destroy_node()


class ObjectDetectThread(Thread):
    def __init__(
            self,
            logger,
            publisher,
            video_source_uri: str,
            video_source_args: List[str] = None,
            network_name: str = 'ssd-mobilenet-v2',
            network_threshold: float = 0.5,
            log_level: str = 'debug'
    ) -> None:
        """
        :param log_level: One of:
            'silent', 'error', 'warning', 'success', 'info', 'verbose', 'debug'
        """

        super().__init__(name='object_detect_thread', daemon=False)

        self.logger = logger
        self.publisher = publisher
        self.video_source_uri = video_source_uri
        self.video_source_args = video_source_args
        self.network_name = network_name
        self.network_threshold = network_threshold
        self.logging_args = [f'--log-level={log_level}']

        self._stop_flag = Event()
        self._image_lock = RLock()

    def run(self) -> None:
        # Build video source args
        video_source_args = self.video_source_args
        if not video_source_args:
            video_source_args = []
        video_source_args += self.logging_args

        # Initialize the video source
        #self.logger.info(f'Initializing video source {self.video_source_uri!r}...')
        #video_source = jetson.utils.videoSource(
        #    self.video_source_uri,
        #    argv=video_source_args
        #)
        video_source = None

        # Test video source
        #self.logger.info('Testing video source...')
        #image = video_source.Capture()
        #if not image:
        #    self.logger.error('Did not receive an image from the camera!')
        #    return

        #video_width = video_source.GetWidth()
        #video_height = video_source.GetHeight()

        # Initialize the detection network
        self.logger.info(f'Initializing detection network {self.network_name!r}...')
        net = jetson.inference.detectNet(
            self.network_name,
            self.logging_args,
            self.network_threshold
        )

        self.logger.info('Detecting objects...')
        try:
            while not self._stop_flag.is_set():
                self._detect_once(video_source, net)
        finally:
            #self.logger.info('Closing video source.')
            #video_source.Close()
            pass

    def stop(self) -> None:
        self._stop_flag.set()

    def set_image(self, image: Image) -> None:
        with self._image_lock:
            self.logger.info(f'Got image: {image.width} x {image.height}; encoding={image.encoding}')
            self._image = image

    def _detect_once(self, video_source, net) -> None:
        # Get the next image
        #image = video_source.Capture()
        #if not image:
        #    self.logger.error('Did not receive an image from the camera!')
        #    return

        with self._image_lock:
            image = self._image
            image = np.reshape(image.data, (image.height, image.width, 3))
            image = jetson.utils.cudaFromNumpy(image)
            self.logger.info('Converted image to CUDA')

        # Detect objects in the image and iterate over each one
        detections = []
        for d in net.Detect(image, overlay='none'):
            # Get the class description, e.g. 'person', 'cat'
            class_desc = net.GetClassDesc(d.ClassID)

            # Convert from jetson Detection to our Detection
            '''
            detections.append(Detection(
                d.ClassID,
                class_desc,
                d.Confidence,
                d.Area,
                d.Bottom,
                d.Center,
                d.Height,
                d.Left,
                d.Right,
                d.Top,
                d.Width,
                video_width,
                video_height
            ))
            '''
            detections.append((class_desc, d))

        # Log detections
        if detections:
            class_descs = sorted(set(class_desc for (class_desc, _) in detections))
            class_descs = ', '.join(class_descs)
        else:
            class_descs = '[None]'
        self.logger.debug(f'Detected: {class_descs}')

        # Log FPS
        fps = round(net.GetNetworkFPS())
        self.logger.debug(f'Network FPS: {fps}')


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

