from typing import List, NamedTuple, Tuple
from typing_extensions import Literal

import jetson.inference
import jetson.utils

from easybot.node import Message, Node


class Detection(NamedTuple):
    """
    Had to create this because jetson.inference.detectNet.Detection cannot be
    pickled, and it does not have some properties I want.
    """

    class_id: int
    class_desc: str
    confidence: float
    area: int
    bottom: int
    center: Tuple[int, int]
    height: int
    left: int
    right: int
    top: int
    width: int
    image_width: int
    image_height: int


class ObjectDetectionNode(Node):
    def __init__(
            self,
            video_source_uri: str,
            video_source_args: List[str] = None,
            network: str = 'ssd-mobilenet-v2',
            threshold: float = 0.5,
            log_level: Literal['silent', 'error', 'warning', 'success', 'info', 'verbose', 'debug'] = 'success',
            log_detections: bool = False,
            log_fps: bool = False
    ) -> None:
        """
        Object detection node.

        Publishes on topic 'detected_objects', with data set to a list of
        Detection instances.

        For reference, the Intel Realsense D435i supports these settings:
        - RGB camera:
          - Resolutions: 320x180, 320x240, 424x240, 640x360, 640x480, 848x480*, 960x540, 1280x720, 1920x1080.
          - FPS: 6, 15*, 30, 60.

        * These are optimal settings.

        :param video_source_args: Includes '--input-width', '--input-height',
            and '--input-rate'.
        """

        super().__init__('Object Detection', 5, worker_type='process')

        self.video_source_uri = video_source_uri
        self.video_source_args = video_source_args
        self.network_name = network
        self.threshold = threshold
        self.logging_args = [f'--log-level={log_level}']
        self.log_detections = log_detections
        self.log_fps = log_fps

        # Initialize the detection network
        self.net = None

        self.class_id_to_desc = {}

    def loop(self) -> None:
        # Initialize the detection network
        if self.net is None:
            self.net = jetson.inference.detectNet(
                self.network_name,
                self.logging_args,
                self.threshold
            )

        # Build video source args
        video_source_args = self.video_source_args
        if not video_source_args:
            video_source_args = []
        video_source_args += self.logging_args

        # Initialize the video source
        video_source = jetson.utils.videoSource(
            self.video_source_uri,
            argv=video_source_args
        )

        video_width = video_source.GetWidth()
        video_height = video_source.GetHeight()

        try:
            while not self._stop_flag.is_set():
                # Get the next image
                image = video_source.Capture()

                # Detect objects in the image and iterate over each one
                detections = []
                for d in self.net.Detect(image, overlay='none'):
                    # Get the class description, e.g. 'person', 'cat'
                    class_desc = self.get_class_desc(d.ClassID)

                    # Convert from jetson Detection to our Detection
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

                # Publish the detections
                self.publish('detected_objects', detections)

                # Log the detections
                if self.log_detections:
                    self.log('Detected: ' + 'TODO: This.')

                # Log the detection FPS
                if self.log_fps:
                    fps = round(self.net.GetNetworkFPS())
                    self.log(f'{fps} FPS')

                if not video_source.IsStreaming():
                    break
        finally:
            video_source.Close()

    def get_class_desc(self, class_id: int) -> str:
        try:
            return self.class_id_to_desc[class_id]
        except KeyError:
            desc = self.class_id_to_desc[class_id] = self.net.GetClassDesc(class_id)
            return desc

