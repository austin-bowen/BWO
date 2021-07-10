import rclpy
import time

from maestro import MicroMaestro
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from std_msgs.msg import Float32
from threading import Event
from utils import grep_serial_ports

PAN = 1
TILT = 0

PAN_CENTER = 1535
PAN_MAX = PAN_CENTER + 500
PAN_MIN = PAN_CENTER - 500
TILT_CENTER = 1450
TILT_MAX = TILT_CENTER + 500
TILT_MIN = TILT_CENTER - 450


class NeckNode(Node):
    LOGGER_LEVEL = LoggingSeverity.DEBUG
    TIMER_PERIOD = 0.1

    def __init__(self) -> None:
        super().__init__('neck')

        # Initialize instance variables
        self._target_pan_pos = 0.0
        self._target_tilt_pos = 0.0
        self._target_pos_changed = Event()

        # Setup logger
        logger = self.get_logger()
        logger.set_level(self.LOGGER_LEVEL)

        # Find maestro
        logger.info('Searching for Maestro...')
        serial_port = None
        while not serial_port:
            try:
                serial_port = next(grep_serial_ports(r'USB VID:PID=1FFB:0089 SER=00233827 LOCATION=.+\.0'))
            except StopIteration:
                logger.warning('Failed to find Maestro.')
        logger.info(f'Found Maestro on serial port: {serial_port}')

        # Connect to Maestro
        logger.info('Connecting...')
        self._servo_control = MicroMaestro(tty=serial_port.device, safe_close=False)
        logger.info('Connected.')

        # Setup Maestro
        #self._servo_control.set_range(PAN, PAN_MIN, PAN_MAX)
        #self._servo_control.set_range(TILT, TILT_MIN, TILT_MAX)
        self._servo_control.set_speed(PAN, 30)
        self._servo_control.set_speed(TILT, 25)
        self._servo_control.set_acceleration(PAN, 5)
        self._servo_control.set_acceleration(TILT, 5)
        self._servo_control.set_targets({
            PAN: PAN_CENTER,
            TILT: TILT_CENTER
        })

        self.create_timer(self.TIMER_PERIOD, self._send_servo_commands)

        # Publishers
        self._pan_pos_publisher = self.create_publisher(
            Float32,
            'neck/pan/pos',
            10
        )
        self._tilt_pos_publisher = self.create_publisher(
            Float32,
            'neck/tilt/pos',
            10
        )

        # Start listening for the position commands
        # - Pan servo
        self.create_subscription(
            Float32,
            'neck/pan/set_pos',
            self._pan_set_pos_callback,
            10
        )
        self.create_subscription(
            Float32,
            'neck/pan/inc_pos',
            self._pan_inc_pos_callback,
            10
        )
        # - Tilt servo
        self.create_subscription(
            Float32,
            'neck/tilt/set_pos',
            self._tilt_set_pos_callback,
            10
        )
        self.create_subscription(
            Float32,
            'neck/tilt/inc_pos',
            self._tilt_inc_pos_callback,
            10
        )

    def destroy_node(self) -> bool:
        self.get_logger().info('Destroying...')

        try:
            # Put the head down before closing
            self._servo_control.set_targets({
                PAN: PAN_CENTER,
                TILT: TILT_MIN
            })
            self._servo_control.close()
        finally:
            return super().destroy_node()

    def _pan_set_pos_callback(self, position: Float32) -> None:
        self._target_pan_pos = min(max(-1.0, position.data), 1.0)
        self._target_pos_changed.set()

    def _pan_inc_pos_callback(self, inc: Float32) -> None:
        position = Float32()
        position.data = self._target_pan_pos + inc.data
        self._pan_set_pos_callback(position)

    def _tilt_set_pos_callback(self, position: Float32) -> None:
        self._target_tilt_pos = min(max(-1.0, position.data), 1.0)
        self._target_pos_changed.set()

    def _tilt_inc_pos_callback(self, inc: Float32) -> None:
        position = Float32()
        position.data = self._target_tilt_pos + inc.data
        self._tilt_set_pos_callback(position)

    def _send_servo_commands(self) -> None:
        if not self._target_pos_changed.is_set():
            return

        self._target_pos_changed.clear()

        # Get and clamp the target unit positions
        target_pan_pos_unit = min(max(-1.0, self._target_pan_pos), 1.0)
        target_tilt_pos_unit = min(max(-1.0, self._target_tilt_pos), 1.0)

        # Convert the unit positions to microseconds
        if target_pan_pos_unit >= 0:
            target_pan_pos_us = PAN_CENTER - (PAN_MAX - PAN_CENTER) * target_pan_pos_unit
        else:
            target_pan_pos_us = PAN_CENTER - (PAN_CENTER - PAN_MIN) * target_pan_pos_unit

        if target_tilt_pos_unit >= 0:
            target_tilt_pos_us = TILT_CENTER + (TILT_MAX - TILT_CENTER) * target_tilt_pos_unit
        else:
            target_tilt_pos_us = TILT_CENTER + (TILT_CENTER - TILT_MIN) * target_tilt_pos_unit

        # Put head down a little bit at the more extreme pan angles so the
        # top of the camera does not scrape against the top level
        pan_inner_range = 300
        tilt_limit_outer_range = 1800
        if target_tilt_pos_us > tilt_limit_outer_range and (
            target_pan_pos_us < PAN_CENTER - pan_inner_range or \
            target_pan_pos_us > PAN_CENTER + pan_inner_range
        ):
            target_tilt_pos_us = tilt_limit_outer_range

        self._servo_control.set_targets({
            PAN: target_pan_pos_us,
            TILT: target_tilt_pos_us
        })

        # Publish the new positions
        position = Float32()
        position.data = target_pan_pos_unit
        self._pan_pos_publisher.publish(position)
        position = Float32()
        position.data = target_tilt_pos_unit
        self._tilt_pos_publisher.publish(position)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = NeckNode()

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

