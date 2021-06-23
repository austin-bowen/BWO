import rclpy
import time

from drive_motor_control import DriveMotorController, ticks_to_distance, differential_to_unicycle
from geometry_msgs.msg import Twist
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from std_msgs.msg import String
from utils import grep_serial_ports, normalize_angle_degrees


class DriveMotorsNode(Node):
    LOGGER_LEVEL = LoggingSeverity.DEBUG
    NODE_NAME = 'drive_motors'
    SAFE = True
    SAFE_STOP_PERIOD = 1.0
    TIMER_PERIOD = 0.2

    def __init__(self) -> None:
        super().__init__(self.NODE_NAME, namespace=self.NODE_NAME)

        # Initialize instance variables
        self._last_set_velocity_time = 0.0
        self._target_linear_velocity = 0.0
        self._target_angular_velocity = 0.0

        # Setup logger
        logger = self.get_logger()
        logger.set_level(self.LOGGER_LEVEL)

        # Find motor controller
        logger.info('Searching for motor controller...')
        try:
            serial_port = next(grep_serial_ports(
                r'USB VID:PID=2886:802F SER=46EB557A50533050352E3120FF121E27 LOCATION=.+'))
        except StopIteration:
            logger.error('Failed to find motor controller.')
            serial_port = None

        # Connect to motor controller
        if serial_port:
            logger.info('Connecting...')
            self.drive_motors = DriveMotorController(
                    controller_serial_port=serial_port.device,
                    set_velocity_resend_period=None
            )
            logger.info('Connected.')

            self.drive_motors.set_acceleration(2000)

            self.create_timer(self.TIMER_PERIOD, self.send_motor_commands)
        else:
            self.drive_motors = None

        # Start listening for the set_velocity command
        self.create_subscription(
            Twist,
            'set_velocity',
            self._set_velocity_callback,
            10
        )

    def destroy_node(self) -> bool:
        result = super().destroy_node()

        if self.drive_motors is not None:
            self.drive_motors.__exit__(None, None, None)

        return result

    def send_motor_commands(self) -> None:
        logger = self.get_logger()

        # drive_motors.set_brake(self._brake)

        # Stop if it has been too long since we received a set_velocity command
        if self.SAFE and (self._target_linear_velocity or self._target_angular_velocity):
            dt = time.monotonic() - self._last_set_velocity_time
            if dt >= self.SAFE_STOP_PERIOD:
                logger.info(f'Stopping drive motors; it has been >= {self.SAFE_STOP_PERIOD}s '
                            f'since the last set_velocity command was received.')
                self._target_linear_velocity = 0
                self._target_angular_velocity = 0

        logger.debug(f'target_linear_velocity = {self._target_linear_velocity}')
        logger.debug(f'target_angular_velocity = {self._target_angular_velocity}')

        state = self.drive_motors.set_velocity_unicycle(
            self._target_linear_velocity,
            self._target_angular_velocity
        )
        logger.debug(str(state))

        #self.publish('drive_motors.state', state)

    def _set_velocity_callback(self, msg) -> None:
        self._last_set_velocity_time = time.monotonic()
        self._target_linear_velocity = msg.linear.x
        self._target_angular_velocity = msg.angular.z


def main(args=None) -> None:
    rclpy.init(args=args)

    node = DriveMotorsNode()

    try:
        print('Spinning node')
        rclpy.spin(node)
    finally:
        print('Shutting down')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
