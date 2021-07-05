import rclpy
import time

from bwo_interfaces.msg import BumperState
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
    TIMER_PERIOD = 0.1

    def __init__(self) -> None:
        super().__init__(self.NODE_NAME, namespace=self.NODE_NAME)

        # Initialize instance variables
        self._last_set_velocity_time = 0.0
        self._target_linear_velocity = 0.0
        self._target_angular_velocity = 0.0
        self._prev_bumper_state = None

        # Setup logger
        logger = self.get_logger()
        logger.set_level(self.LOGGER_LEVEL)

        # Setup publishers
        self._bumpers_changed_publisher = self.create_publisher(
                BumperState, 'bumpers_changed', 10)

        # Find motor controller
        logger.info('Searching for motor controller...')
        try:
            serial_port = next(grep_serial_ports(
                r'USB VID:PID=2886:802F SER=46EB557A50533050352E3120FF121E27 LOCATION=.+'))
        except StopIteration:
            raise RuntimeError('Failed to find motor controller.')

        # Connect to motor controller
        logger.info('Connecting...')
        self.drive_motors = DriveMotorController(
                controller_serial_port=serial_port.device,
                set_velocity_resend_period=None
        )
        logger.info('Connected.')

        self.drive_motors.set_acceleration(2000)

        self.create_timer(self.TIMER_PERIOD, self.send_motor_commands)

        # Start listening for the set_velocity command
        self.create_subscription(
            Twist,
            'set_velocity',
            self._set_velocity_callback,
            10
        )

    def destroy_node(self) -> bool:
        self.get_logger().info('Destroying...')

        try:
            self.drive_motors.__exit__(None, None, None)
        finally:
            return super().destroy_node()

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

        # Publish new bumper state if it has changed
        if self.count_subscribers(self._bumpers_changed_publisher.topic):
            bumper_state = BumperState()
            bumper_state.left = state.left_bumper
            bumper_state.center = state.middle_bumper
            bumper_state.right = state.right_bumper
            bumper_state.any = bumper_state.left or bumper_state.center or bumper_state.right

            if bumper_state != self._prev_bumper_state:
                logger.debug(f'Publishing: {bumper_state}')
                self._bumpers_changed_publisher.publish(bumper_state)
                self._prev_bumper_state = bumper_state

    def _set_velocity_callback(self, msg) -> None:
        self._last_set_velocity_time = time.monotonic()
        self._target_linear_velocity = msg.linear.x
        self._target_angular_velocity = msg.angular.z


def main(args=None) -> None:
    rclpy.init(args=args)

    node = DriveMotorsNode()

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

