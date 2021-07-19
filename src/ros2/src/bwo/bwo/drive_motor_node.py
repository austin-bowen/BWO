import rclpy
import time

from bwo_interfaces.msg import BumperState
from drive_motor_control import DriveMotorController, ticks_to_distance, differential_to_unicycle
from geometry_msgs.msg import Twist
from rclpy.logging import LoggingSeverity
from std_msgs.msg import String
from utils import grep_serial_ports, normalize_angle_degrees
from .node import Node

NODE_NAME = 'drive_motors'
DRIVE_MOTORS_SET_VELOCITY_TOPIC = NODE_NAME + '/set_velocity'


class DriveMotorsNode(Node):
    LOGGER_LEVEL = LoggingSeverity.DEBUG
    SAFE = True
    SAFE_STOP_PERIOD = 1.0
    TIMER_PERIOD = 0.1

    def __init__(self) -> None:
        super().__init__(NODE_NAME, logger_level=self.LOGGER_LEVEL)

        # Initialize instance variables
        self._last_set_velocity_time = 0.0
        self._target_linear_velocity = 0.0
        self._target_angular_velocity = 0.0
        self._prev_bumper_state = None

        # Setup publishers
        self._bumpers_changed_publisher = self.create_publisher(
                BumperState, 'bumpers_changed', 10)

        # Find motor controller
        self.logger.info('Searching for motor controller...')
        try:
            serial_port = next(grep_serial_ports(
                r'USB VID:PID=2886:802F SER=46EB557A50533050352E3120FF121E27 LOCATION=.+'))
        except StopIteration:
            raise RuntimeError('Failed to find motor controller.')

        # Connect to motor controller
        self.logger.info('Connecting...')
        self.drive_motors = DriveMotorController(
                controller_serial_port=serial_port.device,
                set_velocity_resend_period=None
        )
        self.logger.info('Connected.')

        self.drive_motors.set_acceleration(3000)

        self.create_timer(self.TIMER_PERIOD, self.send_motor_commands)

        # Start listening for the set_velocity command
        self.create_subscription(
            Twist,
            DRIVE_MOTORS_SET_VELOCITY_TOPIC,
            self._set_velocity_callback,
            10
        )

    def destroy_node(self) -> bool:
        result = super().destroy_node()

        try:
            self.drive_motors.__exit__(None, None, None)
        finally:
            return result

    def send_motor_commands(self) -> None:
        # drive_motors.set_brake(self._brake)

        # Stop if it has been too long since we received a set_velocity command
        if self.SAFE and (self._target_linear_velocity or self._target_angular_velocity):
            dt = time.monotonic() - self._last_set_velocity_time
            if dt >= self.SAFE_STOP_PERIOD:
                self.logger.info(f'Stopping drive motors; it has been >= {self.SAFE_STOP_PERIOD}s '
                                 f'since the last set_velocity command was received.')
                self._target_linear_velocity = 0
                self._target_angular_velocity = 0

        self.logger.debug(f'target_linear_velocity = {self._target_linear_velocity}')
        self.logger.debug(f'target_angular_velocity = {self._target_angular_velocity}')

        state = self.drive_motors.set_velocity_unicycle(
            self._target_linear_velocity,
            self._target_angular_velocity
        )
        self.logger.debug(str(state))

        # Publish new bumper state if it has changed
        if self.count_subscribers(self._bumpers_changed_publisher.topic):
            bumper_state = BumperState()
            bumper_state.left = state.left_bumper
            bumper_state.center = state.middle_bumper
            bumper_state.right = state.right_bumper
            bumper_state.any = bumper_state.left or bumper_state.center or bumper_state.right

            if bumper_state != self._prev_bumper_state:
                self.logger.debug(f'Publishing: {bumper_state}')
                self._bumpers_changed_publisher.publish(bumper_state)
                self._prev_bumper_state = bumper_state

    def _set_velocity_callback(self, msg) -> None:
        self._last_set_velocity_time = time.monotonic()
        self._target_linear_velocity = msg.linear.x
        self._target_angular_velocity = msg.angular.z


class DriveMotorsControllerMixin:
    """
    Nodes that need to talk to the DriveMotorsNode can include this to easily
    control the drive motors.
    """

    def set_drive_motors_velocity(
        self,
        linear: float = None,
        angular: float = None,
        velocity: Twist = None
    ) -> None:
        """
        :param linear: linear velocity [cm/s]
        :param angular: angular velocity [deg/s]
        :param velocity: an instance of Twist, if you already have one.
        """

        # Build the velocity argument
        if velocity is None:
            velocity = Twist()
            velocity.linear.x = float(linear)
            velocity.angular.z = float(angular)

        # Get the publisher
        try:
            publisher = self.__drive_motors_set_velocity_publisher
        except AttributeError:
            publisher = self.__drive_motors_set_velocity_publisher = self.create_publisher(
                Twist,
                '/' + DRIVE_MOTORS_SET_VELOCITY_TOPIC,
                1
            )

        # Publish the velocity argument
        publisher.publish(velocity)


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

