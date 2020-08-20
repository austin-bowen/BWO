import itertools
import os
import time
from math import atan2, sqrt, pi

import gamesir
from drive_motor_control import DriveMotorController, ticks_to_distance, differential_to_unicycle
from easybot.node import Message, Node, NodeRunner
from maestro import MicroMaestro
from orientation import Orientation
from utils import grep_serial_ports, normalize_angle_degrees


class DriveMotorsNode(Node):
    def __init__(self):
        super().__init__('Drive Motors', 5)

        self._target_linear_velocity = 0
        self._target_angular_velocity = 0
        self.subscribe('remote_control.left_joystick_x', self._handle_left_joystick_x)
        self.subscribe('remote_control.left_joystick_y', self._handle_left_joystick_y)

    def loop(self) -> None:
        self.log('Search for motor controller...')
        try:
            serial_port = next(grep_serial_ports(
                r'USB VID:PID=2886:802F SER=46EB557A50533050352E3120FF121E27 LOCATION=.+'))
        except StopIteration:
            self.log_warning('Failed to find motor controller.')
            return

        self.log('Connecting...')
        try:
            with DriveMotorController(controller_serial_port=serial_port.device) as drive_motors:
                self.log('Connected.')

                while not self._stop_flag.wait(timeout=0.1):
                    drive_motors.set_velocity_unicycle(self._target_linear_velocity, self._target_angular_velocity)
        finally:
            self.log('Disconnected.')

    def _handle_left_joystick_x(self, message: Message):
        self._target_angular_velocity = -90 * message.data

    def _handle_left_joystick_y(self, message: Message):
        self._target_linear_velocity = 40 * message.data


class GamesirNode(Node):
    def __init__(self):
        super().__init__('Gamesir Controller', 5)

    def loop(self) -> None:
        controllers = gamesir.get_controllers()
        if not controllers:
            return

        with controllers[0] as controller:
            for event in controller.read_loop():
                if self._stop_flag.is_set():
                    return

                self.handle_controller_event(controller, event)

    def handle_controller_event(self, controller, event) -> None:
        if event.type not in controller.EVENT_TYPES:
            return

        event_code = controller.EventCode(event.code)

        if event_code == controller.EventCode.LEFT_JOYSTICK_X:
            topic = 'left_joystick_x'
            # Scale from [0, 255] to [-1., 1.]
            value = (event.value - 128) / 128

        elif event_code == controller.EventCode.LEFT_JOYSTICK_Y:
            topic = 'left_joystick_y'
            # Scale from [255, 0] to [-1., 1.]
            value = (128 - event.value) / 128

        elif event_code == controller.EventCode.RIGHT_JOYSTICK_X:
            topic = 'right_joystick_x'
            # Scale from [255, 0] to [-1., 1.]
            value = (128 - event.value) / 128

        elif event_code == controller.EventCode.RIGHT_JOYSTICK_Y:
            topic = 'right_joystick_y'
            # Scale from [0, 255] to [-1., 1.]
            value = (event.value - 128) / 128

        elif event_code == controller.EventCode.LEFT_TRIGGER_PRESSURE:
            topic = 'left_trigger_pressure'
            # Scale from [0, 255] to [0, 1]
            value = event.value / 255

        elif event_code == controller.EventCode.RIGHT_TRIGGER_PRESSURE:
            topic = 'right_trigger_pressure'
            # Scale from [0, 255] to [0, 1]
            value = event.value / 255

        else:
            return

        self.publish(f'remote_control.{topic}', value)


class Point:
    def __init__(self, x, y) -> None:
        self.x = x
        self.y = y

    def __str__(self) -> str:
        return f'[x={self.x:.1f}, y={self.y:.1f}]'

    def angle(self, other: 'Point') -> float:
        """:return: The angle [degrees] of the line drawn from this point to the other point."""
        angle = atan2(other.y - self.y, other.x - self.x) * 180 / pi
        return normalize_angle_degrees(angle)

    def distance(self, other: 'Point') -> float:
        """:return: The distance [no units] of this point from the other point."""
        return sqrt((other.x - self.x) ** 2 + (other.y - self.y) ** 2)


def test_remote_control_nodes():
    node_runner = NodeRunner([
        GamesirNode(),
        DriveMotorsNode()
    ])

    node_runner.run()


def test_remote_control():
    drive_motors_serial_port = next(grep_serial_ports(
        r'USB VID:PID=2886:802F SER=46EB557A50533050352E3120FF121E27 LOCATION=.+'))
    servo_control_serial_port = next(grep_serial_ports(
        r'USB VID:PID=1FFB:0089 SER=00233827 LOCATION=.+\.0'))

    normal_speed = 40
    turbo_speed = 80
    turbo = False

    with gamesir.get_controllers()[0] as controller, \
            DriveMotorController(controller_serial_port=drive_motors_serial_port.device) as drive_motors, \
            MicroMaestro(tty=servo_control_serial_port.device) as servo_control:

        prev_state = drive_motors.set_velocity_differential(0, 0)

        v_scale = w_scale = 0
        for event in controller.read_loop():
            if event.type not in controller.EVENT_TYPES:
                continue

            event_code = controller.EventCode(event.code)

            if event_code == controller.EventCode.LEFT_JOYSTICK_X:
                # Scale from [255, 0] to [-1., 1.]
                w_scale = (128 - event.value) / 128

            elif event_code == controller.EventCode.LEFT_JOYSTICK_Y:
                # Scale from [255, 0] to [-1., 1.]
                v_scale = (128 - event.value) / 128

            elif event_code == controller.EventCode.RIGHT_JOYSTICK_X:
                value = (128 - event.value) / 128
                target = 1510 + 500 * value
                servo_control.set_target(1, target)
                continue

            elif event_code == controller.EventCode.RIGHT_JOYSTICK_Y:
                value = (128 - event.value) / 128
                target = 1450 - 500 * value
                servo_control.set_target(0, target)
                continue

            elif event_code == controller.EventCode.RIGHT_TRIGGER_PRESSURE:
                turbo = event.value >= 200

            else:
                continue

            max_speed = turbo_speed if turbo else normal_speed

            v = max_speed * v_scale
            w = 90 * w_scale
            print(f'Desired:  v:{v:6.1f}  w:{w:6.1f}')

            state = drive_motors.set_velocity_unicycle(v, w)

            dt = state.timestamp - prev_state.timestamp
            d_left_motor_position = ticks_to_distance(state.left_motor_position - prev_state.left_motor_position)
            d_right_motor_position = ticks_to_distance(state.right_motor_position - prev_state.right_motor_position)

            v, w = differential_to_unicycle(d_left_motor_position / dt, d_right_motor_position / dt)

            print(f'Actual :  v:{v:6.1f}  w:{w:6.1f}')

            prev_state = state


def test_waypoint_following():
    # waypoints = (
    #     Point(0, 0),
    #     Point(0, 50),
    #     Point(50, 50),
    #     Point(50, 0),
    #     Point(100, 0),
    #     Point(100, 50),
    #     Point(50, 50),
    #     Point(50, 0),
    # )
    # Starting point in kitchen: -2.5, 4.5
    waypoints = (
        Point(0, 0),
        Point(0, 320),
        Point(137, 320),
        Point(206, 251),
        Point(206, 0),
    )
    waypoints = itertools.cycle(waypoints)

    max_linear_speed = 30
    max_angular_speed = 90

    orientation = Orientation()

    with DriveMotorController('/dev/ttyACM7') as drive_motors:
        drive_motors.state_change_callbacks.add(orientation.drive_motor_state_change_callback)

        waypoint = next(waypoints)
        while time.sleep(0.2) is None:
            robot_point = Point(orientation.x, orientation.y)

            # Robot is at the waypoint?
            if robot_point.distance(waypoint) < 5:
                # Move on to the next waypoint!
                waypoint = next(waypoints)

            # Robot heading is "too far off" from pointing at the waypoint?
            heading_error = normalize_angle_degrees(robot_point.angle(waypoint) - orientation.heading)
            if abs(heading_error) > 10:
                v = 0
                w = heading_error
            # Robot is pointing at the waypoint?
            else:
                v = robot_point.distance(waypoint)
                w = heading_error

            # Cap the velocities
            v = min(max(v, 0), max_linear_speed)
            w = min(max(w, -max_angular_speed), max_angular_speed)

            print(f'Waypoint: {waypoint}  Orientation: {orientation}')

            drive_motors.set_velocity_unicycle(v, w)


def main() -> int:
    print(f'PID: {os.getpid()}\n')

    try:
        # test_remote_control()
        test_remote_control_nodes()
        # test_waypoint_following()
    except KeyboardInterrupt:
        print()
    finally:
        print(f'Exiting.')

    return 0


if __name__ == '__main__':
    import sys

    sys.exit(main())
