import itertools
import os
import signal
import time
from math import atan2, sqrt, pi
from multiprocessing import Event
from threading import Event as ThreadingEvent

import gamesir
from drive_motor_control import DriveMotorController, ticks_to_distance, differential_to_unicycle
from easybot.node import Message, Node, NodeRunner, get_timestamp
from maestro import MicroMaestro
from main_node import MainNode
from object_detection import ObjectDetectionNode
from orientation import Orientation
from tts_node import TTSNode
from utils import grep_serial_ports, normalize_angle_degrees


class DriveMotorsNode(Node):
    MAX_MESSAGE_LATENCY = 0.2

    def __init__(self):
        super().__init__('Drive Motors', 5)

        self._target_linear_velocity = 0
        self._target_angular_velocity = 0
        self._brake = False
        self._brake_changed = Event()
        self._boost = False

        # The data should be a dictionary like:
        #   {'linear': <cm/s>, 'angular': <deg/s>}
        # Each key is optional.
        self.subscribe(
            'drive_motors.set_target_velocities',
            self._handle_set_target_velocities
        )

        self.subscribe('remote_control.right_joystick_x', self._handle_left_joystick_x)
        self.subscribe('remote_control.left_joystick_y', self._handle_left_joystick_y)
        self.subscribe('remote_control.left_trigger_pressure', self._handle_left_trigger_pressure)
        self.subscribe('remote_control.right_trigger_pressure', self._handle_right_trigger_pressure)

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
            with DriveMotorController(
                    controller_serial_port=serial_port.device,
                    set_velocity_resend_period=None
            ) as drive_motors:
                self.log('Connected.')

                drive_motors.set_acceleration(2000)

                while not self._stop_flag.wait(timeout=0.1):
                    if self._brake_changed.is_set():
                        self._brake_changed.clear()
                        drive_motors.set_brake(self._brake)

                    state = drive_motors.set_velocity_unicycle(
                        self._target_linear_velocity * (2 if self._boost else 1),
                        self._target_angular_velocity
                    )

                    self.publish('drive_motors.state', state)
        finally:
            self.log('Disconnected.')

    def _handle_left_joystick_x(self, message: Message):
        self._target_angular_velocity = -90 * message.data

    def _handle_left_joystick_y(self, message: Message):
        self._target_linear_velocity = 40 * message.data

    def _handle_left_trigger_pressure(self, message: Message):
        brake = message.data >= 0.5

        if (brake and not self._brake) or (not brake and self._brake):
            self._brake = brake
            self._brake_changed.set()

    def _handle_right_trigger_pressure(self, message: Message):
        self._boost = message.data >= 0.5

    def _handle_set_target_velocities(self, message: Message) -> None:
        # Stop motors if message is stale
        if message.is_stale(self.MAX_MESSAGE_LATENCY):
            self.log_warning('Received stale message; stopping motors.')
            self._target_linear_velocity = 0
            self._target_angular_velocity = 0
            return

        velocities = message.data

        try:
            self._target_linear_velocity = velocities['linear']
        except KeyError:
            pass

        try:
            self._target_angular_velocity = velocities['angular']
        except KeyError:
            pass


class FaceTrackerNode(Node):
    """
    TODO: Rename this etc. to something like "TargetTrackerNode".
    """

    # Pairs of (class, min confidence) in priority order
    TRACKED_CLASSES = (
        ('sports ball', 0.0),
        ('cat'        , 0.0),
        ('person'     , 0.5)
    )

    TRACKING_PERSISTENCE_TIME = 0.5

    def __init__(self):
        super().__init__('Face Tracker', 5)

        self.person = None
        self.target_timestamp = None

        self.pan_servo_center = ServosNode.POSITIONS[ServosNode.PAN_SERVO][1]
        self.pan_servo_position = None

        self._updated_person = ThreadingEvent()

        self.subscribe(
            'detected_objects',
            self._handle_detected_objects
        )

        self.subscribe(
            f'servos.channel.{ServosNode.PAN_SERVO}.position',
            self._handle_pan_servo_position_change
        )

    def loop(self) -> None:
        error_threshold = 0.05

        was_tracking = False
        while not self._stop_flag.is_set():
            if not self._updated_person.wait(timeout=1):
                continue
            self._updated_person.clear()

            person = self.person

            if not person:
                if was_tracking:
                    was_tracking = False

                    # Stop the drive motors
                    self.publish(
                        'drive_motors.set_target_velocities',
                        {'linear': 0, 'angular': 0}
                    )

                continue

            # Calculate the face center
            if person.class_desc == 'person':
                # TODO: WHY doesn't SUBTRACTING from the top work??
                face_center = [
                    person.center[0],
                    #person.top - round(0.1 * person.height)
                    person.top + 50
                ]
            else:
                face_center = person.center

            # Calculate the error
            image_width, image_height = person.image_width, person.image_height
            face_error = [
                (2 * face_center[0] - image_width) / image_width,
                (2 * face_center[1] - image_height) / image_height
            ]

            self.log(f'{image_width}x{image_height}: {face_center} {face_error}')

            # Only move the camera if one of the errors exceeds the threshold
            max_error = max(abs(e) for e in face_error)
            if max_error >= error_threshold:
                gain = (-50, -40)
                self.publish(
                    'servos.increment_targets',
                    {
                        ServosNode.PAN_SERVO: gain[0] * face_error[0],
                        ServosNode.TILT_SERVO: gain[1] * face_error[1]
                    }
                )

            # Turn the body if the camera is pointed too far to either side
            target_velocities = {}
            if self.pan_servo_position is not None:
                offset = self.pan_servo_position - self.pan_servo_center
                angular = 0.08 * offset if abs(offset) >= 50 else 0
                target_velocities['angular'] = angular

            # Drive towards the target if it takes up less than a certain
            # percentage of the total image width
            error = (image_width - (1 / 0.4) * person.width) / image_width
            if error > 0:
                target_velocities['linear'] = min(max(0, 100 * error), 40)
            elif error < -0.2:
                target_velocities['linear'] = min(max(-15, 50 * error), 0)
            else:
                target_velocities['linear'] = 0

            self.publish(
                'drive_motors.set_target_velocities',
                target_velocities
            )

            was_tracking = True

    def _handle_detected_objects(self, message: Message) -> None:
        detections = message.data

        targets = None
        for class_desc, min_confidence in self.TRACKED_CLASSES:
            targets = filter(
                lambda d: d.class_desc == class_desc and d.confidence >= min_confidence,
                detections
            )
            targets = sorted(targets, key=lambda t: t.area, reverse=True)
            if targets:
                break

        try:
            self.person = targets[0]
            self.target_timestamp = message.timestamp
        except IndexError:
            # How long has it been since the previous target was seen?
            if self.person:
                time_last_seen = get_timestamp() - self.target_timestamp
                if time_last_seen > self.TRACKING_PERSISTENCE_TIME:
                    self.person = None
                else:
                    self.log('Cannot see target but pretending I can')

        self._updated_person.set()

    def _handle_pan_servo_position_change(self, message: Message) -> None:
        self.pan_servo_position = message.data


class GamesirNode(Node):
    def __init__(self):
        super().__init__('Gamesir Controller', 5)

    def loop(self) -> None:
        self.log('Searching for Gamesir controller...')
        controllers = gamesir.get_controllers()
        if not controllers:
            self.log('Failed to find Gamesir controller.')
            return

        self.log('Connecting...')
        try:
            with controllers[0] as controller:
                self.log('Connected.')

                for event in controller.read_loop():
                    if self._stop_flag.is_set():
                        return

                    self.handle_controller_event(controller, event)
        finally:
            print('Disconnected.')

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
            # Scale from [0, 255] to [-1., 1.]
            value = (event.value - 128) / 128

        elif event_code == controller.EventCode.RIGHT_JOYSTICK_Y:
            topic = 'right_joystick_y'
            # Scale from [255, 0] to [-1., 1.]
            value = (128 - event.value) / 128

        elif event_code == controller.EventCode.LEFT_TRIGGER_PRESSURE:
            topic = 'left_trigger_pressure'
            # Scale from [0, 255] to [0, 1]
            value = event.value / 255

        elif event_code == controller.EventCode.RIGHT_TRIGGER_PRESSURE:
            topic = 'right_trigger_pressure'
            # Scale from [0, 255] to [0, 1]
            value = event.value / 255

        elif event_code == controller.EventCode.X_BUTTON:
            topic = 'x_button'
            value = event.value

        elif event_code == controller.EventCode.Y_BUTTON:
            topic = 'y_button'
            value = event.value

        else:
            return

        self.publish(f'remote_control.{topic}', value)


class ServosNode(Node):
    TILT_SERVO = 0
    PAN_SERVO = 1

    MAX_MESSAGE_LATENCY = 0.2

    # Format: (min, center, max)
    POSITIONS = {
        TILT_SERVO: (1450 - 450, 1450, 1450 + 500),
        PAN_SERVO : (1535 - 300, 1535, 1535 + 300)
    }

    def __init__(self):
        super().__init__('Servos', 5)

        self._targets = dict(
            (channel, positions[1]) for channel, positions in self.POSITIONS.items()
        )
        self._targets_changed = Event()
        self._targets_changed.set()

        self.subscribe('servos.set_targets', self._handle_set_targets)
        self.subscribe('servos.increment_targets', self._handle_increment_targets)

        self.subscribe('remote_control.right_joystick_x', self._handle_right_joystick_x)
        self.subscribe('remote_control.right_joystick_y', self._handle_right_joystick_y)

    def loop(self) -> None:
        self.log('Searching for Maestro...')
        try:
            serial_port = next(grep_serial_ports(r'USB VID:PID=1FFB:0089 SER=00233827 LOCATION=.+\.0'))
        except StopIteration:
            self.log_warning('Failed to find Maestro.')
            return

        self.log('Connecting...')
        try:
            with MicroMaestro(tty=serial_port.device) as servo_control:
                self.log('Connected.')

                servo_control.set_acceleration(0, 0)
                servo_control.set_acceleration(1, 0)
                #servo_control.set_speed(0, 50)
                #servo_control.set_speed(1, 50)
                servo_control.set_speed(0, 25)
                servo_control.set_speed(1, 25)

                while not self._stop_flag.wait(timeout=0.1):
                    if self._targets_changed.is_set():
                        self._targets_changed.clear()
                        servo_control.set_targets(self._targets)

                    for channel in sorted(self._targets.keys()):
                        topic = f'servos.channel.{channel}.position'
                        #data = servo_control.get_position(channel)
                        data = self._targets[channel]
                        self.publish(topic, data)

                # Put the head down
                servo_control.set_targets({
                    self.TILT_SERVO: self.POSITIONS[self.TILT_SERVO][0],
                    self.PAN_SERVO: self.POSITIONS[self.PAN_SERVO][1]
                })
                time.sleep(1.5)
        finally:
            self.log('Disconnected.')

    def _handle_set_targets(self, message: Message) -> None:
        if message.is_stale(self.MAX_MESSAGE_LATENCY) or not message.data:
            return

        for channel, target in message.data.items():
            self._targets[channel] = self.normalize(channel, target)

        self._targets_changed.set()

    def _handle_increment_targets(self, message: Message) -> None:
        if message.is_stale(self.MAX_MESSAGE_LATENCY) or not message.data:
            return

        for channel, delta in message.data.items():
            target = self._targets[channel] + delta
            center = self.POSITIONS[channel][1]
            self._targets[channel] = self.normalize(channel, target)

        self._targets_changed.set()

    def _handle_right_joystick_x(self, message: Message) -> None:
        self._targets[1] = self.POSITIONS[self.PAN_SERVO][1] - 500 * message.data
        self._targets_changed.set()

    def _handle_right_joystick_y(self, message: Message) -> None:
        self._targets[0] = self.POSITIONS[self.TILT_SERVO][1] + 500 * message.data
        self._targets_changed.set()

    def normalize(self, servo: int, target: int) -> int:
        try:
            min_target, _, max_target = self.POSITIONS[servo]
        except KeyError:
            return target

        return min(max(min_target, target), max_target)


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


def lpf(current, new, alpha: float) -> float:
    """
    :param alpha: Filter constant. 0 ==> current. 1 ==> new.
    """
    return (1 - alpha) * current + alpha * new


def test_remote_control_nodes():
    def handle_terminate_signal(signum, frame):
        print('Sending stop signal...')
        Node.publish('easybot.stop')

    signal.signal(signal.SIGINT, handle_terminate_signal)
    signal.signal(signal.SIGQUIT, handle_terminate_signal)
    signal.signal(signal.SIGTERM, handle_terminate_signal)

    node_runner = NodeRunner([
        #GamesirNode(),
        DriveMotorsNode(),
        ServosNode(),
        FaceTrackerNode(),
        ObjectDetectionNode(
            video_source_uri='/dev/video2',
            video_source_args=[
                #'--input-width=848',
                #'--input-height=480',
                '--input-width=1280',
                '--input-height=720',
                '--input-rate=15'
            ],
            log_detections=True,
            log_fps=True
        ),
        TTSNode(),
        MainNode(),
    ])

    #node_runner.print_messages_matching(r'remote_control\.._button')

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
