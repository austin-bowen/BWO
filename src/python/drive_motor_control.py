import struct
import time
import traceback
from math import pi
from threading import Event, RLock, Thread
from typing import NamedTuple, Union, Optional, Literal, Tuple

import serial

# Wheel constants
ENCODER_PPR = 2912
WHEEL_DIA_CM = 13.5
WHEEL_CIR_CM = pi * WHEEL_DIA_CM
WHEEL_CM_PER_TICK = WHEEL_CIR_CM / ENCODER_PPR
WHEEL_TICK_PER_CM = ENCODER_PPR / WHEEL_CIR_CM
WHEEL_TRACK_CM = 28

# Types
Real = Union[float, int]


class DriveMotorException(Exception):
    pass


class DriveMotorState(NamedTuple):
    timestamp: float
    left_motor_position: Real
    left_motor_velocity: Real
    right_motor_position: Real
    right_motor_velocity: Real
    left_bumper: bool
    middle_bumper: bool
    right_bumper: bool


class DriveMotorController(Thread):
    """
    Drive Motor Controller

    Properties:
      ``state_change_callbacks`` -- A set of functions that will be called each time a new DriveMotorState is created.
      The function must take arguments of type ``(DriveMotorController, DriveMotorState)``.

    """

    _ACK = b'\xAA'
    _SET_VELOCITY_COMMAND = b'\xC0'
    _SET_VELOCITY_RECV_STRUCT = struct.Struct('<chh')
    _SET_VELOCITY_SEND_STRUCT = struct.Struct('<lhlhc')
    _SET_PID_TUNINGS_COMMAND = b'\xC1'
    _SET_PID_TUNINGS_RECV_STRUCT = struct.Struct('<cfff')
    _SET_ACCELERATION_COMMAND = b'\xC2'
    _SET_ACCELERATION_RECV_STRUCT = struct.Struct('<cH')

    def __init__(
            self,
            controller_serial_port: str,
            baudrate: int = 115200,
            timeout: Optional[Union[float, int]] = 1,
            set_velocity_resend_period: Optional[float] = 1.0
    ) -> None:
        super().__init__(name='DriveMotorControllerThread', daemon=True)

        self._conn = serial.Serial(port=controller_serial_port, baudrate=baudrate, timeout=timeout)
        serial.Serial()
        self._lock = RLock()
        self._set_velocity_resend_period = set_velocity_resend_period
        self._stop_event = Event()

        self.target_left_motor_velocity = 0.0
        self.target_right_motor_velocity = 0.0

        self.state_change_callbacks = set()

        self.set_velocity_differential(0, 0)

        self.start()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self._stop_event.set()

        try:
            self.stop_motors()
        finally:
            self.join()

            with self._lock:
                self._conn.close()

    def get_drive_motor_state(self) -> DriveMotorState:
        return self._send_set_velocity_command()

    def run(self) -> None:
        if self._set_velocity_resend_period is None or self._set_velocity_resend_period <= 0:
            return

        while not self._stop_event.wait(self._set_velocity_resend_period):
            self._send_set_velocity_command()

    def set_acceleration(self, acceleration: int = 8000) -> None:
        """
        Sets the controller acceleration.

        :param acceleration: How quickly the controller should accelerate to a new velocity [ticks / s^2].
            If set to 0, acceleration is instant.
        """

        if acceleration < 0:
            raise ValueError(f'Acceleration must be >= 0. Given acceleration: {acceleration}')

        with self._lock:
            # Send the bytes for the "set acceleration" command
            self._conn.write(self._SET_ACCELERATION_RECV_STRUCT.pack(self._SET_ACCELERATION_COMMAND, acceleration))
            self._conn.flush()

            # Receive the response, hopefully it's an ACK
            self._recv_ack()

    def set_pid_tunings(self, p: float = 0.05, i: float = 0.5, d: float = 0.0) -> None:
        """
        Good default tunings: ``(0.05, 0.5, 0)``
        """

        # Make sure tunings are non-negative
        if p < 0 or i < 0 or d < 0:
            raise ValueError(f'All tunings must be non-negative. Given tunings: p={p}, i={i}, d={d}')

        with self._lock:
            # Send the bytes for the "set PID tunings" command
            self._conn.write(self._SET_PID_TUNINGS_RECV_STRUCT.pack(self._SET_PID_TUNINGS_COMMAND, p, i, d))
            self._conn.flush()

            # Receive the response, hopefully it's an ACK
            self._recv_ack()

    def set_velocity_differential(
            self,
            left_motor_velocity: Real = None,
            right_motor_velocity: Real = None,
            distance_unit: Literal['cm', 'ticks'] = 'cm'
    ) -> DriveMotorState:
        if distance_unit == 'cm':
            left_motor_velocity = distance_to_ticks(left_motor_velocity)
            right_motor_velocity = distance_to_ticks(right_motor_velocity)
        elif distance_unit == 'ticks':
            pass
        else:
            raise ValueError(f'Unrecognized distance_unit "{distance_unit}".')

        with self._lock:
            if left_motor_velocity is not None:
                self.target_left_motor_velocity = left_motor_velocity

            if right_motor_velocity is not None:
                self.target_right_motor_velocity = right_motor_velocity

            return self._send_set_velocity_command()

    def set_velocity_unicycle(self, linear_velocity: Real, angular_velocity: Real) -> DriveMotorState:
        """
        :param linear_velocity: How fast the robot should travel [cm / s]
        :param angular_velocity: How fast the robot should rotate [deg / s]
        """

        return self.set_velocity_differential(
            *unicycle_to_differential(linear_velocity, angular_velocity),
            distance_unit='cm'
        )

    def stop_motors(self) -> DriveMotorState:
        return self.set_velocity_differential(0, 0)

    def _recv_ack(self) -> None:
        with self._lock:
            response = self._conn.read()
            if response != self._ACK:
                raise DriveMotorException('Did not receive ACK from the controller!')

    def _send_set_velocity_command(self) -> DriveMotorState:
        with self._lock:
            # Send the bytes for the "set velocity" command
            self._conn.write(self._SET_VELOCITY_RECV_STRUCT.pack(
                self._SET_VELOCITY_COMMAND,
                int(round(self.target_left_motor_velocity)),
                int(round(self.target_right_motor_velocity))
            ))
            self._conn.flush()

            # Grab the current time and receive the response byte (either ACK or NCK)
            timestamp = time.monotonic()
            self._recv_ack()

            # Receive and unpack the response bytes
            left_motor_position, left_motor_velocity, right_motor_position, right_motor_velocity, bumpers = \
                self._SET_VELOCITY_SEND_STRUCT.unpack(self._conn.read(self._SET_VELOCITY_SEND_STRUCT.size))

        # Interpret the bumpers byte
        bumpers = bumpers[0]
        left_bumper = bool(bumpers & 0x04)
        middle_bumper = bool(bumpers & 0x02)
        right_bumper = bool(bumpers & 0x01)

        # Build a new drive motor state
        state = DriveMotorState(
            timestamp,
            left_motor_position,
            left_motor_velocity,
            right_motor_position,
            right_motor_velocity,
            left_bumper,
            middle_bumper,
            right_bumper
        )

        # Notify callback subscribers of new drive motor state
        for callback in self.state_change_callbacks:
            try:
                callback(self, state)
            except RuntimeError:
                raise
            except Exception:
                print(f'Exception occurred while running callback {callback!r}:')
                traceback.print_last()

        return state


def differential_to_unicycle(left_motor_velocity: Real, right_motor_velocity: Real) -> Tuple[Real, Real]:
    """
    Convert differential steering commands into unicycle steering commands.

    :param left_motor_velocity: [cm / s]
    :param right_motor_velocity: [cm / s]
    :return: A tuple containing (linear_velocity [cm / s], angular_velocity [deg / s])
    """

    linear_velocity = (left_motor_velocity + right_motor_velocity) / 2
    angular_velocity = (180 / (pi * WHEEL_TRACK_CM)) * (right_motor_velocity - left_motor_velocity)

    return linear_velocity, angular_velocity


def unicycle_to_differential(linear_velocity: Real, angular_velocity: Real) -> Tuple[Real, Real]:
    """
    Convert unicycle steering commands into differential steering commands.

    :param linear_velocity: How fast the robot should travel [cm / s]
    :param angular_velocity: How fast the robot should rotate [deg / s]
    :return: A tuple containing (left_motor_velocity, right_motor_velocity) [cm / s]
    """

    # Convert angular velocity from [deg / s] of the body to [cm / s] of the wheel
    angular_velocity = pi * WHEEL_TRACK_CM * (angular_velocity / 360)

    left_motor_velocity = linear_velocity - angular_velocity
    right_motor_velocity = linear_velocity + angular_velocity

    return left_motor_velocity, right_motor_velocity


def ticks_to_distance(ticks: int) -> Real:
    """
    Convert encoder ticks into distance [cm].

    :param ticks: Number of encoder ticks.
    :return: Distance that number of ticks represents [cm].
    """

    return ticks * WHEEL_CM_PER_TICK


def distance_to_ticks(distance: Real) -> int:
    """
    Convert distance [cm] into encoder ticks.

    :param distance: [cm]
    :return: Number of encoder ticks (rounded) that distance represents.
    """

    return int(round(distance * WHEEL_TICK_PER_CM))


def test_set_velocity_differential_gamesir(drive_motors: DriveMotorController):
    import gamesir

    normal_speed = 50
    turbo_speed = 100
    turbo = False

    print('Connecting to GameSir controller...')
    controller = gamesir.get_controllers()[0]
    print('Connected.')

    lv_scale = rv_scale = 0
    for event in controller.read_loop():
        if event.type not in controller.EVENT_TYPES:
            continue

        event_code = controller.EventCode(event.code)

        if event_code == controller.EventCode.LEFT_JOYSTICK_Y:
            value = event.value
            # Scale from [255, 0] to [-1., 1.]
            lv_scale = - (value - 128) / 128

        elif event_code == controller.EventCode.RIGHT_JOYSTICK_Y:
            value = event.value
            # Scale from [255, 0] to [-1., 1.]
            rv_scale = - (value - 128) / 128

        elif event_code == controller.EventCode.RIGHT_TRIGGER_PRESSURE:
            turbo = event.value >= 200

        else:
            continue

        max_speed = turbo_speed if turbo else normal_speed
        lv = max_speed * lv_scale
        rv = max_speed * rv_scale

        print(f'LV: {lv} \tRV: {rv}')
        print(drive_motors.set_velocity_differential(lv, rv))


def test_set_velocity_unicycle_gamesir(drive_motors: DriveMotorController):
    import gamesir

    normal_speed = 40
    turbo_speed = 80
    turbo = False

    prev_state = drive_motors.set_velocity_differential(0, 0)

    print('Connecting to GameSir controller...')
    controller = gamesir.get_controllers()[0]
    print('Connected.')

    v_scale = w_scale = 0
    for event in controller.read_loop():
        if event.type not in controller.EVENT_TYPES:
            continue

        event_code = controller.EventCode(event.code)

        if event_code == controller.EventCode.LEFT_JOYSTICK_Y:
            value = event.value
            # Scale from [255, 0] to [-1., 1.]
            v_scale = - (value - 128) / 128

        elif event_code == controller.EventCode.LEFT_JOYSTICK_X:
            value = event.value
            # Scale from [255, 0] to [-1., 1.]
            w_scale = - (value - 128) / 128

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


def test_set_velocity_unicycle_cli(drive_motors: DriveMotorController):
    while True:
        try:
            result = input('[v,w]: ')
            v, w = result.split(',')
            v, w = float(v), float(w)
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(e)
            continue

        print(drive_motors.set_velocity_unicycle(v, w))


def _find_controller() -> DriveMotorController:
    from glob import iglob
    serial_port = next(iglob('/dev/ttyACM*'))
    return DriveMotorController(serial_port)


def main():
    print('Connecting to drive motor controller...')
    with _find_controller() as drive_motors:
        print('Connected.\n')

        try:
            test_set_velocity_unicycle_cli(drive_motors)
            # test_set_velocity_unicycle_gamesir(drive_motors)
        except KeyboardInterrupt:
            print()


if __name__ == '__main__':
    main()
