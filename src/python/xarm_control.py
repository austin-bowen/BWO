"""
"""

import struct
from multiprocessing import RLock
from typing import Union, NamedTuple, Literal, Tuple

import serial
import serial.tools.list_ports

# General servo constants
BROADCAST_ID = 254
MIN_ANGLE_DEGREES = 0
MAX_ANGLE_DEGREES = 240

# xArm servo IDs
GRIPPER_ID = 1
WRIST_ID = 2
OUTER_ELBOW_ID = 3
INNER_ELBOW_ID = 4
SHOULDER_ID = 5
BASE_ID = 6
SERVO_IDS = (GRIPPER_ID, WRIST_ID, OUTER_ELBOW_ID, INNER_ELBOW_ID, SHOULDER_ID, BASE_ID)

# Packet stuff
_PACKET_HEADER = b'\x55\x55'
_1_SIGNED_CHAR_STRUCT = struct.Struct('<b')
_1_SIGNED_SHORT_STRUCT = struct.Struct('<h')
_2_UNSIGNED_SHORTS_STRUCT = struct.Struct('<HH')

# Servo command numbers
_SERVO_MOVE_TIME_WRITE = 1
_SERVO_MOVE_TIME_READ = 2
_SERVO_MOVE_TIME_WAIT_WRITE = 7
_SERVO_MOVE_TIME_WAIT_READ = 8
_SERVO_MOVE_START = 11
_SERVO_MOVE_STOP = 12
_SERVO_ID_WRITE = 13
_SERVO_ID_READ = 14
_SERVO_ANGLE_OFFSET_ADJUST = 17
_SERVO_ANGLE_OFFSET_WRITE = 18
_SERVO_ANGLE_OFFSET_READ = 19
_SERVO_ANGLE_LIMIT_WRITE = 20
_SERVO_ANGLE_LIMIT_READ = 21
_SERVO_VIN_LIMIT_WRITE = 22
_SERVO_VIN_LIMIT_READ = 23
_SERVO_TEMP_MAX_LIMIT_WRITE = 24
_SERVO_TEMP_MAX_LIMIT_READ = 25
_SERVO_TEMP_READ = 26
_SERVO_VIN_READ = 27
_SERVO_POS_READ = 28
_SERVO_OR_MOTOR_MODE_WRITE = 29
_SERVO_OR_MOTOR_MODE_READ = 30
_SERVO_LOAD_OR_UNLOAD_WRITE = 31
_SERVO_LOAD_OR_UNLOAD_READ = 32
_SERVO_LED_CTRL_WRITE = 33
_SERVO_LED_CTRL_READ = 34
_SERVO_LED_ERROR_WRITE = 35
_SERVO_LED_ERROR_READ = 36

# Custom types
Real = Union[float, int]


class _ServoPacket(NamedTuple):
    servo_id: int
    length: int
    command: int
    parameters: bytes
    checksum: int


class Xarm:
    def __init__(
            self,
            serial_port_regexp: str,
            timeout: float = None,
            on_enter_power_on: bool = False,
            on_exit_power_off: bool = True,
            verify_checksum: bool = True
    ) -> None:
        try:
            port_info = next(serial.tools.list_ports.grep(serial_port_regexp))
        except StopIteration:
            raise XarmException(f'Could not find a serial port matching regexp "{serial_port_regexp}".')

        self.on_enter_power_on = on_enter_power_on
        self.on_exit_power_off = on_exit_power_off
        self.verify_checksum = verify_checksum

        self._conn = serial.Serial(port=port_info.device, baudrate=115200, timeout=timeout)
        self._conn_lock = RLock()

    def __enter__(self):
        if self.on_enter_power_on:
            self.set_powered(BROADCAST_ID, True)

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        try:
            if self.on_exit_power_off:
                self.set_powered(BROADCAST_ID, False)
        finally:
            self._conn.close()

    def _send_packet(self, servo_id: int, command: int, parameters: Union[bytearray, bytes] = None) -> None:
        # The xArm servo command packet format is as follows:
        #
        #     | Header: byte[2] = 0x55 0x55 | ID: byte | Length: byte | Command: byte |
        #     | Parameter: byte[0..n] | Checksum: byte |
        #
        # - ID is the servo ID.
        # - Length is the number of bytes being sent after and including this byte.
        # - Checksum is the LSB of ~(ID + Length + Command + Parameter0 + ...ParameterN).

        if servo_id < 0 or servo_id > 254:
            raise XarmException(f'servo_id must be in range [0, 254]; got {servo_id}.')
        if command < 0 or command > 255:
            raise XarmException(f'command must be in range [0, 255]; got {command}.')

        if parameters is None:
            parameters = b''

        servo_packet = bytearray(_PACKET_HEADER)
        servo_packet.append(servo_id)
        length = 3 + len(parameters)
        servo_packet.append(length)
        servo_packet.append(command)
        if parameters:
            servo_packet.extend(parameters)

        checksum = _calculate_checksum(servo_id, length, command, parameters)
        servo_packet.append(checksum)

        # Insert the total length of the packet at the beginning of the packet
        servo_packet.insert(0, len(servo_packet))
        with self._conn_lock:
            # Send the packet
            self._conn.write(servo_packet)

            # Wait for the ACK
            response = self._conn.read(1)[0]
            if response != 0xAA:
                raise XarmException(f'Expected to receive ACK (0xAA); received 0x{response:x}.')

    def _receive_packet(self) -> _ServoPacket:
        with self._conn_lock:
            header = self._conn.read(2)
            if header != _PACKET_HEADER:
                raise XarmException(rf'Expected header {repr(_PACKET_HEADER)}; received header {repr(header)}.')

            servo_id, length, command = self._conn.read(3)
            param_count = length - 3
            parameters = self._conn.read(param_count)
            checksum = self._conn.read(1)[0]

        if self.verify_checksum:
            actual_checksum = _calculate_checksum(servo_id, length, command, parameters)
            if checksum != actual_checksum:
                raise XarmException(
                    f'Checksum failed for received packet! '
                    f'Received checksum = {checksum}. Actual checksum = {actual_checksum}.'
                )

        return _ServoPacket(servo_id, length, command, parameters, checksum)

    def _send_and_receive_packet(self, *args, **kwargs) -> _ServoPacket:
        with self._conn_lock:
            self._send_packet(*args, **kwargs)
            return self._receive_packet()

    def _move_time_write(self, servo_id: int, angle_degrees: Real, time_s: Real, command: int) -> None:
        """
        TODO: This.

        TODO: Test.

        :param servo_id:
        :param angle_degrees: Should be in the range [0, 240] degrees; will be truncated if outside this range.
        :param time_s: Should be in the range [0, 30] seconds; will be truncated if outside this range.
        :param command: Acceptable values are _SERVO_MOVE_TIME_WRITE, or _SERVO_MOVE_TIME_WAIT_WRITE.
        """

        if command not in {_SERVO_MOVE_TIME_WRITE, _SERVO_MOVE_TIME_WAIT_WRITE}:
            raise ValueError(
                f'Command must be either {_SERVO_MOVE_TIME_WRITE} or {_SERVO_MOVE_TIME_WAIT_WRITE}; got {command}.'
            )

        angle_degrees = _truncate_angle(angle_degrees)
        angle = _degrees_to_ticks(angle_degrees)

        time_s = min(max(0, time_s), 30)
        time_ms = int(round(time_s * 1000))

        params = _2_UNSIGNED_SHORTS_STRUCT.pack(angle, time_ms)
        self._send_packet(servo_id, command, params)

    def move_time_write(self, servo_id: int, angle_degrees: Real, time_s: Real) -> None:
        """
        :param servo_id:
        :param angle_degrees: Should be in the range [0, 240] degrees; will be truncated if outside this range.
        :param time_s: Should be in the range [0, 30] s; will be truncated if outside this range.
        :return:
        """

        return self._move_time_write(servo_id, angle_degrees, time_s, command=_SERVO_MOVE_TIME_WRITE)

    def move_time_wait_write(self, servo_id: int, angle_degrees: Real, time_s: Real) -> None:
        """
        :param servo_id:
        :param angle_degrees: Should be in the range [0, 240] degrees; will be truncated if outside this range.
        :param time_s: Should be in the range [0, 30] s; will be truncated if outside this range.
        :return:
        """

        return self._move_time_write(servo_id, angle_degrees, time_s, command=_SERVO_MOVE_TIME_WAIT_WRITE)

    def _move_time_read(self, servo_id: int, command: int) -> Tuple[float, float]:
        """
        TODO: This docstring.
        TODO: Test.
        :param servo_id:
        :param command: Either _SERVO_MOVE_TIME_READ or _SERVO_MOVE_TIME_WAIT_READ.
        :return: (angle_degrees, time_s)
        """

        if command not in {_SERVO_MOVE_TIME_READ, _SERVO_MOVE_TIME_WAIT_READ}:
            raise ValueError(
                f'Command must be either {_SERVO_MOVE_TIME_READ} or {_SERVO_MOVE_TIME_WAIT_READ}; got {command}.'
            )

        response = self._send_and_receive_packet(servo_id, command)

        angle, time_ms = _2_UNSIGNED_SHORTS_STRUCT.unpack(response.parameters)

        angle_degrees = _ticks_to_degrees(angle)
        time_s = time_ms / 1000

        return angle_degrees, time_s

    def move_time_read(self, servo_id: int) -> Tuple[float, float]:
        return self._move_time_read(servo_id, command=_SERVO_MOVE_TIME_READ)

    def move_time_wait_read(self, servo_id: int) -> Tuple[float, float]:
        return self._move_time_read(servo_id, command=_SERVO_MOVE_TIME_WAIT_READ)

    def move_start(self, servo_id: int) -> None:
        self._send_packet(servo_id, _SERVO_MOVE_START)

    def move_stop(self, servo_id: int) -> None:
        self._send_packet(servo_id, _SERVO_MOVE_STOP)

    def id_write(self, old_id: int, new_id: int) -> None:
        if new_id < 0 or new_id > 253:
            raise ValueError(f'new_id must be in range [0, 253]; got {new_id}.')

        self._send_packet(old_id, _SERVO_ID_WRITE, bytes((new_id,)))

    def angle_offset_adjust(self, servo_id: int, offset_degrees: Real, write: bool = True) -> None:
        """
        Sets the servo's angle offset.

        TODO: Test.

        :param servo_id:
        :param offset_degrees: Servo angle offset, in the range [-30, +30] degrees.
        :param write: If True, then angle_offset_write(servo_id) will be called after the offset adjustment has been
            made. Otherwise, the offset adjustment will be lost after the servo loses power.
        """

        if offset_degrees < -30 or offset_degrees > 30:
            raise ValueError(f'offset_degrees must be in range [-30, 30]; got {offset_degrees}.')

        offset = int(round(offset_degrees * 125 / 30))
        params = _1_SIGNED_CHAR_STRUCT.pack(offset)
        self._send_packet(servo_id, _SERVO_ANGLE_OFFSET_ADJUST, params)

        if write:
            self.angle_offset_write(servo_id)

    def angle_offset_write(self, servo_id: int) -> None:
        """
        Saves the offset adjust value set by angle_offset_adjust()
        so that it will persist after the servo loses power.

        TODO: Test.
        """

        self._send_packet(servo_id, _SERVO_ANGLE_OFFSET_WRITE)

    def angle_offset_read(self, servo_id: int) -> float:
        """
        :return: The angle offset of the servo in degrees. Defaults to 0. Range is [-30, 30].
        """

        response = self._send_and_receive_packet(servo_id, _SERVO_ANGLE_OFFSET_READ)
        offset = _1_SIGNED_CHAR_STRUCT.unpack(response.parameters)[0]
        return offset * 30 / 125

    def angle_limit_write(self, servo_id: int, min_angle_degrees: Real, max_angle_degrees: Real) -> None:
        """
        Sets the minimum and maximum servo angles allowed.
        This setting will persist after the servo loses power.

        TODO: Test.

        :param servo_id:
        :param min_angle_degrees: Minimum servo angle, in the range [0, 240] degrees. Will be truncated if out of range.
            Must be lower than max_angle_degrees.
        :param max_angle_degrees: Maximum servo angle, in the range [0, 240] degrees. Will be truncated if out of range.
            Must be higher than min_angle_degrees.
        """

        min_angle_degrees = _truncate_angle(min_angle_degrees)
        max_angle_degrees = _truncate_angle(max_angle_degrees)

        min_angle = _degrees_to_ticks(min_angle_degrees)
        max_angle = _degrees_to_ticks(max_angle_degrees)

        if min_angle >= max_angle:
            raise ValueError(
                f'min_angle_degrees must be less than max_angle_degrees; '
                f'got min_angle_degrees={min_angle_degrees} (==> min_angle={min_angle}) '
                f'and max_angle_degrees={max_angle_degrees} (==> max_angle={max_angle}).'
            )

        params = _2_UNSIGNED_SHORTS_STRUCT.pack(min_angle, max_angle)
        self._send_packet(servo_id, _SERVO_ANGLE_LIMIT_WRITE, params)

    def angle_limit_read(self, servo_id: int) -> Tuple[float, float]:
        """
        TODO: Test.
        :return: The angle limits as a tuple of (min_angle_degrees, max_angle_degrees).
        """

        response = self._send_and_receive_packet(servo_id, _SERVO_ANGLE_LIMIT_READ)

        min_angle, max_angle = _2_UNSIGNED_SHORTS_STRUCT.unpack(response.parameters)

        min_angle_degrees = _ticks_to_degrees(min_angle)
        max_angle_degrees = _ticks_to_degrees(max_angle)

        return min_angle_degrees, max_angle_degrees

    def vin_limit_write(self, servo_id: int, min_voltage: Real, max_voltage: Real) -> None:
        # TODO: This.
        ...

    def vin_limit_read(self, servo_id: int) -> Tuple[float, float]:
        # TODO: This.
        ...

    def temp_max_limit_write(self, servo_id: int, temp: Real, units: Literal['C', 'F'] = 'F') -> None:
        # TODO: This.
        ...

    def temp_max_limit_read(self, servo_id: int, units: Literal['C', 'F'] = 'F') -> float:
        # TODO: This.
        ...

    def temp_read(self, servo_id: int, units: Literal['C', 'F'] = 'F') -> float:
        """
        Reads the temperature of the servo and returns in units of either Celsius or Fahrenheit.

        :param servo_id:
        :param units: Use 'C' for Celsius or 'F' for Fahrenheit.
        """

        units = units.upper()
        if units not in {'C', 'F'}:
            raise ValueError(f'Units must be either "C" or "F"; got "{units}".')

        response = self._send_and_receive_packet(servo_id, _SERVO_TEMP_READ)

        # This is initially in Celsius
        temp = float(response.parameters[0])

        # Convert to Fahrenheit?
        if units == 'F':
            temp = _celsius_to_fahrenheit(temp)

        return temp

    def vin_read(self, servo_id: int) -> float:
        """Reads the input voltage to the servo."""

        response = self._send_and_receive_packet(servo_id, _SERVO_VIN_READ)

        vin_mv = _1_SIGNED_SHORT_STRUCT.unpack(response.parameters)[0]
        return vin_mv / 1000

    def pos_read(self, servo_id: int) -> float:
        """Reads the servo angle, in degrees. May be negative."""

        response = self._send_and_receive_packet(servo_id, _SERVO_POS_READ)

        angle = _1_SIGNED_SHORT_STRUCT.unpack(response.parameters)[0]
        return _ticks_to_degrees(angle)

    def mode_write(self, servo_id: int, mode: Literal['motor', 'servo']) -> None:
        # TODO: This.
        ...

    def mode_read(self, servo_id: int) -> Literal['motor', 'servo']:
        # TODO: This.
        ...

    def set_powered(self, servo_id: int, powered: bool) -> None:
        self._send_packet(servo_id, _SERVO_LOAD_OR_UNLOAD_WRITE, b'\x01' if powered else b'\x00')

    def is_powered(self, servo_id: int) -> bool:
        response = self._send_and_receive_packet(servo_id, _SERVO_LOAD_OR_UNLOAD_READ)
        return bool(response.parameters[0])

    def led_ctrl_write(self, servo_id: int, state: bool) -> None:
        self._send_packet(servo_id, _SERVO_LED_CTRL_WRITE, b'\x00' if state else b'\x01')

    def led_ctrl_read(self, servo_id: int) -> bool:
        # TODO: This.
        ...

    def led_error_write(self, servo_id: int, stalled: bool, over_voltage: bool, over_temp: bool) -> None:
        # TODO: This.
        ...

    def led_error_read(self, servo_id: int) -> Tuple[bool, bool, bool]:
        # TODO: This.
        ...


class XarmException(Exception):
    pass


def _calculate_checksum(servo_id: int, length: int, command: int, parameters: Union[bytearray, bytes]) -> int:
    checksum = servo_id + length + command + sum(parameters)
    checksum = ~checksum & 0xFF
    return checksum


def _celsius_to_fahrenheit(temp: Real) -> float:
    return (temp * 9 / 5) + 32


def _fahrenheit_to_celsius(temp: Real) -> float:
    return (temp - 32) * 5 / 9


def _degrees_to_ticks(degrees: Real) -> int:
    return int(float(degrees * 1000 / MAX_ANGLE_DEGREES))


def _ticks_to_degrees(ticks: int) -> float:
    return ticks * MAX_ANGLE_DEGREES / 1000


def _truncate_angle(angle_degrees: Real) -> Real:
    """:return: The angle, truncated to be in the range [0, 240] degrees."""

    return min(max(MIN_ANGLE_DEGREES, angle_degrees), MAX_ANGLE_DEGREES)


def control(arm: Xarm):
    print('Enter commands in the format (ID, angle [deg], time [s]):')

    arm.set_powered(BROADCAST_ID, True)

    while True:
        try:
            command = input()
            servo_id, angle, time = [a.strip() for a in command.split(',')]
            servo_id, angle, time = int(servo_id), float(angle), float(time)
        except KeyboardInterrupt:
            print()
            return
        except Exception as e:
            print('Error:', e)
            continue

        arm.move_time_write(servo_id, angle, time)

    arm.set_powered(BROADCAST_ID, False)


def test(arm: Xarm) -> int:
    arm.led_ctrl_write(BROADCAST_ID, False)
    print(f'temp = {arm.temp_read(2)}')
    print(f'vin = {arm.vin_read(2)}')
    arm.move_time_write(2, 90, 1)
    import time
    time.sleep(1)
    arm.move_time_write(2, 180, 10)
    time.sleep(5)
    arm.move_stop(2)
    arm.led_ctrl_write(BROADCAST_ID, True)

    return 0


def main() -> int:
    # Ask the user if they'd like to control or test the arm
    choice = input('Would you like to [t]est or [c]ontrol the arm? ').strip().lower()

    with Xarm(r'/dev/ttyACM\d+') as arm:
        # Control the arm?
        if choice == 'c':
            try:
                control(arm)
            except KeyboardInterrupt:
                print()
            return 0

        # Test the arm?
        elif choice == 't':
            return test(arm)

        # Invalid choice?
        else:
            print(f'Invalid choice "{choice}".')
            return 1


if __name__ == '__main__':
    import sys

    sys.exit(main())
