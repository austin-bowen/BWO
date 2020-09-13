import struct
from multiprocessing import RLock
from typing import Union, NamedTuple, Literal

import serial
import serial.tools.list_ports

Real = Union[float, int]


class ServoPacket(NamedTuple):
    servo_id: int
    length: int
    command: int
    parameters: bytes
    checksum: int


class Xarm:
    ALL_SERVO_IDS = (1, 2, 3, 4, 5, 6)

    _SIGNED_SHORT_STRUCT = struct.Struct('<h')

    def __init__(self, serial_port_regexp: str, timeout: float = None) -> None:
        try:
            port_info = next(serial.tools.list_ports.grep(serial_port_regexp))
        except StopIteration:
            raise XarmException(f'Could not find a serial port matching regexp "{serial_port_regexp}".')

        self._conn = serial.Serial(port=port_info.device, baudrate=115200, timeout=timeout)
        self._conn_lock = RLock()

    def __enter__(self):
        self.set_all_powered(True)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        try:
            self.set_all_powered(False)
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

        servo_packet = bytearray(b'\x55\x55')
        servo_packet.append(servo_id)
        length = 3
        if parameters:
            length += len(parameters)
        servo_packet.append(length)
        servo_packet.append(command)
        if parameters:
            servo_packet.extend(parameters)

        checksum = servo_id + length + command
        if parameters:
            checksum += sum(parameters)
        checksum = ~checksum & 0xFF
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

    def _receive_packet(self) -> ServoPacket:
        with self._conn_lock:
            header = self._conn.read(2)
            if header != b'\x55\x55':
                raise XarmException(rf'Expected header b"\x55\x55"; received header {repr(header)}.')

            servo_id, length, command = self._conn.read(3)
            param_count = length - 3
            parameters = self._conn.read(param_count)
            checksum = self._conn.read(1)[0]

        return ServoPacket(servo_id, length, command, parameters, checksum)

    def _send_and_receive_packet(self, *args, **kwargs) -> ServoPacket:
        with self._conn_lock:
            self._send_packet(*args, **kwargs)
            return self._receive_packet()

    def is_powered(self, servo_id: int) -> bool:
        response = self._send_and_receive_packet(servo_id, 32)
        return bool(response.parameters[0])

    def led_ctrl_write(self, servo_id: int, state: bool) -> None:
        self._send_packet(servo_id, 33, b'\x00' if state else b'\x01')

    def led_ctrl_write_all(self, state: bool) -> None:
        for servo_id in self.ALL_SERVO_IDS:
            self.led_ctrl_write(servo_id, state)

            # TODO: Figure out why we can't just send all the commands ASAP
            import time
            time.sleep(0.1)

    def move_stop(self, servo_id: int) -> None:
        self._send_packet(servo_id, 12)

    def move_time_write(self, servo_id: int, angle_degrees: Real, time_s: Real):
        """
        :param servo_id:
        :param angle_degrees: Should be in the range [0, 240] degrees; will be truncated if outside this range.
        :param time_s: Should be in the range [0, 30] s; will be truncated if outside this range.
        :return:
        """

        angle_degrees = min(max(0, angle_degrees), 240)
        time_s = min(max(0, time_s), 30)

        params = bytearray()
        angle = int(round(angle_degrees * 1000 / 240))
        params.append((angle >> 0) & 0xFF)
        params.append((angle >> 8) & 0xFF)
        time_ms = int(round(time_s * 1000))
        params.append((time_ms >> 0) & 0xFF)
        params.append((time_ms >> 8) & 0xFF)

        self._send_packet(servo_id, 1, params)

    def temp_read(self, servo_id: int, units: Literal['C', 'F'] = 'F') -> float:
        """
        Reads the temperature of the servo and returns in units of either Celsius or Fahrenheit.

        :param servo_id:
        :param units: Use 'C' for Celsius or 'F' for Fahrenheit.
        """

        units = units.upper()
        if units not in {'C', 'F'}:
            raise ValueError(f'Units must be either "C" or "F"; got "{units}".')

        response = self._send_and_receive_packet(servo_id, 26)

        # This is initially in Celsius
        temp = float(response.parameters[0])

        # Convert to Fahrenheit?
        if units == 'F':
            temp = (temp * 9 / 5) + 32

        return temp

    def vin_read(self, servo_id: int) -> float:
        """Reads the input voltage to the servo."""

        response = self._send_and_receive_packet(servo_id, 27)

        vin_mv = self._SIGNED_SHORT_STRUCT.unpack(response.parameters)[0]
        return vin_mv / 1000

    def pos_read(self, servo_id: int) -> float:
        """Reads the servo angle, in degrees. May be negative."""

        response = self._send_and_receive_packet(servo_id, 28)

        angle = self._SIGNED_SHORT_STRUCT.unpack(response.parameters)[0]
        return angle * 240 / 1000

    def set_all_powered(self, powered: bool) -> None:
        """Sets all six servos to be [un]powered."""

        for servo_id in self.ALL_SERVO_IDS:
            self.set_powered(servo_id, powered)

    def set_powered(self, servo_id: int, powered: bool) -> None:
        self._send_packet(servo_id, 31, b'\x01' if powered else b'\x00')


class XarmException(Exception):
    pass


def main():
    with Xarm(r'/dev/ttyACM\d+') as arm:
        arm.led_ctrl_write_all(False)
        print(f'temp = {arm.temp_read(2)}')
        print(f'vin = {arm.vin_read(2)}')
        arm.move_time_write(2, 90, 1)
        import time
        time.sleep(1)
        arm.move_time_write(2, 180, 10)
        time.sleep(5)
        arm.move_stop(2)
        arm.led_ctrl_write_all(True)

        while True:
            try:
                servo_id, angle, time = input('(ID, angle [deg], time [s]): ').split(',')
                servo_id, angle, time = int(servo_id), float(angle), float(time)
            except KeyboardInterrupt:
                print()
                break
            except Exception as e:
                print(e)
                continue

            print('Sending command...')
            arm.move_time_write(servo_id, angle, time)
            print('Done.')


if __name__ == '__main__':
    main()
