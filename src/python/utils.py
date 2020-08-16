from typing import Optional

from serial.tools.list_ports import comports
from serial.tools.list_ports_common import ListPortInfo


def get_serial_port_with_hwid(hwid: str) -> Optional[ListPortInfo]:
    """
    :return: The first serial port (ListPortInfo instance) with a matching hwid, or None if not found.
    """

    for serial_port in comports():
        if serial_port.hwid == hwid:
            return serial_port

    return None


def print_serial_ports() -> None:
    print('Serial Ports:')

    ports = comports()
    if not ports:
        print('[None]')
        return

    for p in sorted(ports):
        print(f'name={p.name!r}, device={p.device!r}, description={p.description!r}, hwid={p.hwid!r}')


def normalize_angle_degrees(angle):
    """
    :return: The angle normalized to (-180, 180] degrees.
    """

    angle = angle % 360
    return angle if angle <= 180 else (angle - 360)
