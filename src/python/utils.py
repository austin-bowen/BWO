"""Utilities."""

from serial.tools.list_ports import comports, grep as grep_serial_ports

__all__ = [
    'grep_serial_ports',
    'normalize_angle_degrees',
    'print_serial_ports'
]


def normalize_angle_degrees(angle):
    """
    :return: The angle normalized to (-180, 180] degrees.
    """

    angle = angle % 360
    return angle if angle <= 180 else (angle - 360)


def print_serial_ports() -> None:
    print('Serial Ports:')

    ports = comports()
    if not ports:
        print('[None]')
        return

    for p in sorted(ports):
        print(f'name={p.name!r}, device={p.device!r}, description={p.description!r}, hwid={p.hwid!r}')
