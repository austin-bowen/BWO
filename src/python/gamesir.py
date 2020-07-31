"""Library for interfacing with the GameSir Bluetooth controller."""

from enum import Enum
from typing import Sequence

import evdev


class GameSirController(evdev.InputDevice):
    EVENT_TYPES = (evdev.ecodes.EV_KEY, evdev.ecodes.EV_ABS)

    class EventCode(Enum):
        """
        Value meanings:
        - *_BUTTON    : 1 is pressed, 0 is not pressed.
        - DPAD_X      : -1 is left, +1 is right, 0 is neither pressed.
        - DPAD_Y      : -1 is up, +1 is down, 0 is neither pressed.
        - *_JOYSTICK_X: 0 is full left, 255 is full right, 128 is centered.
        - *_JOYSTICK_Y: 0 is full up, 255 is full down, 128 is centered.
        - *_BUMPER    : 1 is pressed, 0 is not pressed.
        - *_TRIGGER   : 1 is pressed, 0 is not pressed.
        - *_PRESSURE  : 0 is no pressure, 255 is full pressure.
        """
        A_BUTTON = evdev.ecodes.BTN_A
        B_BUTTON = evdev.ecodes.BTN_B
        X_BUTTON = evdev.ecodes.BTN_X
        Y_BUTTON = evdev.ecodes.BTN_Y

        SELECT_BUTTON = evdev.ecodes.BTN_SELECT
        START_BUTTON = evdev.ecodes.BTN_START

        DPAD_X = evdev.ecodes.ABS_HAT0X
        DPAD_Y = evdev.ecodes.ABS_HAT0Y

        LEFT_JOYSTICK_X = evdev.ecodes.ABS_X
        LEFT_JOYSTICK_Y = evdev.ecodes.ABS_Y
        LEFT_JOYSTICK_BUTTON = evdev.ecodes.BTN_THUMBL

        RIGHT_JOYSTICK_X = evdev.ecodes.ABS_Z
        RIGHT_JOYSTICK_Y = evdev.ecodes.ABS_RZ
        RIGHT_JOYSTICK_BUTTON = evdev.ecodes.BTN_THUMBR

        LEFT_BUMPER = evdev.ecodes.BTN_TL
        LEFT_TRIGGER = evdev.ecodes.BTN_TL2
        LEFT_TRIGGER_PRESSURE = evdev.ecodes.ABS_BRAKE

        RIGHT_BUMPER = evdev.ecodes.BTN_TR
        RIGHT_TRIGGER = evdev.ecodes.BTN_TR2
        RIGHT_TRIGGER_PRESSURE = evdev.ecodes.ABS_GAS

    def __init__(self, device_path: str):
        evdev.InputDevice.__init__(self, device_path)

        if 'Gamesir' not in self.name:
            self.close()
            raise ValueError('Input device at "{}" is not a GameSir controller.'.format(device_path))

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


def get_controllers() -> Sequence[GameSirController]:
    """Returns a list of discovered GameSirController instances."""
    controllers = []
    for device_path in evdev.list_devices():
        try:
            controllers.append(GameSirController(device_path))
        except ValueError:
            pass
    return controllers


def main():
    # Get Gamesir controller
    try:
        controller = get_controllers()[0]
    except IndexError:
        print('ERROR: Did not find a Gamesir device.')
        return

    for event in controller.read_loop():
        if event.type not in controller.EVENT_TYPES:
            continue

        print(GameSirController.EventCode(event.code), event.value)


if __name__ == '__main__':
    main()
