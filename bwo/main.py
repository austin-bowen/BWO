"""
BWO the robot.
"""

from collections import namedtuple

from pynput import keyboard

from bwo.io.motor import DriveMotorController
from maestro import Maestro

prev_velocity = None


def with_maestro(maestro: Maestro):
    FORWARD_KEY = 'w'
    BACKWARD_KEY = 's'
    LEFT_KEY = keyboard.Key.left
    RIGHT_KEY = keyboard.Key.right
    ROTATE_CW_KEY = 'd'
    ROTATE_CCW_KEY = 'a'

    XY_MAGNITUDE = 1
    OMEGA_MAGNITUDE = 1

    Velocity = namedtuple('Velocity', ('x', 'y', 'omega'))

    global prev_velocity
    prev_velocity = Velocity(0.0, 0.0, 0.0)

    motor_controller = DriveMotorController(maestro)

    def update_velocity(x, y, omega):
        print(x, y, omega)
        global prev_velocity
        new_velocity = Velocity(x, y, omega)

        if new_velocity == prev_velocity:
            print('No need to update velocity')

        print('New velocity:', new_velocity)
        motor_controller.set_body_velocity(x, y, omega)

        prev_velocity = new_velocity

    def on_key_press(key):
        global prev_velocity

        try:
            key = key.char
        except AttributeError:
            pass

        if key == FORWARD_KEY:
            print('FORWARD')
            update_velocity(prev_velocity.x, XY_MAGNITUDE, prev_velocity.omega)
        elif key == BACKWARD_KEY:
            print('BACKWARD')
            update_velocity(prev_velocity.x, -XY_MAGNITUDE, prev_velocity.omega)
        elif key == LEFT_KEY:
            print('LEFT')
            update_velocity(-XY_MAGNITUDE, prev_velocity.y, prev_velocity.omega)
        elif key == RIGHT_KEY:
            print('RIGHT')
            update_velocity(XY_MAGNITUDE, prev_velocity.y, prev_velocity.omega)
        elif key == ROTATE_CW_KEY:
            print('CW')
            update_velocity(prev_velocity.x, prev_velocity.y, OMEGA_MAGNITUDE)
        elif key == ROTATE_CCW_KEY:
            print('CCW')
            update_velocity(prev_velocity.x, prev_velocity.y, -OMEGA_MAGNITUDE)
        elif key == keyboard.Key.esc:
            return False

    def on_key_release(key):
        global prev_velocity

        try:
            key = key.char
        except AttributeError:
            pass

        if key == FORWARD_KEY or key == BACKWARD_KEY:
            update_velocity(prev_velocity.x, 0.0, prev_velocity.omega)
        elif key == LEFT_KEY or key == RIGHT_KEY:
            update_velocity(0.0, prev_velocity.y, prev_velocity.omega)
        elif key == ROTATE_CW_KEY or key == ROTATE_CCW_KEY:
            update_velocity(prev_velocity.x, prev_velocity.y, 0.0)

    with keyboard.Listener(on_press=on_key_press, on_release=on_key_release) as keyboard_listener:
        keyboard_listener.join()


def main():
    with Maestro(is_micro=True) as maestro:
        with_maestro(maestro)


if __name__ == '__main__':
    main()
