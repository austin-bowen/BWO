"""
Tracks the orientation of BWO.
"""
from math import cos, radians, sin

from drive_motor_control import DriveMotorController, DriveMotorState, ticks_to_distance, differential_to_unicycle
from utils import normalize_angle_degrees


class Orientation:
    def __init__(self) -> None:
        self.x = self.y = 0.0
        self.heading = 90

        self.prev_drive_motor_state = None

    def __str__(self) -> str:
        return f'[x={self.x:.1f}, y={self.y:.1f}, heading={int(round(self.heading))}°]'

    def drive_motor_state_change_callback(self, drive_motors: DriveMotorController, state: DriveMotorState):
        if self.prev_drive_motor_state is None:
            self.prev_drive_motor_state = state
            return
        prev_state = self.prev_drive_motor_state

        dt = state.timestamp - prev_state.timestamp
        d_left_motor_position = ticks_to_distance(state.left_motor_position - prev_state.left_motor_position)
        d_right_motor_position = ticks_to_distance(state.right_motor_position - prev_state.right_motor_position)

        v, w = differential_to_unicycle(d_left_motor_position / dt, d_right_motor_position / dt)

        d_heading = w * dt
        distance = v * dt
        # TODO: This is not a perfectly accurate calculation; update to use arc calculations.
        theta = self.heading + (d_heading / 2)
        dx = distance * cos(radians(theta))
        dy = distance * sin(radians(theta))

        self.heading = normalize_angle_degrees(self.heading + d_heading)
        self.x += dx
        self.y += dy

        self.prev_drive_motor_state = state
