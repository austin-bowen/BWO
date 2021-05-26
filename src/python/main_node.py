import random; random.seed()
from easybot.node import Node


EXPLETIVES = [
    'Ouch!',
    'Oof!',
    'Fooey!',
    'Drat!',
    'Woops!',
    'Dag nabbit!',
    'Oh dear!',
    'Excuse me!',
    'Pardon me!',
    'Watch where you\'re going!'
]


class MainNode(Node):
    def __init__(self) -> None:
        super().__init__('Main', 5)

        self.first_run = True
        self.prev_drive_motor_state = None
        self.prev_expletive = None

        self.subscribe('drive_motors.state', self.handle_drive_motor_state)

    def loop(self) -> None:
        if self.first_run:
            self.first_run = False
            #self.publish('tts', 'I have started up.')

    def handle_drive_motor_state(self, message) -> None:
        motor_state = message.data

        if any_bumper_pressed(motor_state):
            if self.prev_drive_motor_state is not None \
                    and not any_bumper_pressed(self.prev_drive_motor_state):
                self.publish('tts', self.choose_expletive())

        self.prev_drive_motor_state = motor_state

    def choose_expletive(self) -> str:
        expletive = self.prev_expletive
        while expletive == self.prev_expletive:
            expletive = random.choice(EXPLETIVES)
        self.prev_expletive = expletive
        return expletive


def any_bumper_pressed(motor_state) -> bool:
    return motor_state.left_bumper \
            or motor_state.middle_bumper \
            or motor_state.right_bumper

