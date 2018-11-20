"""Module of PyPubSub topics used within BWO."""

from evdev import InputEvent
from pubsub import pub

# Fired when controller events occur, such as button presses and joystick movement
CONTROLLER_EVENT = 'bwo.controller_event'

# Fired after acquiring a new image from the camera
NEW_CAMERA_IMAGE = 'bwo.new_camera_image'

# Fired when something wants to shutdown BWO
SHUTDOWN = 'bwo.shutdown'


def receive_controller_events(listener: callable):
    pub.subscribe(listener, CONTROLLER_EVENT)


def receive_shutdown_command(listener: callable):
    pub.subscribe(listener, SHUTDOWN)


def send_controller_event(event: InputEvent):
    pub.sendMessage(CONTROLLER_EVENT, event=event)


def send_shutdown_command():
    pub.sendMessage(SHUTDOWN)
