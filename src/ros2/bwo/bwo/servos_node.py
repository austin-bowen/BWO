"""
Published topics:
- ...
"""

import rclpy

from maestro import MicroMaestro
from rclpy.node import Node



class ServosNode(Node):
    TILT_SERVO = 0
    PAN_SERVO = 1

    _centers = {TILT_SERVO: 1450, PAN_SERVO: 1535}

    def __init__(self):
        super().__init__('servos')






class ServosNode(Node):
    TILT_SERVO = 0
    PAN_SERVO = 1

    _centers = {TILT_SERVO: 1450, PAN_SERVO: 1535}

    def __init__(self):
        super().__init__('Servos', 5)

        self._targets = dict(self._centers)
        self._targets_changed = Event()
        self._targets_changed.set()

        self.subscribe('servos.increment_targets', self._handle_increment_targets)

        self.subscribe('remote_control.right_joystick_x', self._handle_right_joystick_x)
        self.subscribe('remote_control.right_joystick_y', self._handle_right_joystick_y)
        self.subscribe('remote_control.x_button', self._handle_x_button)
        self.subscribe('remote_control.y_button', self._handle_y_button)

    def loop(self) -> None:
        self.log('Searching for Maestro...')
        try:
            serial_port = next(grep_serial_ports(r'USB VID:PID=1FFB:0089 SER=00233827 LOCATION=.+\.0'))
        except StopIteration:
            self.log_warning('Failed to find Maestro.')
            return

        self.log('Connecting...')
        try:
            with MicroMaestro(tty=serial_port.device) as servo_control:
                self.log('Connected.')

                servo_control.set_acceleration(0, 0)
                servo_control.set_acceleration(1, 0)
                #servo_control.set_speed(0, 50)
                #servo_control.set_speed(1, 50)
                servo_control.set_speed(0, 25)
                servo_control.set_speed(1, 25)

                while not self._stop_flag.wait(timeout=0.1):
                    if self._targets_changed.is_set():
                        self._targets_changed.clear()
                        servo_control.set_targets(self._targets)

                    for channel in sorted(self._targets.keys()):
                        topic = f'servos.channel.{channel}.position'
                        #data = servo_control.get_position(channel)
                        data = self._targets[channel]
                        self.publish(topic, data)

                # Put the head down
                targets = dict(self._centers)
                targets[0] -= 500
                servo_control.set_targets(targets)
                time.sleep(1)
        finally:
            self.log('Disconnected.')

    def _handle_increment_targets(self, message: Message) -> None:
        for channel, delta in message.data.items():
            target = self._targets[channel] + delta
            center = self._centers[channel]
            target = min(max(center - 500, target), center + 500)
            self._targets[channel] = target

        self._targets_changed.set()







def main(args=None) -> None:
    rclpy.init(args=args)

    object_detector = ObjectDetectorNode()

    try:
        rclpy.spin(object_detector)
    finally:
        object_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

