import gamesir
from ..events import send_controller_event
from ..manager import ManagerThread


class ControllerManager(ManagerThread):
    LOOP_PERIOD_S = 3

    def __init__(self):
        ManagerThread.__init__(self, name='Controller Manager')
        self._disable_loop_period_warning = True

    def main(self):
        # Get the controller
        try:
            controller = gamesir.get_controllers()[0]
        except IndexError:
            print('No controller connected.')
            return

        print('Controller connected!')

        with controller:
            # Clear any events that may have existed prior to connecting to the controller
            clear_events = True

            # Continuously read input events until manager is told to stop or controller is disconnected
            while not self._stop_event.wait(0.1):
                try:
                    # Read all current input events
                    for event in controller.read():
                        if clear_events:
                            continue

                        # Check if told to stop
                        if self._stop_event.is_set():
                            return

                        # Fire controller event!
                        if event.type in controller.EVENT_TYPES:
                            send_controller_event(event)
                except BlockingIOError:
                    # No events to read
                    clear_events = False
                except OSError:
                    # Controller disconnected
                    print('Controller disconnected!')
                    break
