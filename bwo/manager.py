""""""

from threading import Event, Thread
from time import time
from typing import Optional


class ManagerThread(Thread):
    """A thread that tries to execute main() once every LOOP_PERIOD_S seconds."""

    LOOP_PERIOD_S = 0.0

    def __init__(self, *args, **kwargs):
        """Takes the same arguments as Thread()."""
        Thread.__init__(self, *args, **kwargs)

        self._disable_loop_period_warning = False
        self._stop_event = Event()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    def log(self, message: str):
        print(f'[{self.name}] {message}')

    def log_error(self, message: str):
        self.log(f'ERROR: {message}')

    def log_warning(self, message: str):
        self.log(f'WARNING: {message}')

    def main(self):
        raise NotImplementedError(f'{self.__class__}.main is not implemented.')

    def run(self):
        if self.LOOP_PERIOD_S:
            stop_timeout = 0.0
            while not self._stop_event.wait(stop_timeout):
                t0 = time()

                self.main()

                # Calculate stop event timeout to maintain the loop period
                stop_timeout = self.LOOP_PERIOD_S - (time() - t0)
                if stop_timeout <= 0:
                    if not self._disable_loop_period_warning:
                        self.log_warning(f'Loop period has fallen below {self.LOOP_PERIOD_S}s.')
                    stop_timeout = 0
        else:
            while not self._stop_event.is_set():
                self.main()

    def start(self):
        self.log('Starting...')
        self._stop_event.clear()
        Thread.start(self)
        self.log('Done.')

    def stop(self, timeout: Optional[float] = None):
        self.log('Stopping...')
        self._stop_event.set()
        self.join(timeout=timeout)
        self.log('Done.')
