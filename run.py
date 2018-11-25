import signal
import sys

from bwo.bwo import Bwo, BwoMode
from bwo.events import send_shutdown_command


def main() -> int:
    def handle_sigint(signal_number, stack_frame):
        assert signal_number == signal.SIGINT
        print(f'Received SIGINT; shutting down.')
        send_shutdown_command()

    signal.signal(signal.SIGINT, handle_sigint)

    with Bwo(BwoMode.CAT_SEEKER) as bwo:
        bwo.wait_for_shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
