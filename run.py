import sys

from bwo.bwo import Bwo, BwoMode


def main() -> int:
    with Bwo(BwoMode.REMOTE_CONTROL) as bwo:
        bwo.wait_for_shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
