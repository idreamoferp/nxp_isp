import signal
import sys

from . import nxpprog


if __name__ == '__main__':
    signal.signal(signal.SIGINT, )
    sys.exit(nxpprog.main())
