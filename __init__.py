import signal
import sys

import nxpprog


if __name__ == '__main__':
    signal.signal(signal.SIGINT, nxpprog.signal_handler)
    sys.exit(nxpprog.main())
