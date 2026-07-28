#!/usr/bin/env python3
"""Capture serial output from the S3 for N seconds and echo it.

Usage: capture.py [port] [seconds]

Stdlib-only (termios) so it works without pyserial. USB-Serial-JTAG ignores
baud settings. Retries opening the port briefly because the device
re-enumerates after flashing/reset.
"""

import os
import select
import sys
import termios
import time

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem1101"
seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0

fd = None
deadline_open = time.time() + 10.0
last_err = None
while time.time() < deadline_open:
    try:
        fd = os.open(port, os.O_RDONLY | os.O_NONBLOCK | os.O_NOCTTY)
        break
    except OSError as e:
        last_err = e
        time.sleep(0.3)
if fd is None:
    print(f"capture: FAILED to open {port}: {last_err}", file=sys.stderr)
    sys.exit(1)

attrs = termios.tcgetattr(fd)
# cfmakeraw equivalent: no input/output processing.
attrs[0] = 0  # iflag
attrs[1] = 0  # oflag
attrs[3] = 0  # lflag
termios.tcsetattr(fd, termios.TCSANOW, attrs)

end = time.time() + seconds
buf = b""
while time.time() < end:
    r, _, _ = select.select([fd], [], [], 0.2)
    if r:
        try:
            chunk = os.read(fd, 4096)
            if chunk:
                buf += chunk
        except OSError:
            time.sleep(0.1)
os.close(fd)
sys.stdout.write(buf.decode("utf-8", errors="replace"))
