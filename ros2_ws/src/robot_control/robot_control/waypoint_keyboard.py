"""Terminal selection controls; deliberately separate from velocity teleoperation."""
import os
import select
import sys
import termios
import time
import tty


class SelectionKeyboard:
    def __init__(self, enabled=True, stream=None):
        self.enabled = enabled
        self.stream = sys.stdin if stream is None else stream
        self.saved = self.fd = None
        self.last_space = -float('inf')

    def __enter__(self):
        if self.enabled:
            if not self.stream.isatty():
                raise ValueError('Waypoint requires a terminal (SSH: ssh -t); or use --no-keyboard.')
            self.fd = self.stream.fileno()
            self.saved = termios.tcgetattr(self.fd)
            try:
                tty.setcbreak(self.fd, termios.TCSANOW)
                termios.tcflush(self.fd, termios.TCIFLUSH)
            except BaseException:
                self.__exit__()
                raise
        return self

    def poll(self):
        if not self.enabled or not select.select([self.fd], [], [], 0)[0]:
            return []
        data = os.read(self.fd, 4096).decode('ascii', errors='ignore')
        events = []
        for key in data.lower() if data else 'q':
            if key == ' ' and time.monotonic() - self.last_space >= .3:
                self.last_space = time.monotonic()
                events.append('next')
            elif key in ('x', '\x1b'):
                events.append('cancel')
            elif key == 'q':
                events.append('quit')
        return events

    def __exit__(self, *_):
        if self.saved is not None:
            termios.tcsetattr(self.fd, termios.TCSANOW, self.saved)
            self.saved = None
