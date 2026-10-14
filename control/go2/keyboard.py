"""Shared terminal teleoperation with a wall-time deadman and terminal restoration."""
import math
import os
import select
import sys
import termios
import time
import tty

DEFAULT_KEYBOARD_SPEED = 1.5  # m/s; shared by all three keyboard CLIs.
DEFAULT_KEYBOARD_TURN_SPEED = 1.5  # rad/s

HELP = ('GO2_KEYBOARD_READY: W/S forward/back; A/D left/right; Z/C turn left/right; '
        'arrows move; Space/X stop; Q quit; focus this terminal. '
        'Hold a key to repeat; idle input commands zero velocity.')


def add_keyboard_arguments(parser):
    parser.add_argument('--keyboard', action='store_true', help='Control movement from this terminal.')
    parser.add_argument('--keyboard-speed', type=float, default=DEFAULT_KEYBOARD_SPEED,
                        help='Movement speed in m/s (nonnegative, no upper limit).')
    parser.add_argument('--keyboard-turn-speed', type=float, default=DEFAULT_KEYBOARD_TURN_SPEED,
                        help='Turn speed in rad/s (nonnegative, no upper limit).')
    parser.add_argument('--keyboard-timeout', type=float, default=0.6,
                        help='Wall seconds without a movement key before stopping (default 0.6).')


def validate_keyboard_arguments(parser, args):
    if (not all(math.isfinite(v) for v in (args.keyboard_speed, args.keyboard_turn_speed, args.keyboard_timeout))
            or args.keyboard_speed < 0 or args.keyboard_turn_speed < 0
            or args.keyboard_timeout <= 0):
        parser.error('Use finite nonnegative keyboard speeds and a finite positive keyboard timeout.')
    if args.keyboard and not sys.stdin.isatty():
        parser.error('--keyboard requires an interactive terminal (SSH: allocate a TTY).')


class KeyboardCommand:
    def __init__(self, speed=DEFAULT_KEYBOARD_SPEED, turn_speed=DEFAULT_KEYBOARD_TURN_SPEED,
                 timeout=0.6, clock=time.monotonic):
        self.speed, self.turn_speed, self.timeout, self.clock = speed, turn_speed, timeout, clock
        self.values = (0.0, 0.0, 0.0)
        self.received_at = None
        self.events = 0
        self.quit = False
        self.escape = ''

    @property
    def stale(self):
        return self.received_at is None or self.clock() - self.received_at >= self.timeout

    def current(self):
        return (0.0, 0.0, 0.0) if self.quit or self.stale else self.values

    def feed(self, text):
        for key in text:
            if self.quit:
                break
            if self.escape:
                self.escape += key
                if len(self.escape) == 2:
                    if key not in '[O':
                        self.escape = ''
                    continue
                if '@' <= key <= '~':
                    key = {'A': 'w', 'B': 's', 'C': 'd', 'D': 'a'}.get(key, '')
                    self.escape = ''
                elif len(self.escape) > 16:
                    self.escape = ''
                    continue
                else:
                    continue
            elif key == '\x1b':
                self.escape = key
                continue
            key = key.lower()
            directions = {'w': (self.speed, 0., 0.), 's': (-self.speed, 0., 0.),
                          'a': (0., self.speed, 0.), 'd': (0., -self.speed, 0.),
                          'z': (0., 0., self.turn_speed), 'c': (0., 0., -self.turn_speed),
                          ' ': (0., 0., 0.), 'x': (0., 0., 0.), 'q': (0., 0., 0.)}
            if key not in directions:
                continue
            self.values = directions[key]
            self.received_at = self.clock()
            self.events += 1
            self.quit = key == 'q'


class TerminalKeyboard:
    def __init__(self, args, stream=None):
        self.enabled = args.keyboard
        self.stream = sys.stdin if stream is None else stream
        self.state = KeyboardCommand(args.keyboard_speed, args.keyboard_turn_speed, args.keyboard_timeout)
        self.fd = self.saved = None

    def __enter__(self):
        if self.enabled:
            if not self.stream.isatty():
                raise ValueError('--keyboard requires an interactive terminal.')
            self.fd = self.stream.fileno()
            self.saved = termios.tcgetattr(self.fd)
            try:
                tty.setcbreak(self.fd, termios.TCSANOW)
                termios.tcflush(self.fd, termios.TCIFLUSH)
            except BaseException:
                termios.tcsetattr(self.fd, termios.TCSANOW, self.saved)
                self.saved = None
                raise
            print(HELP, flush=True)
        return self

    def poll(self):
        if self.enabled and select.select([self.fd], [], [], 0)[0]:
            data = os.read(self.fd, 4096)
            self.state.feed(data.decode('ascii', errors='ignore') if data else 'q')
        return self.state.current()

    @property
    def quit(self):
        return self.state.quit

    def __exit__(self, *_):
        if self.saved is not None:
            termios.tcsetattr(self.fd, termios.TCSANOW, self.saved)
            self.saved = None
