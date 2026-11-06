"""Ackermann terminal arcs, reverse steering and deadman without key-up events."""
import contextlib
import io
import os
from pathlib import Path
import pty
import sys
import termios
import time
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent))
from hunter.control.keyboard import AckermannKeyboardCommand
from hunter.control.keyboard import TerminalKeyboard


class HunterKeyboardTests(unittest.TestCase):
    def setUp(self):
        self.now = 1.
        self.keys = AckermannKeyboardCommand(.5, .3, clock=lambda: self.now)

    def test_w_then_a_or_d_combines_forward_speed_with_steering(self):
        self.keys.feed('wa')
        self.assertEqual(self.keys.current(), (.5, 0., .3))
        self.keys.feed('d')
        self.assertEqual(self.keys.current(), (.5, 0., -.3))
        self.keys.feed('w')
        self.assertEqual(self.keys.current(), (.5, 0., 0.))

    def test_reverse_keeps_steering_direction_but_flips_yaw_sign(self):
        self.keys.feed('sa')
        self.assertEqual(self.keys.current(), (-.5, 0., -.3))
        self.keys.feed('d')
        self.assertEqual(self.keys.current(), (-.5, 0., .3))

    def test_turn_key_cannot_start_motion_after_idle_timeout_or_stop(self):
        self.keys.feed('a')
        self.assertEqual(self.keys.current(), (0., 0., 0.))
        self.keys.feed('wa')
        self.now += .7
        self.keys.feed('d')
        self.assertEqual(self.keys.current(), (0., 0., 0.))
        self.keys.feed('w a')
        self.assertEqual(self.keys.current(), (0., 0., 0.))
        self.keys.feed('waqa')
        self.assertTrue(self.keys.quit)
        self.assertEqual(self.keys.current(), (0., 0., 0.))

    def test_repeat_deadman_and_arrow_sequences(self):
        self.keys.feed('\x1b[A\x1b[')
        self.keys.feed('D')
        self.assertEqual(self.keys.current(), (.5, 0., .3))
        self.now += .5
        self.keys.feed('a')
        self.now += .5
        self.assertEqual(self.keys.current(), (.5, 0., .3))
        self.keys.feed('zc')  # Holonomic spin keys do not keep Hunter moving.
        self.now += .11
        self.assertEqual(self.keys.current(), (0., 0., 0.))

    def test_terminal_profile_reads_real_key_bytes_and_restores_terminal(self):
        master, slave = pty.openpty()
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        before = termios.tcgetattr(slave)
        args = SimpleNamespace(keyboard=True, keyboard_layout='ackermann', keyboard_speed=.5,
                               keyboard_turn_speed=.3, keyboard_timeout=.6)
        with os.fdopen(os.dup(slave)) as stream, contextlib.redirect_stdout(io.StringIO()):
            with TerminalKeyboard(args, stream=stream) as keyboard:
                os.write(master, b'wa')
                until = time.monotonic() + .5
                while keyboard.poll() == (0., 0., 0.) and time.monotonic() < until:
                    time.sleep(.005)
                self.assertEqual(keyboard.poll(), (.5, 0., .3))
        self.assertEqual(termios.tcgetattr(slave), before)


if __name__ == '__main__':
    unittest.main()
