"""Keyboard mappings, deadman and actual POSIX terminal restoration."""
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

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from control.go2.keyboard import KeyboardCommand, TerminalKeyboard


class KeyboardTests(unittest.TestCase):
    def setUp(self):
        self.now = 1.0
        self.command = KeyboardCommand(speed=.4, turn_speed=.5, clock=lambda: self.now)

    def test_directions_and_immediate_stop(self):
        for key, expected in [('w', (.4, 0., 0.)), ('s', (-.4, 0., 0.)),
                              ('a', (0., .4, 0.)), ('d', (0., -.4, 0.)),
                              ('z', (0., 0., .5)), ('c', (0., 0., -.5))]:
            self.command.feed(key.upper())
            self.assertEqual(self.command.current(), expected)
            self.command.feed(' ')
            self.assertEqual(self.command.current(), (0., 0., 0.))
        self.command.feed('wx')
        self.assertEqual(self.command.current(), (0., 0., 0.))

    def test_repeat_refreshes_and_unknown_keys_do_not_refresh_deadman(self):
        self.assertTrue(self.command.stale)
        self.command.feed('w')
        self.now += .5
        self.command.feed('w')
        self.now += .5
        self.assertEqual(self.command.current(), (.4, 0., 0.))
        self.command.feed('1\n')
        self.now += .11
        self.assertTrue(self.command.stale)
        self.assertEqual(self.command.current(), (0., 0., 0.))

    def test_quit_clears_velocity_and_ignores_queued_motion(self):
        self.command.feed('wqw')
        self.assertTrue(self.command.quit)
        self.assertEqual(self.command.current(), (0., 0., 0.))

    def test_split_arrow_sequences_and_other_escape_sequences(self):
        self.command.feed('\x1b[')
        self.assertEqual(self.command.current(), (0., 0., 0.))
        self.command.feed('A')
        self.assertEqual(self.command.current(), (.4, 0., 0.))
        self.command.feed('\x1b[B')
        self.assertEqual(self.command.current(), (-.4, 0., 0.))
        self.command.feed('\x1b[C')
        self.assertEqual(self.command.current(), (0., -.4, 0.))
        self.command.feed('\x1b[D')
        self.assertEqual(self.command.current(), (0., .4, 0.))
        before = self.command.events
        self.command.feed('\x1b[3~')
        self.assertEqual(self.command.events, before)

    def test_real_pty_reads_without_enter_and_restores_after_exception(self):
        master, slave = pty.openpty()
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        original = termios.tcgetattr(slave)
        args = SimpleNamespace(keyboard=True, keyboard_speed=.3, keyboard_turn_speed=.4,
                               keyboard_timeout=.6)
        with os.fdopen(os.dup(slave)) as stream, contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError, 'test cleanup'):
                with TerminalKeyboard(args, stream=stream) as keyboard:
                    self.assertFalse(termios.tcgetattr(slave)[3] & termios.ICANON)
                    os.write(master, b'w')
                    deadline = time.monotonic() + .5
                    while keyboard.poll() == (0., 0., 0.) and time.monotonic() < deadline:
                        time.sleep(.005)
                    self.assertEqual(keyboard.poll(), (.3, 0., 0.))
                    raise RuntimeError('test cleanup')
        self.assertEqual(termios.tcgetattr(slave), original)


if __name__ == '__main__':
    unittest.main()
