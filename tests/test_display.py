"""Verify local display ignores streaming environment and WebRTC is headless."""
import argparse
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from visualization.launcher import add_display_arguments, launch_display


class DisplayChecks(unittest.TestCase):
    def test_local_mode_needs_no_ip_and_overrides_headless_environment(self):
        with patch.dict('os.environ', {'HEADLESS': '1', 'LIVESTREAM': '1'}), \
                patch('visualization.launcher.run_process', return_value=0) as process, \
                patch('visualization.launcher.check_port_available') as port:
            launch_display(['worker.py'], root=ROOT, robot='display_test', webrtc=False, server_ip=None)
        command = process.call_args.args[0]
        self.assertEqual(command[-2:], ['--livestream', '0'])
        self.assertNotIn('--headless', command)
        env = process.call_args.kwargs['environment']
        self.assertEqual((env['HEADLESS'], env['LIVESTREAM']), ('0', '0'))
        port.assert_not_called()

    def test_webrtc_is_headless_and_uses_streaming_experience(self):
        with patch('visualization.launcher.check_port_available'), \
                patch('visualization.launcher.run_webrtc', return_value=0) as stream:
            launch_display(['worker.py'], root=ROOT, robot='display_test',
                           webrtc=True, server_ip='127.0.0.1')
        command = stream.call_args.args[0]
        self.assertIn('--headless', command)
        self.assertIn('isaacsim.exp.full.streaming.kit', command)

    def test_viewer_defaults_local_and_explicit_modes_parse(self):
        parser = argparse.ArgumentParser()
        add_display_arguments(parser)
        self.assertFalse(parser.parse_args([]).webrtc)
        self.assertTrue(parser.parse_args(['--webrtc']).webrtc)
        self.assertFalse(parser.parse_args(['--no-webrtc']).webrtc)

    def test_webrtc_requires_server_address(self):
        with self.assertRaisesRegex(ValueError, 'PUBLIC_IP'):
            launch_display(['worker.py'], root=ROOT, robot='display_test', webrtc=True, server_ip=None)
