"""Ensure every streamed mode runs a project-owned scenario without importing Kit."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('go2_launcher', ROOT / 'scripts/go2/run_go2_webrtc.py')
launcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher)


class Go2LauncherChecks(unittest.TestCase):
    def test_all_modes_use_local_scenario_runner(self):
        for mode in ('view', 'policy', 'check'):
            with self.subTest(mode=mode):
                args = SimpleNamespace(mode=mode, scenario='shipyard', device='cuda:0',
                                       steps=100, vx=0.4, report=Path('/tmp/go2-test.json'))
                command = launcher.scenario_arguments(args, launcher.WebRTCConfig('127.0.0.1'))
                self.assertEqual(command[0], str(ROOT / 'scripts/go2/run_go2.py'))
                self.assertEqual(command[command.index('--scenario') + 1], 'shipyard')
                self.assertEqual(command[command.index('--mode') + 1], mode)
                self.assertEqual(command[command.index('--steps') + 1], '100')

    def test_demo_is_continuous_and_check_is_bounded_by_default(self):
        for mode in ('view', 'policy', 'check'):
            args = SimpleNamespace(mode=mode, scenario='flat', device='cuda:0',
                                   steps=None, vx=0.4, report=None)
            command = launcher.scenario_arguments(args, launcher.WebRTCConfig('127.0.0.1'))
            if mode == 'check':
                self.assertEqual(command[command.index('--steps') + 1], '1500')
            else:
                self.assertNotIn('--steps', command)


if __name__ == '__main__':
    unittest.main()
