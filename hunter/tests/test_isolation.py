"""Exercise a relocated Hunter folder without the legacy project modules."""
import ast
from contextlib import redirect_stdout
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent))


class HunterIsolationTests(unittest.TestCase):
    def test_application_imports_do_not_use_legacy_packages(self):
        forbidden = {'robot_control', 'robot_sim', 'visualization', 'control', 'scripts', 'scenarios'}
        for path in ROOT.rglob('*.py'):
            if any(part in ('.runtime', '.cache', '__pycache__') for part in path.parts):
                continue
            for node in ast.walk(ast.parse(path.read_text())):
                if isinstance(node, ast.Import):
                    names = [alias.name for alias in node.names]
                elif isinstance(node, ast.ImportFrom) and node.level == 0:
                    names = [node.module or '']
                else:
                    continue
                for name in names:
                    self.assertNotIn(name.split('.')[0], forbidden, f'{path}: {name}')

    def test_relocated_folder_imports_configuration_and_entrypoints(self):
        with tempfile.TemporaryDirectory(prefix='hunter_isolation_') as directory:
            parent = Path(directory)
            destination = parent / 'hunter'
            shutil.copytree(ROOT, destination,
                            ignore=shutil.ignore_patterns('__pycache__', '.cache', '.runtime'))
            script = '''
import sys
from pathlib import Path
sys.path.insert(0, sys.argv[1])
import hunter
from hunter.control.controller import Hunter2Controller, load_parameters
from hunter.control.keyboard import AckermannKeyboardCommand
from hunter.control.commands import DriveCommand
from hunter.ros.bridge import RobotRosBridge
from hunter.ros.teleop import main
from hunter.sim.scene import prepare_hunter2_stage
from hunter.sim.runtime import arguments, validate_arguments
root = Path(hunter.__file__).parent
parameters = load_parameters(root)
assert (root / parameters['usd_path']).is_file()
assert (root / parameters['usd_path']).resolve().is_relative_to(root)
keys = AckermannKeyboardCommand(.5, .3)
keys.feed('wa')
targets = Hunter2Controller(parameters).update_drive(DriveCommand(*keys.current()), 10.)
assert targets.steering_positions[0] > targets.steering_positions[1] > 0.
parser = arguments(root)
args = parser.parse_args(['--ros'])
validate_arguments(parser, args, parameters)
assert args.ackermann_topic == '/hunter2_0/ackermann_cmd'
assert args.duration is None
assert not any(name.split('.')[0] in ('robot_control', 'robot_sim', 'visualization') for name in sys.modules)
print('Hunter imports and model path are independent of the old workspace.')
'''
            result = subprocess.run([sys.executable, '-I', '-c', script, str(parent)],
                                    cwd=parent, capture_output=True, text=True, timeout=20)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            for entry in ('simulate.py', 'teleop.py', 'waypoint.py', 'nav2.py', 'export_map.py', 'patrol.py'):
                result = subprocess.run([sys.executable, '-I', str(destination / 'scripts' / entry), '--help'],
                                        cwd=parent, capture_output=True, text=True, timeout=20)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn('usage:', result.stdout)

    def test_worker_launch_preserves_caller_relative_paths(self):
        from hunter.sim import environment, launcher, runtime

        commands = []

        def launch(command, *, cwd, environment):
            commands.append((command, cwd, environment))
            Path(command[command.index('--result-file') + 1]).write_text(json.dumps({'successful': True}))
            return 0

        with tempfile.TemporaryDirectory(prefix='hunter_launch_') as directory:
            caller = Path(directory)
            argv = ['simulate.py', '--ros', '--config', 'relative/robot.yaml']
            # Supply the real settings through the loader while checking that
            # a caller-relative CLI path survives the parent -> worker handoff.
            from hunter.control.controller import load_parameters
            parameters = load_parameters(ROOT)
            with (patch.object(sys, 'argv', argv),
                  patch.dict(os.environ, {'HUNTER_ISAAC_PREFIX': sys.prefix}),
                  patch('hunter.control.controller.load_parameters', return_value=parameters),
                  patch.object(environment, 'isaac_environment',
                               side_effect=lambda python, python_paths: {'PYTHONPATH': os.pathsep.join(map(str, python_paths))}),
                  patch.object(launcher, 'run_process', side_effect=launch),
                  patch.object(runtime.Path, 'cwd', return_value=caller),
                  redirect_stdout(io.StringIO()),
                  self.assertRaises(SystemExit) as stopped):
                runtime.main(ROOT)
            self.assertEqual(stopped.exception.code, 0)
            command, cwd, environment = commands[0]
            self.assertEqual(command[1], str(ROOT / 'scripts/simulate.py'))
            self.assertEqual(command[2:5], ['--ros', '--config', 'relative/robot.yaml'])
            self.assertEqual(cwd, caller)
            self.assertEqual(environment['PYTHONPATH'], str(ROOT.parent))


if __name__ == '__main__':
    unittest.main()
