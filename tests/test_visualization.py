"""Check display timing and prevent streamed simulator processes from being orphaned."""
import os
from pathlib import Path
import selectors
import signal
import socket
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
from visualization.runtime import FramePacer
from visualization.launcher import check_port_available, run_process


class VisualizationChecks(unittest.TestCase):
    def test_slow_frame_does_not_add_delay(self):
        pacer = FramePacer(True)
        with patch('visualization.runtime.time.perf_counter', side_effect=[1.0, 1.1]), \
                patch('visualization.runtime.time.sleep') as sleep:
            pacer.start()
            pacer.wait(0.02)
        sleep.assert_called_once_with(0.0)

    def test_headless_pacing_does_not_sleep(self):
        with patch('visualization.runtime.time.sleep') as sleep:
            FramePacer(False).wait(0.02)
        sleep.assert_not_called()

    def test_occupied_port_rejected(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.bind(('127.0.0.1', 0))
            server.listen()
            with self.assertRaisesRegex(ValueError, 'Stop the existing Isaac Sim'):
                check_port_available(server.getsockname()[1])

    def test_child_failure_is_returned(self):
        with tempfile.TemporaryDirectory() as directory:
            result = run_process([sys.executable, '-c', 'raise SystemExit(7)'],
                                 cwd=Path(directory), environment=os.environ.copy())
        self.assertEqual(result, 7)

    def test_termination_reaps_unresponsive_simulator(self):
        child_code = (
            'import os, signal, time; '
            'signal.signal(signal.SIGTERM, signal.SIG_IGN); '
            'print("READY:" + str(os.getpid()), flush=True); time.sleep(60)'
        )
        launcher_code = (
            'from pathlib import Path; import os, sys; '
            'from visualization.launcher import run_process; '
            f'raise SystemExit(run_process([sys.executable, "-u", "-c", {child_code!r}], '
            'cwd=Path.cwd(), environment=os.environ.copy(), shutdown_timeout=0.2))'
        )
        environment = os.environ.copy()
        environment['PYTHONPATH'] = str(SCRIPTS)
        launcher = subprocess.Popen([sys.executable, '-u', '-c', launcher_code],
                                    env=environment, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        child_pid = None
        try:
            with selectors.DefaultSelector() as selector:
                selector.register(launcher.stdout, selectors.EVENT_READ)
                self.assertTrue(selector.select(timeout=5), 'Simulator did not start')
            line = launcher.stdout.readline().strip()
            self.assertTrue(line.startswith('READY:'), line)
            child_pid = int(line.split(':')[1])
            launcher.send_signal(signal.SIGTERM)
            _, errors = launcher.communicate(timeout=5)
            self.assertEqual(launcher.returncode, 128 + signal.SIGTERM, errors)
            with self.assertRaises(ProcessLookupError):
                os.kill(child_pid, 0)
        finally:
            if child_pid is not None:
                try:
                    os.killpg(child_pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            if launcher.poll() is None:
                launcher.kill()
            launcher.communicate(timeout=5)


if __name__ == '__main__':
    unittest.main()
