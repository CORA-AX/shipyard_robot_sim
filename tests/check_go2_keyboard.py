#!/usr/bin/env python3
"""Exercise the three keyboard CLIs through real PTYs, TCP/DDS and optional GPU physics.

Use --transport-only for the control client and ROS command publisher without a GPU.
The default also starts ros.py --headless --keyboard and verifies input/clock/cleanup.
"""
import argparse
import json
import os
from pathlib import Path
import pty
import subprocess
import sys
import termios
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from control.control_channel import ControlServer
from control.go2.policy import CONTRACT, JOINT_NAMES
from control.go2.ros_bridge import Go2RosBridge, ros_environment


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--transport-only', action='store_true')
    parser.add_argument('--test-speed', type=float, default=.4)
    parser.add_argument('--test-turn-speed', type=float, default=.5)
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if not args.worker:
        environment = ros_environment()
        environment.update(ROS_DOMAIN_ID='82', PYTHONUNBUFFERED='1')
        os.execve(sys.executable, [sys.executable, str(Path(__file__).resolve()),
                                  *sys.argv[1:], '--worker'], environment)
    out = ROOT / '.runtime/go2_keyboard'
    out.mkdir(parents=True, exist_ok=True)
    results = {}

    def exercise(name, arguments, tick, get_command, *, ready='GO2_KEYBOARD_READY',
                 simulator=False):
        master, slave = pty.openpty()
        original = termios.tcgetattr(slave)
        log_path = out / f'{name}.log'
        report = out / f'{name}.json'
        report.unlink(missing_ok=True)
        with log_path.open('w') as log:
            process = subprocess.Popen([sys.executable, *arguments, '--keyboard-speed', str(args.test_speed),
                                        '--keyboard-turn-speed', str(args.test_turn_speed)], cwd=ROOT,
                                       stdin=slave, stdout=log, stderr=subprocess.STDOUT)

            def pump(duration):
                until = time.monotonic() + duration
                while time.monotonic() < until:
                    tick()
                    time.sleep(.005)

            def hold(key, command):
                seen = False
                until = time.monotonic() + .5
                next_key = 0.
                while time.monotonic() < until:
                    if time.monotonic() >= next_key:
                        os.write(master, key)
                        next_key = time.monotonic() + .08
                    tick()
                    seen |= get_command() == command
                    time.sleep(.005)
                assert seen, f'{name}: key {key!r} did not produce {command}; see {log_path}'

            try:
                deadline = time.monotonic() + (180. if simulator else 15.)
                while ready not in log_path.read_text(errors='replace'):
                    assert process.poll() is None, f'{name} exited early; see {log_path}'
                    if time.monotonic() > deadline:
                        raise TimeoutError(f'{name}: readiness timeout; see {log_path}')
                    pump(.05)
                hold(b'w', (args.test_speed, 0., 0.))
                hold(b' ', (0., 0., 0.))
                hold(b'z', (0., 0., args.test_turn_speed))
                hold(b' ', (0., 0., 0.))
                if not simulator:
                    hold(b'a', (0., args.test_speed, 0.))
                pump(1.)
                assert get_command() == (0., 0., 0.), f'{name}: idle did not stop'
                os.write(master, b'q')
                deadline = time.monotonic() + 20.
                while process.poll() is None and time.monotonic() < deadline:
                    pump(.02)
                assert process.poll() == 0, f'{name}: Q did not exit successfully; see {log_path}'
                assert termios.tcgetattr(slave) == original, f'{name}: terminal settings not restored'
                result = {'passed': True, 'terminal_restored': True, 'stop_and_quit': True}
                if report.is_file():
                    data = json.loads(report.read_text())
                    assert data['input_mode'] == 'keyboard' and data.get('stopped_by_user'), data
                    if simulator:
                        assert data['passed'] and data['command_observation_matches'], data
                        assert data['received_commands'] == 0, data
                        assert data['clock_messages'] == data['control_steps'] + 1, data
                        result.update(control_steps=data['control_steps'], resets=data['resets'])
                results[name] = result
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=15.)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                os.close(master)
                os.close(slave)

    velocity = [0., 0., 0.]
    def update(command):
        velocity[:] = [command['vx'], command.get('vy', 0.), command.get('wz', 0.)]
    server = ControlServer(0, 'go2', on_configure=update, on_command=update)
    step = 0
    def control_tick():
        nonlocal step
        observation = [0.] * 48
        observation[8] = -1.
        observation[9:12] = velocity
        action = server.poll({'step': step, 'episode': 0, 'contract': CONTRACT,
            'action_size': 12, 'joint_names': JOINT_NAMES, 'step_dt': .02,
            'observation': observation, 'state': {}})
        if action is not None:
            step += 1
    try:
        exercise('control', ['scripts/go2/control.py', '--keyboard', '--control-port', str(server.port),
                            '--report', str(out / 'control.json')],
                 control_tick, lambda: tuple(velocity))
        assert server.last_reason == 'controller_paused', server.last_reason
        assert server.rejections == 0, server.rejections
    finally:
        server.close()

    bridge = Go2RosBridge()
    try:
        exercise('ros_command', ['scripts/go2/ros_command.py', '--keyboard', '--ros-domain-id', '82'],
                 bridge.poll, bridge.command.current)
    finally:
        bridge.close()

    if not args.transport_only:
        def simulated_command():
            lines = (out / 'ros.log').read_text(errors='replace').splitlines()
            for line in reversed(lines):
                if line.startswith('GO2_ROS_COMMAND '):
                    return tuple(json.loads(line.split(' ', 1)[1])['command'])
            return None
        exercise('ros', ['scripts/go2/ros.py', '--keyboard', '--scenario', 'flat', '--headless',
                         '--ros-domain-id', '83', '--report', str(out / 'ros.json')],
                 lambda: None, simulated_command, ready='GO2_ROS_READY', simulator=True)
    (out / 'checks.json').write_text(json.dumps(results, indent=2) + '\n')
    print('GO2_KEYBOARD_CHECK_PASS ' + json.dumps(results), flush=True)


if __name__ == '__main__':
    main()
