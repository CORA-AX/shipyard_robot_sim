#!/usr/bin/env python3
"""Attach a standalone Go2 policy to a robot spawned by scripts/viewer.py."""
import argparse
import json
from pathlib import Path
import socket
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from control.control_channel import PROTOCOL, ProtocolError, encode, receive, vector
from control.go2.policy import CONTRACT, JOINT_NAMES, FlatPolicy
from control.go2.keyboard import TerminalKeyboard, add_keyboard_arguments, validate_keyboard_arguments


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-id', default='go2')
    parser.add_argument('--control-port', type=int, default=8765)
    parser.add_argument('--vx', type=float, default=0.4, help='Forward velocity command, 0.2 to 0.8 m/s.')
    parser.add_argument('--device', default='cpu', help='Policy inference device; independent of viewer GPU.')
    parser.add_argument('--checkpoint', type=Path,
                        help='Override the selected control checkpoint; defaults to the project manifest.')
    parser.add_argument('--steps', type=int, default=0, help='Applied control steps; 0 runs until stopped.')
    parser.add_argument('--connect-timeout', type=float, default=180.0)
    parser.add_argument('--response-timeout', type=float, default=30.0)
    parser.add_argument('--report', type=Path)
    add_keyboard_arguments(parser)
    args = parser.parse_args()
    validate_keyboard_arguments(parser, args)
    if (args.steps < 0 or not 1 <= args.control_port <= 65535 or args.connect_timeout <= 0
            or args.response_timeout <= 0 or (not args.keyboard and not 0.2 <= args.vx <= 0.8)):
        parser.error('Use a valid port, nonnegative steps, positive timeouts; constant --vx must be 0.2 to 0.8.')
    report = {'passed': False, 'robot_id': args.robot_id, 'applied_steps': 0,
              'commanded_forward_m_s': 0.0 if args.keyboard else args.vx,
              'input_mode': 'keyboard' if args.keyboard else 'constant'}
    sock = None
    try:
        policy = FlatPolicy(ROOT, args.device, checkpoint=args.checkpoint)
        report['policy'] = {'path': str(policy.checkpoint_path), 'sha256': policy.checkpoint_sha256,
                            'iteration': policy.iteration}
        print('GO2_POLICY_LOADED ' + json.dumps(report['policy']), flush=True)
        deadline = time.monotonic() + args.connect_timeout
        while True:
            try:
                sock = socket.create_connection(('127.0.0.1', args.control_port), timeout=1.0)
                break
            except OSError:
                if time.monotonic() >= deadline:
                    raise TimeoutError('Viewer is not listening; start viewer.py --robot go2 first.')
                time.sleep(0.2)
        sock.settimeout(args.connect_timeout)  # First observation may wait for scene loading.
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        with sock, sock.makefile('rb') as stream, TerminalKeyboard(args) as keyboard:
            first_step = expected_step = session = None
            configured = False
            previous_command = None
            while True:
                message = receive(stream)
                sock.settimeout(args.response_timeout)
                if message.get('protocol') != PROTOCOL or message.get('robot_id') != args.robot_id:
                    raise ProtocolError('Controller/robot protocol mismatch.')
                if session is None:
                    session = message.get('session')
                    first_step = expected_step = message.get('step')
                    if not isinstance(session, str) or type(first_step) is not int:
                        raise ProtocolError('Invalid session or step.')
                if message.get('session') != session or message.get('step') != expected_step:
                    raise ProtocolError('Unexpected session or observation step.')
                report['applied_steps'] = expected_step - first_step
                report['last_state'] = message.get('state')
                if message.get('type') == 'hello':
                    if configured or message.get('contract') != CONTRACT:
                        raise ProtocolError('Unexpected controller handshake.')
                    if args.keyboard and not message.get('runtime_commands'):
                        raise ProtocolError('Restart viewer.py to enable keyboard command updates.')
                    reply = {k: message[k] for k in ('protocol', 'robot_id', 'session', 'step', 'episode')}
                    sock.sendall(encode({**reply, 'type': 'configure',
                                         'command': {'vx': 0.0 if args.keyboard else args.vx}}))
                    configured = True
                    continue
                if not configured:
                    raise ProtocolError('Viewer did not request controller configuration.')
                if message.get('type') == 'done':
                    if args.steps and report['applied_steps'] != args.steps:
                        raise ProtocolError('Viewer stopped before the requested control steps completed.')
                    report['passed'] = True
                    break
                if (message.get('type') != 'observation' or message.get('contract') != CONTRACT
                        or message.get('joint_names') != JOINT_NAMES
                        or message.get('action_size') != 12 or message.get('step_dt') != 0.02):
                    raise ProtocolError('Viewer observation/action contract is incompatible with the flat policy.')
                observation = vector(message.get('observation'), 48)
                reply = {k: message[k] for k in ('protocol', 'robot_id', 'session', 'step', 'episode')}
                command = keyboard.poll() if args.keyboard else None
                if args.keyboard and keyboard.quit:
                    sock.sendall(encode({**reply, 'type': 'pause'}))
                    report['stopped_by_user'] = True
                    report['passed'] = not args.steps
                    break
                if args.steps and report['applied_steps'] >= args.steps:
                    sock.sendall(encode({**reply, 'type': 'pause'}))
                    report['passed'] = True
                    break
                if expected_step == first_step:
                    print(f'GO2_CONTROL_CONNECTED: {args.robot_id}, step={first_step}, device={args.device}', flush=True)
                payload = {**reply, 'type': 'action'}
                if command is not None:
                    # The viewer applies this command with the same action, so both
                    # policy inference and physics use the new command this step.
                    observation[9:12] = list(command)
                    payload['command'] = dict(zip(('vx', 'vy', 'wz'), command))
                    if command != previous_command:
                        print('GO2_KEYBOARD_COMMAND ' + json.dumps(list(command)), flush=True)
                        previous_command = command
                    report['last_command'] = list(command)
                    report['commanded_forward_m_s'] = command[0]
                payload['actions'] = policy.actions(observation)
                sock.sendall(encode(payload))
                expected_step += 1
        print('GO2_CONTROL_COMPLETE ' + json.dumps(report), flush=True)
    except KeyboardInterrupt:
        report['stopped_by_user'] = True
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
        raise
    finally:
        if sock is not None:
            sock.close()
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
