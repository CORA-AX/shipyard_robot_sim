#!/usr/bin/env python3
"""Verify DDS command/clock transport, optionally with real headless Go2 physics.

Run in the project Conda environment. --transport-only needs no GPU. Full mode
launches ros.py, publishes movement/turn/stop/expiry commands, and inspects its report.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from control.go2.ros_bridge import Go2RosBridge, clock_qos, ros_environment
from scenarios.registry import GO2_SCENARIOS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--transport-only', action='store_true')
    parser.add_argument('--scenario', choices=GO2_SCENARIOS, default='flat')
    parser.add_argument('--ros-domain-id', type=int, default=79)
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if not args.worker:
        environment = ros_environment()
        environment['ROS_DOMAIN_ID'] = str(args.ros_domain_id)
        os.execve(sys.executable, [sys.executable, str(Path(__file__).resolve()),
                                  *sys.argv[1:], '--worker'], environment)
    from geometry_msgs.msg import Twist
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rosgraph_msgs.msg import Clock

    output = ROOT / '.runtime/go2_ros'
    output.mkdir(parents=True, exist_ok=True)
    context = Context()
    context.init(args=[])
    node = Node('shipyard_go2_ros_check', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    clocks = []
    subscription = node.create_subscription(Clock, '/clock', lambda m: clocks.append(
        m.clock.sec + m.clock.nanosec / 1e9), clock_qos())
    publisher = node.create_publisher(Twist, '/unitree_go2/cmd_vel', 1)
    bridge = process = log = None
    report_path = output / 'simulation_check.json'
    log_path = output / 'simulation_check.log'

    def spin_for(seconds, command=None):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            if command is not None:
                message = Twist()
                message.linear.x, message.linear.y, message.angular.z = command
                publisher.publish(message)
            if bridge is not None:
                bridge.poll()
            executor.spin_once(timeout_sec=0.01)
            time.sleep(0.01)

    try:
        if args.transport_only:
            bridge = Go2RosBridge(timeout=.2)
        else:
            report_path.unlink(missing_ok=True)
            log = log_path.open('w')
            process = subprocess.Popen([sys.executable, str(ROOT / 'scripts/go2/ros.py'),
                '--scenario', args.scenario, '--headless', '--steps', '900', '--command-timeout', '0.5',
                '--ros-domain-id', str(args.ros_domain_id), '--report', str(report_path)],
                cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 360.0
        while publisher.get_subscription_count() == 0 or node.count_publishers('/clock') == 0:
            if process is not None and process.poll() is not None:
                raise RuntimeError(f'ROS simulator exited early; see {log_path}')
            if time.monotonic() >= deadline:
                raise TimeoutError(f'ROS discovery failed; see {log_path}')
            executor.spin_once(timeout_sec=.05)
        if args.transport_only:
            spin_for(.5, (.4, -.3, .5))
            assert bridge.command.current() == (.4, -.3, .5)
            assert bridge.command.received > 0
            for timestamp in (0., .02, .04, .06):
                bridge.publish_clock(timestamp)
                spin_for(.05)
            assert clocks and clocks[-1] == .06, clocks
            spin_for(.3)
            assert bridge.command.stale and bridge.command.current() == (0., 0., 0.)
            result = {'passed': True, 'transport': 'FastDDS Humble',
                      'received_commands': bridge.command.received, 'clocks': clocks,
                      'watchdog_stop': True}
            (output / 'transport_check.json').write_text(json.dumps(result, indent=2) + '\n')
        else:
            spin_for(1.)
            for command in ((.4, 0., 0.), (0., 0., .5), (0., .3, 0.), (0., 0., 0.), (-.3, 0., 0.)):
                spin_for(2., command)
            # Leave the final backward command to expire, without sending an explicit stop.
            spin_for(1.)
            deadline = time.monotonic() + 90.
            while process.poll() is None and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.05)
            assert process.poll() == 0, f'Simulator failed or timed out; see {log_path}'
            result = json.loads(report_path.read_text())
            assert result['passed'] and result['control_steps'] == 900, result
            manifest = json.loads((ROOT / 'assets/go2/config/sources.lock.json').read_text())
            selected = manifest['control_policy']
            assert Path(result['policy']['path']) == (ROOT / selected['path']).resolve(), result['policy']
            assert result['policy']['sha256'] == selected['sha256'], result['policy']
            assert result['command_observation_matches'], result
            assert result['resets'] == 0, result
            history = result['command_history']
            commands = [tuple(entry['command']) for entry in history]
            for expected in ((.4, 0., 0.), (0., 0., .5), (0., .3, 0.), (0., 0., 0.), (-.3, 0., 0.)):
                assert expected in commands, (expected, history)
            assert history[-1]['stale'] and history[-1]['command'] == [0., 0., 0.], history
            assert len(clocks) > 100 and clocks[-1] >= 17.9, clocks[-10:]
            assert all(b > a for a, b in zip(clocks, clocks[1:])), clocks[-10:]
            assert result['clock_messages'] == 901, result
            responses = {tuple(entry['command']): entry['mean_body_velocity']
                         for entry in result['command_responses'] if not entry['stale']}
            assert responses[(.4, 0., 0.)][0] > .2, responses
            assert responses[(0., 0., .5)][2] > .2, responses
            assert responses[(0., .3, 0.)][1] > .15, responses
            assert responses[(-.3, 0., 0.)][0] < -.15, responses
            assert max(abs(v) for v in responses[(0., 0., 0.)]) < .15, responses
            result['observed_clock_messages'] = len(clocks)
            (output / 'integration_check.json').write_text(json.dumps(result, indent=2) + '\n')
        print('GO2_ROS_CHECK_PASS ' + json.dumps(result), flush=True)
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=15.)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        if log is not None:
            log.close()
        if bridge is not None:
            bridge.close()
        executor.shutdown()
        node.destroy_node()
        context.try_shutdown()


if __name__ == '__main__':
    main()
