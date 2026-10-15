#!/usr/bin/env python3
"""System Python 3.10 -> ROS launch -> Isaac Python 3.11 -> real Lekiwi wheel/roller physics.

Source /opt/ros/humble/setup.bash and ros2_ws/install/setup.bash, then use /usr/bin/python3.
Checks body motion, passive rollers, stopping, namespaces, odometry, TF and clock.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=('flat', 'warehouse'), default='flat')
    parser.add_argument('--display', choices=('headless', 'local', 'webrtc'), default='headless')
    parser.add_argument('--server-ip', default='')
    parser.add_argument('--startup-timeout', type=float, default=600.,
                        help='Cold warehouse/material preparation can take several minutes.')
    args = parser.parse_args()
    assert sys.version_info[:2] == (3, 10), sys.version
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from tf2_msgs.msg import TFMessage
    import yaml

    os.environ['ROS_DOMAIN_ID'] = '92'
    rclpy.init(args=[])
    node = rclpy.create_node('lekiwi_integration_check')
    out = ROOT / '.runtime/lekiwi' / (args.scenario + '_' + args.display)
    out.mkdir(exist_ok=True, parents=True)
    (out / 'integration_check.json').unlink(missing_ok=True)
    report_path, log_path = out / 'simulation.json', out / 'simulation.log'
    report_path.unlink(missing_ok=True)
    namespace = 'validation/lekiwi_0'
    config = {'world': {'scenario': args.scenario, 'seed': 42}, 'robots': [
        {'id': 'lekiwi_check', 'type': 'lekiwi', 'namespace': namespace, 'position': [0., 0., .04], 'yaw': .3}]}
    robots_file = out / 'robots.yaml'
    robots_file.write_text(yaml.safe_dump(config))
    clocks, odometry, joint_states, transforms, static_transforms = [], [], [], [], []
    clock_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    static_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        node.create_subscription(Clock, '/clock', lambda m: clocks.append(
            m.clock.sec + m.clock.nanosec / 1e9), clock_qos),
        node.create_subscription(Odometry, '/' + namespace + '/odom', odometry.append, 100),
        node.create_subscription(JointState, '/' + namespace + '/joint_states', joint_states.append, 100),
        node.create_subscription(TFMessage, '/tf', lambda m: transforms.extend(m.transforms), 100),
        node.create_subscription(TFMessage, '/tf_static', lambda m: static_transforms.extend(m.transforms), static_qos),
    ]
    publisher = node.create_publisher(Twist, '/' + namespace + '/cmd_vel', 1)
    wrong_publisher = node.create_publisher(Twist, '/unitree_go2/cmd_vel', 1)
    process = None
    def spin(seconds, command=None):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if command is not None:
                message = Twist()
                message.linear.x, message.linear.y, message.angular.z = command
                publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=.005)
            time.sleep(.005)

    def spin_sim(seconds, command):
        # GPU rendering may run below real time. Keep refreshing cmd_vel until
        # a full measurement interval has elapsed in physics, not wall time.
        assert clocks
        target = clocks[-1] + seconds
        deadline = time.monotonic() + 90.
        while clocks[-1] < target:
            assert process.poll() is None, f'Simulation ended during command {command}'
            assert time.monotonic() < deadline, f'Physics stalled during command {command}'
            spin(.02, command)
    try:
        with log_path.open('w') as log:
            process = subprocess.Popen(['ros2', 'launch', 'robot_bringup', 'simulation.launch.py',
                'project_root:=' + str(ROOT), 'robots_file:=' + str(robots_file),
                'display:=' + args.display, 'steps:=900', 'ros_domain_id:=92', 'report:=' + str(report_path),
                *(['server_ip:=' + args.server_ip] if args.server_ip else [])],
                cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + args.startup_timeout
            while publisher.get_subscription_count() == 0 or not odometry:
                assert process.poll() is None, f'Simulator exited early; see {log_path}'
                assert time.monotonic() < deadline, f'Discovery timeout; see {log_path}'
                spin(.05)
            assert node.count_publishers('/clock') == 1
            assert wrong_publisher.get_subscription_count() == 0
            if args.display == 'webrtc':
                with socket.create_connection(('127.0.0.1', 49100), timeout=5.):
                    pass
            spin(.5)
            # Exercise the installed system-Humble command executable as well.
            with (out / 'teleop.log').open('w') as command_log:
                teleop = subprocess.Popen(['ros2', 'run', 'robot_control', 'teleop',
                    '--namespace', namespace, '--vx', '0', '--duration', '1'],
                    cwd=ROOT, stdout=command_log, stderr=subprocess.STDOUT)
                deadline = time.monotonic() + 10.
                while teleop.poll() is None and time.monotonic() < deadline:
                    spin(.02)
                if teleop.poll() is None:
                    teleop.terminate()
                    teleop.wait(timeout=10.)
                    raise AssertionError('teleop timed out')
                assert teleop.returncode == 0, (out / 'teleop.log').read_text()
            commands = ((.2, 0., 0.), (0., .2, 0.), (0., 0., .5), (0., 0., 0.), (-.2, 0., 0.), (0., -.2, 0.), (0., 0., -.5))
            for command in commands:
                spin_sim(1.5, command)
            spin(1.)  # Stop publishing and confirm the simulator's wall-time watchdog.
            deadline = time.monotonic() + 90.
            while process.poll() is None and time.monotonic() < deadline:
                spin(.05)
            assert process.poll() == 0, f'Simulation failed; see {log_path}'
        report = json.loads(report_path.read_text())
        assert report['passed'] and report['control_steps'] == 900 and report['resets'] == 0, report
        assert report['namespace'] == namespace and report['robot_id'] == 'lekiwi_check', report
        assert report['robot_type'] == 'lekiwi'
        assert abs(report['physics_elapsed_seconds'] - report['simulated_seconds']) < 1e-6, report
        assert report['asset']['contact_model'] == 'passive_capsule_rollers'
        assert report['asset']['passive_rollers'] == 36
        assert len(report['asset']['locked_arm_joints']) == 6
        assert not report['asset']['composition_errors']
        assert report['command_history'][-1]['stale']
        assert report['command_history'][-1]['command'] == [0., 0., 0.]
        assert report['clock_messages'] == report['state_messages'] == 901
        assert len(clocks) > 100 and clocks[-1] > 17.9 and all(b > a for a, b in zip(clocks, clocks[1:]))
        assert len(odometry) > 100 and len(joint_states) > 100
        assert all(m.header.frame_id == namespace + '/odom' and m.child_frame_id == namespace + '/base_link'
                   for m in odometry)
        assert all(len(m.name) == len(m.position) == len(m.velocity) == 39 for m in joint_states)
        assert all(math.isfinite(v) for m in joint_states for v in m.position)
        assert transforms and all(t.header.frame_id == namespace + '/odom'
                                  and t.child_frame_id == namespace + '/base_link' for t in transforms)
        assert any(t.header.frame_id == 'map' and t.child_frame_id == namespace + '/odom'
                   for t in static_transforms)
        latest = odometry[-1]
        position = [latest.pose.pose.position.x, latest.pose.pose.position.y, latest.pose.pose.position.z]
        assert max(abs(a - b) for a, b in zip(position, report['end_position'])) < .05
        responses = {tuple(m['command']): m['mean_body_velocity'] for m in report['command_responses']
                     if not m['stale']}
        assert responses[(.2, 0., 0.)][0] > .1, responses
        assert responses[(0., .2, 0.)][1] > .1, responses
        assert responses[(0., 0., .5)][2] > .25, responses
        assert responses[(-.2, 0., 0.)][0] < -.1, responses
        assert responses[(0., -.2, 0.)][1] < -.1, responses
        assert responses[(0., 0., -.5)][2] < -.25, responses
        stop = responses[(0., 0., 0.)]
        assert math.hypot(*stop[:2]) < .08 and abs(stop[2]) < .15, responses
        stale = [r for r in report['command_responses'] if r['stale']][-1]['mean_body_velocity']
        assert math.hypot(*stale[:2]) < .08 and abs(stale[2]) < .15, stale
        assert report['minimum_base_height_m'] > .015, report['minimum_base_height_m']
        result = {'passed': True, 'external_python': sys.version.split()[0],
                  'simulator_python': '3.11', 'scenario': args.scenario, 'namespace': namespace,
                  'steps': report['control_steps'], 'resets': report['resets'],
                  'clock_received': len(clocks), 'odom_received': len(odometry),
                  'joint_states_received': len(joint_states), 'tf_received': len(transforms),
                  'static_tf_received': len(static_transforms), 'watchdog_stop': True,
                  'installed_teleop': True, 'passive_rollers': 36, 'body_motion': {str(k): v for k, v in responses.items()},
                  'physical_stop': True, 'display': args.display,
                  'webrtc_signaling_port_open': args.display == 'webrtc'}
        (out / 'integration_check.json').write_text(json.dumps(result, indent=2) + '\n')
        print('LEKIWI_ROS_CHECK_PASS ' + json.dumps(result), flush=True)
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=20.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
