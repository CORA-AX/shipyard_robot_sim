#!/usr/bin/env python3
"""System Python 3.10 -> ROS launch -> Isaac Python 3.11 -> real Go2 physics.

Source /opt/ros/humble/setup.bash and ros2_ws/install/setup.bash, then use /usr/bin/python3.
Checks commands, expiry, odometry, joints, TF, one clock and the selected checkpoint.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    assert sys.version_info[:2] == (3, 10), sys.version
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from tf2_msgs.msg import TFMessage
    import yaml

    os.environ['ROS_DOMAIN_ID'] = '89'
    rclpy.init(args=[])
    node = rclpy.create_node('workspace_integration_check')
    out = ROOT / '.runtime/ros2_workspace'
    out.mkdir(exist_ok=True, parents=True)
    report_path, log_path = out / 'simulation.json', out / 'simulation.log'
    report_path.unlink(missing_ok=True)
    namespace = 'validation/go2_0'
    config = {'world': {'scenario': 'warehouse', 'seed': 42}, 'robots': [
        {'id': 'go2_check', 'type': 'go2', 'namespace': namespace, 'position': [0., 0., .4]}]}
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

    def spin_sim(seconds, command=None):
        assert clocks
        target, deadline = clocks[-1] + seconds, time.monotonic() + 90.
        while clocks[-1] < target:
            assert process.poll() is None, 'Simulator ended during command measurement'
            assert time.monotonic() < deadline, 'Physics stalled during command measurement'
            spin(.02, command)
    try:
        with log_path.open('w') as log:
            process = subprocess.Popen(['ros2', 'launch', 'robot_bringup', 'simulation.launch.py',
                'project_root:=' + str(ROOT), 'robots_file:=' + str(robots_file),
                'display:=headless', 'steps:=900', 'ros_domain_id:=89', 'report:=' + str(report_path)],
                cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 180.
            while publisher.get_subscription_count() == 0 or not odometry:
                assert process.poll() is None, f'Simulator exited early; see {log_path}'
                assert time.monotonic() < deadline, f'Discovery timeout; see {log_path}'
                spin(.05)
            assert node.count_publishers('/clock') == 1
            assert wrong_publisher.get_subscription_count() == 0
            spin_sim(1.)
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
            commands = ((.4, 0., 0.), (0., 0., .5), (0., .3, 0.), (0., 0., 0.), (-.3, 0., 0.))
            for command in commands:
                spin_sim(2., command)
                if command == (0., 0., .5):
                    # Let yaw settle before measuring the independent lateral response.
                    spin_sim(1., (0., 0., 0.))
            spin(1.)  # Stop publishing and confirm the simulator's wall-time watchdog.
            deadline = time.monotonic() + 90.
            while process.poll() is None and time.monotonic() < deadline:
                spin(.05)
            assert process.poll() == 0, f'Simulation failed; see {log_path}'
        report = json.loads(report_path.read_text())
        assert report['passed'] and report['control_steps'] == 900 and report['resets'] == 0, report
        assert report['namespace'] == namespace and report['robot_id'] == 'go2_check', report
        assert report['command_observation_matches']
        selected = json.loads((ROOT / 'assets/go2/config/sources.lock.json').read_text())['control_policy']
        assert report['policy']['sha256'] == selected['sha256']
        assert report['command_history'][-1]['stale']
        assert report['command_history'][-1]['command'] == [0., 0., 0.]
        assert report['clock_messages'] == report['state_messages'] == 901
        assert len(clocks) > 100 and clocks[-1] > 17.9 and all(b > a for a, b in zip(clocks, clocks[1:]))
        assert len(odometry) > 100 and len(joint_states) > 100
        assert all(m.header.frame_id == namespace + '/odom' and m.child_frame_id == namespace + '/base_link'
                   for m in odometry)
        assert all(len(m.name) == len(m.position) == len(m.velocity) == 12 for m in joint_states)
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
        assert responses[(.4, 0., 0.)][0] > .2, responses
        assert responses[(0., 0., .5)][2] > .2, responses
        assert responses[(0., .3, 0.)][1] > .15, responses
        assert responses[(-.3, 0., 0.)][0] < -.15, responses
        result = {'passed': True, 'external_python': sys.version.split()[0],
                  'simulator_python': '3.11', 'scenario': 'warehouse', 'namespace': namespace,
                  'steps': report['control_steps'], 'resets': report['resets'],
                  'clock_received': len(clocks), 'odom_received': len(odometry),
                  'joint_states_received': len(joint_states), 'tf_received': len(transforms),
                  'static_tf_received': len(static_transforms), 'watchdog_stop': True,
                  'installed_teleop': True, 'checkpoint_sha256': selected['sha256']}
        (out / 'integration_check.json').write_text(json.dumps(result, indent=2) + '\n')
        print('ROS_WORKSPACE_CHECK_PASS ' + json.dumps(result), flush=True)
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
