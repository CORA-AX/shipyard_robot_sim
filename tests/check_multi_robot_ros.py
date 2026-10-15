#!/usr/bin/env python3
"""Real shared-world DDS, independent motion/watchdogs/resets, and robot collision check."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--display', choices=('headless', 'webrtc', 'local'), default='headless')
    parser.add_argument('--server-ip', default='127.0.0.1')
    args = parser.parse_args()
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger
    from tf2_msgs.msg import TFMessage
    import yaml

    os.environ['ROS_DOMAIN_ID'] = '93'
    rclpy.init(args=[])
    node = rclpy.create_node('multi_robot_check')
    out = ROOT / '.runtime/multi_ros' / args.display
    out.mkdir(parents=True, exist_ok=True)
    report_path, result_path = out / 'simulation.json', out / 'integration_check.json'
    report_path.unlink(missing_ok=True)
    result_path.unlink(missing_ok=True)
    config = yaml.safe_load((ROOT / 'ros2_ws/src/robot_bringup/config/go2_lekiwi.yaml').read_text())
    for robot in config['robots']:
        robot['namespace'] = 'validation/' + robot['id']
    robots_file = out / 'robots.yaml'
    robots_file.write_text(yaml.safe_dump(config))
    ids = ['go2_0', 'lekiwi_0']
    namespaces = {r['id']: r['namespace'] for r in config['robots']}
    clocks, odom, joints, transforms, static = [], {i: [] for i in ids}, {i: [] for i in ids}, [], []
    subscriptions = [node.create_subscription(Clock, '/clock', lambda m: clocks.append(
        m.clock.sec + m.clock.nanosec / 1e9), QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)),
        node.create_subscription(TFMessage, '/tf', lambda m: transforms.extend(m.transforms), 100),
        node.create_subscription(TFMessage, '/tf_static', lambda m: static.extend(m.transforms),
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))]
    publishers, clients = {}, {}
    for identity in ids:
        namespace = namespaces[identity]
        subscriptions.extend([
            node.create_subscription(Odometry, '/' + namespace + '/odom', odom[identity].append, 100),
            node.create_subscription(JointState, '/' + namespace + '/joint_states', joints[identity].append, 100)])
        publishers[identity] = node.create_publisher(Twist, '/' + namespace + '/cmd_vel', 1)
        clients[identity] = node.create_client(Trigger, '/' + namespace + '/reset')
    process = None
    phases = []

    def spin(seconds, commands=None):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            for identity, command in (commands or {}).items():
                message = Twist()
                message.linear.x, message.linear.y, message.angular.z = command
                publishers[identity].publish(message)
            rclpy.spin_once(node, timeout_sec=.003)
            time.sleep(.003)

    def phase(name, seconds, commands):
        assert clocks
        start, until = clocks[-1], clocks[-1] + seconds
        counts = {i: len(odom[i]) for i in ids}
        deadline = time.monotonic() + 120.
        while clocks[-1] < until:
            assert process.poll() is None, f'Simulation stopped during {name}'
            assert time.monotonic() < deadline, f'Physics stalled during {name}'
            spin(.02, commands)
        measured = {}
        for identity in ids:
            samples = [m for m in odom[identity][counts[identity]:] if stamp(m.header.stamp) > start + .4]
            assert samples, (name, identity)
            measured[identity] = [sum(getattr(m.twist.twist, kind).__getattribute__(axis) for m in samples) / len(samples)
                                  for kind, axis in [('linear', 'x'), ('linear', 'y'), ('angular', 'z')]]
        phases.append({'name': name, 'start': start, 'end': clocks[-1], 'velocity': measured})
        print('MULTI_CHECK_PHASE ' + json.dumps(phases[-1]), flush=True)
        return measured

    def stamp(value):
        return value.sec + value.nanosec / 1e9

    def xy(message):
        return (message.pose.pose.position.x, message.pose.pose.position.y)

    def reset(identity, commands=None):
        other = next(i for i in ids if i != identity)
        before, before_time = xy(odom[other][-1]), stamp(odom[other][-1].header.stamp)
        started = clocks[-1]
        future = clients[identity].call_async(Trigger.Request())
        deadline = time.monotonic() + 15.
        while not future.done():
            assert time.monotonic() < deadline, 'Reset service timed out'
            spin(.01, commands)
        assert future.result().success, future.result().message
        until = clocks[-1] + .16
        while clocks[-1] < until:
            assert time.monotonic() < deadline
            spin(.01, commands)
        after, after_time = xy(odom[other][-1]), stamp(odom[other][-1].header.stamp)
        distance = math.dist(before, after)
        assert clocks[-1] > started and distance < .08 + .8 * (after_time - before_time), (identity, distance)
        spawn = next(r['position'][:2] for r in config['robots'] if r['id'] == identity)
        assert math.dist(xy(odom[identity][-1]), spawn) < .12, (identity, xy(odom[identity][-1]), spawn)

    zero = (0., 0., 0.)
    both_zero = {i: zero for i in ids}
    try:
        with (out / 'simulation.log').open('w') as log:
            process = subprocess.Popen(['ros2', 'launch', 'robot_bringup', 'simulation.launch.py',
                'robots_file:=' + str(robots_file), 'display:=' + args.display, 'steps:=1400',
                'ros_domain_id:=93', 'report:=' + str(report_path),
                *(['server_ip:=' + args.server_ip] if args.display == 'webrtc' else [])],
                cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 600.
            while (not clocks or not all(odom.values()) or not all(c.service_is_ready() for c in clients.values())):
                assert process.poll() is None, f'Simulation exited; see {out / "simulation.log"}'
                assert time.monotonic() < deadline, 'Discovery timed out'
                spin(.05)
            assert node.count_publishers('/clock') == 1
            assert all(p.get_subscription_count() == 1 for p in publishers.values())
            if args.display == 'webrtc':
                with socket.create_connection(('127.0.0.1', 49100), timeout=5.):
                    pass
            phase('settle', .8, both_zero)
            stationary = xy(odom['lekiwi_0'][-1])
            velocity = phase('go2_only', 1.5, {'go2_0': (.3, 0., 0.), 'lekiwi_0': zero})
            assert velocity['go2_0'][0] > .15, velocity
            assert math.dist(stationary, xy(odom['lekiwi_0'][-1])) < .05
            moving_lekiwi = {'lekiwi_0': (0., .2, 0.)}
            reset('go2_0', moving_lekiwi)
            velocity = phase('go2_reset_lekiwi_moving', 1.2, moving_lekiwi)
            assert velocity['lekiwi_0'][1] > .1, velocity
            velocity = phase('different_commands', 1.5, {'go2_0': (0., 0., .5), 'lekiwi_0': (.2, 0., 0.)})
            assert velocity['go2_0'][2] > .2 and velocity['lekiwi_0'][0] > .1, velocity
            moving_go2 = {'go2_0': (-.3, 0., 0.)}
            velocity = phase('lekiwi_watchdog_go2_moving', 2., moving_go2)
            assert velocity['go2_0'][0] < -.15, velocity
            assert math.hypot(*velocity['lekiwi_0'][:2]) < .05 and abs(velocity['lekiwi_0'][2]) < .15, velocity
            reset('lekiwi_0', moving_go2)
            velocity = phase('lekiwi_reset_go2_moving', 1., moving_go2)
            assert velocity['go2_0'][0] < -.15, velocity
            reset('go2_0', both_zero)
            reset('lekiwi_0', both_zero)
            phase('before_collision', 1., both_zero)
            collision_start = clocks[-1]
            phase('collision', 5.5, {'go2_0': zero, 'lekiwi_0': (0., -.3, 0.)})
            phase('after_collision_stop', 1., both_zero)
            reset('go2_0', both_zero)
            reset('lekiwi_0', both_zero)
            phase('recovered', 1., both_zero)
            deadline = time.monotonic() + 180.
            while process.poll() is None:
                assert time.monotonic() < deadline, 'Simulation did not finish'
                spin(.05)
            assert process.returncode == 0, f'Simulation failed; see {out / "simulation.log"}'
        report = json.loads(report_path.read_text())
        assert report['passed'] and report['control_steps'] == 1400, report.get('error')
        assert report['clock_messages'] == 1401
        assert abs(report['physics_elapsed_seconds'] - 28.) < 1e-5
        assert len(clocks) > 1000 and all(b > a for a, b in zip(clocks, clocks[1:]))
        assert any(pair['events'] and pair['impulse_sum'] > 0 for pair in report['robot_contacts']), report['robot_contacts']
        assert all(pair['first_step'] * .02 >= collision_start - .04 for pair in report['robot_contacts'])
        manifest = json.loads((ROOT / 'assets/go2/config/sources.lock.json').read_text())
        assert report['policy']['sha256'] == manifest['control_policy']['sha256']
        for identity, expected_joints in zip(ids, (12, 39)):
            robot = report['robots'][identity]
            assert robot['state_messages'] == 1401
            assert sum(r['reason'] == 'service' for r in robot['reset_history']) == 3
            assert all(r['reason'] != 'fall' or r['step'] * .02 >= collision_start for r in robot['reset_history'])
            assert len(odom[identity]) > 1000 and len(joints[identity]) > 1000
            assert all(m.header.frame_id == namespaces[identity] + '/odom' for m in odom[identity])
            assert all(len(m.name) == len(m.position) == len(m.velocity) == expected_joints for m in joints[identity])
            assert all(math.isfinite(v) for m in joints[identity] for v in m.position)
            assert any(t.child_frame_id == namespaces[identity] + '/base_link' for t in transforms)
            assert any(t.header.frame_id == 'map' and t.child_frame_id == namespaces[identity] + '/odom' for t in static)
            assert robot['command_history'][-1]['stale']
        common_stamps = [{stamp(m.header.stamp) for m in odom[i]} for i in ids]
        assert len(common_stamps[0] & common_stamps[1]) > 1000
        result = dict(passed=True, display=args.display, scenario='warehouse', steps=1400,
                      single_clock=True, isolated_commands=True, isolated_watchdog=True, isolated_resets=True,
                      robot_contacts=report['robot_contacts'], reset_counts={i: report['robots'][i]['resets'] for i in ids},
                      clock_received=len(clocks), odom_received={i: len(odom[i]) for i in ids},
                      checkpoint_sha256=report['policy']['sha256'])
        result_path.write_text(json.dumps(result, indent=2) + '\n')
        print('MULTI_ROBOT_ROS_CHECK_PASS ' + json.dumps(result), flush=True)
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
