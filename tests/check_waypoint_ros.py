"""PTY Space switching and DDS waypoint tracking, with synthetic or actual Isaac physics."""
import argparse
import json
import math
import os
from pathlib import Path
import pty
import signal
import subprocess
import termios
import time
from types import SimpleNamespace

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]


class SyntheticWorld:
    def __init__(self, config):
        from robot_sim.ros_bridge import RobotRosBridge, SimulationClock
        self.clock = SimulationClock()
        self.robots = []
        self.time = 0.
        self.next_tick = time.monotonic()
        self.drop = set()
        for specification in config['robots']:
            namespace, identity = specification['namespace'], specification['id']
            bridge = RobotRosBridge('/' + namespace + '/cmd_vel', robot_id=identity,
                namespace=namespace, publish_clock=False, enable_reset=True)
            joints = 12 if specification['type'] == 'go2' else 39
            data = SimpleNamespace(root_pos_w=np.array([specification['position']], dtype=float),
                root_quat_w=np.array([[1., 0., 0., 0.]]), root_lin_vel_b=np.zeros((1, 3)),
                root_ang_vel_b=np.zeros((1, 3)), joint_pos=np.zeros((1, joints)), joint_vel=np.zeros((1, joints)))
            model = SimpleNamespace(data=data, joint_names=[f'joint_{i}' for i in range(joints)])
            self.robots.append((specification, bridge, model))

    def tick(self):
        if time.monotonic() < self.next_tick:
            return
        self.next_tick = time.monotonic() + .02
        self.time += .02
        self.clock.publish(self.time)
        for specification, bridge, model in self.robots:
            command = bridge.poll()
            data = model.data
            if bridge.reset_requested:
                data.root_pos_w[0] = specification['position']
                data.root_quat_w[0] = (1., 0., 0., 0.)
                data.root_lin_vel_b[:] = data.root_ang_vel_b[:] = 0.
                bridge.reset_requested = False
                bridge.notify_reset()
                command = (0., 0., 0.)
            data.root_lin_vel_b[0, :2] += .2 * (np.array(command[:2]) - data.root_lin_vel_b[0, :2])
            data.root_ang_vel_b[0, 2] += .2 * (command[2] - data.root_ang_vel_b[0, 2])
            angle = 2 * math.atan2(data.root_quat_w[0, 3], data.root_quat_w[0, 0])
            vx, vy = data.root_lin_vel_b[0, :2]
            data.root_pos_w[0, 0] += (math.cos(angle) * vx - math.sin(angle) * vy) * .02
            data.root_pos_w[0, 1] += (math.sin(angle) * vx + math.cos(angle) * vy) * .02
            angle += data.root_ang_vel_b[0, 2] * .02
            data.root_quat_w[0] = (math.cos(angle / 2), 0., 0., math.sin(angle / 2))
            if specification['id'] not in self.drop:
                bridge.publish_robot_state(self.time, model)

    def close(self):
        for _, bridge, _ in self.robots:
            bridge.close()
        self.clock.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=('synthetic', 'isaac'), default='synthetic')
    parser.add_argument('--scenario', choices=('flat', 'warehouse'), default='warehouse')
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = '97'
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    output = ROOT / '.runtime/waypoint' / args.backend
    output.mkdir(parents=True, exist_ok=True)
    result_file = output / 'integration.json'
    result = {'passed': False, 'backend': args.backend, 'scenario': args.scenario}
    result_file.write_text(json.dumps(result) + '\n')
    config = yaml.safe_load((ROOT / 'ros2_ws/src/robot_bringup/config/go2_lekiwi.yaml').read_text())
    config['world']['scenario'] = args.scenario
    for robot in config['robots']:
        robot['namespace'] = 'waypoint_check/' + robot['id']
    robots_file = output / 'robots.yaml'
    robots_file.write_text(yaml.safe_dump(config))
    rclpy.init(args=[])
    node = rclpy.create_node('waypoint_integration_check')
    robots = [r['id'] for r in config['robots']]
    clocks, statuses, odometry = [], [], {r: [] for r in robots}
    commands = {r: [] for r in robots}
    transient = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        node.create_subscription(Clock, '/clock', lambda msg: clocks.append(
            msg.clock.sec + msg.clock.nanosec / 1e9),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)),
        node.create_subscription(String, '/waypoint/status', lambda msg: statuses.append(json.loads(msg.data)), transient)]
    reset_clients = {}
    for robot in config['robots']:
        identity, prefix = robot['id'], '/' + robot['namespace']
        subscriptions += [node.create_subscription(Odometry, prefix + '/odom', odometry[identity].append, 10),
            node.create_subscription(Twist, prefix + '/cmd_vel',
                lambda msg, identity=identity: commands[identity].append(
                    (msg.linear.x, msg.linear.y, msg.angular.z)), 10)]
        reset_clients[identity] = node.create_client(Trigger, prefix + '/reset')
    publisher = node.create_publisher(PoseStamped, '/waypoint/goal_pose', 1)
    synthetic = simulation = controller = None
    master, slave = pty.openpty()
    saved_terminal = termios.tcgetattr(slave)
    quitting = False

    def spin(seconds=.02):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            if synthetic:
                synthetic.tick()
            for _ in range(16):
                rclpy.spin_once(node, timeout_sec=0.)
            if controller is not None and not quitting:
                assert controller.poll() is None, (output / 'controller.log').read_text()
            if simulation is not None:
                assert simulation.poll() is None, (output / 'simulation.log').read_text()[-3000:]
            time.sleep(.003)

    def wait(predicate, timeout=30):
        until = time.monotonic() + timeout
        while not predicate():
            assert time.monotonic() < until, ('Timed out', statuses[-5:], (output / 'controller.log').read_text()[-3000:])
            spin()

    def phase(seconds):
        end = clocks[-1] + seconds
        wait(lambda: clocks[-1] >= end, timeout=max(30, seconds * 8))

    def xy(identity):
        position = odometry[identity][-1].pose.pose.position
        return (position.x, position.y)

    def send_goal(identity, offset, *, stale=False):
        base = xy(identity)
        target = (base[0] + offset[0], base[1] + offset[1])
        message = PoseStamped()
        message.header.frame_id = 'map'
        seconds = max(0., clocks[-1] - (5. if stale else 0.))
        message.header.stamp.sec = int(seconds)
        message.header.stamp.nanosec = round((seconds % 1) * 1e9)
        message.pose.position.x, message.pose.position.y = target
        message.pose.orientation.w = 1.
        sequence = len(statuses)
        publisher.publish(message)
        if not stale:
            wait(lambda: any(s['selected_robot'] == identity and s['phase'] in ('TRACKING', 'SWITCHING')
                             for s in statuses[sequence:]))
        return target, sequence

    def reached(identity, sequence, target):
        wait(lambda: any(s['selected_robot'] == identity and s['phase'] == 'REACHED'
                         for s in statuses[sequence:]), timeout=100)
        error = math.dist(xy(identity), target)
        assert error < (.13 if identity == 'go2_0' else .09), (identity, error, xy(identity), target)
        result.setdefault('arrivals', []).append(dict(robot=identity, error=error, target=target))
        print('WAYPOINT_CHECK_REACHED ' + json.dumps(result['arrivals'][-1]), flush=True)

    def assert_zero(identity, start):
        samples = commands[identity][start:]
        assert len(samples) > 5, (identity, len(samples))
        assert all(max(map(abs, command)) < 1e-8 for command in samples), (identity, samples[:10])

    try:
        if args.backend == 'synthetic':
            synthetic = SyntheticWorld(config)
        else:
            with (output / 'simulation.log').open('w') as log:
                simulation = subprocess.Popen(['ros2', 'launch', 'robot_bringup', 'simulation.launch.py',
                    f'robots_file:={robots_file}', 'display:=headless', 'rviz_display:=off',
                    'steps:=2400', 'ros_domain_id:=97', f'report:={output / "simulation.json"}'],
                    cwd=ROOT, env=dict(os.environ, ROS_LOG_DIR=str(output / 'ros_logs')),
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        # The first discovery wait precedes the controller log creation.
        (output / 'controller.log').write_text('')
        wait(lambda: clocks and all(odometry.values()), timeout=300)
        with (output / 'controller.log').open('w') as log:
            controller = subprocess.Popen(['ros2', 'run', 'robot_control', 'waypoint',
                '--robots-file', str(robots_file), '--report', str(output / 'controller.json')],
                cwd=ROOT, stdin=slave, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        wait(lambda: statuses and publisher.get_subscription_count() > 0)
        assert statuses[-1]['selected_robot'] == 'go2_0'
        phase(.6)
        stationary = xy('lekiwi_0')
        start = len(commands['lekiwi_0'])
        target, sequence = send_goal('go2_0', (.7, 0.))
        reached('go2_0', sequence, target)
        assert_zero('lekiwi_0', start)
        assert math.dist(stationary, xy('lekiwi_0')) < .06
        send_goal('go2_0', (2., 0.))
        phase(.5)
        os.write(master, b' ')
        wait(lambda: statuses[-1]['selected_robot'] == 'lekiwi_0')
        target, sequence = send_goal('lekiwi_0', (.5, .25))
        if synthetic:
            # Reset the old robot while a new goal is queued: only that robot's state may be canceled.
            future = reset_clients['go2_0'].call_async(Trigger.Request())
            wait(future.done)
            assert future.result().success
        wait(lambda: any(s['selected_robot'] == 'lekiwi_0' and s['phase'] == 'TRACKING' for s in statuses[sequence:]))
        stopped_go2, go2_commands = xy('go2_0'), len(commands['go2_0'])
        reached('lekiwi_0', sequence, target)
        assert_zero('go2_0', go2_commands)
        assert math.dist(stopped_go2, xy('go2_0')) < .15
        os.write(master, b' ')
        wait(lambda: statuses[-1]['selected_robot'] == 'go2_0')
        wait(lambda: statuses[-1]['phase'] == 'WAITING_GOAL')
        start = len(commands['go2_0'])
        phase(1.)
        assert_zero('go2_0', start)  # The old far goal must never resume after selection wraps.
        send_goal('go2_0', (.8, 0.))
        phase(.4)
        os.write(master, b'x')
        wait(lambda: statuses[-1]['phase'] == 'CANCELED')
        start = len(commands['go2_0'])
        phase(.4)
        assert_zero('go2_0', start)
        send_goal('go2_0', (.8, 0.))
        phase(.4)
        assert reset_clients['go2_0'].service_is_ready()
        future = reset_clients['go2_0'].call_async(Trigger.Request())
        wait(future.done)
        assert future.result().success
        wait(lambda: statuses[-1]['phase'] in ('RESET', 'POSE_RESET'))
        start = len(commands['go2_0'])
        phase(.8)
        assert_zero('go2_0', start)
        if synthetic:
            send_goal('go2_0', (.8, 0.))
            phase(.2)
            synthetic.drop.add('go2_0')
            wait(lambda: statuses[-1]['phase'] == 'ODOM_TIMEOUT')
            synthetic.drop.clear()
            start = len(commands['go2_0'])
            phase(.5)
            assert_zero('go2_0', start)
            send_goal('go2_0', (.8, 0.), stale=True)
            phase(.3)
        quitting = True
        os.write(master, b'q')
        while controller.poll() is None:
            spin()
        assert controller.returncode == 0, (output / 'controller.log').read_text()
        assert termios.tcgetattr(slave) == saved_terminal, 'Terminal settings were not restored.'
        report = json.loads((output / 'controller.json').read_text())
        assert report['switches'] == 2, report
        assert report['goals_accepted'] >= 5, report
        if synthetic:
            assert report['goals_rejected'] >= 1, report
        if simulation is not None:
            # Stop controller first, then let the finite simulation produce its normal final report.
            process, simulation = simulation, None
            deadline = time.monotonic() + 180
            while process.poll() is None:
                assert time.monotonic() < deadline
                spin()
            assert process.returncode == 0
            sim_report = json.loads((output / 'simulation.json').read_text())
            assert sim_report['passed'] and sim_report['control_steps'] == 2400, sim_report.get('error')
            assert sim_report['robots']['go2_0']['resets'] == 1
            assert sim_report['robots']['lekiwi_0']['resets'] == 0
            manifest = json.loads((ROOT / 'assets/go2/config/sources.lock.json').read_text())
            assert sim_report['policy']['sha256'] == manifest['control_policy']['sha256']
            result['checkpoint_sha256'] = sim_report['policy']['sha256']
        result.update(passed=True, space_switching=True, inactive_commands_zero=True,
                      reset_cancels_goal=True, terminal_restored=True, controller=report)
    finally:
        for process in (controller, simulation):
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        if synthetic:
            synthetic.close()
        os.close(master)
        os.close(slave)
        node.destroy_node()
        rclpy.shutdown()
        result_file.write_text(json.dumps(result, indent=2) + '\n')
    print('WAYPOINT_ROS_CHECK_PASS ' + json.dumps({k: v for k, v in result.items() if k != 'controller'}), flush=True)


if __name__ == '__main__':
    main()
