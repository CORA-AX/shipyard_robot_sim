"""Exercise real RViz/noVNC in Chrome, goal input, native display and process cleanup.

Run with system Humble/workspace sourced. Optional test dependencies:
  /usr/bin/python3 -m pip install --target .runtime/rviz_test_tools playwright
Uses synthetic state through the actual simulator ROS bridge; does not start Isaac.
"""
import argparse
import json
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / '.runtime/rviz_test_tools')]
import numpy as np
from playwright.sync_api import sync_playwright
import yaml


def wait_until(predicate, timeout=25):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = predicate()
        if result:
            return result
        time.sleep(.1)
    raise RuntimeError('Condition did not become true before timeout.')


def port_open(port):
    try:
        with socket.create_connection(('127.0.0.1', port), timeout=.2):
            return True
    except OSError:
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=6081)
    parser.add_argument('--domain', type=int, default=94)
    args = parser.parse_args()
    if port_open(args.port):
        raise RuntimeError(f'Test port {args.port} is already occupied.')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    from geometry_msgs.msg import PoseStamped
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from robot_sim.ros_bridge import RobotRosBridge, SimulationClock

    output = ROOT / '.runtime/rviz_check'
    output.mkdir(parents=True, exist_ok=True)
    (output / 'result.json').write_text('{"passed": false}\n')
    robots_file = output / 'robots.yaml'
    world = yaml.safe_load((ROOT / 'ros2_ws/src/robot_bringup/config/go2_lekiwi.yaml').read_text())
    for robot in world['robots']:
        robot['namespace'] = 'rviz_check/' + robot['id']
    robots_file.write_text(yaml.safe_dump(world))
    goal_topic = '/waypoint/goal_pose'
    context = Context()
    context.init(args=[])
    node = Node('rviz_input_check', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    goals, other_goals = [], []
    node.create_subscription(PoseStamped, goal_topic, goals.append, 10)
    node.create_subscription(PoseStamped, '/rviz_check/lekiwi_0/goal_pose', other_goals.append, 10)
    bridges, models = [], []
    clock = SimulationClock()
    for robot in world['robots']:
        bridge = RobotRosBridge('/' + robot['namespace'] + '/cmd_vel', robot_id=robot['id'],
            namespace=robot['namespace'], accept_commands=False, publish_clock=False)
        bridges.append(bridge)
        joints = 12 if robot['type'] == 'go2' else 39
        data = SimpleNamespace(root_pos_w=np.array([robot['position']]),
            root_quat_w=np.array([[1., 0., 0., 0.]]), root_lin_vel_b=np.zeros((1, 3)),
            root_ang_vel_b=np.zeros((1, 3)), joint_pos=np.zeros((1, joints)), joint_vel=np.zeros((1, joints)))
        models.append(SimpleNamespace(data=data, joint_names=[f'joint_{i}' for i in range(joints)]))
    stop = threading.Event()
    def publish():
        step = 0
        while not stop.is_set():
            step += 1
            clock.publish(step * .02)
            for bridge, model in zip(bridges, models):
                bridge.publish_robot_state(step * .02, model)
            executor.spin_once(timeout_sec=0.)
            stop.wait(.02)
    worker = threading.Thread(target=publish)
    worker.start()
    processes = []
    results = {'passed': False, 'goal_topic': goal_topic}

    def launch(mode, label, extra_env=None, software=False):
        before = set((ROOT / '.runtime/rviz').glob('*/session.json'))
        env = dict(os.environ, ROS_LOG_DIR=str(output / 'ros_logs'))
        if extra_env:
            env.update(extra_env)
        command = ['ros2', 'launch', 'robot_bringup', 'rviz.launch.py',
                   f'project_root:={ROOT}', f'robots_file:={robots_file}',
                   f'rviz_display:={mode}', f'rviz_port:={args.port}', f'ros_domain_id:={args.domain}']
        if software:
            command.append('rviz_rendering:=software')
        with (output / (label + '.log')).open('w') as log:
            process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
        processes.append(process)
        def ready():
            if process.poll() is not None:
                raise RuntimeError((output / (label + '.log')).read_text())
            for path in set((ROOT / '.runtime/rviz').glob('*/session.json')) - before:
                try:
                    record = json.loads(path.read_text())
                except json.JSONDecodeError:
                    continue
                if record.get('state') == 'ready':
                    return path, record
        return process, *wait_until(ready)

    def terminate(process):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=15)

    try:
        process, session_path, record = launch('web', 'web')
        listeners = subprocess.check_output(['ss', '-H', '-ltnp'], text=True).splitlines()
        addresses = []
        for name in ('x11vnc', 'websockify'):
            rows = [row for row in listeners if f'pid={record["processes"][name]},' in row]
            assert rows, f'No listening socket for {name}'
            for row in rows:
                address = row.split()[3]
                assert address.startswith('127.0.0.1:'), f'Unexpected listener: {row}'
                addresses.append(address)
        results['listeners'] = addresses
        wait_until(lambda: all(bridge.odom_pub.get_subscription_count() > 0 for bridge in bridges))
        with sync_playwright() as p:
            browser = p.chromium.launch(executable_path=shutil.which('google-chrome'),
                headless=True, args=['--no-sandbox', '--disable-dev-shm-usage'])
            try:
                page = browser.new_page(viewport={'width': 1500, 'height': 1000})
                errors = []
                page.on('pageerror', lambda error: errors.append(str(error)))
                page.goto(f'http://127.0.0.1:{args.port}/vnc.html?autoconnect=true&resize=scale')
                page.wait_for_function("document.documentElement.classList.contains('noVNC_connected')")
                page.wait_for_timeout(1500)
                canvas = page.locator('canvas')
                bounds = canvas.bounding_box()
                width = canvas.evaluate('(el) => el.width')
                height = canvas.evaluate('(el) => el.height')
                def point(x, y):
                    return (bounds['x'] + x * bounds['width'] / width,
                            bounds['y'] + y * bounds['height'] / height)
                # Select the actual toolbar button rather than relying on remote keyboard focus.
                page.mouse.click(*point(360, 72))
                page.wait_for_timeout(500)
                page.mouse.move(*point(900, 400))
                page.mouse.down()
                page.mouse.move(*point(950, 400), steps=10)
                page.mouse.up()
                page.wait_for_timeout(800)
                page.screenshot(path=str(output / 'browser.png'))
                wait_until(lambda: goals, timeout=5)
                assert not errors, errors
                assert not other_goals, 'Goal leaked into another robot namespace.'
                goal = goals[-1]
                assert goal.header.frame_id == 'map'
                assert goal.header.stamp.sec > 0 or goal.header.stamp.nanosec > 0
                assert all(math.isfinite(v) for v in (goal.pose.position.x, goal.pose.position.y))
                assert node.count_publishers('/rviz_check/go2_0/cmd_vel') == 0
                results.update(goal={'frame': goal.header.frame_id, 'x': goal.pose.position.x,
                                     'y': goal.pose.position.y, 'orientation_w': goal.pose.orientation.w},
                               browser_errors=errors, web_session=str(session_path))
            finally:
                browser.close()
        log = (session_path.parent / 'rviz.log').read_text()
        assert 'OpenGl version:' in log, log
        # Exercise native mode against a real X server without touching the host desktop.
        local, local_path, local_record = launch('local', 'local',
            {'DISPLAY': record['display'], 'XAUTHORITY': str(session_path.parent / 'Xauthority')}, True)
        assert 'xvfb' not in local_record['processes']
        terminate(local)
        assert local.returncode == 0
        assert port_open(args.port), 'Local viewer shutdown killed the independent remote display.'
        results['local_session'] = str(local_path)
        terminate(process)
        assert process.returncode == 0
        wait_until(lambda: not port_open(args.port))
        for pid in record['processes'].values():
            assert not Path(f'/proc/{pid}').exists(), f'Leaked helper PID {pid}'
        # An unexpected display helper exit must stop RViz and release the port too.
        failed, failed_path, failed_record = launch('web', 'helper_failure')
        os.kill(failed_record['processes']['x11vnc'], signal.SIGTERM)
        failed.wait(timeout=15)
        assert failed.returncode != 0
        wait_until(lambda: not port_open(args.port))
        for pid in failed_record['processes'].values():
            assert not Path(f'/proc/{pid}').exists(), f'Leaked helper PID {pid}'
        assert 'x11vnc exited' in json.loads(failed_path.read_text())['error']
        results.update(passed=True, normal_shutdown=True, helper_failure_cleanup=True)
    finally:
        for process in processes:
            terminate(process)
        stop.set()
        worker.join(timeout=5)
        executor.shutdown()
        node.destroy_node()
        context.try_shutdown()
        for bridge in bridges:
            bridge.close()
        clock.close()
        (output / 'result.json').write_text(json.dumps(results, indent=2) + '\n')
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
