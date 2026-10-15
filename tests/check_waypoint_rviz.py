"""Real browser RViz clicks -> selected PD robot, then PTY Space -> second robot, without RViz restart."""
import json
import os
from pathlib import Path
import pty
import shutil
import signal
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / '.runtime/rviz_test_tools'))
from playwright.sync_api import sync_playwright
import yaml

from check_waypoint_ros import SyntheticWorld


def main():
    os.environ['ROS_DOMAIN_ID'] = '98'
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    from visualization_msgs.msg import MarkerArray
    output = ROOT / '.runtime/waypoint/browser'
    output.mkdir(parents=True, exist_ok=True)
    result = {'passed': False}
    (output / 'result.json').write_text(json.dumps(result) + '\n')
    config = yaml.safe_load((ROOT / 'ros2_ws/src/robot_bringup/config/go2_lekiwi.yaml').read_text())
    for robot in config['robots']:
        robot['namespace'] = 'waypoint_browser/' + robot['id']
    robots_file = output / 'robots.yaml'
    robots_file.write_text(yaml.safe_dump(config))
    rclpy.init(args=[])
    node = rclpy.create_node('waypoint_browser_check')
    world = SyntheticWorld(config)
    statuses, markers, commands = [], [], {r['id']: [] for r in config['robots']}
    transient = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        node.create_subscription(String, '/waypoint/status', lambda msg: statuses.append(json.loads(msg.data)), transient),
        node.create_subscription(MarkerArray, '/waypoint/markers', markers.append, transient)]
    for robot in config['robots']:
        identity = robot['id']
        subscriptions.append(node.create_subscription(Twist, '/' + robot['namespace'] + '/cmd_vel',
            lambda msg, identity=identity: commands[identity].append((msg.linear.x, msg.linear.y, msg.angular.z)), 10))
    stop = threading.Event()
    def spin():
        while not stop.is_set():
            world.tick()
            for _ in range(8):
                rclpy.spin_once(node, timeout_sec=0.)
            stop.wait(.003)
    worker = threading.Thread(target=spin)
    worker.start()
    master, slave = pty.openpty()
    processes = []
    def launch(command, name, stdin=None):
        with (output / (name + '.log')).open('w') as log:
            process = subprocess.Popen(command, cwd=ROOT, env=dict(os.environ, ROS_LOG_DIR=str(output / 'ros_logs')),
                stdin=stdin, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process
    def wait(predicate, timeout=30):
        until = time.monotonic() + timeout
        while not predicate():
            assert all(p.poll() is None for p in processes), 'A child exited; inspect browser test logs.'
            assert time.monotonic() < until, statuses[-5:]
            time.sleep(.03)
    try:
        before = set((ROOT / '.runtime/rviz').glob('*/session.json'))
        launch(['ros2', 'launch', 'robot_bringup', 'rviz.launch.py', 'rviz_display:=web',
                'rviz_port:=6082', 'ros_domain_id:=98', f'robots_file:={robots_file}'], 'rviz')
        launch(['ros2', 'run', 'robot_control', 'waypoint', '--robots-file', str(robots_file)], 'controller', slave)
        def ready():
            for path in set((ROOT / '.runtime/rviz').glob('*/session.json')) - before:
                data = json.loads(path.read_text())
                if data['state'] == 'ready':
                    return data
        wait(lambda: ready() and statuses)
        assert statuses[-1]['selected_robot'] == 'go2_0'
        with sync_playwright() as p:
            browser = p.chromium.launch(executable_path=shutil.which('google-chrome'),
                headless=True, args=['--no-sandbox', '--disable-dev-shm-usage'])
            try:
                page = browser.new_page(viewport={'width': 1500, 'height': 1000})
                page.goto('http://127.0.0.1:6082/vnc.html?autoconnect=true&resize=scale')
                page.wait_for_function("document.documentElement.classList.contains('noVNC_connected')")
                page.wait_for_timeout(1200)
                canvas = page.locator('canvas')
                bounds = canvas.bounding_box()
                width, height = canvas.evaluate('(el) => [el.width, el.height]')
                def point(x, y):
                    return bounds['x'] + x * bounds['width'] / width, bounds['y'] + y * bounds['height'] / height
                def click_goal(x, y):
                    page.mouse.click(*point(360, 72))
                    page.wait_for_timeout(500)
                    page.mouse.move(*point(x, y))
                    page.mouse.down()
                    page.mouse.move(*point(x + 30, y), steps=8)
                    page.mouse.up()
                click_goal(900, 400)
                wait(lambda: statuses[-1]['phase'] == 'TRACKING' and statuses[-1]['selected_robot'] == 'go2_0')
                wait(lambda: any(max(map(abs, c)) > .01 for c in commands['go2_0']))
                assert all(max(map(abs, c)) == 0. for c in commands['lekiwi_0'])
                result['first_selected'] = statuses[-1]['selected_robot']
                os.write(master, b' ')
                wait(lambda: statuses[-1]['selected_robot'] == 'lekiwi_0')
                click_goal(880, 500)
                wait(lambda: statuses[-1]['phase'] == 'TRACKING' and statuses[-1]['selected_robot'] == 'lekiwi_0')
                wait(lambda: any(max(map(abs, c)) > .01 for c in commands['lekiwi_0']))
                wait(lambda: markers and any('SELECTED: lekiwi_0' in m.text for m in markers[-1].markers))
                page.wait_for_timeout(800)
                assert all(max(map(abs, c)) == 0. for c in commands['go2_0'][-10:])
                page.screenshot(path=str(output / 'selected_lekiwi.png'))
                result.update(passed=True, second_selected='lekiwi_0', rviz_restart_required=False,
                              marker_selection=True, inactive_commands_zero=True)
            finally:
                browser.close()
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                # Terminal Ctrl+C targets the whole foreground group, including ros2 run's child.
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        result['clean_shutdown'] = all(p.returncode == 0 for p in processes)
        result['passed'] = result.get('passed', False) and result['clean_shutdown']
        stop.set()
        worker.join(timeout=5)
        world.close()
        node.destroy_node()
        rclpy.shutdown()
        os.close(master)
        os.close(slave)
        (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    assert result['passed'], result
    print('WAYPOINT_RVIZ_CHECK_PASS ' + json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
