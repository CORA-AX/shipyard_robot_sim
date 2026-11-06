"""Real browser RViz clicks -> selected PD robot, then PTY Space -> second robot, without RViz restart."""
import argparse
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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--nav2', action='store_true', help='Exercise single-Go2 Nav2 and terminal cancellation instead of PD switching.')
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = '98'
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    from visualization_msgs.msg import MarkerArray
    output = ROOT / ('.runtime/navigation/browser' if args.nav2 else '.runtime/waypoint/browser')
    output.mkdir(parents=True, exist_ok=True)
    result = {'passed': False}
    (output / 'result.json').write_text(json.dumps(result) + '\n')
    config = yaml.safe_load((ROOT / 'ros2_ws/src/robot_bringup/config/go2_lekiwi.yaml').read_text())
    if args.nav2:
        config['robots'] = [config['robots'][0]]
        config['robots'][0]['position'] = [0.,0.,.4]
    for robot in config['robots']:
        robot['namespace'] = 'waypoint_browser/' + robot['id']
    robots_file = output / 'robots.yaml'
    robots_file.write_text(yaml.safe_dump(config))
    rclpy.init(args=[])
    node = rclpy.create_node('waypoint_browser_check')
    world = SyntheticWorld(config)
    statuses, markers, commands, plans = [], [], {r['id']: [] for r in config['robots']}, []
    transient = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        node.create_subscription(String, '/waypoint/status', lambda msg: statuses.append(json.loads(msg.data)), transient),
        node.create_subscription(MarkerArray, '/waypoint/markers', markers.append, transient)]
    if args.nav2:
        from nav_msgs.msg import Path as RosPath
        subscriptions.append(node.create_subscription(RosPath, '/waypoint_browser/go2_0/plan', plans.append, 10))
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
                'rviz_port:=6082', 'ros_domain_id:=98', f'robots_file:={robots_file}',
                'rviz_navigation:='+str(args.nav2).lower()], 'rviz')
        if args.nav2:
            launch(['ros2','launch','robot_bringup','navigation.launch.py','start_simulator:=false',
                    'goal_selector:=false','rviz_display:=off','ros_domain_id:=98',f'robots_file:={robots_file}'], 'nav2')
        launch(['ros2', 'run', 'robot_navigation' if args.nav2 else 'robot_control',
                'goal_selector' if args.nav2 else 'waypoint', '--robots-file', str(robots_file)], 'controller', slave)
        def ready():
            for path in set((ROOT / '.runtime/rviz').glob('*/session.json')) - before:
                data = json.loads(path.read_text())
                if data['state'] == 'ready':
                    return data
        wait(lambda: ready() and statuses)
        if args.nav2:
            wait(lambda: statuses[-1]['phase']=='READY')
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
                tracking = 'NAVIGATING' if args.nav2 else 'TRACKING'
                wait(lambda: statuses[-1]['phase'] == tracking and statuses[-1]['selected_robot'] == 'go2_0')
                wait(lambda: any(max(map(abs, c)) > .01 for c in commands['go2_0']))
                if not args.nav2:
                    assert all(max(map(abs, c)) == 0. for c in commands['lekiwi_0'])
                result['first_selected'] = statuses[-1]['selected_robot']
                os.write(master, b' ')
                second = 'go2_0' if args.nav2 else 'lekiwi_0'
                wait(lambda: statuses[-1]['phase']=='CANCELED' if args.nav2 else statuses[-1]['selected_robot']==second)
                if args.nav2:
                    time.sleep(.3)
                    assert all(max(map(abs,c)) == 0. for c in commands['go2_0'][-5:])
                click_goal(880, 500)
                wait(lambda: statuses[-1]['phase']==tracking and statuses[-1]['selected_robot']==second)
                wait(lambda: any(max(map(abs,c)) > .01 for c in commands[second]))
                text = 'go2_0 Nav2' if args.nav2 else 'SELECTED: lekiwi_0'
                wait(lambda: markers and any(text in m.text for m in markers[-1].markers))
                page.wait_for_timeout(800)
                if args.nav2:
                    wait(lambda: plans and plans[-1].poses)
                    displays = yaml.safe_load(Path(ready()['config']).read_text())['Visualization Manager']['Displays']
                    assert any(d['Name']=='Warehouse occupancy map' for d in displays)
                    assert any(d['Class']=='rviz_default_plugins/Path' for d in displays)
                    result.update(nav2_map_and_path=True,space_cancels_single_robot=True)
                else:
                    assert all(max(map(abs,c)) == 0. for c in commands['go2_0'][-10:])
                page.screenshot(path=str(output / ('navigation.png' if args.nav2 else 'selected_lekiwi.png')))
                result.update(passed=True, second_selected=second, rviz_restart_required=False,
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
