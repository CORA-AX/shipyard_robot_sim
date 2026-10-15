"""Supervise native RViz or an isolated X11/noVNC session over an SSH tunnel."""
import argparse
import json
import os
from pathlib import Path
import re
import secrets
import select
import shutil
import signal
import socket
import subprocess
import tempfile
import time

import yaml

from robot_sim.config import load_world_config


def resolve_display(requested, simulation_display=None):
    if requested not in ('auto', 'local', 'web', 'off'):
        raise ValueError('rviz_display must be auto, local, web or off.')
    if requested == 'auto':
        return {'webrtc': 'web', 'headless': 'off'}.get(simulation_display, 'local')
    return requested


def validate_options(mode, resolution, port, rendering, domain):
    match = re.fullmatch(r'(\d+)x(\d+)', resolution)
    if not match or not (640 <= int(match[1]) <= 4096 and 480 <= int(match[2]) <= 2160):
        raise ValueError('rviz_resolution must be WIDTHxHEIGHT, within 640x480..4096x2160.')
    if mode not in ('local', 'web') or not 1024 <= port <= 65535 or not 0 <= domain <= 232:
        raise ValueError('Use local/web, a port in 1024..65535 and a ROS domain in 0..232.')
    if rendering not in ('auto', 'software', 'native'):
        raise ValueError('rviz_rendering must be auto, software or native.')
    return int(match[1]), int(match[2])


def system_environment():
    """RViz and its helper tools use system Humble, never Isaac's Python/Qt/GL libraries."""
    env = os.environ.copy()
    for key in ('PYTHONHOME', 'QT_PLUGIN_PATH', 'QT_QPA_PLATFORM_PLUGIN_PATH', 'QML2_IMPORT_PATH'):
        env.pop(key, None)
    for key in ('PATH', 'PYTHONPATH', 'LD_LIBRARY_PATH'):
        if key in env:
            env[key] = ':'.join(path for path in env[key].split(':')
                                if path and not any(part in path.lower() for part in
                                    ('/conda', '/.conda', '/miniconda', '/anaconda', '/isaacsim', '/omni.')))
    env['PATH'] = '/opt/ros/humble/bin:/usr/bin:/bin:' + env.get('PATH', '')
    return env


def web_tools(root):
    sysroot = Path(root) / '.runtime/rviz_web/sysroot'
    tools = {name: sysroot / 'usr/bin' / name for name in ('Xvfb', 'x11vnc', 'openbox')}
    tools['web'] = sysroot / 'usr/share/novnc'
    tools['python'] = sysroot / 'usr/lib/python3/dist-packages'
    tools['lib'] = sysroot / 'usr/lib/x86_64-linux-gnu'
    tools['data'] = sysroot / 'usr/share'
    tools['wm_config'] = sysroot / 'etc/xdg/openbox/rc.xml'
    required = [tools[name] for name in ('Xvfb', 'x11vnc', 'openbox', 'wm_config')]
    required += [tools['web'] / 'vnc.html', tools['python'] / 'websockify/__main__.py']
    if not all(path.is_file() for path in required):
        raise ValueError('RViz web tools missing. Run: bash ros2_ws/setup_rviz_web.sh')
    return tools


def make_config(world, robot_id, template, width, height, goal_topic='/waypoint/goal_pose'):
    robots = world['robots']
    selected = next((robot for robot in robots if robot['robot_id'] == robot_id), None) if robot_id else robots[0]
    if selected is None:
        raise ValueError(f'Unknown rviz_robot {robot_id!r}; choose a robot id from robots_file.')
    config = yaml.safe_load(Path(template).read_text())
    manager = config['Visualization Manager']
    # RViz sends unassigned goals; the terminal tracker owns live robot selection.
    manager['Views']['Current']['Focal Point'].update(X=float(selected['position'][0]),
                                                     Y=float(selected['position'][1]))
    for tool in manager['Tools']:
        if tool['Class'] == 'rviz_default_plugins/SetGoal':
            tool['Topic'] = {'Value': goal_topic, 'Reliability Policy': 'Reliable',
                             'Durability Policy': 'Volatile', 'Depth': 5}
    for index, robot in enumerate(robots):
        manager['Displays'].append({
            'Class': 'rviz_default_plugins/Odometry', 'Name': robot['robot_id'] + ' odom',
            'Enabled': True, 'Keep': 1,
            'Shape': {'Value': 'Arrow',
                      'Color': '80; 200; 255' if index % 2 == 0 else '120; 230; 130',
                      'Shaft Length': .4, 'Shaft Radius': .035,
                      'Head Length': .15, 'Head Radius': .08},
            'Covariance': {'Value': False},
            'Topic': {'Value': '/' + robot['namespace'] + '/odom',
                      'Reliability Policy': 'Reliable', 'Durability Policy': 'Volatile', 'Depth': 10}})
    manager['Displays'].append({
        'Class': 'rviz_default_plugins/MarkerArray', 'Name': 'Waypoint selection and goal',
        'Enabled': True,
        'Topic': {'Value': '/waypoint/markers', 'Reliability Policy': 'Reliable',
                  'Durability Policy': 'Transient Local', 'Depth': 1}})
    config['Window Geometry'].update(Width=width - 40, Height=height - 60, X=10, Y=10)
    return config, goal_topic


class Viewer:
    def __init__(self, args):
        self.args = args
        self.root = Path(args.project_root).resolve()
        runtime = self.root / '.runtime/rviz'
        runtime.mkdir(parents=True, exist_ok=True)
        self.directory = Path(tempfile.mkdtemp(prefix='view_', dir=runtime))
        self.children = []
        self.stopping = False
        self.record = {'pid': os.getpid(), 'mode': args.display, 'state': 'starting',
                       'directory': str(self.directory), 'processes': {}}

    def spawn(self, name, command, env, **kwargs):
        log_path = self.directory / (name + '.log')
        with log_path.open('w') as log:
            child = subprocess.Popen(list(map(str, command)), env=env, stdout=log,
                                     stderr=subprocess.STDOUT, start_new_session=True, **kwargs)
        self.children.append((name, child))
        self.record['processes'][name] = child.pid
        self.save()
        return child

    def save(self):
        pending = self.directory / 'session.json.tmp'
        pending.write_text(json.dumps(self.record, indent=2) + '\n')
        pending.replace(self.directory / 'session.json')

    def check(self):
        if self.stopping:
            raise InterruptedError('Viewer shutdown requested.')
        for name, child in self.children:
            if child.poll() is not None:
                tail = (self.directory / (name + '.log')).read_text(errors='replace')[-2500:]
                raise RuntimeError(f'{name} exited with code {child.returncode}.\n{tail}')

    def wait_port(self, port):
        until = time.monotonic() + 15
        while time.monotonic() < until:
            self.check()
            try:
                with socket.create_connection(('127.0.0.1', port), timeout=.2):
                    return
            except OSError:
                time.sleep(.1)
        raise RuntimeError(f'Timed out waiting for localhost:{port}.')

    def start_web(self, env, width, height):
        tools = web_tools(self.root)
        # Check the public-facing tunnel endpoint before creating any GUI process.
        with socket.socket() as probe:
            try:
                probe.bind(('127.0.0.1', self.args.port))
            except OSError as error:
                raise ValueError(f'RViz port {self.args.port} is in use; set rviz_port to another port.') from error
        helpers = env.copy()
        helpers['LD_LIBRARY_PATH'] = str(tools['lib']) + ':' + env.get('LD_LIBRARY_PATH', '')
        helpers['PYTHONPATH'] = str(tools['python']) + ':' + env.get('PYTHONPATH', '')
        helpers['XDG_DATA_DIRS'] = str(tools['data']) + ':/usr/local/share:/usr/share'
        authority = self.directory / 'Xauthority'
        authority.touch(mode=0o600)
        cookie = secrets.token_hex(16)
        subprocess.run(['/usr/bin/xauth', '-f', str(authority), 'add', ':0', '.', cookie], check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        read_fd, write_fd = os.pipe()
        try:
            self.spawn('xvfb', [tools['Xvfb'], '-displayfd', write_fd, '-screen', '0',
                               f'{width}x{height}x24', '-nolisten', 'tcp', '-noreset',
                               '-auth', authority], helpers, pass_fds=(write_fd,))
            os.close(write_fd)
            write_fd = None
            deadline, number = time.monotonic() + 15, b''
            while b'\n' not in number and time.monotonic() < deadline:
                self.check()
                if select.select([read_fd], [], [], .1)[0]:
                    chunk = os.read(read_fd, 64)
                    if not chunk:
                        raise RuntimeError('Xvfb closed its display readiness pipe.')
                    number += chunk
            if not number.strip().isdigit():
                raise RuntimeError('Xvfb did not allocate a display.')
        finally:
            os.close(read_fd)
            if write_fd is not None:
                os.close(write_fd)
        display = ':' + number.decode().strip()
        subprocess.run(['/usr/bin/xauth', '-f', str(authority), 'add', display, '.', cookie], check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        env.update(DISPLAY=display, XAUTHORITY=str(authority), QT_QPA_PLATFORM='xcb',
                   QT_X11_NO_MITSHM='1')
        env.pop('WAYLAND_DISPLAY', None)
        helpers.update(DISPLAY=display, XAUTHORITY=str(authority))
        self.spawn('openbox', [tools['openbox'], '--sm-disable', '--config-file', tools['wm_config']], helpers)
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 0))
            vnc_port = probe.getsockname()[1]
        self.spawn('x11vnc', [tools['x11vnc'], '-norc', '-display', display, '-auth', authority,
                             '-localhost', '-no6', '-noipv6', '-listen', '127.0.0.1',
                             '-rfbport', vnc_port, '-rfbportv6', '-1', '-nopw',
                             '-forever', '-shared', '-xkb'], helpers)
        self.wait_port(vnc_port)
        self.spawn('websockify', ['/usr/bin/python3', '-m', 'websockify', '--web', tools['web'],
                                 f'127.0.0.1:{self.args.port}', f'127.0.0.1:{vnc_port}'], helpers)
        self.wait_port(self.args.port)
        self.record.update(display=display, vnc_port=vnc_port, web_port=self.args.port)

    def run(self):
        args = self.args
        width, height = validate_options(args.display, args.resolution, args.port,
                                         args.rendering, args.ros_domain_id)
        world = load_world_config(args.robots_file)
        from ament_index_python.packages import get_package_share_directory
        template = Path(get_package_share_directory('robot_bringup')) / 'config/robots.rviz'
        config, goal_topic = make_config(world, args.robot, template, width, height, args.goal_topic)
        config_path = self.directory / 'robots.rviz'
        config_path.write_text(yaml.safe_dump(config, sort_keys=False))
        env = system_environment()
        env['ROS_DOMAIN_ID'] = str(args.ros_domain_id)
        env['ROS_LOG_DIR'] = str(self.directory / 'ros_logs')
        if args.display == 'web':
            xdg = self.directory / 'xdg'
            xdg.mkdir(mode=0o700)
            env['XDG_RUNTIME_DIR'] = str(xdg)
            self.start_web(env, width, height)
        elif not (env.get('DISPLAY') or env.get('WAYLAND_DISPLAY')):
            raise ValueError('No local GUI display. In SSH use rviz_display:=web.')
        software = args.rendering == 'software' or (args.rendering == 'auto' and args.display == 'web')
        if software:
            env.update(LIBGL_ALWAYS_SOFTWARE='1', __GLX_VENDOR_LIBRARY_NAME='mesa')
        rviz = shutil.which('rviz2', path=env['PATH'])
        if not rviz:
            raise ValueError('rviz2 not found; source /opt/ros/humble/setup.bash.')
        child = self.spawn('rviz', [rviz, '-d', config_path, '--ros-args', '-p', 'use_sim_time:=true'], env)
        # Keep startup failures distinct from a user closing an established RViz window.
        until = time.monotonic() + 2
        while time.monotonic() < until:
            self.check()
            time.sleep(.1)
        self.record.update(state='ready', goal_topic=goal_topic, config=str(config_path),
                           software_rendering=software)
        self.save()
        print(f'RVIZ_{args.display.upper()}_READY: goal={goal_topic}; logs={self.directory}', flush=True)
        if args.display == 'web':
            print(f'RVIZ_BROWSER: http://localhost:{args.port}/vnc.html?autoconnect=true&resize=scale', flush=True)
            print(f'RVIZ_SSH: ssh -N -L {args.port}:127.0.0.1:{args.port} USER@SERVER', flush=True)
        while not self.stopping:
            if child.poll() == 0:
                return
            self.check()
            time.sleep(.1)

    def close(self):
        for _, child in reversed(self.children):
            try:
                os.killpg(child.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + 5
        for _, child in reversed(self.children):
            try:
                child.wait(timeout=max(.01, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        self.record.update(state='closed', returncodes={name: child.returncode for name, child in self.children})
        self.save()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', required=True)
    parser.add_argument('--robots-file', required=True)
    parser.add_argument('--display', choices=('local', 'web'), default='local')
    parser.add_argument('--robot', default='')
    parser.add_argument('--goal-topic', default='/waypoint/goal_pose')
    parser.add_argument('--port', type=int, default=6080)
    parser.add_argument('--resolution', default='1440x900')
    parser.add_argument('--rendering', choices=('auto', 'software', 'native'), default='auto')
    parser.add_argument('--ros-domain-id', type=int, default=int(os.environ.get('ROS_DOMAIN_ID', '0')))
    viewer = Viewer(parser.parse_args())
    def stop(signum, frame):
        viewer.stopping = True
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop)
    try:
        viewer.run()
    except InterruptedError:
        pass
    except Exception as error:
        viewer.record['error'] = str(error)
        print(f'RVIZ_ERROR: {error}', flush=True)
        raise SystemExit(1) from error
    finally:
        viewer.close()


if __name__ == '__main__':
    main()
