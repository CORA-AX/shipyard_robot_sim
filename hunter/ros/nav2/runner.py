"""Own the Hunter-only Nav2 processes and optional local RViz window."""
import argparse
from datetime import datetime
import os
from pathlib import Path
import signal
import subprocess
import time

import yaml
from hunter.control.controller import load_parameters
from hunter.navigation.nav2.config import configured_parameters, namespace_value, rviz_configuration
from hunter.environments.config import load_world
from hunter.navigation.maps import load_map
from hunter.ros.nav2.rviz import rviz_environment


def arguments(root, argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='hunter2_0')
    parser.add_argument('--speed', type=float, default=.4, help='Forward cruise speed, m/s.')
    parser.add_argument('--environment', choices=('flat', 'shipyard'), default='flat')
    parser.add_argument('--config', type=Path, default=Path(root)/'config/nav2.yaml')
    parser.add_argument('--robot-config', type=Path, default=Path(root)/'config/robot.yaml')
    parser.add_argument('--no-rviz', action='store_true', help='Only run the ROS stack (no Isaac startup).')
    parser.add_argument('--rviz-rendering', choices=('software','native'), default='software',
                        help='RViz only: software avoids indexed-map GLSL driver errors (default).')
    parser.add_argument('--route', choices=('smoke', 'flat_only'),
                        help='Stored shipyard round trip instead of RViz click goals.')
    parser.add_argument('--route-idle', action='store_true',
                        help='Wait for /<namespace>/patrol/start rather than automatically starting.')
    args = parser.parse_args(argv)
    if args.route and args.environment != 'shipyard':
        parser.error('Stored routes require --environment shipyard.')
    if args.route_idle and not args.route:
        parser.error('--route-idle requires --route.')
    try:
        args.namespace = namespace_value(args.namespace)
        robot = load_parameters(root, args.robot_config)
        parameters = configured_parameters(root, robot, args.namespace, args.speed, args.config)
    except (ValueError, KeyError, OSError) as error:
        parser.error(str(error))
    return args, robot, parameters


def main(root, argv=None):
    root = Path(root)
    args, robot, parameters = arguments(root, argv)
    try:
        world = load_world(root,args.environment)
        grid = load_map(root,world)
        route = None
        if args.route:
            from hunter.navigation.routes import load_route, patrol_parameters
            route = load_route(root, args.route, grid)
            parameters = patrol_parameters(parameters, args.namespace)
    except (OSError, ValueError) as error:
        raise SystemExit(str(error)) from error
    try:
        from ament_index_python.packages import get_package_prefix, get_package_share_directory
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from hunter.ros.nav2.goal import HunterNav2Goal
        binaries = {}
        for package, executable in (('nav2_planner', 'planner_server'), ('nav2_controller', 'controller_server'),
                                    ('nav2_bt_navigator', 'bt_navigator'),
                                    ('nav2_lifecycle_manager', 'lifecycle_manager'), ('rviz2', 'rviz2')):
            if package == 'rviz2' and args.no_rviz:
                continue
            binaries[executable] = Path(get_package_prefix(package))/'lib'/package/executable
            if not binaries[executable].is_file():
                raise RuntimeError(f'Missing {package}/{executable}')
        for package in ('nav2_smac_planner', 'nav2_regulated_pure_pursuit_controller'):
            get_package_share_directory(package)
    except (ImportError, LookupError, RuntimeError) as error:
        raise SystemExit(f'{error}\nRun bash hunter/scripts/setup_nav2.sh, then source hunter/scripts/nav2_env.bash '
                         'in a system-Python terminal.') from error
    if not args.no_rviz and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        raise SystemExit('No GUI display. Use a local desktop terminal for RViz, or --no-rviz '
                         'and connect RViz separately. Isaac Sim is launched separately.')
    directory = root/'.runtime/nav2/sessions'/f'{datetime.now():%Y%m%d_%H%M%S_%f}'
    directory.mkdir(parents=True)
    params_file, rviz_file = directory/'parameters.yaml', directory/'hunter.rviz'
    params_file.write_text(yaml.safe_dump(parameters, sort_keys=False))
    rviz_file.write_text(yaml.safe_dump(rviz_configuration(args.namespace,world), sort_keys=False))
    environment = dict(os.environ, ROS_LOG_DIR=str(directory/'ros_logs'))
    context = Context()
    context.init(args=[])
    executor = SingleThreadedExecutor(context=context)
    node = None
    children, logs = [], []
    exit_code = 0

    def start(name, extra):
        log = (directory/(name+'.log')).open('w')
        logs.append(log)
        command = [str(binaries[name]), '--ros-args', '-r', '__ns:=/'+args.namespace, *extra]
        child_environment = rviz_environment(environment,args.rviz_rendering) if name == 'rviz2' else environment
        children.append((name, subprocess.Popen(command, env=child_environment, stdout=log,
                                                 stderr=subprocess.STDOUT, start_new_session=True)))

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    previous_sigterm = signal.signal(signal.SIGTERM, interrupt)
    try:
        if route is None:
            node = HunterNav2Goal(root, args.namespace, robot, context, world=world, grid=grid)
        else:
            from hunter.ros.nav2.patrol import HunterPatrol
            node = HunterPatrol(root, args.namespace, robot, context, world=world, grid=grid,
                                route=route, idle=args.route_idle)
        executor.add_node(node)
        for name in ('planner_server', 'controller_server', 'bt_navigator'):
            extra = ['--params-file', str(params_file)]
            if name == 'controller_server':
                extra += ['-r', 'cmd_vel:=nav2/cmd_vel']
            if name == 'bt_navigator':
                extra += ['-r', 'goal_pose:=nav2/internal_goal_pose']
            start(name, extra)
        start('lifecycle_manager', ['-r', '__node:=hunter_navigation_lifecycle',
            '-p', 'use_sim_time:=true', '-p', 'autostart:=true',
            '-p', 'bond_timeout:=10.0',
            '-p', 'node_names:=[planner_server, controller_server, bt_navigator]'])
        if not args.no_rviz:
            start('rviz2', ['-p', 'use_sim_time:=true', '--', '-d', str(rviz_file)])
        print(f'HUNTER_NAV2_STARTING logs={directory}\n'
              f'Environment: {world.name}; RViz rendering: {args.rviz_rendering}.\n'
              f'Waiting for simulator --ros and active Nav2. Cancel: /{args.namespace}/nav2/cancel', flush=True)
        if route is None:
            print(f'Point: /{args.namespace}/nav2/goal_point; pose: /{args.namespace}/nav2/goal_pose', flush=True)
        if route is not None:
            print(f'HUNTER_PATROL route={route.name}; RViz click goals disabled. '
                  f'{"Waiting for start service" if args.route_idle else "Starts automatically when READY"}.\n'
                  f'Start: /{args.namespace}/patrol/start; cancel: /{args.namespace}/patrol/cancel', flush=True)
        while context.ok():
            executor.spin_once(timeout_sec=.02)
            for _ in range(10):
                executor.spin_once(timeout_sec=0.)
            for name, child in children:
                if child.poll() is not None:
                    tail = (directory/(name+'.log')).read_text(errors='replace')[-5000:]
                    raise RuntimeError(f'{name} exited ({child.returncode}). Log: {directory}/{name}.log\n{tail}')
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f'HUNTER_NAV2_ERROR: {error}', flush=True)
        exit_code = 1
    finally:
        # Close the actuator command gate before stopping any asynchronous node.
        if node is not None:
            node.abort('cancelled', 'navigation_shutdown')
            for _ in range(10):
                node.send_stop()
                executor.spin_once(timeout_sec=0.)
                time.sleep(.02)
        for _, child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
        deadline = time.monotonic()+5.
        for _, child in children:
            if child.poll() is None:
                try:
                    child.wait(timeout=max(.1, deadline-time.monotonic()))
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait(timeout=2.)
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        context.try_shutdown()
        for log in logs:
            log.close()
        signal.signal(signal.SIGTERM, previous_sigterm)
    return exit_code
