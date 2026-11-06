"""Single forward waypoint: ROS transport, lifecycle and simulation-time result."""
import argparse
from dataclasses import asdict
from datetime import datetime
import json
import math
from pathlib import Path
import re
import time

from hunter.control.controller import load_parameters
from hunter.navigation.custom.follower import Mission, load_navigation
from hunter.navigation.state import Pose
from hunter.ros.odometry import read_odometry


def arguments(root, argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--x', type=float, required=True, help='Absolute rear-axle goal x in odom, metres.')
    parser.add_argument('--y', type=float, required=True, help='Absolute rear-axle goal y in odom, metres.')
    parser.add_argument('--yaw-deg', type=float, default=0., help='Absolute vehicle heading, degrees (default 0).')
    parser.add_argument('--speed', type=float, help='Override forward cruise speed, m/s.')
    parser.add_argument('--namespace', default='hunter2_0')
    parser.add_argument('--ackermann-topic')
    parser.add_argument('--odom-topic')
    parser.add_argument('--config', type=Path, default=root/'config/navigation.yaml')
    parser.add_argument('--robot-config', type=Path, default=root/'config/robot.yaml')
    parser.add_argument('--rate', type=float, default=20.)
    parser.add_argument('--odom-timeout', type=float, default=1., help='Abort after this many wall seconds without odometry.')
    parser.add_argument('--max-sim-time', type=float, help='Optional mission time limit; default has no time limit.')
    parser.add_argument('--output', type=Path, help='Result JSON file; default hunter/outputs/waypoint/<timestamp>.json.')
    args = parser.parse_args(argv)
    args.namespace = args.namespace.strip('/')
    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', args.namespace):
        parser.error('Use a valid namespace, such as hunter2_0.')
    args.ackermann_topic = args.ackermann_topic or '/'+args.namespace+'/ackermann_cmd'
    args.odom_topic = args.odom_topic or '/'+args.namespace+'/odom'
    if any(not re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', topic)
           for topic in (args.ackermann_topic, args.odom_topic)):
        parser.error('Use absolute valid topic names.')
    if (not all(math.isfinite(v) for v in (args.x, args.y, args.yaw_deg, args.rate, args.odom_timeout))
            or min(args.rate, args.odom_timeout) <= 0.
            or (args.max_sim_time is not None and (not math.isfinite(args.max_sim_time) or args.max_sim_time <= 0.))):
        parser.error('Use finite goals and positive rates/timeouts.')
    try:
        robot = load_parameters(root, args.robot_config)
        config = load_navigation(root, robot, args.config)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    if args.speed is not None:
        if not math.isfinite(args.speed) or not 0. < args.speed <= robot['max_speed']:
            parser.error('--speed must be positive and within the vehicle speed limit.')
        config['cruise_speed'] = args.speed
    args.output = args.output or root/'outputs/waypoint'/f'{datetime.now():%Y%m%d_%H%M%S_%f}.json'
    if args.output.exists():
        parser.error('Output file already exists; choose a new filename.')
    return args, robot, config


def main(root, argv=None):
    args, robot, config = arguments(Path(root), argv)
    try:
        from ackermann_msgs.msg import AckermannDriveStamped
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry, Path as RosPath
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import String, UInt64
    except ImportError as error:
        raise SystemExit('Run in a system ROS terminal after source hunter/scripts/ros_env.bash. '
                         'If needed, run bash hunter/scripts/setup_ros.sh once.') from error
    from hunter.ros.bridge import stamp

    context = Context()
    context.init(args=[])
    node = Node('single_waypoint', namespace=args.namespace, context=context,
                parameter_overrides=[Parameter('use_sim_time', value=True)])
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    command_pub = node.create_publisher(AckermannDriveStamped, args.ackermann_topic, 1)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    path_pub = node.create_publisher(RosPath, 'waypoint/path', latched)
    status_pub = node.create_publisher(String, 'waypoint/status', latched)
    received = {'observation': None, 'wall_time': None, 'error': None, 'reset': None}
    mission = None
    last_steering = 0.

    def on_odometry(message):
        try:
            received['observation'] = read_odometry(message, args.namespace)
            received['wall_time'] = time.monotonic()
        except ValueError as error:
            received['error'] = str(error)

    def on_reset(message):
        if mission is not None and received['reset'] is not None and message.data != received['reset']:
            received['error'] = 'simulation_reset'
        received['reset'] = message.data

    node.create_subscription(Odometry, args.odom_topic, on_odometry, 1)
    node.create_subscription(UInt64, 'reset_count', on_reset, latched)

    def send(speed, steering):
        message = AckermannDriveStamped()
        observation = received['observation']
        message.header.stamp = stamp(observation.seconds) if observation else node.get_clock().now().to_msg()
        message.header.frame_id = args.namespace+'/base_link'
        message.drive.speed, message.drive.steering_angle = float(speed), float(steering)
        message.drive.acceleration = float(config['deceleration'])
        command_pub.publish(message)

    def publish_status(result):
        status_pub.publish(String(data=json.dumps(result, allow_nan=False)))

    result = None
    try:
        print('HUNTER_WAYPOINT_WAITING: close teleop/other command publishers; '
              'waiting for fresh odometry and a stopped robot.', flush=True)
        next_command = next_log = 0.
        last_state = None
        while context.ok():
            executor.spin_once(timeout_sec=.005)
            # Drain enough callbacks for odom + /clock at the simulator's 50 Hz.
            for _ in range(4):
                executor.spin_once(timeout_sec=0.)
            now = time.monotonic()
            if now < next_command:
                continue
            next_command = now + 1./args.rate
            observation = received['observation']
            if node.count_publishers(args.ackermann_topic) > 1:
                received['error'] = 'another_command_publisher_is_active'
            if received['error']:
                if mission:
                    mission.stop(received['error'])
                    result = mission.result()
                else:
                    result = dict(status='failed', reason=received['error'], time_cost_s=None)
                break
            if observation is None:
                send(0., last_steering)
                continue
            if now-received['wall_time'] > args.odom_timeout:
                if mission:
                    mission.stop('odometry_timeout')
                    result = mission.result()
                else:
                    result = dict(status='failed', reason='odometry_timeout', time_cost_s=None)
                break
            if command_pub.get_subscription_count() == 0:
                send(0., last_steering)
                continue
            if mission is None:
                if abs(observation.speed) > config['stopped_speed'] or abs(observation.yaw_rate) > config['stopped_yaw_rate']:
                    send(0., last_steering)
                    continue
                goal = Pose(args.x, args.y, math.radians(args.yaw_deg))
                mission = Mission(observation, goal, robot, config)
                path_message = RosPath()
                path_message.header.frame_id = args.namespace+'/odom'
                path_message.header.stamp = stamp(observation.seconds)
                for _, pose in mission.path.samples:
                    point = PoseStamped()
                    point.header = path_message.header
                    point.pose.position.x, point.pose.position.y = float(pose.x), float(pose.y)
                    point.pose.orientation.w, point.pose.orientation.z = math.cos(pose.yaw/2.), math.sin(pose.yaw/2.)
                    path_message.poses.append(point)
                path_pub.publish(path_message)
                print('HUNTER_WAYPOINT_STARTED '+json.dumps(dict(goal=asdict(goal),
                      planned_distance_m=mission.path.length, start_sim_s=observation.seconds)), flush=True)
            command = mission.update(observation)
            if args.max_sim_time is not None and observation.seconds-mission.start.seconds >= args.max_sim_time and not mission.done:
                command = mission.stop('simulation_time_limit')
            last_steering = command.steering_angle
            send(command.speed, command.steering_angle)
            if now >= next_log or mission.state != last_state:
                metrics = mission.result()
                summary = {key: metrics[key] for key in ('status', 'elapsed_sim_s', 'position_error_m', 'yaw_error_rad', 'progress_m')}
                print('HUNTER_WAYPOINT_STATE '+json.dumps(summary), flush=True)
                publish_status(summary)
                next_log, last_state = now+1., mission.state
            if mission.done:
                result = mission.result()
                break
        if result is None:
            if mission:
                mission.stop('ros_shutdown')
                result = mission.result()
            else:
                result = dict(status='failed', reason='ros_shutdown', time_cost_s=None)
    except KeyboardInterrupt:
        if mission:
            mission.stop('user_cancelled', cancelled=True)
            result = mission.result()
        else:
            result = dict(status='cancelled', reason='user_cancelled', time_cost_s=None)
    except Exception as error:
        if mission:
            mission.stop(f'{type(error).__name__}: {error}')
            result = mission.result()
        else:
            result = dict(status='failed', reason=f'{type(error).__name__}: {error}', time_cost_s=None)
    finally:
        if context.ok():
            for _ in range(5):
                send(0., last_steering)
                executor.spin_once(timeout_sec=0.)
                time.sleep(.05)
        if result is not None:
            result['result_file'] = str(args.output.resolve())
            result['odom_frame'] = args.namespace+'/odom'
            if mission:
                result['planned_path'] = [dict(s=s, **asdict(pose)) for s, pose in mission.path.samples]
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open('x') as output:
                json.dump(result, output, indent=2, allow_nan=False)
            summary = {key: value for key, value in result.items() if key != 'planned_path'}
            if context.ok():
                publish_status(summary)
            print('HUNTER_WAYPOINT_RESULT '+json.dumps(summary, allow_nan=False), flush=True)
        executor.shutdown()
        node.destroy_node()
        context.try_shutdown()
    return 0 if result and result['status'] == 'succeeded' else (130 if result and result['status'] == 'cancelled' else 1)
