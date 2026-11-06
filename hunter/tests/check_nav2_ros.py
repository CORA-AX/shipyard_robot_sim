"""Actual Smac/RPP Nav2 + DDS against a CPU bicycle plant; no Isaac or RViz.

Goals use RViz's PointStamped and PoseStamped topics. This verifies ROS integration and
Ackermann tracking, not rendered GUI interaction or Isaac contact dynamics.
"""
import json
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent))


def check(environment='flat', patrol=False, patrol_route='smoke'):
    os.environ['ROS_DOMAIN_ID'] = ('98' if patrol_route == 'flat_only' else '97') if patrol else (
        '95' if environment == 'flat' else '96')
    check_name = ('patrol_full_check' if patrol_route == 'flat_only' else 'patrol_check') if patrol else (
        'check' if environment == 'flat' else 'shipyard_check')
    directory = ROOT/'.runtime/nav2'/check_name
    directory.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_LOG_DIR'] = str(directory/'ros_logs')
    from ackermann_msgs.msg import AckermannDriveStamped
    from geometry_msgs.msg import PointStamped, PoseStamped
    from nav_msgs.msg import Odometry, Path as RosPath
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String, UInt64
    from std_srvs.srv import Trigger
    from tf2_msgs.msg import TFMessage
    from hunter.control.commands import DriveCommand
    from hunter.control.controller import Hunter2Controller, load_parameters
    from hunter.navigation.state import Pose, Segment, advance
    from hunter.ros.bridge import RobotRosBridge, clock_qos, stamp
    from hunter.environments.config import load_world
    from hunter.navigation.maps import load_map
    from hunter.navigation.nav2.config import FOOTPRINT

    world = load_world(ROOT,environment)
    grid = load_map(ROOT,world)

    namespace = 'hunter_nav2_check' if environment == 'flat' else 'hunter_shipyard_check'
    context = Context()
    context.init(args=[])
    node = Node('cpu_hunter', namespace=namespace, context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    odom_pub = node.create_publisher(Odometry, 'odom', 1)
    clock_pub = node.create_publisher(Clock, '/clock', clock_qos())
    tf_pub = node.create_publisher(TFMessage, '/tf', 10)
    static_pub = node.create_publisher(TFMessage, '/tf_static', latched)
    reset_pub = node.create_publisher(UInt64, 'reset_count', latched)
    environment_pub = node.create_publisher(String, 'environment', latched)
    goal_pub = node.create_publisher(PoseStamped, 'nav2/goal_pose', 1)
    point_pub = node.create_publisher(PointStamped, 'nav2/goal_point', 1)
    cancel = node.create_client(Trigger, 'nav2/cancel')
    start_route = node.create_client(Trigger, 'patrol/start')
    route_statuses = []
    node.create_subscription(String, 'patrol/status', lambda msg: route_statuses.append(json.loads(msg.data)), latched)
    statuses, paths, requests = [], [], []
    command = [DriveCommand(), -math.inf]

    def on_command(message):
        assert message.header.frame_id == namespace+'/base_link'
        command[:] = [DriveCommand(message.drive.speed, message.drive.steering_angle,
                                   acceleration=message.drive.acceleration), time.monotonic()]
        requests.append([time.monotonic(), command[0].speed, command[0].steering_angle])

    node.create_subscription(AckermannDriveStamped, 'ackermann_cmd', on_command, 1)
    node.create_subscription(String, 'nav2/status', lambda msg: statuses.append(json.loads(msg.data)), latched)
    node.create_subscription(RosPath, 'nav2/selected_path', paths.append, latched)
    static_pub.publish(TFMessage(transforms=[RobotRosBridge._transform(
        'map', namespace+'/odom', 0., (0., 0., 0.), (1., 0., 0., 0.))]))
    reset_pub.publish(UInt64(data=0))
    environment_pub.publish(String(data=environment))
    robot = load_parameters(ROOT)
    controller = Hunter2Controller(robot)
    pose = Pose(world.position[0]-.28009*math.cos(world.yaw)+.00001*math.sin(world.yaw),
                world.position[1]-.28009*math.sin(world.yaw)-.00001*math.cos(world.yaw),world.yaw)
    seconds = 0.
    footprint_collisions = []
    last_tick = time.monotonic()
    publish_odom = True
    child = None

    def spin():
        nonlocal pose, seconds, last_tick
        for _ in range(16):
            executor.spin_once(timeout_sec=0.)
        now = time.monotonic()
        if now-last_tick >= .01:
            last_tick = now
            targets = (controller.update_drive(command[0], .02) if now-command[1] < .5 else controller.stop(.02))
            pose = advance(pose, Segment(targets.speed*.02, math.tan(targets.steering_angle)/robot['wheelbase']))
            if environment == 'shipyard' and not grid.footprint_free(pose.x,pose.y,pose.yaw,FOOTPRINT):
                footprint_collisions.append([pose.x,pose.y,pose.yaw])
            seconds += .02
            clock_pub.publish(Clock(clock=stamp(seconds)))
            if publish_odom:
                transform = RobotRosBridge._transform(namespace+'/odom', namespace+'/base_link', seconds,
                    (pose.x, pose.y, .17), (math.cos(pose.yaw/2.), 0., 0., math.sin(pose.yaw/2.)))
                tf_pub.publish(TFMessage(transforms=[transform]))
                odom = Odometry()
                odom.header, odom.child_frame_id = transform.header, transform.child_frame_id
                odom.pose.pose.position.x, odom.pose.pose.position.y = pose.x, pose.y
                odom.pose.pose.orientation = transform.transform.rotation
                odom.twist.twist.linear.x, odom.twist.twist.angular.z = targets.speed, targets.yaw_rate
                odom_pub.publish(odom)
        if child and child.poll() is not None:
            raise AssertionError((directory/'runner.log').read_text()[-6000:])
        time.sleep(.002)

    def wait(predicate, timeout=60.):
        deadline = time.monotonic()+timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise AssertionError(dict(last_status=statuses[-2:], pose=vars(pose),
                    log=(directory/'runner.log').read_text()[-6000:]))
            spin()

    def advance_time(duration):
        end = seconds+duration
        wait(lambda: seconds >= end)

    def send(x, y, yaw=0., frame='map'):
        goal = PoseStamped()
        goal.header.frame_id, goal.header.stamp = frame, stamp(seconds)
        goal.pose.position.x, goal.pose.position.y = float(x), float(y)
        goal.pose.orientation.w, goal.pose.orientation.z = math.cos(yaw/2.), math.sin(yaw/2.)
        goal_pub.publish(goal)

    def send_point(x, y, frame='map'):
        goal = PointStamped()
        goal.header.frame_id, goal.header.stamp = frame, stamp(seconds)
        goal.point.x, goal.point.y = float(x), float(y)
        point_pub.publish(goal)

    def arrival(x, y, yaw):
        start = len(statuses)
        send_point(x, y) if yaw is None else send(x, y, yaw)
        wait(lambda: any(s['phase'] == 'NAVIGATING' for s in statuses[start:]))
        wait(lambda: statuses[-1]['phase'] in ('SUCCEEDED', 'FAILED'), timeout=100.)
        assert statuses[-1]['phase'] == 'SUCCEEDED', statuses[-3:]
        result = statuses[-1]['last_result']
        assert result['position_error_m'] <= .15, result
        if yaw is None:
            assert result['goal'] == [x, y, None] and result['yaw_error_rad'] is None, result
            assert result['goal_mode'] == 'position', result
            lengths = [candidate['distance_m'] for candidate in result['heading_candidates']
                       if candidate['distance_m'] is not None]
            assert len(result['heading_candidates']) >= 12 and len(lengths) >= 2, result
            assert abs(result['planned_distance_m']-min(lengths)) < 1e-9, result
            assert max(lengths) > min(lengths)+1., result
        else:
            assert result['goal_mode'] == 'pose' and result['yaw_error_rad'] <= .15, result
        assert abs(result['final_speed_m_s']) <= .03 and result['time_cost_s'] > 0., result
        assert Path(statuses[-1]['result_file']).is_file()
        print('NAV2_ARRIVAL '+json.dumps(result), flush=True)
        advance_time(.5)
        assert command[0].speed == 0.
        return result

    result = dict(passed=False, backend='cpu_bicycle_actual_nav2', environment=environment, arrivals=[])
    (directory/'result.json').write_text(json.dumps(result))
    try:
        with (directory/'runner.log').open('w') as log:
            child = subprocess.Popen([sys.executable, str(ROOT/'scripts'/('patrol.py' if patrol else 'nav2.py')), '--no-rviz',
                                       '--namespace', namespace,'--environment',environment,
                                       *(['--route-idle','--route',patrol_route] if patrol else [])], stdout=log, stderr=subprocess.STDOUT)
        wait(lambda: statuses and statuses[-1]['phase'] == 'READY', timeout=90.)
        assert node.count_publishers('/'+namespace+'/ackermann_cmd') == 1
        if patrol:
            assert route_statuses[-1]['phase'] == 'WAITING'
            send_point(-22.,18.)
            advance_time(.5)
            assert command[0].speed == 0. and route_statuses[-1]['phase'] == 'WAITING'
            wait(start_route.service_is_ready)
            future = start_route.call_async(Trigger.Request())
            wait(future.done)
            assert future.result().success
            wait(lambda: route_statuses[-1]['phase'] in ('SUCCEEDED','FAILED'), timeout=400.)
            report = route_statuses[-1]['last_result']
            assert route_statuses[-1]['phase'] == 'SUCCEEDED', report
            from hunter.navigation.routes import load_route
            route = load_route(ROOT,patrol_route,grid)
            assert report['completed_waypoints'] == report['total_waypoints'] == len(route.points)
            assert [p['checkpoint'] for p in report['checkpoints']] == [p.name for p in route.points]
            assert report['time_cost_s'] > 0.
            assert report['time_cost_s']+1e-9 >= sum(
                p['result']['time_cost_s'] for p in report['checkpoints'])
            assert all(p['result']['position_error_m'] <= .15 and
                       abs(p['result']['final_speed_m_s']) <= .03 for p in report['checkpoints'])
            assert Path(route_statuses[-1]['result_file']).is_file()
            result['round_trip'] = report
            advance_time(1.)
            assert command[0].speed == 0. and abs(controller.speed) < .03
            future = start_route.call_async(Trigger.Request())
            wait(future.done)
            assert future.result().success
            wait(lambda: statuses[-1]['phase'] == 'NAVIGATING' and command[0].speed > .05)
            future = cancel.call_async(Trigger.Request())
            wait(future.done)
            wait(lambda: route_statuses[-1]['phase'] == 'CANCELLED')
            advance_time(2.)
            assert route_statuses[-1]['phase'] == 'CANCELLED' and command[0].speed == 0.
            assert route_statuses[-1]['last_result']['time_cost_s'] is None
            future = start_route.call_async(Trigger.Request())
            wait(future.done)
            assert future.result().success
            wait(lambda: statuses[-1]['phase'] == 'PLANNING')
            publish_odom = False
            wait(lambda: route_statuses[-1]['phase'] == 'FAILED')
            assert route_statuses[-1]['last_result']['reason'] == 'odometry_timeout'
            publish_odom = True
            advance_time(2.)
            assert route_statuses[-1]['phase'] == 'FAILED' and command[0].speed == 0.
            assert route_statuses[-1]['last_result']['time_cost_s'] is None
            assert not footprint_collisions, footprint_collisions[:5]
            result.update(passed=True, click_rejected=True, cancelled_queue_does_not_resume=True,
                          odometry_failure_stops_queue=True, footprint_collisions=0)
            (directory/'result.json').write_text(json.dumps(result,indent=2)+'\n')
            print('PASS: stored Hunter patrol, all stopped arrivals, round trip, time costs, '
                  'RViz input rejection, queue cancellation and odometry failure; actual Nav2 + CPU bicycle.',flush=True)
            return
        send(2., 0., frame='wrong_frame')
        send_point(2., 0., frame='wrong_frame')
        advance_time(.5)
        assert statuses[-1]['phase'] == 'READY' and command[0].speed == 0.
        result['invalid_goal_rejected'] = True
        goals = ((3.,0.,None),(5.,3.,None),(8.,5.,0.)) if environment == 'flat' else (
            (-22.,18.,None),(-25.,12.,None),(-25.,8.,-math.pi/2.))
        for goal in goals:
            result['arrivals'].append(arrival(*goal))
        if environment == 'shipyard':
            index = len(statuses)
            send_point(0.,0.)  # Dry dock/no supported ground in the exported map.
            advance_time(.5)
            assert not any(s['phase'] in ('PLANNING','NAVIGATING') for s in statuses[index:])
            assert command[0].speed == 0.
            result['unsupported_goal_rejected'] = True
        assert paths and all(path.header.frame_id == 'map' for path in paths)
        assert all(0. <= v <= robot['max_speed'] and abs(a) <= robot['max_steering_angle']
                   for _, v, a in requests)
        result['path_and_ackermann_limits'] = True
        index = len(statuses)
        send_point(pose.x, pose.y)
        wait(lambda: any(s['phase'] == 'SETTLING' for s in statuses[index:]))
        wait(lambda: statuses[-1]['phase'] == 'SUCCEEDED')
        assert statuses[-1]['last_result']['planned_distance_m'] == 0.
        assert statuses[-1]['last_result']['heading_candidates'] == []
        assert command[0].speed == 0.
        result['already_near_position_stays_stopped'] = True
        index = len(statuses)
        far1 = (12.,5.,0.) if environment == 'flat' else (-25.,-6.,-math.pi/2.)
        far2 = (14.,5.,0.) if environment == 'flat' else (-25.,-12.,-math.pi/2.)
        send_point(*far1[:2])
        wait(lambda: statuses[-1]['phase'] == 'PLANNING' and statuses[-1]['heading_search'] is not None)
        future = cancel.call_async(Trigger.Request())
        wait(future.done)
        wait(lambda: statuses[-1]['phase'] == 'CANCELLED')
        advance_time(1.5)
        assert command[0].speed == 0.
        assert not any(s['phase'] == 'NAVIGATING' for s in statuses[index:]), statuses[index:]
        result['cancel_during_heading_search_does_not_start_following'] = True
        send(*far1)
        wait(lambda: statuses[-1]['phase'] == 'NAVIGATING' and command[0].speed > .05)
        # A second RViz click must not silently replace the active target.
        send(10., 8.)
        advance_time(.4)
        assert statuses[-1]['goal'][:2] == list(far1[:2]), statuses[-1]
        assert abs(statuses[-1]['goal'][2]-far1[2]) < 1e-9, statuses[-1]
        future = cancel.call_async(Trigger.Request())
        wait(future.done)
        wait(lambda: statuses[-1]['phase'] == 'CANCELLED')
        advance_time(1.5)
        assert command[0].speed == 0. and abs(controller.speed) < .03
        assert statuses[-1]['last_result']['time_cost_s'] is None
        result['cancel_and_no_automatic_resume'] = True
        send(*far2)
        wait(lambda: statuses[-1]['phase'] == 'NAVIGATING' and command[0].speed > .05)
        publish_odom = False
        wait(lambda: statuses[-1]['phase'] == 'FAILED')
        assert statuses[-1]['reason'] == 'odometry_timeout', statuses[-1]
        advance_time(.5)
        assert command[0].speed == 0.
        result['stale_odometry_stops'] = True
        publish_odom = True
        advance_time(1.5)
        send(*far2)
        wait(lambda: statuses[-1]['phase'] == 'NAVIGATING' and command[0].speed > .05)
        reset_pub.publish(UInt64(data=1))
        wait(lambda: statuses[-1]['phase'] == 'FAILED' and statuses[-1]['reason'] == 'simulation_reset')
        advance_time(1.)
        assert command[0].speed == 0.
        result['reset_stops'] = True
        send(*far2)
        wait(lambda: statuses[-1]['phase'] == 'NAVIGATING' and command[0].speed > .05)
        competing = node.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 1)
        wait(lambda: statuses[-1]['phase'] == 'FAILED' and
             statuses[-1]['reason'] == 'another_command_publisher_is_active')
        advance_time(1.)
        assert command[0].speed == 0.
        node.destroy_publisher(competing)
        result['command_owner_conflict_stops'] = True
        assert not footprint_collisions, footprint_collisions[:5]
        result['footprint_collisions'] = len(footprint_collisions)
        environment_pub.publish(String(data='shipyard' if environment == 'flat' else 'flat'))
        advance_time(.5)
        send(*far2)
        advance_time(.3)
        assert command[0].speed == 0. and statuses[-1]['phase'] == 'WAITING'
        result['wrong_environment_rejected'] = True
        result['passed'] = True
        (directory/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        print('PASS: actual Nav2 Smac/RPP, RViz point/pose goals, shortest heading candidate, '
              'straight/turning arrivals, already-near stop, planning cancel, '
              'goal rejection, cancel, odometry loss, reset, command ownership. '
              'CPU plant; no Isaac/RViz GUI.', flush=True)
    finally:
        if child and child.poll() is None:
            child.send_signal(signal.SIGINT)
            child.wait(timeout=12.)
        executor.shutdown()
        node.destroy_node()
        context.try_shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--environment',choices=('flat','shipyard'),default='flat')
    parser.add_argument('--worker',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--patrol',action='store_true',help='Test stored shipyard round-trip queue.')
    parser.add_argument('--patrol-route',choices=('smoke','flat_only'),default='smoke')
    args = parser.parse_args()
    if args.patrol:
        args.environment = 'shipyard'
    if args.worker:
        check(args.environment,args.patrol,args.patrol_route)
    else:
        environment = dict(os.environ)
        for key in ('PYTHONPATH', 'LD_LIBRARY_PATH', 'AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH',
                    'CMAKE_PREFIX_PATH', 'VIRTUAL_ENV', 'PYTHONHOME'):
            environment.pop(key, None)
        raise SystemExit(subprocess.call(['bash', '--noprofile', '--norc', '-c',
            'source "$2"\nexec /usr/bin/python3 "$1" --environment "$3" --worker ${4:+--patrol} --patrol-route "$5"',
            'hunter_nav2_ros_check', str(Path(__file__).resolve()), str(ROOT/'scripts/nav2_env.bash'),args.environment,
            'yes' if args.patrol else '',args.patrol_route], env=environment))
