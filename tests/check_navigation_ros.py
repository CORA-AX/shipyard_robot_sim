"""Real Humble Nav2 against synthetic dynamics or Isaac Go2 warehouse physics."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import yaml

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=('synthetic', 'isaac'), default='synthetic')
    parser.add_argument('--domain', type=int, default=99, help='Isolated ROS domain for this check.')
    args = parser.parse_args()
    if not 0 <= args.domain <= 232:
        parser.error('--domain must be 0..232')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry, OccupancyGrid, Path as RosPath
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String, UInt64
    from std_srvs.srv import Trigger
    from robot_navigation.goal_selector import goal_is_free
    from check_waypoint_ros import SyntheticWorld

    directory = ROOT / '.runtime/navigation' / args.backend
    directory.mkdir(parents=True, exist_ok=True)
    result_file = directory / 'result.json'
    result = dict(passed=False, backend=args.backend)
    result_file.write_text(json.dumps(result) + '\n')
    (directory/'simulation.json').unlink(missing_ok=True)
    # The authored forklift near (-6,14) gives a short, real obstruction test.
    configuration = {'world': {'scenario': 'warehouse', 'seed': 42}, 'robots': [
        dict(id='go2_0', type='go2', namespace='nav_check/go2_0', position=[-6.,12.,.4], yaw=0.)]}
    robots_file = directory / 'robots.yaml'
    robots_file.write_text(yaml.safe_dump(configuration))
    prefix = '/nav_check/go2_0'
    rclpy.init(args=[])
    node = rclpy.create_node('navigation_integration_check')
    transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    odom, statuses, commands, paths, maps, resets = [], [], [], [], [], []
    subscriptions = [
        node.create_subscription(Odometry, prefix+'/odom', odom.append, 10),
        node.create_subscription(String, '/waypoint/status', lambda msg: statuses.append(json.loads(msg.data)), transient),
        node.create_subscription(Twist, prefix+'/cmd_vel', lambda msg: commands.append(
            (time.monotonic(), (msg.linear.x,msg.linear.y,msg.angular.z))), 10),
        node.create_subscription(RosPath, prefix+'/plan', paths.append, 10),
        node.create_subscription(OccupancyGrid, '/map', maps.append, transient),
        node.create_subscription(UInt64, prefix+'/reset_count', lambda msg: resets.append(msg.data), transient)]
    goal_pub = node.create_publisher(PoseStamped, '/waypoint/goal_pose', 1)
    cancel = node.create_client(Trigger, '/waypoint/cancel')
    reset = node.create_client(Trigger, prefix+'/reset')
    launch = synthetic = None
    history = []

    def spin(duration=.02):
        until = time.monotonic()+duration
        while time.monotonic() < until:
            if synthetic:
                synthetic.tick()
            for _ in range(24):
                rclpy.spin_once(node, timeout_sec=0.)
            if launch and launch.poll() is not None:
                raise RuntimeError((directory/'launch.log').read_text()[-7000:])
            time.sleep(.002)
        if odom:
            pose = odom[-1].pose.pose
            history.append([pose.position.x,pose.position.y,2*math.atan2(pose.orientation.z,pose.orientation.w)])

    def wait(predicate, timeout=40):
        deadline = time.monotonic()+timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise AssertionError(('Timeout', statuses[-3:], history[-3:],
                                      (directory/'launch.log').read_text()[-4000:]))
            spin()

    def phase(seconds):
        stamp = odom[-1].header.stamp
        end = stamp.sec+stamp.nanosec/1e9+seconds
        wait(lambda: odom[-1].header.stamp.sec+odom[-1].header.stamp.nanosec/1e9 >= end,
             timeout=max(20, seconds*8))

    def send(x, y, yaw=0.):
        message = PoseStamped()
        message.header.frame_id = 'map'
        message.header.stamp = odom[-1].header.stamp
        message.pose.position.x, message.pose.position.y = float(x),float(y)
        message.pose.orientation.w, message.pose.orientation.z = math.cos(yaw/2),math.sin(yaw/2)
        index = len(statuses)
        goal_pub.publish(message)
        return index

    def arrival(x,y,yaw=0.):
        index = send(x,y,yaw)
        wait(lambda: any(s['phase'] == 'NAVIGATING' for s in statuses[index:]))
        # A queued REACHED from the previous goal can arrive between send()
        # and NAVIGATING. Wait for the current phase after navigation starts.
        wait(lambda: statuses[-1]['phase'] in ('REACHED','FAILED','RESET','ODOM_TIMEOUT'), timeout=150)
        assert statuses[-1]['phase'] == 'REACHED', statuses[-3:]
        phase(.4)
        p = odom[-1].pose.pose.position
        error = math.hypot(p.x-x,p.y-y)
        assert error <= .15, (error,p,x,y)
        q = odom[-1].pose.pose.orientation
        measured_yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        yaw_error = math.atan2(math.sin(yaw-measured_yaw),math.cos(yaw-measured_yaw))
        assert abs(yaw_error) <= .15, (yaw_error,measured_yaw,yaw)
        result.setdefault('arrivals',[]).append(dict(goal=[x,y,yaw],position_error=error,yaw_error=yaw_error))
        print('NAVIGATION_ARRIVAL '+json.dumps(result['arrivals'][-1]),flush=True)

    def zero_after(start):
        samples = [values for stamp,values in commands if stamp >= start+.10]
        assert len(samples) > 8 and all(max(map(abs,v)) < 1e-8 for v in samples), samples[-10:]

    try:
        if args.backend == 'synthetic':
            synthetic = SyntheticWorld(configuration)
        with (directory/'launch.log').open('w') as log:
            launch = subprocess.Popen(['ros2','launch','robot_bringup','navigation.launch.py',
                f'robots_file:={robots_file}', 'navigation_robot:=go2_0',
                'start_simulator:='+('true' if args.backend == 'isaac' else 'false'),
                'display:=headless','rviz_display:=off',f'ros_domain_id:={args.domain}',
                f'report:={directory / "simulation.json"}'], cwd=ROOT,
                env=dict(os.environ,ROS_LOG_DIR=str(directory/'ros_logs')),
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        wait(lambda: odom and maps and statuses and statuses[-1]['phase']=='READY',timeout=300)
        assert node.count_publishers(prefix+'/cmd_vel') == 1
        assert node.count_publishers('/clock') == 1
        assert abs(maps[-1].info.resolution-.05) < 1e-7
        result['single_command_and_clock_owner'] = True
        phase(.5)
        arrival(-5.2,12.,math.pi/2)
        path_start, history_start = len(paths),len(history)
        # This straight segment actually crosses the authored forklift.
        a,b=(-5.2,12.),(-6.1,15.8)
        blocked = sum(not goal_is_free(maps[-1],a[0]+(b[0]-a[0])*i/100,
                                       a[1]+(b[1]-a[1])*i/100,0.) for i in range(101))
        assert blocked > 0, 'Obstacle test does not cross an actual obstacle.'
        arrival(*b,-math.pi/2)
        route = [pose for path in paths[path_start:] for pose in path.poses]
        def deviation(x,y):
            return abs((b[0]-a[0])*(a[1]-y)-(a[0]-x)*(b[1]-a[1]))/math.dist(a,b)
        maximum = max(deviation(p.pose.position.x,p.pose.position.y) for p in route)
        assert maximum > .45, maximum
        collisions = [(x,y,yaw) for x,y,yaw in history[history_start:]
                      if not goal_is_free(maps[-1],x,y,yaw,pad_cells=False)]
        assert not collisions, collisions[:5]
        result['obstacle_detour'] = dict(maximum_path_deviation=maximum,
            straight_blocked_samples=blocked,checked_robot_poses=len(history)-history_start,footprint_collisions=0,
            collision_check='Occupied/unknown grid cell centers inside the padded rectangular footprint')
        phase(.7)
        index, start = len(statuses),time.monotonic()
        send(-6.1,13.8)  # Inside the forklift: reject without moving.
        phase(.8)
        assert not any(s['phase'] in ('NAVIGATING','SENDING','STOPPING') for s in statuses[index:])
        zero_after(start)
        result['occupied_goal_rejected'] = True
        index = send(-4.3,15.8)
        wait(lambda: any(s['phase']=='NAVIGATING' for s in statuses[index:]))
        wait(lambda: max(map(abs,commands[-1][1])) > .05)
        assert cancel.service_is_ready()
        future = cancel.call_async(Trigger.Request())
        wait(future.done)
        wait(lambda: statuses[-1]['phase']=='CANCELED')
        start = time.monotonic()
        phase(1.)
        zero_after(start)
        result['cancel_stops_without_resuming'] = True
        index = send(-4.3,15.8)
        wait(lambda: any(s['phase']=='NAVIGATING' for s in statuses[index:]))
        wait(lambda: max(map(abs,commands[-1][1])) > .05)
        future = reset.call_async(Trigger.Request())
        wait(future.done)
        wait(lambda: resets and resets[-1] > 0 and statuses[-1]['phase']=='RESET')
        start = time.monotonic()
        phase(1.2)
        zero_after(start)
        assert resets[-1] == 1, resets
        result['reset_stops_without_resuming'] = True
        # A fresh, explicit post-reset goal must work.
        arrival(-5.2,12.,-math.pi/2)
        # Same-position goals must also turn to the requested heading.
        p = odom[-1].pose.pose.position
        arrival(p.x,p.y,math.pi/2)
        if synthetic:
            index = send(-4.2,12.)
            wait(lambda: any(s['phase']=='NAVIGATING' for s in statuses[index:]))
            synthetic.drop.add('go2_0')
            wait(lambda: statuses[-1]['phase']=='ODOM_TIMEOUT')
            start = time.monotonic()
            spin(.4)
            zero_after(start)
            synthetic.drop.clear()
            phase(.5)
            assert statuses[-1]['phase']=='ODOM_TIMEOUT'
            result['stale_odometry_closes_gate'] = True
        policy = ROOT/'assets/go2/policies/upstream_flat_6800.pt'
        result['policy_sha256'] = hashlib.sha256(policy.read_bytes()).hexdigest()
        assert result['policy_sha256'] == 'd358d4a99a327f9c34f8538755655affd50cecf9a194a0ce0d374ba3228df8b0'
        result['passed'] = True
    except BaseException as exc:
        result['error'] = repr(exc)
        raise
    finally:
        (directory/'trajectory.json').write_text(json.dumps(history)+'\n')
        if launch and launch.poll() is None:
            os.killpg(launch.pid,signal.SIGINT)
            try:
                launch.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid,signal.SIGKILL)
                launch.wait()
        if synthetic:
            synthetic.close()
        node.destroy_node()
        rclpy.shutdown()
        if args.backend == 'isaac' and (directory/'simulation.json').exists():
            simulation = json.loads((directory/'simulation.json').read_text())
            actor = simulation.get('robots',{}).get('go2_0',simulation)
            result['simulation'] = dict(resets=actor.get('resets'),
                minimum_base_height_m=actor.get('minimum_base_height_m'),
                reset_history=actor.get('reset_history'),policy=simulation.get('policy'),error=simulation.get('error'))
            if result['passed'] and (actor.get('resets') != 1 or simulation.get('error') or
                                    simulation['policy']['sha256'] != result['policy_sha256']):
                result.update(passed=False,error='Simulation report did not confirm exactly one manual reset and the selected policy.')
        elif args.backend == 'isaac' and result['passed']:
            result.update(passed=False,error='Simulator did not produce a fresh final report.')
        result_file.write_text(json.dumps(result,indent=2)+'\n')
    assert result['passed'],result
    print('NAVIGATION_CHECK_PASS '+json.dumps(result),flush=True)


if __name__ == '__main__':
    main()
