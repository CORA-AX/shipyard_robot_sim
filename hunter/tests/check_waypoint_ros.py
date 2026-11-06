"""Real waypoint CLI + DDS + CPU bicycle model; no Kit or Isaac physics.

The simplified plant checks ROS lifecycle and recording, not physical accuracy.
Uses ROS domain 94, independent of the user's GUI and keyboard teleoperation.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent))


def check():
    os.environ['ROS_DOMAIN_ID'] = '94'
    os.environ['ROS_HOME'] = str(ROOT/'.runtime/waypoint_ros')
    Path(os.environ['ROS_HOME']).mkdir(parents=True, exist_ok=True)
    from ackermann_msgs.msg import AckermannDriveStamped
    from nav_msgs.msg import Odometry, Path as RosPath
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import UInt64
    from hunter.control.commands import DriveCommand
    from hunter.control.controller import Hunter2Controller, load_parameters
    from hunter.navigation.state import Pose, Segment, advance
    from hunter.ros.bridge import clock_qos, stamp

    robot = load_parameters(ROOT)
    for scenario in ('arrive', 'odometry_loss', 'cancel'):
        context = Context()
        context.init(args=[])
        namespace = 'hunter_waypoint_check'
        node = Node('cpu_bicycle', namespace=namespace, context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        command = [DriveCommand()]
        last_received = [0.]
        paths = []

        def on_command(message):
            assert message.header.frame_id == namespace+'/base_link'
            drive = message.drive
            command[0] = DriveCommand(drive.speed, drive.steering_angle, drive.steering_angle_velocity,
                                      drive.acceleration, drive.jerk)
            last_received[0] = time.monotonic()

        node.create_subscription(AckermannDriveStamped, 'ackermann_cmd', on_command, 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(RosPath, 'waypoint/path', lambda msg: paths.append(msg), latched)
        odom_pub = node.create_publisher(Odometry, 'odom', 1)
        clock_pub = node.create_publisher(Clock, '/clock', clock_qos())
        reset_pub = node.create_publisher(UInt64, 'reset_count', latched)
        reset_pub.publish(UInt64(data=0))
        controller = Hunter2Controller(robot)
        pose, elapsed, moving_since = Pose(-.28009, -.00001, 0), 0., None
        child = None
        try:
            with tempfile.TemporaryDirectory(prefix='result_', dir=ROOT/'.runtime/waypoint_ros') as folder:
                output = Path(folder)/'result.json'
                child = subprocess.Popen([sys.executable, str(ROOT/'scripts/waypoint.py'),
                    '--namespace', namespace, '--x', '3', '--y', '1', '--yaw-deg', '45',
                    '--output', str(output)], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                deadline, interrupted = time.monotonic()+35., False
                while child.poll() is None:
                    assert time.monotonic() < deadline, f'{scenario}: waypoint CLI timed out.'
                    for _ in range(5):
                        executor.spin_once(timeout_sec=0.)
                    request = command[0]
                    target = (controller.stop(.02) if time.monotonic()-last_received[0] > .5
                              else controller.update_drive(request, .02))
                    pose = advance(pose, Segment(target.speed*.02, math.tan(target.steering_angle)/robot['wheelbase']))
                    elapsed += .02
                    if target.speed > .05 and moving_since is None:
                        moving_since = time.monotonic()
                    clock_pub.publish(Clock(clock=stamp(elapsed)))
                    loss = scenario == 'odometry_loss' and moving_since is not None and time.monotonic()-moving_since > .3
                    if not loss:
                        odom = Odometry()
                        odom.header.stamp = stamp(elapsed)
                        odom.header.frame_id, odom.child_frame_id = namespace+'/odom', namespace+'/base_link'
                        odom.pose.pose.position.x, odom.pose.pose.position.y = pose.x, pose.y
                        odom.pose.pose.orientation.w, odom.pose.pose.orientation.z = math.cos(pose.yaw/2.), math.sin(pose.yaw/2.)
                        odom.twist.twist.linear.x, odom.twist.twist.angular.z = target.speed, target.yaw_rate
                        odom_pub.publish(odom)
                    if scenario == 'cancel' and moving_since is not None and not interrupted and time.monotonic()-moving_since > .3:
                        child.send_signal(signal.SIGINT)
                        interrupted = True
                    time.sleep(.01)
                stdout, stderr = child.communicate(timeout=2.)
                assert output.exists(), stdout+'\n'+stderr
                result = json.loads(output.read_text())
                for _ in range(10):
                    executor.spin_once(timeout_sec=.01)
                assert paths and paths[0].header.frame_id == namespace+'/odom'
                assert len(paths[0].poses) >= 2
                assert command[0].speed == 0., 'CLI did not send its final stop command.'
                if scenario == 'arrive':
                    assert child.returncode == 0, stdout+'\n'+stderr
                    assert result['status'] == 'succeeded', result
                    assert result['time_cost_s'] > 0., result
                    assert result['position_error_m'] <= .12 and result['yaw_error_rad'] <= .12, result
                    assert abs(result['final']['speed']) <= .03, result
                else:
                    assert child.returncode != 0, result
                    assert result['time_cost_s'] is None, result
                    expected = 'odometry_timeout' if scenario == 'odometry_loss' else 'user_cancelled'
                    assert result['reason'] == expected, result
                print(json.dumps(dict(scenario=scenario, status=result['status'],
                    reason=result['reason'], time_cost_s=result['time_cost_s'],
                    position_error_m=result.get('position_error_m'))), flush=True)
        finally:
            if child is not None and child.poll() is None:
                child.terminate()
                child.communicate(timeout=5.)
            executor.shutdown()
            node.destroy_node()
            context.try_shutdown()
    print('PASS: real waypoint CLI, DDS, planned path, arrival-time JSON, odometry loss and cancel. '
          'CPU bicycle model only; no Isaac Sim or physical validation.', flush=True)


if __name__ == '__main__':
    if '--worker' in sys.argv:
        check()
    else:
        environment = dict(os.environ)
        for key in ('PYTHONPATH', 'LD_LIBRARY_PATH', 'AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH',
                    'CMAKE_PREFIX_PATH', 'VIRTUAL_ENV', 'PYTHONHOME'):
            environment.pop(key, None)
        raise SystemExit(subprocess.call(['bash', '--noprofile', '--norc', '-c',
            'source "$2"\nexec /usr/bin/python3 "$1" --worker',
            'hunter_waypoint_ros_check', str(Path(__file__).resolve()), str(ROOT/'scripts/ros_env.bash')], env=environment))
