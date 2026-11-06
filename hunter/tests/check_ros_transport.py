"""CPU-only DDS check: system Humble <-> Isaac-bundled rclpy, without Isaac Sim.

Uses the Hunter-local environment helper to select Isaac-bundled rclpy.
Robot state is synthetic; this check makes no claim about physical motion.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent))
REAR_OFFSET = (-.28009, -.00001, -.19558)  # Synthetic reference geometry, metres.


def publisher():
    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from tf2_msgs.msg import TFMessage
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

    rclpy.init(args=[])
    node = rclpy.create_node('hunter2_transport_check')
    received = set()
    steering_received = [False]
    turning_received = [False]
    chassis_tf_received = [False]
    subscriptions = []

    def state(topic, msg):
        received.add(topic)
        if topic.endswith('/joint_states'):
            steering_received[0] |= any(abs(v) > .01 for v in msg.position[:2])
        if topic == '/hunter2_0/odom':
            assert msg.header.frame_id == 'hunter2_0/odom'
            assert msg.child_frame_id == 'hunter2_0/base_link'
            assert abs(msg.pose.pose.position.x - REAR_OFFSET[0]) < 1e-6
            assert abs(msg.pose.pose.position.y - REAR_OFFSET[1]) < 1e-6
            assert abs(msg.pose.pose.position.z - (.35 + REAR_OFFSET[2])) < 1e-6
            assert abs(msg.twist.twist.linear.y) < 1e-6
            turning_received[0] |= abs(msg.twist.twist.angular.z) > .01
        if topic == '/tf':
            assert len(msg.transforms) == 1
            transform = msg.transforms[0]
            assert transform.header.frame_id == 'hunter2_0/odom'
            assert transform.child_frame_id == 'hunter2_0/base_link'
            assert abs(transform.transform.translation.x - REAR_OFFSET[0]) < 1e-6
        if topic == '/tf_static':
            for transform in msg.transforms:
                if transform.child_frame_id == 'hunter2_0/chassis_link':
                    assert transform.header.frame_id == 'hunter2_0/base_link'
                    translation = transform.transform.translation
                    assert all(abs(a-b) < 1e-6 for a, b in zip(
                        (translation.x, translation.y, translation.z), tuple(-v for v in REAR_OFFSET)))
                    chassis_tf_received[0] = True

    for kind, topic, qos in (
        (Odometry, '/hunter2_0/odom', 10),
        (JointState, '/hunter2_0/joint_states', 10),
        (TFMessage, '/tf', 10),
        (TFMessage, '/tf_static', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        (Clock, '/clock', QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)),
    ):
        subscriptions.append(node.create_subscription(kind, topic,
                            lambda msg, topic=topic: state(topic, msg), qos))
    pub = node.create_publisher(AckermannDriveStamped, '/hunter2_0/ackermann_cmd', 1)
    try:
        until = time.monotonic() + 10.
        while pub.get_subscription_count() == 0:
            assert time.monotonic() < until, 'No Hunter command subscriber discovered.'
            rclpy.spin_once(node, timeout_sec=.02)
        # speed, virtual steering angle, test variant (not a Twist/yaw command).
        for command in ((0., .3, ''), (.5, 0., ''), (.5, .2, ''), (.5, -.2, ''),
                        (-.5, .2, ''), (-.5, -.2, ''), (0., 0., ''), (.5, .2, 'wrong_frame'),
                        (.5, .2, 'jerk'), (float('nan'), 0., ''), (.5, 0., ''),
                        None, (-.5, 0., ''), (0., 0., '')):
            until = time.monotonic() + .7
            while time.monotonic() < until:
                if command is not None:
                    msg = AckermannDriveStamped()
                    msg.header.frame_id = 'map' if command[2] == 'wrong_frame' else 'hunter2_0/base_link'
                    msg.drive.speed, msg.drive.steering_angle = command[:2]
                    msg.drive.jerk = 1. if command[2] == 'jerk' else 0.
                    pub.publish(msg)
                rclpy.spin_once(node, timeout_sec=.01)
                time.sleep(.03)
        assert len(received) == 5, f'Missing state topics: {received}'
        assert steering_received[0], 'No nonzero steering state received.'
        assert turning_received[0] and chassis_tf_received[0], 'Missing rear-centred odometry / chassis TF.'
        print(json.dumps({'received_state_topics': sorted(received)}), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def check():
    import numpy as np
    from hunter.ros.bridge import RobotRosBridge
    from hunter.control.controller import Hunter2Controller, load_parameters

    # The checker owns an isolated ROS domain, leaving user GUI sessions alone.
    os.environ['ROS_DOMAIN_ID'] = '93'
    os.environ['ROS_HOME'] = str(ROOT / '.runtime/ros_transport')
    parameters = load_parameters(ROOT)
    controller = Hunter2Controller(parameters)
    bridge = RobotRosBridge('/hunter2_0/ackermann_cmd', .3, rear_axle_offset=REAR_OFFSET,
                            robot_id='hunter2', namespace='hunter2_0')
    child = None
    try:
        environment = dict(os.environ)
        for key in ('PYTHONPATH', 'LD_LIBRARY_PATH', 'AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH',
                    'CMAKE_PREFIX_PATH', 'VIRTUAL_ENV', 'PYTHONHOME'):
            environment.pop(key, None)
        child = subprocess.Popen(['bash', '--noprofile', '--norc', '-c',
            'source "$2"\nexec /usr/bin/python3 "$1" --publisher',
            'hunter2_ros_check', str(Path(__file__).resolve()), str(ROOT / 'scripts/ros_env.bash')],
            env=environment, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        seen, stale_after_command, elapsed = set(), False, 0.
        positive = negative = left = right = stationary_steering = False
        robot = SimpleNamespace(joint_names=[entry['joint'] for group in ('steering', 'rear_wheels')
                                for entry in parameters[group]] + parameters['passive_front_wheels'],
            data=SimpleNamespace(root_link_pos_w=np.array([[0., 0., .35]]),
                root_link_quat_w=np.array([[1., 0., 0., 0.]]), root_link_lin_vel_b=np.zeros((1, 3)),
                root_link_ang_vel_b=np.zeros((1, 3)), root_lin_vel_b=np.full((1, 3), 999.),
                joint_pos=np.zeros((1, 6)), joint_vel=np.zeros((1, 6))))
        deadline = time.monotonic() + 30.
        while child.poll() is None:
            assert time.monotonic() < deadline, 'ROS transport check timed out.'
            command = bridge.poll()
            seen.add((round(command.speed, 2), round(command.steering_angle, 2)))
            stale_after_command |= bridge.command.stale and bridge.command.received > 0
            targets = controller.stop(.02) if bridge.command.stale else controller.update_drive(command, .02)
            positive |= targets.speed > 0.
            negative |= targets.speed < 0.
            left |= targets.steering_angle > .01
            right |= targets.steering_angle < -.01
            stationary_steering |= targets.speed == 0. and abs(targets.steering_angle) > .2
            robot.data.joint_pos[0, :2] = targets.steering_positions
            robot.data.joint_vel[0, 2:4] = targets.rear_velocities
            robot.data.root_link_lin_vel_b[0, 0] = targets.speed + targets.yaw_rate*REAR_OFFSET[1]
            robot.data.root_link_lin_vel_b[0, 1] = -targets.yaw_rate*REAR_OFFSET[0]
            robot.data.root_link_ang_vel_b[0, 2] = targets.yaw_rate
            bridge.publish_clock(elapsed)
            bridge.publish_robot_state(elapsed, robot)
            elapsed += .02
            time.sleep(.02)
        stdout, stderr = child.communicate(timeout=2.)
        assert child.returncode == 0, stderr
        assert {(.5, 0.), (-.5, 0.), (0., 0.), (0., .3), (.5, .2),
                (.5, -.2), (-.5, .2), (-.5, -.2)} <= seen, seen
        assert bridge.command.rejected > 0, 'Invalid commands were not rejected.'
        assert positive and negative and left and right and stationary_steering and stale_after_command
        print(stdout.strip())
        # Exercise the real fixed-command CLI, its message type, clock and stop-on-exit.
        child = subprocess.Popen(['bash', '--noprofile', '--norc', '-c',
            'source "$2"\nexec /usr/bin/python3 "$1" --drive-speed 0.4 --steering-angle -0.2 --duration 0.5',
            'hunter2_teleop_check', str(ROOT / 'scripts/teleop.py'), str(ROOT / 'scripts/ros_env.bash')],
            env=environment, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        teleop_seen = set()
        deadline = time.monotonic() + 15.
        while child.poll() is None:
            assert time.monotonic() < deadline, 'Real teleop timed out.'
            command = bridge.poll()
            teleop_seen.add((round(command.speed, 2), round(command.steering_angle, 2)))
            bridge.publish_clock(elapsed)
            elapsed += .02
            time.sleep(.01)
        stdout, stderr = child.communicate(timeout=2.)
        assert child.returncode == 0, stderr
        assert {(.4, -.2), (0., -.2)} <= teleop_seen, teleop_seen
        print(stdout.strip())
        print('PASS: Ackermann forward/reverse/stop, steering at rest, invalid input, command expiry/recovery; '
              'rear-centred odometry and TF; real teleop; system Humble and bundled rclpy communicate. '
              'No Isaac physics was started.')
    finally:
        if child is not None and child.poll() is None:
            child.terminate()
            child.communicate(timeout=5.)
        bridge.close()


if __name__ == '__main__':
    if '--publisher' in sys.argv:
        publisher()
    elif '--worker' in sys.argv:
        check()
    else:
        from hunter.sim.environment import isaac_environment
        prefix = Path(os.environ.get('HUNTER_ISAAC_PREFIX', ROOT.parent / '.conda/isaacsim-5.0'))
        python = prefix / 'bin/python'
        environment = isaac_environment(python, python_paths=[ROOT.parent])
        raise SystemExit(subprocess.call([str(python), str(Path(__file__).resolve()), '--worker'],
                                        env=environment))
