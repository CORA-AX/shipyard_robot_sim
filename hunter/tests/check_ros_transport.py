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


def publisher():
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from tf2_msgs.msg import TFMessage
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

    rclpy.init(args=[])
    node = rclpy.create_node('hunter2_transport_check')
    received = set()
    steering_received = [False]
    subscriptions = []

    def state(topic, msg):
        received.add(topic)
        if topic.endswith('/joint_states'):
            steering_received[0] |= any(abs(v) > .01 for v in msg.position[:2])

    for kind, topic, qos in (
        (Odometry, '/hunter2_0/odom', 10), (JointState, '/hunter2_0/joint_states', 10),
        (TFMessage, '/tf', 10),
        (TFMessage, '/tf_static', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        (Clock, '/clock', QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)),
    ):
        subscriptions.append(node.create_subscription(kind, topic,
                            lambda msg, topic=topic: state(topic, msg), qos))
    pub = node.create_publisher(Twist, '/hunter2_0/cmd_vel', 1)
    try:
        until = time.monotonic() + 10.
        while pub.get_subscription_count() == 0:
            assert time.monotonic() < until, 'No Hunter command subscriber discovered.'
            rclpy.spin_once(node, timeout_sec=.02)
        for command in ((.5, 0., 0.), (.5, 0., .2), (.5, 0., -.2),
                        (-.5, 0., .2), (-.5, 0., -.2), (0., 0., 0.), (.5, .1, 0.), (0., 0., 1.),
                        (float('nan'), 0., 0.), (.5, 0., 0.), None, (-.5, 0., 0.), (0., 0., 0.)):
            until = time.monotonic() + .7
            while time.monotonic() < until:
                if command is not None:
                    msg = Twist()
                    msg.linear.x, msg.linear.y, msg.angular.z = command
                    pub.publish(msg)
                rclpy.spin_once(node, timeout_sec=.01)
                time.sleep(.03)
        assert len(received) == 5, f'Missing state topics: {received}'
        assert steering_received[0], 'No nonzero steering state received.'
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
    bridge = RobotRosBridge('/hunter2_0/cmd_vel', .3, robot_id='hunter2', namespace='hunter2_0')
    child = None
    try:
        environment = dict(os.environ)
        for key in ('PYTHONPATH', 'LD_LIBRARY_PATH', 'AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH',
                    'CMAKE_PREFIX_PATH', 'VIRTUAL_ENV', 'PYTHONHOME'):
            environment.pop(key, None)
        child = subprocess.Popen(['bash', '--noprofile', '--norc', '-c',
            'source /opt/ros/humble/setup.bash\nexec /usr/bin/python3 "$1" --publisher',
            'hunter2_ros_check', str(Path(__file__).resolve())],
            env=environment, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        seen, stale_after_command, elapsed = set(), False, 0.
        positive = negative = left = right = False
        robot = SimpleNamespace(joint_names=[entry['joint'] for group in ('steering', 'rear_wheels')
                                for entry in parameters[group]] + parameters['passive_front_wheels'],
            data=SimpleNamespace(root_pos_w=np.array([[0., 0., .35]]),
                root_quat_w=np.array([[1., 0., 0., 0.]]), root_lin_vel_b=np.zeros((1, 3)),
                root_ang_vel_b=np.zeros((1, 3)), joint_pos=np.zeros((1, 6)), joint_vel=np.zeros((1, 6))))
        deadline = time.monotonic() + 30.
        while child.poll() is None:
            assert time.monotonic() < deadline, 'ROS transport check timed out.'
            command = bridge.poll()
            seen.add(command)
            stale_after_command |= bridge.command.stale and bridge.command.received > 0
            targets = controller.update_command(command, .02)
            positive |= targets.speed > 0.
            negative |= targets.speed < 0.
            left |= targets.steering_angle > .01
            right |= targets.steering_angle < -.01
            robot.data.joint_pos[0, :2] = targets.steering_positions
            robot.data.joint_vel[0, 2:4] = targets.rear_velocities
            robot.data.root_lin_vel_b[0, 0] = targets.speed
            robot.data.root_ang_vel_b[0, 2] = targets.yaw_rate
            bridge.publish_clock(elapsed)
            bridge.publish_robot_state(elapsed, robot)
            elapsed += .02
            time.sleep(.02)
        stdout, stderr = child.communicate(timeout=2.)
        assert child.returncode == 0, stderr
        assert {(.5, 0., 0.), (-.5, 0., 0.), (0., 0., 0.), (.5, 0., .2),
                (.5, 0., -.2), (-.5, 0., .2), (-.5, 0., -.2), (.5, .1, 0.), (0., 0., 1.)} <= seen, seen
        assert bridge.command.rejected > 0, 'NaN was not rejected.'
        assert positive and negative and left and right and stale_after_command
        print(stdout.strip())
        print('PASS: forward/reverse/stop, left/right steering, invalid input, command expiry/recovery; '
              'system Humble and bundled rclpy communicate. No Isaac physics was started.')
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
