"""Publish Hunter speed and steering angle using AckermannDriveStamped."""
import argparse
import math
import re
import time

from hunter.control.keyboard import TerminalKeyboard, add_keyboard_arguments, validate_keyboard_arguments


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    add_keyboard_arguments(parser)
    parser.add_argument('--robot-id', default='hunter2_0')
    parser.add_argument('--namespace')
    parser.add_argument('--topic')
    parser.add_argument('--drive-speed', type=float, default=0., help='Fixed signed rear-axle speed in m/s.')
    parser.add_argument('--steering-angle', type=float, default=0., help='Fixed virtual steering angle in radians; left positive.')
    parser.add_argument('--rate', type=float, default=20.)
    parser.add_argument('--duration', type=float)
    parser.add_argument('--wait-timeout', type=float, default=15.)
    args = parser.parse_args()
    validate_keyboard_arguments(parser, args)
    namespace = (args.namespace or args.robot_id).strip('/')
    topic = args.topic or '/' + namespace + '/ackermann_cmd'
    if (not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', namespace)
            or not re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', topic)
            or not all(math.isfinite(v) for v in
                (args.drive_speed, args.steering_angle, args.rate, args.wait_timeout))
            or abs(args.steering_angle) >= math.pi / 2
            or args.rate <= 0 or args.wait_timeout <= 0
            or (args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0))):
        parser.error('Use valid namespace/topic, finite velocities and positive duration/rate/timeouts.')
    try:
        from ackermann_msgs.msg import AckermannDriveStamped
    except ImportError as error:
        raise SystemExit('Ackermann messages unavailable. Run bash hunter/scripts/setup_ros.sh once, '
                         'then source hunter/scripts/ros_env.bash in this terminal.') from error
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    context = Context()
    context.init(args=[])
    node = Node('teleop', namespace=namespace, context=context,
                parameter_overrides=[Parameter('use_sim_time', value=True)])
    publisher = node.create_publisher(AckermannDriveStamped, topic, 1)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    last_steering = 0.

    def publish(speed, steering):
        executor.spin_once(timeout_sec=0.)
        message = AckermannDriveStamped()
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = namespace + '/base_link'
        message.drive.speed, message.drive.steering_angle = speed, steering
        publisher.publish(message)
    try:
        until = time.monotonic() + args.wait_timeout
        while publisher.get_subscription_count() == 0:
            if time.monotonic() >= until:
                raise RuntimeError(f'No subscriber for {topic}; start hunter/scripts/simulate.py --ros first.')
            time.sleep(.05)
        duration = args.duration
        until = float('inf') if duration is None else time.monotonic() + duration
        print(f'HUNTER_COMMAND_READY: {topic}; speed m/s, steering angle rad; '
              'base_link is the rear axle midpoint.', flush=True)
        try:
            with TerminalKeyboard(args) as keyboard:
                while context.ok() and time.monotonic() < until:
                    command = keyboard.poll() if args.keyboard else (args.drive_speed, args.steering_angle)
                    last_steering = command[1]
                    if keyboard.quit:
                        break
                    publish(*command)
                    time.sleep(1. / args.rate)
        finally:
            for _ in range(5):
                publish(0., last_steering)
                time.sleep(.05)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        context.try_shutdown()
