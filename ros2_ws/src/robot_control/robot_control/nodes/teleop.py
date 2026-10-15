"""Send body velocity from the system Humble Python, optionally using a terminal."""
import argparse
import math
import re
import time

from robot_control.keyboard import TerminalKeyboard, add_keyboard_arguments, validate_keyboard_arguments


def main():
    from geometry_msgs.msg import Twist
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.utilities import remove_ros_args
    parser = argparse.ArgumentParser(description=__doc__)
    add_keyboard_arguments(parser)
    parser.add_argument('--robot-id', default='go2_0')
    parser.add_argument('--namespace')
    parser.add_argument('--topic')
    parser.add_argument('--vx', type=float, default=0.)
    parser.add_argument('--vy', type=float, default=0.)
    parser.add_argument('--wz', type=float, default=0.)
    parser.add_argument('--rate', type=float, default=20.)
    parser.add_argument('--duration', type=float)
    parser.add_argument('--wait-timeout', type=float, default=15.)
    args = parser.parse_args(remove_ros_args()[1:])
    validate_keyboard_arguments(parser, args)
    namespace = (args.namespace or args.robot_id).strip('/')
    topic = args.topic or '/' + namespace + '/cmd_vel'
    if (not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', namespace)
            or not topic.startswith('/') or not all(math.isfinite(v) for v in
                (args.vx, args.vy, args.wz, args.rate, args.wait_timeout))
            or args.rate <= 0 or args.wait_timeout <= 0
            or (args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0))):
        parser.error('Use valid namespace/topic, finite velocities and positive duration/rate/timeouts.')
    context = Context()
    context.init(args=[])
    node = Node('teleop', namespace=namespace, context=context)
    publisher = node.create_publisher(Twist, topic, 1)
    try:
        until = time.monotonic() + args.wait_timeout
        while publisher.get_subscription_count() == 0:
            if time.monotonic() >= until:
                raise RuntimeError(f'No subscriber for {topic}; start simulation.launch.py first.')
            time.sleep(.05)
        duration = args.duration if args.duration is not None else (None if args.keyboard else 5.)
        until = float('inf') if duration is None else time.monotonic() + duration
        print(f'ROBOT_COMMAND_READY: {topic}; system Humble publisher.', flush=True)
        try:
            with TerminalKeyboard(args) as keyboard:
                while context.ok() and time.monotonic() < until:
                    command = keyboard.poll() if args.keyboard else (args.vx, args.vy, args.wz)
                    if keyboard.quit:
                        break
                    message = Twist()
                    message.linear.x, message.linear.y, message.angular.z = command
                    publisher.publish(message)
                    time.sleep(1. / args.rate)
        finally:
            for _ in range(5):
                publisher.publish(Twist())
                time.sleep(.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        context.try_shutdown()
