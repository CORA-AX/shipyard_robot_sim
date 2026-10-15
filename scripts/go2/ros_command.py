#!/usr/bin/env python3
"""Publish Go2 cmd_vel using the bundled Humble libraries; no system ROS installation needed."""
import argparse
import math
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from control.go2.ros_bridge import ros_environment
from control.go2.keyboard import TerminalKeyboard, add_keyboard_arguments, validate_keyboard_arguments


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    add_keyboard_arguments(parser)
    parser.add_argument('--vx', type=float, default=0.0)
    parser.add_argument('--vy', type=float, default=0.0)
    parser.add_argument('--wz', type=float, default=0.0, help='Yaw velocity in rad/s.')
    parser.add_argument('--duration', type=float,
                        help='Wall seconds to publish; default 5, or unlimited with --keyboard.')
    parser.add_argument('--rate', type=float, default=20.0)
    parser.add_argument('--topic', default='/unitree_go2/cmd_vel')
    parser.add_argument('--ros-domain-id', type=int, default=int(os.environ.get('ROS_DOMAIN_ID', '0')))
    args = parser.parse_args()
    validate_keyboard_arguments(parser, args)
    if (not all(math.isfinite(v) for v in (args.vx, args.vy, args.wz, args.rate))
            or (args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0))
            or args.rate <= 0 or not 0 <= args.ros_domain_id <= 232):
        parser.error('Use finite velocities, positive duration/rate and ROS domain 0..232.')
    if not os.environ.get('WAREHOUSE_ROS_COMMAND_WORKER'):
        environment = ros_environment()
        environment['ROS_DOMAIN_ID'] = str(args.ros_domain_id)
        environment['WAREHOUSE_ROS_COMMAND_WORKER'] = '1'
        os.execve(sys.executable, [sys.executable, str(Path(__file__).resolve()), *sys.argv[1:]], environment)
    from geometry_msgs.msg import Twist
    from rclpy.context import Context
    from rclpy.node import Node
    context = Context()
    context.init(args=[])
    node = None
    try:
        node = Node('warehouse_go2_command', context=context)
        publisher = node.create_publisher(Twist, args.topic, 1)
        deadline = time.monotonic() + 5.0
        while publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                raise SystemExit('No cmd_vel subscriber found. Start ros.py with the same topic/domain first.')
            time.sleep(0.05)
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = args.vx, args.vy, args.wz
        print(f'Publishing ({args.vx}, {args.vy}, {args.wz}) on {args.topic}', flush=True)
        duration = args.duration if args.duration is not None else (None if args.keyboard else 5.0)
        deadline = time.monotonic() + duration if duration is not None else float('inf')
        try:
            with TerminalKeyboard(args) as keyboard:
                while time.monotonic() < deadline and context.ok():
                    if args.keyboard:
                        command = keyboard.poll()
                        if keyboard.quit:
                            break
                        message.linear.x, message.linear.y, message.angular.z = command
                    publisher.publish(message)
                    time.sleep(1.0 / args.rate)
        finally:
            # Explicit stop, with the simulator watchdog covering abrupt publisher loss.
            stop = Twist()
            for _ in range(5):
                publisher.publish(stop)
                time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        context.try_shutdown()


if __name__ == '__main__':
    main()
