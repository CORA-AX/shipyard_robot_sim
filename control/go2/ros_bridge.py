"""Minimal ROS command/clock transport and command expiry, without simulator imports."""
import math
import os
from pathlib import Path
import sys
import time


def ros_environment(environment=None):
    """Use Isaac Sim's Python 3.11 Humble libraries in a fresh child process.

    LD_LIBRARY_PATH must be present when the interpreter starts. Do not source
    Ubuntu's Python 3.10 ROS installation in the simulator's interpreter.
    """
    env = dict(os.environ if environment is None else environment)
    bridge = (Path(sys.prefix) / 'lib/python3.11/site-packages/isaacsim/exts'
              / 'isaacsim.ros2.bridge/humble')
    if not (bridge / 'rclpy/rclpy').is_dir():
        raise ValueError(f'Isaac Sim internal Humble libraries not found: {bridge}')
    env['ROS_DISTRO'] = 'humble'
    env['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    # Exclude externally sourced ROS Python paths to avoid mixing Python ABIs.
    env['PYTHONPATH'] = str(bridge / 'rclpy')
    env['LD_LIBRARY_PATH'] = str(bridge / 'lib') + ':' + env.get('LD_LIBRARY_PATH', '')
    for key in ('AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'OLD_PYTHONPATH'):
        env.pop(key, None)
    return env


class VelocityCommand:
    """Body-frame vx/vy/yaw command; stale or invalid input means stand."""
    def __init__(self, timeout=0.5, clock=time.monotonic):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Command timeout must be finite and positive.')
        self.timeout = timeout
        self.clock = clock
        self.values = (0.0, 0.0, 0.0)
        self.received_at = None
        self.received = self.rejected = self.clamped = 0

    def update(self, vx, vy, wz):
        values = (vx, vy, wz)
        if not all(math.isfinite(value) for value in values):
            self.values = (0.0, 0.0, 0.0)
            self.received_at = None
            self.rejected += 1
            return False
        self.values = values
        self.received_at = self.clock()
        self.received += 1
        return True

    @property
    def stale(self):
        return self.received_at is None or self.clock() - self.received_at >= self.timeout

    def current(self):
        return (0.0, 0.0, 0.0) if self.stale else self.values


def clock_qos():
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)


class Go2RosBridge:
    """Own a ROS context, nonblocking subscriber and simulation-time publisher."""
    def __init__(self, topic='/unitree_go2/cmd_vel', timeout=0.5, *, accept_commands=True):
        from geometry_msgs.msg import Twist
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rosgraph_msgs.msg import Clock

        self.command = VelocityCommand(timeout)
        self.context = Context()
        self.context.init(args=[])
        self.node = None
        self.executor = None
        self.clock_count = 0
        try:
            self.node = Node('shipyard_go2_control', context=self.context)
            self.subscription = (self.node.create_subscription(Twist, topic, self._on_command, 1)
                                 if accept_commands else None)
            self.clock_publisher = self.node.create_publisher(Clock, '/clock', clock_qos())
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
        except BaseException:
            self.close()
            raise

    def _on_command(self, message):
        valid = self.command.update(message.linear.x, message.linear.y, message.angular.z)
        if not valid:
            self.node.get_logger().warning('Rejected non-finite cmd_vel; commanding zero velocity.')

    def poll(self):
        # Queue depth one retains the latest command; never wait for a ROS message.
        self.executor.spin_once(timeout_sec=0.0)
        return self.command.current()

    def publish_clock(self, seconds):
        from rosgraph_msgs.msg import Clock
        stamp = Clock()
        nanoseconds = round(seconds * 1_000_000_000)
        stamp.clock.sec, stamp.clock.nanosec = divmod(nanoseconds, 1_000_000_000)
        self.clock_publisher.publish(stamp)
        self.clock_count += 1

    def close(self):
        if self.executor is not None:
            self.executor.shutdown()
        if self.node is not None:
            self.node.destroy_node()
        self.context.try_shutdown()
