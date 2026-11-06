"""Ackermann commands and rear-axle-centred ground-truth ROS state."""
from hunter.control.commands import CommandBuffer, DriveCommand
from hunter.ros.odometry import robot_frames, vector


def clock_qos():
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)


def stamp(seconds):
    from builtin_interfaces.msg import Time
    total = round(seconds * 1_000_000_000)
    sec, nanosec = divmod(total, 1_000_000_000)
    return Time(sec=sec, nanosec=nanosec)


class RobotRosBridge:
    def __init__(self, topic='/hunter2_0/ackermann_cmd', timeout=.5, *, rear_axle_offset, robot_id='hunter2',
                 namespace='hunter2_0', accept_commands=True, publish_state=True,
                 publish_clock=True, enable_reset=False, environment='flat'):
        from ackermann_msgs.msg import AckermannDriveStamped
        from nav_msgs.msg import Odometry
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import JointState
        from tf2_msgs.msg import TFMessage
        from std_msgs.msg import String, UInt64

        self.namespace = namespace.strip('/')
        self.base_frame = self.namespace + '/base_link'
        # ROS base_link is the navigation point; the USD base_link stays intact.
        self.chassis_frame = self.namespace + '/chassis_link'
        self.rear_axle_offset = vector(rear_axle_offset)
        self.odom_frame = self.namespace + '/odom'
        self.command = CommandBuffer(timeout)
        self.context = Context()
        self.context.init(args=[])
        self.node = self.executor = None
        self.clock_count = self.state_count = 0
        self.reset_count = 0
        self.reset_requested = False
        self.publish_state = publish_state
        try:
            self.node = Node(robot_id + '_sim_bridge', context=self.context, namespace=self.namespace)
            self.subscription = (self.node.create_subscription(AckermannDriveStamped, topic, self._on_command, 1)
                                 if accept_commands else None)
            # Shared worlds use a separate simulator clock owner.
            self.clock_publisher = (self.node.create_publisher(Clock, '/clock', clock_qos())
                                    if publish_clock else None)
            if enable_reset:
                from std_srvs.srv import Trigger
                self.reset_service = self.node.create_service(Trigger, 'reset', self._on_reset)
            if publish_state:
                self.odom_pub = self.node.create_publisher(Odometry, 'odom', 10)
                self.joint_pub = self.node.create_publisher(JointState, 'joint_states', 10)
                self.tf_pub = self.node.create_publisher(TFMessage, '/tf', 10)
                static_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
                self.static_tf_pub = self.node.create_publisher(TFMessage, '/tf_static', static_qos)
                self.reset_pub = self.node.create_publisher(UInt64, 'reset_count', static_qos)
                self.environment_pub = self.node.create_publisher(String, 'environment', static_qos)
                self.environment_pub.publish(String(data=environment))
                self.reset_pub.publish(UInt64(data=0))
                self.static_tf_pub.publish(TFMessage(transforms=[
                    self._transform('map', self.odom_frame, 0., (0., 0., 0.), (1., 0., 0., 0.)),
                    self._transform(self.base_frame, self.chassis_frame, 0.,
                                    tuple(-v for v in self.rear_axle_offset), (1., 0., 0., 0.))]))
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
        except BaseException:
            self.close()
            raise

    def _on_command(self, message):
        drive = message.drive
        command = DriveCommand(drive.speed, drive.steering_angle, drive.steering_angle_velocity,
                               drive.acceleration, drive.jerk)
        if message.header.frame_id not in ('', self.base_frame):
            self.command.clear()
            self.command.rejected += 1
            self.node.get_logger().warning(f'Rejected command frame; expected {self.base_frame}.')
        elif not self.command.update(command):
            self.node.get_logger().warning('Rejected Ackermann command: use finite values, nonnegative '
                                           'rate/acceleration and jerk=0; requesting stop.')

    def poll(self):
        self.executor.spin_once(timeout_sec=0.)
        return self.command.current()

    def _on_reset(self, request, response):
        self.reset_requested = True
        self.command.clear()
        response.success = True
        response.message = 'Reset queued for this robot at the next control boundary; command cleared.'
        return response

    def publish_clock(self, seconds):
        from rosgraph_msgs.msg import Clock
        if self.clock_publisher is None:
            raise RuntimeError('This bridge does not own /clock.')
        self.clock_publisher.publish(Clock(clock=stamp(seconds)))
        self.clock_count += 1

    def notify_reset(self):
        """Notify upstream controllers after an actual robot reset, including automatic falls."""
        from std_msgs.msg import UInt64
        self.command.clear()
        self.reset_count += 1
        if self.publish_state:
            self.reset_pub.publish(UInt64(data=self.reset_count))

    @staticmethod
    def _transform(parent, child, seconds, position, quaternion):
        from geometry_msgs.msg import TransformStamped
        msg = TransformStamped()
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp(seconds), parent, child
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, position)
        w, x, y, z = map(float, quaternion)
        msg.transform.rotation.w, msg.transform.rotation.x = w, x
        msg.transform.rotation.y, msg.transform.rotation.z = y, z
        return msg

    def _odometry(self, seconds, frame, state):
        from nav_msgs.msg import Odometry
        transform = self._transform(self.odom_frame, frame, seconds, state.position, state.quaternion)
        odom = Odometry()
        odom.header, odom.child_frame_id = transform.header, frame
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = state.position
        odom.pose.pose.orientation = transform.transform.rotation
        twist = odom.twist.twist
        twist.linear.x, twist.linear.y, twist.linear.z = state.linear_velocity
        twist.angular.x, twist.angular.y, twist.angular.z = state.angular_velocity
        return odom, transform

    def publish_robot_state(self, seconds, robot):
        if not self.publish_state:
            return
        from sensor_msgs.msg import JointState
        from tf2_msgs.msg import TFMessage
        data = robot.data
        _, rear = robot_frames(data, self.rear_axle_offset)
        odom, transform = self._odometry(seconds, self.base_frame, rear)
        joints = JointState()
        joints.header.stamp, joints.header.frame_id = stamp(seconds), self.chassis_frame
        joints.name = list(robot.joint_names)
        joints.position, joints.velocity = data.joint_pos[0].tolist(), data.joint_vel[0].tolist()
        self.odom_pub.publish(odom)
        self.joint_pub.publish(joints)
        self.tf_pub.publish(TFMessage(transforms=[transform]))
        self.state_count += 1

    def close(self):
        if self.executor is not None:
            self.executor.shutdown()
        if self.node is not None:
            self.node.destroy_node()
        self.context.try_shutdown()


class SimulationClock:
    """The sole /clock publisher in a shared world."""
    def __init__(self):
        from rclpy.context import Context
        from rclpy.node import Node
        from rosgraph_msgs.msg import Clock
        self.context = Context()
        self.context.init(args=[])
        self.node = Node('simulation_clock', context=self.context)
        self.publisher = self.node.create_publisher(Clock, '/clock', clock_qos())
        self.count = 0

    def publish(self, seconds):
        from rosgraph_msgs.msg import Clock
        self.publisher.publish(Clock(clock=stamp(seconds)))
        self.count += 1

    def close(self):
        self.node.destroy_node()
        self.context.try_shutdown()
