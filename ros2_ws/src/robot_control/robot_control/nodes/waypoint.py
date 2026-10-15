"""One selected robot follows RViz goals; Space changes selection and cancels old goals."""
import argparse
import json
import math
from pathlib import Path
import signal
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String, UInt64
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray

from robot_control.waypoint import ZERO, State, WaypointPD, load_settings, multiply, quaternion, rotate, yaw
from robot_control.waypoint_keyboard import SelectionKeyboard


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def xyz(vector):
    values = (vector.x, vector.y, vector.z)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Non-finite position or velocity.')
    return values


def xyzw(value):
    return quaternion((value.x, value.y, value.z, value.w))


class WaypointNode(Node):
    def __init__(self, args, context):
        super().__init__('waypoint_tracker', context=context,
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.settings, profiles, self.robots = load_settings(args.config, args.robots_file)
        self.args = args
        self.controllers = {r['id']: WaypointPD(profiles[r['type']]) for r in self.robots}
        self.received = {r['id']: None for r in self.robots}
        self.reset_counts = {}
        initial = args.robot or next((r['id'] for r in self.robots if r['type'] == 'go2'), self.robots[0]['id'])
        if initial not in self.controllers:
            raise ValueError('--robot must match an id from robots_file/config.')
        self.index = next(i for i, robot in enumerate(self.robots) if robot['id'] == initial)
        self.phase = 'WAITING_GOAL'
        self.pending = None
        self.braking = set()
        self.stopped_since = None
        self.switch_started = self.selection_floor = 0
        self.switch_fault = False
        self.last_visual = self.last_ownership_check = -float('inf')
        self.report = {'goals_accepted': 0, 'goals_rejected': 0, 'switches': 0, 'events': []}
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.selected_pub = self.create_publisher(String, '/waypoint/selected_robot', transient)
        self.status_pub = self.create_publisher(String, '/waypoint/status', transient)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint/markers', transient)
        self.command_publishers = {}
        self.inputs = [self.create_subscription(PoseStamped, args.goal_topic, self.on_goal, 1)]
        for robot in self.robots:
            identity, namespace = robot['id'], robot['namespace']
            self.command_publishers[identity] = self.create_publisher(Twist, '/' + namespace + '/cmd_vel', 1)
            self.inputs.append(self.create_subscription(Odometry, '/' + namespace + '/odom',
                lambda msg, robot=robot: self.on_odom(robot, msg), latest))
            self.inputs.append(self.create_subscription(UInt64, '/' + namespace + '/reset_count',
                lambda msg, identity=identity: self.on_reset(identity, msg.data), transient))
        self.next_service = self.create_service(Trigger, '/waypoint/next_robot', self.on_next)
        self.cancel_service = self.create_service(Trigger, '/waypoint/cancel', self.on_cancel)
        self.status('WAITING_GOAL')
        print(f'WAYPOINT_READY: goal={args.goal_topic}; Space=next robot; X/Esc=cancel; Q=quit; '
              'focus this terminal. Stop other cmd_vel publishers before tracking.', flush=True)

    @property
    def selected(self):
        return self.robots[self.index]['id']

    def status(self, phase):
        self.phase = phase
        event = {'selected_robot': self.selected, 'phase': phase,
                 'goal': self.controllers[self.selected].goal,
                 'simulation_time': self.get_clock().now().nanoseconds / 1e9}
        self.report['events'].append(event)
        self.report['events'] = self.report['events'][-1000:]
        self.selected_pub.publish(String(data=self.selected))
        self.status_pub.publish(String(data=json.dumps(event)))
        print('WAYPOINT_STATUS ' + json.dumps(event), flush=True)
        self.last_visual = -float('inf')

    def fresh(self, identity):
        received = self.received[identity]
        return received is not None and time.monotonic() - received < self.settings['odom_timeout']

    def transform(self, frame, stamp):
        frame = frame.strip('/')
        if frame == self.args.frame:
            return ZERO, (0., 0., 0., 1.)
        transform = self.buffer.lookup_transform(self.args.frame, frame, Time(nanoseconds=stamp)).transform
        return xyz(transform.translation), xyzw(transform.rotation)

    def on_odom(self, robot, message):
        identity = robot['id']
        controller = self.controllers[identity]
        try:
            if message.child_frame_id.strip('/') != robot['namespace'] + '/base_link':
                raise ValueError('Unexpected odometry child_frame_id.')
            nanoseconds = stamp_ns(message.header.stamp)
            translation, reference_q = self.transform(message.header.frame_id, nanoseconds)
            position = rotate(reference_q, xyz(message.pose.pose.position))
            orientation = multiply(reference_q, xyzw(message.pose.pose.orientation))
            velocity = rotate(orientation, xyz(message.twist.twist.linear))
            angular = rotate(orientation, xyz(message.twist.twist.angular))
            state = State(nanoseconds / 1e9, position[0] + translation[0], position[1] + translation[1],
                          yaw(orientation), velocity[0], velocity[1], angular[2])
            before = controller.status
            if controller.update(state):
                self.received[identity] = time.monotonic()
            if (identity == self.selected and controller.status != before
                    and controller.status in ('CLOCK_RESET', 'POSE_RESET')):
                self.pending = None
                self.selection_floor = self.get_clock().now().nanoseconds
                self.status(controller.status)
        except (ValueError, TransformException) as error:
            if controller.goal is not None:
                controller.cancel('ODOM_INVALID')
                self.status('ODOM_INVALID')
            self.get_logger().warning(f'{identity}: {error}', throttle_duration_sec=5.)

    def on_reset(self, identity, count):
        previous = self.reset_counts.get(identity)
        self.reset_counts[identity] = count
        if previous is not None and count != previous:
            self.controllers[identity].cancel('RESET')
            self.received[identity] = None  # A post-reset odometry sample is required.
            if identity == self.selected:
                self.pending = None
                self.selection_floor = self.get_clock().now().nanoseconds
                self.status('RESET')

    def on_goal(self, message):
        try:
            if not self.fresh(self.selected) or not self.command_publishers[self.selected].get_subscription_count():
                raise ValueError('Selected robot has no fresh odometry/command subscriber.')
            self.check_ownership()
            nanoseconds = stamp_ns(message.header.stamp)
            now = self.get_clock().now().nanoseconds
            if nanoseconds and (nanoseconds < self.selection_floor or
                    (now - nanoseconds) / 1e9 > self.settings['goal_max_age'] or nanoseconds > now + 250_000_000):
                raise ValueError('Goal predates selection/reset, is stale, or has a future timestamp.')
            translation, reference_q = self.transform(message.header.frame_id, nanoseconds)
            position = rotate(reference_q, xyz(message.pose.position))
            heading = yaw(multiply(reference_q, xyzw(message.pose.orientation))) if self.args.track_yaw else None
            goal = (position[0] + translation[0], position[1] + translation[1], heading)
            self.report['goals_accepted'] += 1
            if self.braking:
                self.pending = goal
                self.status('SWITCHING')
            else:
                self.controllers[self.selected].set_goal(*goal)
                self.status('TRACKING')
        except (ValueError, TransformException) as error:
            self.report['goals_rejected'] += 1
            self.get_logger().warning(f'Goal rejected: {error}')

    def next_robot(self):
        old = self.selected
        if self.controllers[old].state is not None:
            self.braking.add(old)
        for controller in self.controllers.values():
            controller.cancel('WAITING_GOAL')
        self.pending = None
        self.index = (self.index + 1) % len(self.robots)
        self.selection_floor = self.get_clock().now().nanoseconds
        self.switch_started = time.monotonic()
        self.stopped_since = None
        self.switch_fault = False
        self.report['switches'] += 1
        self.publish_commands()
        self.status('SWITCHING' if self.braking else 'WAITING_GOAL')

    def cancel(self, reason='CANCELED'):
        self.pending = None
        for controller in self.controllers.values():
            controller.cancel(reason)
        self.selection_floor = self.get_clock().now().nanoseconds
        self.publish_commands()
        self.status(reason)

    def on_next(self, request, response):
        self.next_robot()
        response.success, response.message = True, self.selected
        return response

    def on_cancel(self, request, response):
        self.cancel()
        response.success, response.message = True, 'Goal canceled; zero velocities published.'
        return response

    def check_ownership(self):
        for robot in self.robots:
            if self.count_publishers('/' + robot['namespace'] + '/cmd_vel') > 1:
                raise RuntimeError(f'Another cmd_vel publisher owns {robot["id"]}; stop teleop/other trackers first.')

    def publish_commands(self):
        for identity, publisher in self.command_publishers.items():
            command = self.controllers[identity].command if identity == self.selected and not self.braking else ZERO
            message = Twist()
            message.linear.x, message.linear.y, message.angular.z = command
            publisher.publish(message)

    def tick(self):
        now = time.monotonic()
        if now - self.last_ownership_check > .5:
            self.check_ownership()
            self.last_ownership_check = now
        controller = self.controllers[self.selected]
        if (controller.goal is not None or self.pending is not None) and not self.fresh(self.selected):
            self.cancel('ODOM_TIMEOUT')
        if self.braking:
            stopped = all(self.fresh(i) and self.controllers[i].stopped() for i in self.braking)
            sample_time = min((self.controllers[i].state.time for i in self.braking), default=0.)
            if not stopped:
                self.stopped_since = None
            elif self.stopped_since is None:
                self.stopped_since = sample_time
            elif sample_time - self.stopped_since >= .2:
                self.braking.clear()
                self.switch_fault = False
                if self.pending is not None:
                    controller.set_goal(*self.pending)
                    self.pending = None
                    self.status('TRACKING')
                else:
                    self.status('WAITING_GOAL')
            if self.braking and not self.switch_fault and now - self.switch_started > self.settings['switch_timeout']:
                self.pending = None
                self.switch_fault = True
                self.status('SWITCH_TIMEOUT')
        if not self.braking:
            before = controller.status
            controller.step()
            if controller.status != before:
                self.status(controller.status)
        self.publish_commands()
        if now - self.last_visual > .2:
            self.publish_visuals()
            self.last_visual = now

    def publish_visuals(self):
        robot = self.robots[self.index]
        controller = self.controllers[self.selected]
        markers = []
        for identity in range(4):
            marker = Marker()
            marker.header.frame_id = self.args.frame
            marker.ns, marker.id = 'waypoint', identity
            marker.pose.orientation.w = 1.
            markers.append(marker)
        text, halo, goal, line = markers
        text.header.frame_id = robot['namespace'] + '/base_link'
        text.type = Marker.TEXT_VIEW_FACING
        text.frame_locked = True
        text.pose.position.z = .65
        text.scale.z = .18
        text.color.r, text.color.g, text.color.b, text.color.a = 1., .85, .2, 1.
        text.text = f'SELECTED: {self.selected}\n{self.phase}'
        halo.type = Marker.CYLINDER
        halo.scale.x = halo.scale.y = .75
        halo.scale.z = .025
        halo.color.r, halo.color.g, halo.color.a = 1., .7, .25
        if controller.state is not None:
            halo.pose.position.x, halo.pose.position.y = controller.state.x, controller.state.y
        else:
            halo.action = Marker.DELETE
        if controller.goal is not None:
            gx, gy, heading, _ = controller.goal
            goal.type = Marker.ARROW
            goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = gx, gy, .05
            goal.pose.orientation.z, goal.pose.orientation.w = math.sin(heading / 2), math.cos(heading / 2)
            goal.scale.x, goal.scale.y, goal.scale.z = .6, .1, .1
            goal.color.r, goal.color.g, goal.color.a = 1., .65, 1.
            line.type = Marker.LINE_STRIP
            line.scale.x = .025
            line.color.r, line.color.g, line.color.a = 1., .65, .8
            line.points = [Point(x=controller.state.x, y=controller.state.y, z=.04), Point(x=gx, y=gy, z=.04)]
        else:
            goal.action = line.action = Marker.DELETE
        self.marker_pub.publish(MarkerArray(markers=markers))

    def close(self):
        self.cancel('STOPPED')
        for _ in range(5):
            self.publish_commands()
            time.sleep(.02)
        self.marker_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
        time.sleep(.05)
        if self.args.report:
            path = Path(self.args.report)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(self.report, indent=2) + '\n')


def main():
    from ament_index_python.packages import get_package_share_directory
    from rclpy.utilities import remove_ros_args
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=get_package_share_directory('robot_control') + '/config/waypoint.yaml')
    parser.add_argument('--robots-file', help='Same robot list as simulation.launch.py; defaults to Go2 + Lekiwi.')
    parser.add_argument('--robot', help='Initial robot id; defaults to the first Go2, otherwise the first robot.')
    parser.add_argument('--goal-topic', default='/waypoint/goal_pose')
    parser.add_argument('--frame', default='map')
    parser.add_argument('--track-yaw', action='store_true', help='Also track the orientation dragged in RViz.')
    parser.add_argument('--no-keyboard', action='store_true', help='Use next_robot/cancel ROS services without a TTY.')
    parser.add_argument('--report')
    args = parser.parse_args(remove_ros_args()[1:])
    if not args.goal_topic.startswith('/') or not args.frame.strip('/'):
        parser.error('Use an absolute goal topic and a nonempty reference frame.')
    args.frame = args.frame.strip('/')
    context = Context()
    context.init(args=None)
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    node = executor = None
    try:
        with SelectionKeyboard(not args.no_keyboard) as keyboard:
            node = WaypointNode(args, context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            quit_requested = False
            while context.ok() and not quit_requested:
                started = time.monotonic()
                # Drain enough callbacks for both robots, TF and clock without queueing stale odometry.
                for _ in range(32):
                    executor.spin_once(timeout_sec=0.)
                for event in keyboard.poll():
                    if event == 'next':
                        node.next_robot()
                    elif event == 'cancel':
                        node.cancel()
                    elif event == 'quit':
                        quit_requested = True
                node.tick()
                time.sleep(max(0., 1. / node.settings['rate'] - (time.monotonic() - started)))
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            if executor is not None:
                executor.shutdown()
            node.destroy_node()
        context.try_shutdown()


if __name__ == '__main__':
    main()
