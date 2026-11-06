"""Assign RViz goals to one Go2 and own the final velocity command gate.

Nav2's controller and smoother publish intermediate topics only. Cancellation,
reset and stale-state handling close this gate before any asynchronous action
operation; a new goal waits for the old result and measured stopping.
"""
import argparse
import copy
import json
import math
from pathlib import Path
import signal
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, UInt64
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from robot_control.waypoint_keyboard import SelectionKeyboard
from robot_sim.config import load_world_config
from .geometry import goal_is_free


def seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def yaw(orientation):
    q = orientation
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def pose_errors(current, target):
    position = math.hypot(current.position.x-target.position.x, current.position.y-target.position.y)
    angle = yaw(target.orientation) - yaw(current.orientation)
    return position, math.atan2(math.sin(angle), math.cos(angle))


class GoalSelector(Node):
    def __init__(self, robot, goal_topic):
        super().__init__('navigation_goal_selector', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.robot = robot
        self.prefix = '/' + robot['namespace']
        self.goal_topic = goal_topic
        self.phase, self.reason = 'STARTING', 'Waiting for map, odometry and active Nav2.'
        self.pending = self.request = self.target = None
        self.refinements = 0
        self.heading_bias = 0.
        self.position_bias = (0., 0.)
        self.raw, self.raw_time = Twist(), -math.inf
        self.odom = self.map = None
        self.odom_time, self.odom_stamp, self.stop_stamp = -math.inf, None, None
        self.reset_count = None
        self.goal_after = 0.
        self.nav_active, self.state_future, self.state_poll = False, None, -math.inf
        self.stopping_since = time.monotonic()
        self.last_status = -math.inf
        transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.command_pub = self.create_publisher(Twist, self.prefix + '/cmd_vel', 1)
        self.status_pub = self.create_publisher(String, '/waypoint/status', transient)
        self.selected_pub = self.create_publisher(String, '/waypoint/selected_robot', transient)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint/markers', transient)
        self.subscriptions_ = [
            self.create_subscription(PoseStamped, goal_topic, self.on_goal, 1),
            self.create_subscription(Twist, self.prefix + '/cmd_vel_smoothed', self.on_command, 1),
            self.create_subscription(Odometry, self.prefix + '/odom', self.on_odom, latest),
            self.create_subscription(UInt64, self.prefix + '/reset_count', self.on_reset, transient),
            self.create_subscription(OccupancyGrid, '/map', self.on_map, transient),
        ]
        self.cancel_service = self.create_service(Trigger, '/waypoint/cancel', self.on_cancel)
        self.next_service = self.create_service(Trigger, '/waypoint/next_robot', self.on_next)
        self.client = ActionClient(self, NavigateToPose, self.prefix + '/navigate_to_pose')
        self.state_client = self.create_client(GetState, self.prefix + '/bt_navigator/get_state')
        self.timer = self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_status()

    def set_phase(self, phase, reason):
        if (phase, reason) != (self.phase, self.reason):
            self.phase, self.reason = phase, reason
            self.get_logger().info(f'{phase}: {reason}')
            self.publish_status()

    def publish_status(self):
        position_error = yaw_error = None
        if self.target is not None and self.odom is not None:
            position_error, yaw_error = pose_errors(self.odom.pose.pose, self.target.pose)
        state = dict(mode='nav2', selected_robot=self.robot['robot_id'], namespace=self.robot['namespace'],
                     phase=self.phase, reason=self.reason, position_error=position_error, yaw_error=yaw_error,
                     refinements=self.refinements,
                     heading_bias=self.heading_bias,
                     position_bias=self.position_bias,
                     nav2_active=self.nav_active, reset_count=self.reset_count,
                     goal=None if self.target is None else [self.target.pose.position.x, self.target.pose.position.y,
                                                           yaw(self.target.pose.orientation)])
        self.status_pub.publish(String(data=json.dumps(state)))
        self.selected_pub.publish(String(data=self.robot['robot_id']))
        markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        label = Marker()
        label.header.frame_id = self.robot['namespace'] + '/base_link'
        label.ns, label.id, label.type, label.action = 'navigation', 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        label.pose.position.z, label.pose.orientation.w = .75, 1.
        label.scale.z = .18
        label.color.r, label.color.g, label.color.b, label.color.a = 1., .8, .1, 1.
        label.text = f'{self.robot["robot_id"]} Nav2: {self.phase}'
        markers.markers.append(label)
        if self.target is not None:
            arrow = Marker()
            arrow.header.frame_id, arrow.ns, arrow.id = 'map', 'navigation', 1
            arrow.type, arrow.action, arrow.pose = Marker.ARROW, Marker.ADD, copy.deepcopy(self.target.pose)
            arrow.pose.position.z = .08
            arrow.scale.x, arrow.scale.y, arrow.scale.z = .6, .08, .08
            arrow.color.r, arrow.color.g, arrow.color.a = 1., .7, 1.
            markers.markers.append(arrow)
        self.marker_pub.publish(markers)
        self.last_status = time.monotonic()

    def on_map(self, message):
        self.map = message

    def on_command(self, message):
        values = (message.linear.x, message.linear.y, message.angular.z)
        if not all(math.isfinite(value) for value in values):
            self.stop('INVALID_COMMAND', 'Non-finite Nav2 command.')
        elif self.phase == 'NAVIGATING' and self.request and not self.request['cancel']:
            self.raw, self.raw_time = message, time.monotonic()

    def on_odom(self, message):
        stamp = seconds(message.header.stamp)
        p, q = message.pose.pose.position, message.pose.pose.orientation
        v = message.twist.twist
        values = (p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.linear.x,v.linear.y,v.angular.z,stamp)
        if (message.header.frame_id != self.robot['namespace'] + '/odom'
                or message.child_frame_id != self.robot['namespace'] + '/base_link'
                or not all(math.isfinite(value) for value in values)):
            self.stop('INVALID_ODOM', 'Invalid odometry/frame.')
            return
        if self.odom is not None:
            previous = self.odom.pose.pose.position
            if stamp < self.odom_stamp or math.hypot(p.x-previous.x,p.y-previous.y) > .75:
                self.stop('RESET', 'Odometry jumped or simulation clock rewound.')
                self.goal_after = max(0., stamp)
        if stamp != self.odom_stamp:
            self.odom_time = time.monotonic()
        self.odom, self.odom_stamp = message, stamp

    def on_reset(self, message):
        if self.reset_count is not None and message.data != self.reset_count:
            self.stop('RESET', 'Robot reset; send a new goal after the reset.')
            self.goal_after = self.get_clock().now().nanoseconds / 1e9
        self.reset_count = message.data

    def on_goal(self, message):
        now = self.get_clock().now().nanoseconds / 1e9
        p, q = message.pose.position, message.pose.orientation
        valid = all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w))
        stamp = seconds(message.header.stamp)
        norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w) if valid else 0.
        if not valid or not .99 < norm < 1.01 or abs(q.x)+abs(q.y) > .01:
            self.get_logger().warning('Rejected non-finite or non-planar goal pose.')
            return
        if message.header.frame_id != 'map':
            self.get_logger().warning('Goal must use the map frame.')
            return
        if stamp and (stamp < self.goal_after or now-stamp > 2. or stamp-now > .5):
            self.get_logger().warning('Rejected stale/future goal; use RViz simulation time.')
            return
        if not self.nav_active or self.map is None or time.monotonic()-self.odom_time > .75:
            self.get_logger().warning('Nav2/map/odometry is not ready; send the goal after READY.')
            return
        goal_yaw = yaw(q)
        if not goal_is_free(self.map, p.x, p.y, goal_yaw, any_heading=True):
            self.get_logger().warning('Rejected goal: Go2 footprint overlaps occupied/unknown cells.')
            return
        self.stop('STOPPING', 'Waiting for previous action and measured stop before new goal.')
        self.pending = copy.deepcopy(message)
        self.pending.pose.position.z = 0.
        self.target = self.pending
        self.refinements = 0
        self.heading_bias = 0.
        self.position_bias = (0., 0.)
        self.publish_status()

    def on_cancel(self, request, response):
        self.stop('CANCELED', 'Goal canceled by user.')
        response.success, response.message = True, 'Command gate closed; canceling Nav2 action.'
        return response

    def on_next(self, request, response):
        self.stop('CANCELED', 'Single-Go2 Nav2 phase; selection stays on this Go2.')
        response.success, response.message = True, self.reason
        return response

    def stop(self, phase, reason):
        self.pending = None
        self.target = None
        self.heading_bias = 0.
        self.position_bias = (0., 0.)
        self.raw, self.raw_time = Twist(), -math.inf
        self.stop_stamp = None
        self.stopping_since = time.monotonic()
        if self.request:
            self.request['cancel'] = True
            self.cancel_request()
        self.command_pub.publish(Twist())
        self.set_phase(phase, reason)

    def cancel_request(self):
        request = self.request
        if request and request.get('handle') and not request.get('cancel_sent'):
            request['cancel_sent'] = True
            request['handle'].cancel_goal_async()

    def dispatch(self):
        goal = self.pending
        self.pending = None
        self.request = request = dict(goal=goal, cancel=False, handle=None)
        request['sent_at'] = time.monotonic()
        goal.header.stamp = self.get_clock().now().to_msg()
        action = NavigateToPose.Goal(pose=goal)
        future = self.client.send_goal_async(action)
        self.set_phase('SENDING', 'Sending NavigateToPose to ' + self.prefix)

        def accepted(done):
            try:
                handle = done.result()
                request['handle'] = handle
                if not handle.accepted:
                    self.request = None
                    if self.pending is None:
                        self.set_phase('FAILED', 'Nav2 rejected the action.')
                    return
                handle.get_result_async().add_done_callback(finished)
                if request['cancel']:
                    self.cancel_request()
                else:
                    self.raw_time = -math.inf
                    self.set_phase('NAVIGATING', 'Following Nav2 path.')
            except Exception as exc:
                self.request = None
                self.stop('FAILED', f'Action transport failed: {exc}')

        def finished(done):
            try:
                status = done.result().status
            except Exception as exc:
                status = GoalStatus.STATUS_UNKNOWN
                self.get_logger().error(str(exc))
            self.request = None
            self.raw, self.raw_time = Twist(), -math.inf
            self.command_pub.publish(Twist())
            if request['cancel']:
                if self.pending is not None:
                    self.set_phase('STOPPING', 'Previous action ended; waiting for measured stop.')
                return
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.stop_stamp = None
                self.stopping_since = time.monotonic()
                self.set_phase('SETTLING', 'Nav2 succeeded; gate closed, checking actual stopped pose.')
            else:
                self.set_phase('FAILED', f'Nav2 action ended with status {status}.')
        future.add_done_callback(accepted)

    def refinement_goal(self, position_error, yaw_error):
        """Compensate bounded gait settling bias without changing the user's goal."""
        goal = copy.deepcopy(self.target)
        if position_error <= .25 and abs(yaw_error) <= .25:
            p, target = self.odom.pose.pose.position, self.target.pose.position
            bx = self.position_bias[0] + target.x-p.x
            by = self.position_bias[1] + target.y-p.y
            scale = min(1., .15/max(math.hypot(bx, by), 1e-9))
            bx, by = bx*scale, by*scale
            if goal_is_free(self.map, target.x+bx, target.y+by, yaw(goal.pose.orientation), any_heading=True):
                self.position_bias = bx, by
            else:
                self.position_bias = (0., 0.)
        goal.pose.position.x += self.position_bias[0]
        goal.pose.position.y += self.position_bias[1]
        if position_error <= .12 and abs(yaw_error) <= .35:
            self.heading_bias = max(-.30, min(.30, self.heading_bias+yaw_error))
        goal_yaw = yaw(self.target.pose.orientation) + self.heading_bias
        q = goal.pose.orientation
        q.x, q.y, q.z, q.w = 0., 0., math.sin(goal_yaw/2), math.cos(goal_yaw/2)
        return goal

    def tick(self):
        now = time.monotonic()
        if self.count_publishers(self.prefix + '/cmd_vel') > 1 or self.count_publishers('/waypoint/status') > 1:
            self.stop('CONFLICT', 'Another command/waypoint controller is running; stop it first.')
            raise RuntimeError(self.reason)
        if self.state_future is not None and self.state_future.done():
            try:
                active = self.state_future.result().current_state.id == 3
            except Exception:
                active = False
            self.state_future = None
            if self.nav_active and not active and self.request:
                self.stop('NAV2_INACTIVE', 'Nav2 lifecycle is no longer active.')
            self.nav_active = active
        if now-self.state_poll > 1. and self.state_future is None and self.state_client.service_is_ready():
            self.state_poll = now
            self.state_future = self.state_client.call_async(GetState.Request())
        fresh = now-self.odom_time <= .75
        if not fresh and (self.phase == 'SETTLING' or self.pending is not None or
                          (self.request and not self.request['cancel'])):
            self.stop('ODOM_TIMEOUT', 'Odometry stopped advancing; send a new goal after recovery.')
        if self.request and self.request['cancel'] and now-self.stopping_since > 10.:
            self.pending = None
            self.set_phase('CANCEL_TIMEOUT', 'Nav2 did not finish cancellation; command gate remains closed.')
        if self.phase == 'SENDING' and self.request and now-self.request['sent_at'] > 10.:
            self.stop('ACTION_TIMEOUT','Nav2 did not acknowledge the goal; command gate remains closed.')
        if self.pending is not None and self.request is None and fresh and self.nav_active:
            velocity = self.odom.twist.twist
            stopped = math.hypot(velocity.linear.x,velocity.linear.y) < .12 and abs(velocity.angular.z) < .25
            if stopped:
                if self.stop_stamp is None:
                    self.stop_stamp = self.odom_stamp
                if self.odom_stamp-self.stop_stamp >= .25 and self.client.server_is_ready():
                    self.dispatch()
            else:
                self.stop_stamp = None
        if self.phase == 'SETTLING' and fresh:
            velocity = self.odom.twist.twist
            stopped = math.hypot(velocity.linear.x,velocity.linear.y) < .04 and abs(velocity.angular.z) < .10
            if stopped:
                if self.stop_stamp is None:
                    self.stop_stamp = self.odom_stamp
                if self.odom_stamp-self.stop_stamp >= 1.0:
                    error, yaw_error = pose_errors(self.odom.pose.pose, self.target.pose)
                    reason = (f'Stopped after Nav2 success; position error={error:.3f} m, '
                              f'yaw error={yaw_error:.3f} rad.')
                    if error <= .12 and abs(yaw_error) <= .12:
                        self.set_phase('REACHED', reason)
                    elif self.refinements < 5:
                        # Retry only measured drift after Nav2 success. Cancellation,
                        # resets and stale state still clear the target and cannot retry.
                        self.refinements += 1
                        self.pending = self.refinement_goal(error, yaw_error)
                        self.stop_stamp, self.stopping_since = None, now
                        self.set_phase('STOPPING', reason + f' Refining pose ({self.refinements}/5).')
                    else:
                        self.set_phase('FAILED', reason + ' Pose refinement limit reached.')
            else:
                self.stop_stamp = None
            if self.phase == 'SETTLING' and now-self.stopping_since > 10.:
                self.stop('FAILED','Robot did not settle after Nav2 success.')
        if self.phase == 'STARTING' and fresh and self.map is not None and self.nav_active:
            self.set_phase('READY', 'RViz 2D Goal Pose → ' + self.robot['robot_id'])
        command = (self.raw if self.phase == 'NAVIGATING' and self.request and
                   not self.request['cancel'] and fresh and now-self.raw_time <= .3 else Twist())
        self.command_pub.publish(command)
        if now-self.last_status > .5:
            self.publish_status()

    def close(self):
        self.timer.cancel()
        self.stop('CANCELED', 'Navigation selector stopped.')
        # Process cancellation/result callbacks briefly while publishing zero.
        deadline = time.monotonic() + 1.
        while rclpy.ok() and self.request and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.02)
            self.command_pub.publish(Twist())
        self.marker_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
        self.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robots-file', type=Path, required=True)
    parser.add_argument('--robot', default='go2_0')
    parser.add_argument('--goal-topic', default='/waypoint/goal_pose')
    parser.add_argument('--no-keyboard', action='store_true')
    args, ros_args = parser.parse_known_args()
    world = load_world_config(args.robots_file)
    robots = [robot for robot in world['robots'] if robot['robot_id'] == args.robot and robot['robot_type'] == 'go2']
    if len(robots) != 1:
        parser.error('--robot must select one Go2 from robots-file.')
    if not args.goal_topic.startswith('/'):
        parser.error('--goal-topic must be absolute.')
    rclpy.init(args=ros_args)
    node = None
    stopped = False

    def interrupt(*_):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    try:
        with SelectionKeyboard(enabled=not args.no_keyboard) as keyboard:
            node = GoalSelector(robots[0], args.goal_topic)
            while rclpy.ok() and not stopped:
                for event in keyboard.poll():
                    if event == 'quit':
                        stopped = True
                    else:
                        node.stop('CANCELED', 'Terminal cancel; single-Go2 selection is unchanged.')
                rclpy.spin_once(node, timeout_sec=.02)
    finally:
        if node is not None:
            node.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
