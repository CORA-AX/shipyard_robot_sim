"""RViz point/pose -> Nav2 actions -> guarded Ackermann commands.

The selected world supplies either a blank plane map or an offline shipyard
map. Identity map->odom uses simulator ground truth, not sensor localization.
"""
import copy
from datetime import datetime
import json
import math
from pathlib import Path
import time

from ackermann_msgs.msg import AckermannDriveStamped
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String, UInt64
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from hunter.navigation.nav2.config import FOOTPRINT, planning_turning_radius, twist_to_drive
from hunter.environments.config import load_world
from hunter.navigation.maps import load_map, occupancy_message
from hunter.navigation.state import wrap
from hunter.navigation.nav2.position_goal import heading_candidates, path_distance
from hunter.ros.bridge import stamp
from hunter.ros.odometry import read_odometry


class HunterNav2Goal(Node):
    ready_instruction = ('RViz Publish Point: position only; 2D Goal Pose: position and heading. '
                         'Rear axle target, forward only.')

    def __init__(self, root, namespace, robot, context, *, world=None, grid=None):
        super().__init__('hunter_nav2_goal', namespace=namespace, context=context,
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.root, self.namespace, self.robot = Path(root), namespace, robot
        self.world = world or load_world(root)
        self.grid = grid or load_map(root,self.world)
        self.simulator_environment = None
        self.phase, self.reason = 'WAITING', 'Waiting for fresh odometry, stopped vehicle and active Nav2.'
        self.observation, self.odom_wall, self.reset_count = None, -math.inf, None
        self.request = self.mission = None
        self.position_search = None
        self.raw, self.raw_wall, self.last_steering = Twist(), -math.inf, 0.
        self.settle_start = None
        self.settle_wall = None
        self.last_visual = self.last_map = self.last_health_poll = -math.inf
        self.states = {name: False for name in ('planner_server', 'controller_server', 'bt_navigator')}
        self.state_futures = {}
        self.printed_ready = False
        self.last_result = None
        self.result_file = None
        self.goal_floor = 0.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.command_pub = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 1)
        self.status_pub = self.create_publisher(String, 'nav2/status', latched)
        self.marker_pub = self.create_publisher(MarkerArray, 'nav2/markers', latched)
        self.map_pub = self.create_publisher(OccupancyGrid, 'nav2/map', latched)
        self.path_pub = self.create_publisher(RosPath, 'nav2/selected_path', latched)
        self.subscriptions_ = [
            self.create_subscription(PoseStamped, 'nav2/goal_pose', self.on_goal, 1),
            self.create_subscription(PointStamped, 'nav2/goal_point', self.on_point, 1),
            self.create_subscription(RosPath, 'plan', self.on_plan, 1),
            self.create_subscription(Twist, 'nav2/cmd_vel', self.on_command, 1),
            self.create_subscription(Odometry, 'odom', self.on_odom, 1),
            self.create_subscription(UInt64, 'reset_count', self.on_reset, latched)]
        self.subscriptions_.append(self.create_subscription(String, 'environment', self.on_environment, latched))
        self.cancel_service = self.create_service(Trigger, 'nav2/cancel', self.on_cancel)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.planner_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.follower_client = ActionClient(self, FollowPath, 'follow_path')
        self.state_clients = {name: self.create_client(GetState, name+'/get_state') for name in self.states}
        self.timer = self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.map = occupancy_message(self.grid,self.get_clock().now().to_msg())
        self.publish_map()
        self.publish_visuals()

    def active(self):
        return (all(self.states.values()) and self.client.server_is_ready()
                and self.planner_client.server_is_ready() and self.follower_client.server_is_ready())

    def fresh(self):
        return self.observation is not None and time.monotonic()-self.odom_wall <= 1.

    def stopped(self):
        o = self.observation
        return o is not None and abs(o.speed) <= .03 and abs(o.yaw_rate) <= .05

    def owns_command(self):
        return self.count_publishers('/'+self.namespace+'/ackermann_cmd') == 1

    def world_matches(self):
        return self.simulator_environment == self.world.name

    def on_environment(self, message):
        self.simulator_environment = message.data
        if not self.world_matches():
            if self.mission is not None:
                self.abort('failed', 'simulator_environment_mismatch')
            else:
                self.set_phase('WAITING', f'Expected {self.world.name}; simulator reports {message.data}.')

    def set_phase(self, phase, reason):
        if (phase, reason) != (self.phase, self.reason):
            self.phase, self.reason = phase, reason
            self.get_logger().info(f'{phase}: {reason}')
            self.publish_visuals()

    def on_odom(self, message):
        try:
            observation = read_odometry(message, self.namespace)
        except ValueError as error:
            self.odom_wall = -math.inf
            self.abort('failed', 'invalid_odometry: '+str(error))
            return
        previous = self.observation
        if previous is not None:
            if observation.seconds < previous.seconds or math.hypot(
                    observation.pose.x-previous.pose.x, observation.pose.y-previous.pose.y) > .75:
                self.goal_floor = observation.seconds
                self.abort('failed', 'simulation_reset')
            elif observation.seconds == previous.seconds:
                return
        self.observation, self.odom_wall = observation, time.monotonic()
        if self.mission is not None:
            self.mission['travelled_distance_m'] += math.hypot(
                observation.pose.x-self.mission['last_x'], observation.pose.y-self.mission['last_y'])
            self.mission['last_x'], self.mission['last_y'] = observation.pose.x, observation.pose.y

    def on_reset(self, message):
        if self.reset_count is not None and self.reset_count != message.data:
            self.abort('failed', 'simulation_reset')
            self.odom_wall = -math.inf
            self.goal_floor = self.get_clock().now().nanoseconds*1e-9
        self.reset_count = message.data

    def on_command(self, message):
        if self.phase != 'NAVIGATING' or not self.request or self.request['cancel']:
            return
        if not all(math.isfinite(value) for value in (
                message.linear.x, message.linear.y, message.linear.z,
                message.angular.x, message.angular.y, message.angular.z)):
            self.abort('failed', 'non_finite_nav2_command')
            return
        if max(abs(message.linear.z), abs(message.angular.x), abs(message.angular.y)) > 1e-6:
            self.abort('failed', 'non_planar_nav2_command')
            return
        self.raw, self.raw_wall = message, time.monotonic()

    def goal_available(self):
        if self.mission is not None or self.request is not None:
            self.get_logger().warning('Goal rejected: finish or cancel the current goal first.')
            return False
        if not (self.fresh() and self.stopped() and self.active() and self.owns_command() and self.world_matches()
                and self.command_pub.get_subscription_count()):
            self.get_logger().warning('Goal rejected: wait for READY, stopped robot, and stop other command publishers.')
            return False
        return True

    def validate_location(self, header, point):
        if not all(math.isfinite(v) for v in (point.x, point.y, point.z)):
            raise ValueError('Non-finite goal position.')
        if header.frame_id not in ('map', self.namespace+'/odom'):
            raise ValueError('Goal frame must be map or the Hunter odometry frame.')
        now_sim = self.get_clock().now().nanoseconds*1e-9
        seconds = header.stamp.sec + header.stamp.nanosec*1e-9
        if seconds and (seconds < self.goal_floor or seconds > now_sim+.5 or now_sim-seconds > 2.):
            raise ValueError('Stale/future goal. Set RViz use_sim_time=true and click again.')
        if not self.grid.inside(point.x,point.y,.2) or not self.grid.free(point.x,point.y):
            raise ValueError('Goal must be on supported, connected free ground in this environment map.')

    def begin_mission(self, point, yaw=None):
        self.mission = dict(goal=[point.x, point.y, yaw], goal_mode='position' if yaw is None else 'pose',
                            start_sim_s=self.observation.seconds,
                            start_pose=[self.observation.pose.x, self.observation.pose.y, self.observation.pose.yaw],
                            last_x=self.observation.pose.x, last_y=self.observation.pose.y,
                            travelled_distance_m=0., planned_distance_m=None,
                            selected_goal_yaw_rad=yaw, heading_candidates=[])
        self.settle_start = self.settle_wall = None
        self.raw, self.raw_wall = Twist(), -math.inf
        empty = RosPath()
        empty.header.frame_id, empty.header.stamp = 'map', stamp(self.observation.seconds)
        self.path_pub.publish(empty)

    def on_goal(self, message):
        if not self.goal_available():
            return
        try:
            p, q = message.pose.position, message.pose.orientation
            self.validate_location(message.header, p)
            if not all(math.isfinite(v) for v in (q.w, q.x, q.y, q.z)):
                raise ValueError('Non-finite goal.')
            norm = math.sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z)
            if abs(norm-1.) > .01 or abs(q.x)+abs(q.y) > .01:
                raise ValueError('Goal requires a normalized planar orientation.')
            yaw = math.atan2(2.*q.w*q.z, 1.-2.*q.z*q.z)
            if not self.grid.footprint_free(p.x,p.y,yaw,FOOTPRINT):
                raise ValueError('Vehicle footprint overlaps blocked or unsupported ground at the goal.')
        except ValueError as error:
            self.get_logger().warning('Goal rejected: '+str(error))
            return
        goal = copy.deepcopy(message)
        goal.header.frame_id = 'map'  # Simulator owns an identity map->odom transform.
        goal.header.stamp = stamp(self.observation.seconds)
        goal.pose.position.z = 0.
        self.begin_mission(p, yaw)
        self.set_phase('PLANNING', 'Accepted RViz goal; planning a forward-only path.')
        self.send_action(self.client, NavigateToPose.Goal(pose=goal), self.on_result, moving=True)

    def on_point(self, message):
        if not self.goal_available():
            return
        try:
            self.validate_location(message.header, message.point)
        except ValueError as error:
            self.get_logger().warning('Point goal rejected: '+str(error))
            return
        self.begin_mission(message.point)
        if self.errors()[0] <= .10:
            self.mission['planned_distance_m'] = 0.
            self.settle_wall = time.monotonic()
            self.set_phase('SETTLING', 'Already near the clicked position; verifying actual stop.')
            return
        x, y, _ = self.mission['goal']
        radius = planning_turning_radius(self.robot)
        start = PoseStamped()
        start.header.frame_id, start.header.stamp = 'map', stamp(self.observation.seconds)
        start.pose.position.x, start.pose.position.y = self.observation.pose.x, self.observation.pose.y
        start.pose.orientation.w = math.cos(self.observation.pose.yaw/2.)
        start.pose.orientation.z = math.sin(self.observation.pose.yaw/2.)
        self.position_search = dict(headings=heading_candidates(self.observation.pose, x, y, radius),
                                    index=0, start=start, best=None, began=time.monotonic())
        self.set_phase('PLANNING', 'Position-only goal: comparing terminal headings with Smac.')
        self.next_candidate()

    def next_candidate(self):
        search = self.position_search
        if search is None or self.mission is None or self.request is not None:
            return
        if search['index'] == len(search['headings']):
            self.follow_selected_path()
            return
        heading = search['headings'][search['index']]
        search['index'] += 1
        goal = PoseStamped()
        goal.header.frame_id, goal.header.stamp = 'map', stamp(self.observation.seconds)
        goal.pose.position.x, goal.pose.position.y = self.mission['goal'][:2]
        goal.pose.orientation.w, goal.pose.orientation.z = math.cos(heading/2.), math.sin(heading/2.)
        action = ComputePathToPose.Goal(goal=goal, start=search['start'], planner_id='GridBased', use_start=True)
        request = self.send_action(self.planner_client, action, self.on_candidate_result)
        if request is not None:
            request['heading'] = heading

    def on_candidate_result(self, request, future):
        if self.request is not request:
            return
        self.request = None
        search = self.position_search
        if request['cancel'] or self.mission is None or search is None:
            return
        try:
            result = future.result()
            candidate = dict(yaw_rad=request['heading'], status=result.status, distance_m=None)
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                path = result.result.path
                points = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
                length = path_distance(points)
                x, y, _ = self.mission['goal']
                if path.header.frame_id != 'map' or math.hypot(points[-1][0]-x, points[-1][1]-y) > .12:
                    raise ValueError('Planner returned a path outside the requested goal/frame.')
                if self.path_is_usable(path):
                    candidate['distance_m'] = length
                    if search['best'] is None or length < search['best']['length']:
                        search['best'] = dict(path=path, length=length, heading=request['heading'])
                else:
                    candidate['rejection'] = 'insufficient_footprint_clearance'
            self.mission['heading_candidates'].append(candidate)
        except Exception as error:
            self.abort('failed', 'heading_search_exception: '+str(error))
            return
        self.next_candidate()

    def path_is_usable(self, path):
        return True

    def follow_selected_path(self):
        search, self.position_search = self.position_search, None
        if self.mission is None or search is None:
            return
        best = search['best']
        self.mission['heading_search_wall_s'] = time.monotonic()-search['began']
        if best is None:
            self.abort('failed', 'no_feasible_terminal_heading')
            return
        self.mission['selected_goal_yaw_rad'] = best['heading']
        self.mission['planned_distance_m'] = best['length']
        path = best['path']
        path.header.stamp = stamp(self.observation.seconds)
        for pose in path.poses:
            pose.header = path.header
        self.path_pub.publish(path)
        action = FollowPath.Goal(path=path, controller_id='FollowPath', goal_checker_id='position_goal_checker')
        self.set_phase('PLANNING', 'Selected shortest valid candidate; requesting Nav2 path following.')
        self.send_action(self.follower_client, action, self.on_result, moving=True)

    def on_plan(self, message):
        # Smac publishes each candidate to /plan. Only expose the executed path
        # to RViz; pose-mode NavigateToPose still owns its own planning phase.
        if self.mission is not None and self.mission['goal_mode'] == 'pose':
            self.path_pub.publish(message)
            if len(message.poses) >= 2:
                self.mission['planned_distance_m'] = path_distance(
                    (p.pose.position.x, p.pose.position.y) for p in message.poses)

    def send_action(self, client, goal, callback, moving=False):
        request = self.request = dict(handle=None, cancel=False, cancel_sent=False)
        try:
            future = client.send_goal_async(goal)
        except Exception as error:
            self.request = None
            self.abort('failed', 'nav2_goal_send_failed: '+str(error))
            return None

        def accepted(done):
            try:
                handle = done.result()
                request['handle'] = handle
                if not handle.accepted:
                    if self.request is request:
                        self.request = None
                        self.abort('failed', 'nav2_goal_rejected')
                    return
                handle.get_result_async().add_done_callback(lambda result: callback(request, result))
                if request['cancel'] or self.request is not request:
                    self.cancel_request(request)
                elif moving:
                    self.set_phase('NAVIGATING', 'Following Nav2 path.')
            except Exception as error:
                if self.request is request:
                    self.request = None
                    self.abort('failed', 'nav2_goal_exception: '+str(error))

        future.add_done_callback(accepted)
        return request

    def on_result(self, request, future):
        if self.request is not request:
            return
        self.request = None
        self.raw, self.raw_wall = Twist(), -math.inf
        self.send_stop()
        if request['cancel'] or self.mission is None:
            return
        try:
            status = future.result().status
        except Exception as error:
            self.abort('failed', 'nav2_result_exception: '+str(error))
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.settle_wall = time.monotonic()
            self.set_phase('SETTLING', 'Nav2 finished; verifying rear-axle pose and actual stop.')
        else:
            self.abort('failed', f'nav2_action_status_{status}')

    def cancel_request(self, request=None):
        request = request or self.request
        if request is None:
            return
        request['cancel'] = True
        if request['handle'] is not None and not request['cancel_sent']:
            request['cancel_sent'] = True
            request['handle'].cancel_goal_async()

    def on_cancel(self, request, response):
        self.abort('cancelled', 'user_cancelled')
        response.success, response.message = True, 'Goal cancelled; braking. Click a new goal after stopping.'
        return response

    def errors(self):
        if self.mission is None or self.observation is None:
            return None, None
        x, y, yaw = self.mission['goal']
        p = self.observation.pose
        return math.hypot(p.x-x, p.y-y), None if yaw is None else abs(wrap(p.yaw-yaw))

    def finish(self, status, reason):
        if self.mission is None:
            return
        distance, angle = self.errors()
        elapsed = max(0., self.observation.seconds-self.mission['start_sim_s'])
        result = dict(self.mission, status=status, reason=reason,
                      mode='nav2_forward_only_'+self.world.name, environment=self.world.name,
                      map_source=self.grid.source, reference_point='rear_axle_midpoint',
                      elapsed_sim_s=elapsed, time_cost_s=elapsed if status == 'succeeded' else None,
                      position_error_m=distance, yaw_error_rad=angle,
                      final_pose=[self.observation.pose.x, self.observation.pose.y, self.observation.pose.yaw],
                      final_speed_m_s=self.observation.speed, final_yaw_rate_rad_s=self.observation.yaw_rate)
        result.pop('last_x')
        result.pop('last_y')
        folder = self.root/'outputs/nav2'
        folder.mkdir(parents=True, exist_ok=True)
        path = folder/f'{datetime.now():%Y%m%d_%H%M%S_%f}.json'
        # Close the motion gate even if recording fails.
        self.mission = None
        self.position_search = None
        self.last_result, self.result_file = result, str(path)
        self.set_phase(status.upper(), reason)
        try:
            with path.open('x') as stream:
                json.dump(result, stream, indent=2, allow_nan=False)
        except OSError as error:
            self.get_logger().error(f'Could not save navigation result: {error}')
        print('HUNTER_NAV2_RESULT '+json.dumps(dict(result, result_file=str(path)), allow_nan=False), flush=True)

    def abort(self, status, reason):
        self.cancel_request()
        self.position_search = None
        self.raw, self.raw_wall = Twist(), -math.inf
        self.send_stop()
        if self.mission is not None:
            self.finish(status, reason)
        else:
            self.set_phase(status.upper(), reason)

    def send_stop(self):
        self.send(0., self.last_steering, 0.)

    def send(self, speed, angle, acceleration):
        message = AckermannDriveStamped()
        message.header.frame_id = self.namespace+'/base_link'
        message.header.stamp = stamp(self.observation.seconds) if self.observation else self.get_clock().now().to_msg()
        message.drive.speed, message.drive.steering_angle = float(speed), float(angle)
        message.drive.acceleration = float(acceleration)
        self.command_pub.publish(message)

    def tick(self):
        now = time.monotonic()
        if now-self.last_map >= 1.:
            self.publish_map()
        if now-self.last_health_poll >= .5:
            self.last_health_poll = now
            for name, client in self.state_clients.items():
                if name in self.state_futures and not self.state_futures[name].done():
                    continue
                if not client.service_is_ready():
                    self.states[name] = False
                    continue
                future = self.state_futures[name] = client.call_async(GetState.Request())

                def state_done(done, identity=name):
                    try:
                        self.states[identity] = done.result().current_state.id == 3
                    except Exception:
                        self.states[identity] = False

                future.add_done_callback(state_done)
        if self.mission is not None:
            if not self.fresh():
                self.abort('failed', 'odometry_timeout')
            elif not self.world_matches():
                self.abort('failed', 'simulator_environment_mismatch')
            elif not self.owns_command():
                self.abort('failed', 'another_command_publisher_is_active')
            elif not self.active():
                self.abort('failed', 'nav2_inactive')
        if self.phase == 'NAVIGATING' and self.mission is not None:
            if now-self.raw_wall <= .3:
                try:
                    drive = twist_to_drive(self.raw.linear.x, self.raw.linear.y, self.raw.angular.z, self.robot)
                    if drive.speed > 0.:
                        self.last_steering = drive.steering_angle
                    self.send(drive.speed, self.last_steering, drive.acceleration)
                except ValueError as error:
                    self.abort('failed', str(error))
            else:
                # Planner/controller delays must not leave an old command driving.
                self.send_stop()
        else:
            self.send_stop()
        if self.phase == 'SETTLING' and self.mission is not None:
            distance, angle = self.errors()
            if distance <= .15 and (angle is None or angle <= .15) and self.stopped():
                if self.settle_start is None:
                    self.settle_start = self.observation.seconds
                elif self.observation.seconds-self.settle_start >= .3:
                    self.finish('succeeded', 'goal_reached_and_stopped')
            else:
                self.settle_start = None
            if self.mission is not None and now-self.settle_wall > 10.:
                self.abort('failed', 'final_pose_or_stop_not_verified')
        if (self.phase == 'WAITING' and self.fresh() and self.stopped() and self.active()
                and self.owns_command() and self.world_matches() and self.command_pub.get_subscription_count()):
            self.set_phase('READY', self.ready_instruction)
            if not self.printed_ready:
                self.printed_ready = True
                print('HUNTER_NAV2_READY: '+self.ready_instruction, flush=True)
        if now-self.last_visual >= .5:
            self.publish_visuals()

    def publish_map(self):
        self.map.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map)
        self.last_map = time.monotonic()

    def publish_visuals(self):
        distance, angle = self.errors()
        state = dict(mode='nav2', phase=self.phase, reason=self.reason, namespace=self.namespace,
                     environment=self.world.name, simulator_environment=self.simulator_environment,
                     nav2_active=self.active(), odometry_fresh=self.fresh(),
                     command_owner=self.owns_command(), goal=None if self.mission is None else self.mission['goal'],
                     goal_mode=None if self.mission is None else self.mission['goal_mode'],
                     heading_search=None if self.position_search is None else dict(
                         completed=len(self.mission['heading_candidates']), total=len(self.position_search['headings'])),
                     position_error_m=distance, yaw_error_rad=angle,
                     last_result=self.last_result, result_file=self.result_file)
        self.status_pub.publish(String(data=json.dumps(state, allow_nan=False)))
        markers = [Marker(action=Marker.DELETEALL)]
        # Publish Point picks rendered geometry, not an abstract XY plane.
        # Supply a selectable floor under the map/grid, independent of USD.
        floor = Marker()
        floor.header.frame_id = 'map'
        floor.ns, floor.id, floor.type = 'hunter_nav2', 3, Marker.CUBE
        floor.pose.orientation.w, floor.pose.position.z = 1., -.04
        x0,y0,x1,y1 = self.grid.bounds
        floor.pose.position.x, floor.pose.position.y = (x0+x1)/2., (y0+y1)/2.
        floor.scale.x, floor.scale.y, floor.scale.z = x1-x0, y1-y0, .02
        floor.color.r, floor.color.g, floor.color.b, floor.color.a = .18, .20, .23, 1.
        markers.append(floor)
        boundary = Marker()
        boundary.header.frame_id = 'map'
        boundary.ns,boundary.id,boundary.type = 'hunter_nav2',4,Marker.LINE_STRIP
        boundary.pose.orientation.w,boundary.scale.x = 1.,.06
        boundary.color.r,boundary.color.g,boundary.color.b,boundary.color.a = 1.,.65,.1,1.
        boundary.points = [Point(x=x,y=y,z=.03) for x,y in
                           ((x0,y0),(x1,y0),(x1,y1),(x0,y1),(x0,y0))]
        markers.append(boundary)
        body = Marker()
        body.header.frame_id = self.namespace+'/base_link'
        body.ns, body.id, body.type = 'hunter_nav2', 0, Marker.LINE_STRIP
        body.pose.orientation.w = 1.
        body.scale.x = .03
        body.color.r, body.color.g, body.color.b, body.color.a = .2, .8, 1., 1.
        body.points = [Point(x=x, y=y, z=.04) for x, y in FOOTPRINT+[FOOTPRINT[0]]]
        markers.append(body)
        text = Marker()
        text.header.frame_id = self.namespace+'/base_link'
        text.ns, text.id, text.type = 'hunter_nav2', 1, Marker.TEXT_VIEW_FACING
        text.pose.orientation.w, text.pose.position.z = 1., .7
        text.scale.z = .2
        text.color.r, text.color.g, text.color.a = 1., .8, 1.
        text.text = 'Hunter Nav2: '+self.phase
        markers.append(text)
        if self.mission is not None:
            x, y, yaw = self.mission['goal']
            goal = Marker()
            goal.header.frame_id = 'map'
            goal.ns, goal.id = 'hunter_nav2', 2
            goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = x, y, .05
            if yaw is None:
                goal.type, goal.pose.orientation.w = Marker.CYLINDER, 1.
                goal.scale.x, goal.scale.y, goal.scale.z = .25, .25, .06
            else:
                goal.type = Marker.ARROW
                goal.pose.orientation.w, goal.pose.orientation.z = math.cos(yaw/2.), math.sin(yaw/2.)
                goal.scale.x, goal.scale.y, goal.scale.z = .6, .09, .09
            goal.color.r, goal.color.g, goal.color.a = 1., .7, 1.
            markers.append(goal)
        self.marker_pub.publish(MarkerArray(markers=markers))
        self.last_visual = time.monotonic()
