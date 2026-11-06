"""Stop-confirmed waypoint queue on the existing single Nav2 command gate."""
from datetime import datetime
import json
import math

from geometry_msgs.msg import Point, PointStamped
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from hunter.ros.bridge import stamp
from hunter.ros.nav2.goal import HunterNav2Goal
from hunter.navigation.nav2.config import FOOTPRINT
from hunter.navigation.routes import route_path_clear


class HunterPatrol(HunterNav2Goal):
    ready_instruction = 'Stored waypoint route ready. RViz click goals disabled; use patrol/start or auto-start.'

    def __init__(self, *args, route, idle=False, **kwargs):
        self.route = route
        self.route_phase = 'WAITING'
        self.auto_start = not idle
        self.route_index = 0
        self.route_results = []
        self.route_start = None
        self.route_result = None
        self.route_file = None
        self.route_publishers_ready = False
        super().__init__(*args, **kwargs)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.route_pub = self.create_publisher(String, 'patrol/status', latched)
        self.route_marker_pub = self.create_publisher(MarkerArray, 'patrol/markers', latched)
        self.start_service = self.create_service(Trigger, 'patrol/start', self.on_start)
        self.route_cancel_service = self.create_service(Trigger, 'patrol/cancel', self.on_cancel)
        self.route_publishers_ready = True
        self.publish_visuals()

    def on_goal(self, message):
        self.get_logger().warning('Stored-route mode: RViz goals are disabled. Use hunter/scripts/nav2.py for click goals.')

    def on_point(self, message):
        self.on_goal(message)

    def path_is_usable(self, path):
        # Check the complete rectangle against occupied AND unknown cells,
        # with tracking room, before comparing candidate lengths.
        def poses():
            for stamped in path.poses:
                p, q = stamped.pose.position, stamped.pose.orientation
                yield p.x, p.y, math.atan2(2.*q.w*q.z, 1.-2.*q.z*q.z)
        return route_path_clear(self.grid, poses())

    def on_start(self, request, response):
        response.success, response.message = self.start_route()
        return response

    def start_route(self):
        if self.route_phase == 'RUNNING':
            return False, 'Route already running.'
        if not self.goal_available():
            return False, 'Wait for fresh odometry, stopped vehicle and active Nav2.'
        p = self.observation.pose
        if not self.route.at_start(p.x, p.y):
            return False, 'Return/reset to the configured starting position before starting the route.'
        self.auto_start = False
        self.route_index, self.route_results = 0, []
        self.route_start = self.observation.seconds
        self.route_result = self.route_file = None
        self.route_phase = 'RUNNING'
        self.publish_visuals()
        print(f'HUNTER_PATROL_STARTED route={self.route.name} checkpoints={len(self.route.points)}', flush=True)
        return True, 'Route started; each checkpoint requires arrival and an actual stop.'

    def finish(self, status, reason):
        had_mission = self.mission is not None
        super().finish(status, reason)
        if had_mission and self.route_phase == 'RUNNING':
            point = self.route.points[self.route_index]
            self.route_results.append(dict(checkpoint=point.name, index=self.route_index+1,
                                           result=self.last_result, result_file=self.result_file))
            if status == 'succeeded':
                self.route_index += 1
                if self.route_index == len(self.route.points):
                    self.end_route('succeeded', 'round_trip_completed')
            else:
                self.end_route(status, reason)

    def end_route(self, status, reason):
        self.route_phase = status.upper()
        elapsed = max(0., self.observation.seconds-self.route_start) if self.observation else 0.
        result = dict(route=self.route.name, status=status, reason=reason,
                      environment=self.world.name, reference_point='rear_axle_midpoint',
                      namespace=self.namespace, map_source=self.grid.source,
                      start_sim_s=self.route_start, elapsed_sim_s=elapsed,
                      time_cost_s=elapsed if status == 'succeeded' else None,
                      completed_waypoints=sum(p['result']['status'] == 'succeeded' for p in self.route_results),
                      total_waypoints=len(self.route.points), checkpoints=self.route_results,
                      return_heading_required=False)
        folder = self.root/'outputs/patrol'
        folder.mkdir(parents=True, exist_ok=True)
        path = folder/f'{datetime.now():%Y%m%d_%H%M%S_%f}.json'
        self.route_result, self.route_file = result, str(path)
        try:
            with path.open('x') as stream:
                json.dump(result, stream, indent=2, allow_nan=False)
        except OSError as error:
            self.get_logger().error(f'Could not save patrol result: {error}')
        self.publish_visuals()
        print('HUNTER_PATROL_RESULT '+json.dumps(dict(result, result_file=str(path)), allow_nan=False), flush=True)

    def abort(self, status, reason):
        self.auto_start = False
        super().abort(status, reason)
        # Also stop a queue cancelled between checkpoints or before startup.
        if self.route_phase == 'RUNNING':
            self.end_route(status, reason)
        elif self.route_phase == 'WAITING':
            self.route_phase = status.upper()
            self.publish_visuals()

    def on_cancel(self, request, response):
        self.abort('cancelled', 'user_cancelled')
        response.success = True
        response.message = 'Entire route cancelled; no automatic resume. Restart at the configured start.'
        return response

    def tick(self):
        if self.route_phase == 'RUNNING' and self.fresh():
            p = self.observation.pose
            if not self.grid.footprint_free(p.x, p.y, p.yaw, FOOTPRINT):
                self.abort('failed', 'vehicle_footprint_outside_supported_ground')
        super().tick()
        if self.auto_start and self.phase == 'READY':
            ok, reason = self.start_route()
            if not ok:
                self.abort('failed', reason)
        if self.route_phase != 'RUNNING' or self.mission is not None or self.request is not None:
            return
        if not (self.fresh() and self.active() and self.owns_command() and self.world_matches()
                and self.command_pub.get_subscription_count()):
            self.abort('failed', 'route_readiness_lost')
            return
        if not self.stopped():
            return
        point = self.route.points[self.route_index]
        goal = PointStamped()
        goal.header.frame_id, goal.header.stamp = 'map', stamp(self.observation.seconds)
        goal.point.x, goal.point.y = point.x, point.y
        # Bypass only our RViz-input rejection; retain all base validation/gating.
        HunterNav2Goal.on_point(self, goal)
        if self.mission is None and self.route_phase == 'RUNNING':
            self.abort('failed', 'stored_checkpoint_rejected')

    def publish_visuals(self):
        super().publish_visuals()
        if not self.route_publishers_ready:
            return
        self.route_pub.publish(String(data=json.dumps(dict(
            route=self.route.name, phase=self.route_phase, reason=self.reason,
            next_waypoint=self.route_index+1 if self.route_index < len(self.route.points) else None,
            total_waypoints=len(self.route.points), last_result=self.route_result,
            result_file=self.route_file), allow_nan=False)))
        markers = [Marker(action=Marker.DELETEALL)]
        line = Marker()
        line.header.frame_id, line.ns, line.id, line.type = 'map', 'hunter_patrol', 0, Marker.LINE_STRIP
        line.pose.orientation.w, line.scale.x = 1., .04
        line.color.r, line.color.b, line.color.a = .8, 1., .8
        line.points = [Point(x=x, y=y, z=.06) for x,y in
                       [self.route.start]+[(p.x,p.y) for p in self.route.points]]
        markers.append(line)
        groups = {}
        for i,p in enumerate(self.route.points, 1):
            groups.setdefault((p.x, p.y), []).append((i, p.name))
        for (x,y), entries in groups.items():
            text = Marker()
            text.header.frame_id, text.ns, text.id = 'map', 'hunter_patrol', entries[0][0]
            text.type, text.pose.orientation.w = Marker.TEXT_VIEW_FACING, 1.
            text.pose.position.x, text.pose.position.y, text.pose.position.z = x, y, .4
            text.scale.z = .3
            text.color.r, text.color.g, text.color.b, text.color.a = 1., .8, 1., 1.
            text.text = '\n'.join(f'{i}: {name}' for i,name in entries)
            markers.append(text)
        self.route_marker_pub.publish(MarkerArray(markers=markers))
