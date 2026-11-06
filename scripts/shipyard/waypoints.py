#!/usr/bin/env python3
"""Sequence shipyard routes using the unchanged WaypointNode/WaypointPD controller."""
import argparse
import json
import math
from pathlib import Path
import signal
import time

from common import ROOT, FLEET, HINTS, PD_CONFIG, load_fleet
from route_plan import build_plan, distance_to_segment


def create_node(args,context):
    from geometry_msgs.msg import Point, PoseStamped
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from visualization_msgs.msg import Marker, MarkerArray
    from robot_control.nodes.waypoint import WaypointNode

    class ShipyardWaypointNode(WaypointNode):
        def __init__(self):
            self.route_phase = 'IDLE'
            self.route_points = []
            self.route_index = 0
            self.route_events = []
            self.completed_routes = 0
            self.wait_started = time.monotonic()
            super().__init__(args,context)
            self.fleet = {r['robot_id']:r for r in load_fleet(args.robots_file)['robots']}
            qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.route_pub = self.create_publisher(String,'/shipyard/route_status',qos)
            self.route_markers = self.create_publisher(MarkerArray,'/shipyard/route_markers',qos)
            self.start_service = self.create_service(Trigger,'/shipyard/start_route',self.on_start)
            self.route_status('IDLE')
            if not args.idle:
                self.request_start()

        def route_status(self,phase,reason=''):
            self.route_phase = phase
            event = dict(phase=phase,robot=self.selected,route=args.route,
                         waypoint_index=self.route_index,waypoint_count=len(self.route_points),
                         completed_routes=self.completed_routes,reason=reason)
            self.route_events.append(event)
            self.route_events = self.route_events[-2000:]
            self.route_pub.publish(String(data=json.dumps(event)))
            print('SHIPYARD_ROUTE '+json.dumps(event),flush=True)

        def request_start(self):
            if self.route_phase in ('RUNNING','WAITING_READY'):
                raise ValueError('A route is already active; cancel it before restarting.')
            plan = build_plan(args.hints,self.fleet[self.selected],args.route,args.allow_ramps,args.spacing)
            self.route_points = plan['points']
            self.route_index = 0
            self.wait_started = time.monotonic()
            self.route_status('WAITING_READY')
            marker = Marker()
            marker.header.frame_id = args.frame
            marker.ns,marker.id = 'shipyard_route',0
            marker.type = Marker.LINE_STRIP
            marker.pose.orientation.w = 1.
            marker.scale.x = .05
            marker.color.r,marker.color.g,marker.color.b,marker.color.a = .1,.8,1.,1.
            marker.points = [Point(x=float(x),y=float(y),z=float(z+.06)) for x,y,z in self.route_points]
            self.route_markers.publish(MarkerArray(markers=[marker]))

        def on_start(self,request,response):
            try:
                self.request_start()
                response.success,response.message = True,'Route queued for '+self.selected
            except ValueError as exc:
                response.success,response.message = False,str(exc)
            return response

        def on_goal(self,message):
            # Do not let an RViz click replace a validated graph segment mid-route.
            self.get_logger().warning('Use /shipyard/start_route; arbitrary 2D goals are disabled in this runner.',
                                      throttle_duration_sec=2.)

        def on_reset(self,identity,count):
            previous = self.reset_counts.get(identity)
            super().on_reset(identity,count)
            if (identity==self.selected and previous is not None and previous!=count
                    and self.route_phase in ('RUNNING','WAITING_READY')):
                self.abort('RESET')

        def cancel(self,reason='CANCELED'):
            super().cancel(reason)
            if self.route_phase in ('RUNNING','WAITING_READY'):
                self.route_status('CANCELED',reason)

        def next_robot(self):
            self.cancel('ROBOT_SWITCH')
            super().next_robot()
            self.request_start()

        def on_next(self,request,response):
            try:
                self.next_robot()
                response.success,response.message = True,self.selected
            except ValueError as exc:
                self.abort(str(exc))
                response.success,response.message = False,str(exc)
            return response

        def abort(self,reason):
            super().cancel(reason)
            self.route_status('FAILED',reason)

        def ready(self):
            return not self.braking and all(self.fresh(r) for r in self.controllers) and bool(
                self.command_publishers[self.selected].get_subscription_count())

        def obstacle(self,target):
            state = self.controllers[self.selected].state
            for identity,controller in self.controllers.items():
                if identity==self.selected:
                    continue
                if not self.fresh(identity):
                    return 'OTHER_ROBOT_ODOM_STALE: '+identity
                if distance_to_segment((controller.state.x,controller.state.y),
                                       (state.x,state.y),target)<1.0:
                    return 'OTHER_ROBOT_BLOCKING: '+identity
            return None

        def dispatch(self):
            target = self.route_points[self.route_index]
            reason = self.obstacle(target)
            if reason:
                self.abort(reason)
                return
            message = PoseStamped()
            message.header.frame_id = args.frame
            message.header.stamp = self.get_clock().now().to_msg()
            message.pose.position.x,message.pose.position.y,message.pose.position.z = map(float,target)
            message.pose.orientation.w = 1.
            # Directly reuse the original TF, freshness, ownership and goal validation.
            accepted = self.report['goals_accepted']
            super().on_goal(message)
            if self.report['goals_accepted']!=accepted+1:
                self.abort('GOAL_REJECTED')
            else:
                self.route_status('RUNNING')

        def tick(self):
            if self.route_phase=='RUNNING':
                controller = self.controllers[self.selected]
                if not self.fresh(self.selected):
                    self.abort('ODOM_TIMEOUT')
                elif self.obstacle(self.route_points[self.route_index]):
                    self.abort(self.obstacle(self.route_points[self.route_index]))
            super().tick()
            if self.route_phase=='WAITING_READY':
                if time.monotonic()-self.wait_started>args.ready_timeout:
                    self.abort('READY_TIMEOUT')
                elif self.ready():
                    state = self.controllers[self.selected].state
                    if math.dist((state.x,state.y),self.route_points[0][:2])>.4:
                        self.abort('NOT_AT_PARKING: reset the selected robot before starting a route')
                    else:
                        self.dispatch()
            elif self.route_phase=='RUNNING':
                controller = self.controllers[self.selected]
                if controller.status=='REACHED':
                    self.route_index += 1
                    if self.route_index==len(self.route_points):
                        self.completed_routes += 1
                        self.route_status('COMPLETED')
                    else:
                        self.dispatch()
                elif controller.status!='TRACKING':
                    self.abort(controller.status)

        def close(self):
            super().close()
            path = Path(args.route_report)
            path.parent.mkdir(parents=True,exist_ok=True)
            path.write_text(json.dumps(dict(completed_routes=self.completed_routes,
                final_phase=self.route_phase,events=self.route_events),indent=2)+'\n')

    return ShipyardWaypointNode()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robots-file',default=str(FLEET))
    parser.add_argument('--hints',default=str(HINTS))
    parser.add_argument('--config',default=str(PD_CONFIG))
    parser.add_argument('--robot',default='go2_0')
    parser.add_argument('--route',default='flat_only')
    parser.add_argument('--allow-ramps',action='store_true')
    parser.add_argument('--spacing',type=float,default=2.)
    parser.add_argument('--ready-timeout',type=float,default=300.)
    parser.add_argument('--idle',action='store_true',help='Wait for /shipyard/start_route instead of starting automatically.')
    parser.add_argument('--no-keyboard',action='store_true')
    parser.add_argument('--exit-on-complete',action='store_true')
    parser.add_argument('--preview',action='store_true',help='Print the route without starting ROS or moving robots.')
    parser.add_argument('--report',default=str(ROOT/'outputs/shipyard/waypoints/controller.json'))
    parser.add_argument('--route-report',default=str(ROOT/'outputs/shipyard/waypoints/route.json'))
    args = parser.parse_args()
    fleet = load_fleet(args.robots_file)
    selected = next((r for r in fleet['robots'] if r['robot_id']==args.robot),None)
    if selected is None:
        parser.error('--robot must match the fleet.')
    if not math.isfinite(args.ready_timeout) or args.ready_timeout<=0:
        parser.error('ready-timeout must be finite and positive.')
    try:
        plan = build_plan(args.hints,selected,args.route,args.allow_ramps,args.spacing)
    except ValueError as exc:
        parser.error(str(exc))
    if args.preview:
        print(json.dumps(plan,indent=2))
        return
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from robot_control.waypoint_keyboard import SelectionKeyboard
    args.frame,args.goal_topic,args.track_yaw = 'map','/shipyard/manual_goal_disabled',False
    context = Context()
    context.init(args=[])
    def terminate(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,terminate)
    signal.signal(signal.SIGINT,terminate)
    node,executor = None,None
    result = 0
    try:
        with SelectionKeyboard(not args.no_keyboard) as keyboard:
            node = create_node(args,context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            quit_requested = False
            while context.ok() and not quit_requested:
                started = time.monotonic()
                for _ in range(32):
                    executor.spin_once(timeout_sec=0.)
                for event in keyboard.poll():
                    if event=='next':
                        try:
                            node.next_robot()
                        except ValueError as exc:
                            node.abort(str(exc))
                    elif event=='cancel':
                        node.cancel()
                    elif event=='quit':
                        quit_requested = True
                node.tick()
                if args.exit_on_complete and node.route_phase in ('COMPLETED','FAILED','CANCELED'):
                    result = 0 if node.route_phase=='COMPLETED' else 1
                    break
                time.sleep(max(0.,1/node.settings['rate']-(time.monotonic()-started)))
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            if executor is not None:
                executor.shutdown()
            node.destroy_node()
        context.try_shutdown()
    raise SystemExit(result)


if __name__=='__main__':
    main()
