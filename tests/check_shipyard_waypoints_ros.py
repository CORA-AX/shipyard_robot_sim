#!/usr/bin/env python3
"""Real ROS sequencing, with synthetic dynamics or both robots in Isaac shipyard."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'scripts/shipyard'),str(ROOT/'tests')]
from common import FLEET, HINTS, PD_CONFIG


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend',choices=('synthetic','isaac'),default='synthetic')
    parser.add_argument('--domain',type=int,default=86)
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    import yaml
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from check_waypoint_ros import SyntheticWorld
    from waypoints import create_node

    out = ROOT/'outputs/shipyard/waypoints'/args.backend
    out.mkdir(parents=True,exist_ok=True)
    result = dict(passed=False,backend=args.backend)
    result_path = out/'integration.json'
    result_path.write_text(json.dumps(result)+'\n')
    opts = SimpleNamespace(robots_file=str(FLEET),config=str(PD_CONFIG),hints=str(HINTS),
        robot='go2_0',route='smoke',allow_ramps=False,spacing=1.,ready_timeout=300.,idle=False,
        frame='map',goal_topic='/shipyard/manual_goal_disabled',track_yaw=False,
        report=str(out/'controller.json'),route_report=str(out/'route.json'))
    context = Context()
    context.init(args=[])
    node = executor = synthetic = simulator = log = None
    try:
        if args.backend=='synthetic':
            synthetic = SyntheticWorld(yaml.safe_load(FLEET.read_text()))
        else:
            log = (out/'simulation.log').open('w')
            simulator = subprocess.Popen([str(ROOT/'.conda/isaacsim-5.0/bin/python'),
                str(ROOT/'scripts/shipyard/simulate.py'),'--display','headless',
                '--ros-domain-id',str(args.domain),'--report',str(out/'simulation.json')],
                cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        node = create_node(opts,context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        def tick():
            if simulator and simulator.poll() is not None:
                raise RuntimeError((out/'simulation.log').read_text()[-5000:])
            if synthetic:
                synthetic.tick()
            for _ in range(32):
                executor.spin_once(timeout_sec=0.)
            node.tick()
            time.sleep(.01)
        def wait(condition,timeout=90):
            until = time.monotonic()+timeout
            while not condition():
                if time.monotonic()>until:
                    raise AssertionError(('Timed out',node.route_events[-4:]))
                if node.route_phase=='FAILED':
                    raise AssertionError(node.route_events[-1])
                tick()
        wait(lambda:node.completed_routes==1,360 if simulator else 90)
        first = node.controllers['go2_0'].state
        assert math.dist((first.x,first.y),(-16,18))<.14
        node.next_robot()
        wait(lambda:node.completed_routes==2,150)
        second = node.controllers['lekiwi_0'].state
        assert math.dist((second.x,second.y),(-23.5,-20))<.10
        result.update(two_robot_round_trips=True,go2_end=[first.x,first.y],lekiwi_end=[second.x,second.y])
        # Cancel is sticky: queued waypoints must not restart on a later tick.
        node.request_start()
        wait(lambda:node.route_phase=='RUNNING')
        node.cancel()
        accepted = node.report['goals_accepted']
        for _ in range(40):
            tick()
        assert node.route_phase=='CANCELED' and node.report['goals_accepted']==accepted
        assert all(c.command==(0.,0.,0.) for c in node.controllers.values())
        result['cancel_stops_queue'] = True
        if synthetic:
            # Reset notification and stale odometry must terminate the queue.
            node.request_start()
            wait(lambda:node.route_phase=='RUNNING')
            synthetic.robots[1][1].reset_requested = True
            for _ in range(100):
                tick()
                if node.route_phase=='FAILED':
                    break
            assert node.route_phase=='FAILED' and node.route_events[-1]['reason']=='RESET'
            result['reset_stops_queue'] = True
            node.request_start()
            wait(lambda:node.route_phase=='RUNNING')
            synthetic.drop.add('lekiwi_0')
            for _ in range(150):
                tick()
                if node.route_phase=='FAILED':
                    break
            assert node.route_phase=='FAILED' and node.route_events[-1]['reason']=='ODOM_TIMEOUT'
            result['stale_odometry_stops_queue'] = True
            synthetic.drop.clear()
            for _ in range(30):
                tick()
            node.request_start()
            wait(lambda:node.route_phase=='RUNNING')
            selected = node.controllers['lekiwi_0'].state
            synthetic.robots[0][2].data.root_pos_w[0,:2] = (selected.x+.5,selected.y)
            for _ in range(50):
                tick()
                if node.route_phase=='FAILED':
                    break
            assert node.route_phase=='FAILED' and node.route_events[-1]['reason']=='OTHER_ROBOT_BLOCKING: go2_0'
            result['parked_robot_blocks_route'] = True
        result['passed'] = True
    finally:
        if node is not None:
            node.close()
            executor.shutdown()
            node.destroy_node()
        if synthetic:
            synthetic.close()
        context.try_shutdown()
        if simulator and simulator.poll() is None:
            os.killpg(simulator.pid,signal.SIGINT)
            try:
                simulator.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(simulator.pid,signal.SIGKILL)
                simulator.wait(timeout=10)
        if log:
            log.close()
        if args.backend=='isaac' and (out/'simulation.json').exists():
            sim_report = json.loads((out/'simulation.json').read_text())
            result['simulation_resets'] = sim_report.get('resets')
            result['robot_contacts'] = sim_report.get('robot_contacts')
            result['passed'] = result['passed'] and sim_report.get('passed',False) and sim_report.get('resets')==0 and not sim_report.get('robot_contacts')
        elif args.backend=='isaac':
            result.update(passed=False,error='Simulator did not produce a final report.')
        result_path.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    assert result['passed'],result


if __name__=='__main__':
    main()
