#!/usr/bin/env python3
"""Shipyard-only shared simulator; existing launchers and modules are not patched."""
import argparse
import json
import os
import signal
import sys

from common import ROOT, FLEET, load_fleet


def main():
    from robot_sim.environment import isaac_environment
    from visualization.launcher import launch_display, run_process
    from visualization.runtime import prepare_environment, kit_arguments
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robots-file',default=str(FLEET))
    parser.add_argument('--display',choices=('local','webrtc','headless'),default='local')
    parser.add_argument('--server-ip',default='')
    parser.add_argument('--device',default='cuda:0')
    parser.add_argument('--policy-device',default='cpu')
    parser.add_argument('--ros-domain-id',type=int,default=int(os.environ.get('ROS_DOMAIN_ID','0')))
    parser.add_argument('--steps',type=int,default=0)
    parser.add_argument('--report',default=str(ROOT/'outputs/shipyard/waypoints/simulation.json'))
    parser.add_argument('--worker',action='store_true',help=argparse.SUPPRESS)
    # launch_display supplies these switches to the worker.
    parser.add_argument('--headless',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--livestream',type=int,default=0,help=argparse.SUPPRESS)
    parser.add_argument('--experience',default='',help=argparse.SUPPRESS)
    parser.add_argument('--kit_args',default='',help=argparse.SUPPRESS)
    args = parser.parse_args()
    world = load_fleet(args.robots_file)
    if not 0 <= args.ros_domain_id <= 232 or args.steps < 0:
        parser.error('Use ROS domain 0..232 and nonnegative steps.')
    if args.display=='webrtc' and not args.server_ip:
        parser.error('WebRTC needs --server-ip.')
    from pathlib import Path
    if Path(sys.prefix).resolve() != (ROOT/'.conda/isaacsim-5.0').resolve():
        parser.error('Use .conda/isaacsim-5.0/bin/python.')
    report_path = Path(args.report).resolve()
    report_path.parent.mkdir(parents=True,exist_ok=True)
    if not args.worker:
        environment = isaac_environment(sys.executable,python_paths=[ROOT,ROOT/'scripts',
            ROOT/'ros2_ws/src/robot_sim',ROOT/'ros2_ws/src/robot_control'])
        environment['ROS_DOMAIN_ID'] = str(args.ros_domain_id)
        command = [str(Path(__file__).resolve()),*sys.argv[1:],'--worker']
        report_path.write_text(json.dumps(dict(passed=False,status='starting'))+'\n')
        if args.display=='headless':
            environment = prepare_environment(ROOT,'shipyard_waypoints',environment)
            status = run_process([sys.executable,*command,'--headless'],cwd=ROOT,environment=environment)
        else:
            for key in set(os.environ)-set(environment):
                os.environ.pop(key,None)
            os.environ.update(environment)
            status = launch_display(command,root=ROOT,robot='shipyard_waypoints',
                                    webrtc=args.display=='webrtc',server_ip=args.server_ip)
        report = json.loads(report_path.read_text())
        raise SystemExit(status or (0 if report.get('passed') or report.get('stopped_by_user') else 1))
    os.environ['ROS_DOMAIN_ID'] = str(args.ros_domain_id)
    prepare_environment(ROOT,'shipyard_waypoints')
    from isaaclab.app import AppLauncher
    app = AppLauncher(headless=args.headless or args.display=='headless',device=args.device,
                      livestream=args.livestream,experience=args.experience,
                      kit_args=args.kit_args+' '+kit_arguments(ROOT,'shipyard_waypoints')+
                      ' --/isaac/startup/ros_bridge_extension=').app
    args.stop_requested = False
    args.command_timeout = .5
    def stop(*_):
        args.stop_requested = True
    signal.signal(signal.SIGTERM,stop)
    signal.signal(signal.SIGINT,stop)
    report = dict(passed=False,scenario='shipyard')
    try:
        from runtime import run
        run(app,args,world,report)
    except Exception as exc:
        import traceback
        traceback.print_exc()
        report.update(passed=False,error=str(exc))
    finally:
        report_path.write_text(json.dumps(report,indent=2)+'\n')
        app.close()


if __name__=='__main__':
    main()
