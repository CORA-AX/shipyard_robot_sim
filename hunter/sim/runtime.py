"""Hunter CLI straight driving or ROS Ackermann driving; one physics-loop owner."""
import argparse
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import signal
import sys
import tempfile
import time


def startup_status(stage, **details):
    print('HUNTER2_BOOT ' + json.dumps(dict(stage=stage, **details)), flush=True)


def arguments(root):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=root / 'config/robot.yaml')
    parser.add_argument('--usd-path', type=Path, help='Override the saved robot USD path.')
    parser.add_argument('--environment', choices=('flat', 'shipyard'), default='flat')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--speed', type=float, default=0., help='Straight body speed in m/s; negative reverses.')
    mode.add_argument('--ros', action='store_true', help='Drive from ROS2 AckermannDriveStamped commands.')
    parser.add_argument('--namespace', default='hunter2_0')
    parser.add_argument('--ackermann-topic', help='Absolute command topic; default /<namespace>/ackermann_cmd.')
    parser.add_argument('--command-timeout', type=float, default=.5,
                        help='Wall seconds without a command before requesting a ramped stop.')
    parser.add_argument('--ros-domain-id', type=int, default=int(os.environ.get('ROS_DOMAIN_ID', '0')))
    parser.add_argument('--duration', type=float,
                        help='Optional command duration in simulation seconds; omit to run until closed.')
    parser.add_argument('--settle-time', type=float, default=2.)
    parser.add_argument('--brake-time', type=float,
                        help='Braking seconds for timed runs; default allows the speed ramp to reach zero.')
    parser.add_argument('--device', default='cuda:0')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--result-file', type=Path, help=argparse.SUPPRESS)
    return parser


def validate_arguments(parser, args, parameters):
    import re

    args.namespace = args.namespace.strip('/')
    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', args.namespace):
        parser.error('Use a valid ROS namespace, such as hunter2_0.')
    args.ackermann_topic = args.ackermann_topic or '/' + args.namespace + '/ackermann_cmd'
    if not re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', args.ackermann_topic):
        parser.error('--ackermann-topic must be an absolute ROS topic name.')
    if (not math.isfinite(args.command_timeout) or args.command_timeout <= 0
            or not 0 <= args.ros_domain_id <= 232):
        parser.error('Use a positive finite command-timeout and ROS domain 0..232.')
    if args.ros and (args.duration is not None or args.brake_time is not None):
        parser.error('--duration and --brake-time are for CLI speed mode; --ros runs until closed.')
    if args.brake_time is None:
        args.brake_time = max(2., abs(args.speed) / parameters['max_acceleration'] + .1)
    timings = (args.speed, args.settle_time, args.brake_time)
    if not all(math.isfinite(v) for v in timings) or (args.duration is not None and not math.isfinite(args.duration)):
        parser.error('Speed and timings must be finite.')
    if (args.duration is not None and args.duration <= 0) or args.settle_time < 0 or args.brake_time < 0:
        parser.error('Use positive duration and nonnegative settling/braking times.')
    if abs(args.speed) > parameters['max_speed']:
        parser.error(f"Straight speed limit is +/-{parameters['max_speed']} m/s.")
    if args.duration is not None and args.brake_time < abs(args.speed) / parameters['max_acceleration'] + .1:
        parser.error('brake-time must allow the speed ramp to reach zero plus 0.1 s.')


def main(root):
    from hunter.control.controller import load_parameters
    from hunter.environments.config import load_world
    from .environment import isaac_environment
    from .launcher import run_process, kit_arguments, prepare_environment

    root = Path(root)
    parser = arguments(root)
    args = parser.parse_args()
    try:
        load_world(root,args.environment)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    parameters = load_parameters(root, args.config)
    if args.usd_path is not None:
        parameters['usd_path'] = str(args.usd_path.resolve())
    validate_arguments(parser, args, parameters)
    expected_prefix = Path(os.environ.get('HUNTER_ISAAC_PREFIX', root.parent / '.conda/isaacsim-5.0'))
    if Path(sys.prefix).resolve() != expected_prefix.resolve():
        parser.error(f'Activate {expected_prefix} before running hunter/scripts/simulate.py.')
    if not args.worker:
        environment = isaac_environment(sys.executable, python_paths=[root.parent])
        environment['ROS_DOMAIN_ID'] = str(args.ros_domain_id)
        environment = prepare_environment(root, 'hunter2_control', environment)
        if not args.headless:
            environment.update(HEADLESS='0', LIVESTREAM='0')
            if not environment.get('DISPLAY') and Path('/tmp/.X11-unix/X0').exists():
                environment['DISPLAY'] = ':0'
                if (Path.home() / '.Xauthority').is_file():
                    environment.setdefault('XAUTHORITY', str(Path.home() / '.Xauthority'))
        # Kit may terminate Python inside app.close(). Record the outcome before
        # closing, so an import/physics error cannot masquerade as exit code 0.
        with tempfile.TemporaryDirectory(prefix='hunter2_', dir=root / '.runtime/hunter2_control') as directory:
            result_file = Path(directory) / 'result.json'
            command = [sys.executable, str(root / 'scripts/simulate.py'), *sys.argv[1:],
                       '--worker', '--result-file', str(result_file)]
            status = run_process(command, cwd=Path.cwd(), environment=environment)
            result = json.loads(result_file.read_text()) if result_file.is_file() else {}
            if status == 0 and not result.get('successful'):
                print('Hunter run failed: ' + result.get('error', 'worker did not finish initialization'),
                      file=sys.stderr)
                status = 1
        raise SystemExit(status)

    prepare_environment(root, 'hunter2_control')
    startup_status('starting_app',
                   message='Initializing Isaac Sim graphics/shaders; first startup may take several minutes.',
                   kit_log_directory=str(root / '.runtime/hunter2_control/kit/logs/Kit/Isaac-Sim/5.0'))
    from isaaclab.app import AppLauncher
    app = AppLauncher(headless=args.headless, device=args.device,
                      kit_args=kit_arguments(root, 'hunter2_control') +
                      ' --/isaac/startup/ros_bridge_extension=').app
    startup_status('app_started')
    stopping = False

    def stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    result = {'successful': False}
    try:
        run(app, args, root, parameters, lambda: stopping)
        result['successful'] = True
    except Exception as error:
        import traceback
        traceback.print_exc()
        result['error'] = f'{type(error).__name__}: {error}'
    finally:
        if args.result_file is not None:
            args.result_file.write_text(json.dumps(result))
        app.close()
    if not result['successful']:
        raise SystemExit(1)


def run(app, args, root, parameters, stop_requested):
    import torch
    import isaaclab.sim as sim
    from carb.settings import get_settings
    from omni.physx.bindings._physx import SETTING_COLLISION_APPROXIMATE_CYLINDERS
    from hunter.control.controller import Hunter2Controller
    from hunter.control.commands import DriveCommand
    from hunter.ros.odometry import robot_frames
    from .scene import spawn_hunter2
    from .world import spawn_world
    from hunter.environments.config import load_world

    physics_dt, decimation = .005, 4
    control_dt = physics_dt * decimation
    startup_status('creating_scene')
    simulator = sim.SimulationContext(sim.SimulationCfg(dt=physics_dt, render_interval=decimation,
                                                       device=args.device))
    # GUI settings and referenced asset metadata do not configure this new stage.
    # Keep the saved wheel cylinders analytic instead of cooking convex meshes.
    get_settings().set_bool(SETTING_COLLISION_APPROXIMATE_CYLINDERS, False)
    robot = bridge = None
    try:
        world = load_world(root,args.environment)
        spawn_world(world)
        simulator.set_camera_view(eye=world.camera_eye, target=world.camera_target)
        startup_status('loading_model', usd_path=parameters['usd_path'])
        robot, asset = spawn_hunter2(root, '/World/Robots/hunter2_0', parameters,
                                    position=world.position, yaw=world.yaw)
        startup_status('initializing_physics')
        simulator.reset()
        state = robot.data.default_root_state.clone()
        robot.write_root_pose_to_sim(state[:, :7])
        robot.write_root_velocity_to_sim(state[:, 7:])
        robot.write_joint_state_to_sim(robot.data.default_joint_pos, robot.data.default_joint_vel)
        robot.reset()
        robot.update(0.)
        controller = Hunter2Controller(parameters)
        steering_ids = [robot.joint_names.index(name) for name in controller.steering_names]
        rear_ids = [robot.joint_names.index(name) for name in controller.rear_names]
        steering = torch.zeros((1, 2), device=simulator.device)
        rear = torch.zeros_like(steering)
        print('HUNTER2_READY ' + json.dumps(dict(stage='ackermann_drive' if args.ros else 'straight_drive', physics_hz=200,
            control_hz=50, input_mode='ros' if args.ros else 'cli',
            wheel_radius_m=parameters['wheel_radius'],
            approximate_cylinders_with_convex_meshes=get_settings().get_as_bool(SETTING_COLLISION_APPROXIMATE_CYLINDERS),
            requested_speed_m_s=args.speed, environment=world.name,
            joint_names=robot.joint_names, **asset)), flush=True)
        for _ in range(5):
            simulator.render()
        total = None if args.duration is None else args.settle_time + args.duration + args.brake_time
        steps = None if total is None else math.ceil(total / control_dt)
        _, initial_rear = robot_frames(robot.data, asset['rear_axle_offset_base_m'])
        step, previous_command = 0, None
        with torch.inference_mode():
            while app.is_running() and not stop_requested() and (steps is None or step < steps):
                tick = time.monotonic()
                seconds = step * control_dt
                driving = seconds >= args.settle_time and (
                    args.duration is None or seconds < args.settle_time + args.duration)
                stale = False
                if args.ros:
                    if bridge is None and driving:
                        from hunter.ros.bridge import RobotRosBridge
                        bridge = RobotRosBridge(args.ackermann_topic, args.command_timeout,
                                                rear_axle_offset=asset['rear_axle_offset_base_m'],
                                                robot_id='hunter2', namespace=args.namespace, environment=world.name)
                        bridge.publish_clock(seconds)
                        bridge.publish_robot_state(seconds, robot)
                        print(f'HUNTER2_ROS_READY: topic={args.ackermann_topic}; AckermannDriveStamped; '
                              f'odom=/{args.namespace}/odom (rear axle); '
                              f'timeout={args.command_timeout}s; ROS_DOMAIN_ID={args.ros_domain_id}', flush=True)
                    if bridge is not None and not bridge.context.ok():
                        break
                    command = bridge.poll() if bridge is not None else DriveCommand()
                    stale = bridge is None or bridge.command.stale
                    if (command, stale) != previous_command:
                        print('HUNTER2_ROS_COMMAND ' + json.dumps(dict(command=asdict(command), stale=stale,
                            action='request_stop' if stale else ('steer_at_rest' if command.speed == 0.
                                                              else 'ackermann_drive'))), flush=True)
                        previous_command = (command, stale)
                    targets = controller.stop(control_dt) if stale else controller.update_drive(command, control_dt)
                else:
                    command = DriveCommand(args.speed if driving else 0.)
                    targets = controller.update_drive(command, control_dt)
                steering[0] = torch.tensor(targets.steering_positions, device=simulator.device)
                rear[0] = torch.tensor(targets.rear_velocities, device=simulator.device)
                robot.set_joint_position_target(steering, joint_ids=steering_ids)
                robot.set_joint_velocity_target(rear, joint_ids=rear_ids)
                for _ in range(decimation):
                    robot.write_data_to_sim()
                    simulator.step(render=False)
                    robot.update(physics_dt)
                data = robot.data
                if not torch.isfinite(data.root_state_w).all() or not torch.isfinite(data.joint_pos).all():
                    raise RuntimeError('Non-finite Hunter physics state.')
                if float(data.projected_gravity_b[0, 2]) > -.5 or float(data.root_pos_w[0, 2]) < .15:
                    raise RuntimeError('Hunter tipped over or fell through the ground.')
                if simulator.has_gui():
                    simulator.render()
                if bridge is not None:
                    bridge.publish_clock(seconds + control_dt)
                    bridge.publish_robot_state(seconds + control_dt, robot)
                if step % 50 == 0:
                    base_state, rear_state = robot_frames(data, asset['rear_axle_offset_base_m'])
                    print('HUNTER2_STATE ' + json.dumps(dict(time_s=round(seconds + control_dt, 3),
                        phase='drive' if driving else ('settle' if seconds < args.settle_time else 'brake'),
                        input_mode='ros' if args.ros else 'cli', command=asdict(command), stale=stale,
                        ramped_speed_m_s=targets.speed, body_speed_m_s=rear_state.linear_velocity[0],
                        chassis_speed_m_s=base_state.linear_velocity[0],
                        rear_axle_speed_m_s=rear_state.linear_velocity[0],
                        rear_axle_lateral_speed_m_s=rear_state.linear_velocity[1],
                        steering_centre_target_rad=targets.steering_angle,
                        steering_target_rad=list(targets.steering_positions),
                        steering_actual_rad=data.joint_pos[0, steering_ids].tolist(),
                        yaw_target_rad_s=targets.yaw_rate, yaw_actual_rad_s=float(data.root_ang_vel_b[0, 2]),
                        rear_target_rad_s=list(targets.rear_velocities),
                        rear_actual_rad_s=data.joint_vel[0, rear_ids].tolist(),
                        position_m=list(rear_state.position))), flush=True)
                if not args.headless:
                    time.sleep(max(0., control_dt - (time.monotonic() - tick)))
                step += 1
        _, final_rear = robot_frames(robot.data, asset['rear_axle_offset_base_m'])
        print('HUNTER2_FINISHED ' + json.dumps(dict(completed=steps is not None and step == steps,
            simulated_seconds=step * control_dt,
            displacement_m=[p-s for p, s in zip(final_rear.position, initial_rear.position)],
            final_body_speed_m_s=final_rear.linear_velocity[0])), flush=True)
    finally:
        if bridge is not None:
            bridge.close()
        simulator.clear_all_callbacks()
        simulator.clear_instance()
