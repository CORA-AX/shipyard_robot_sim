"""Single-process Go2 policy/physics loop driven by ROS, independent of control_channel."""
import json
import math
import time

from robot_control.go2.policy import FlatPolicy
from robot_control.go2.controller import Go2Controller
from .ros_bridge import RobotRosBridge
from robot_control.keyboard import TerminalKeyboard


def run_ros(app, args, root, report):
    import torch
    from isaaclab.envs import ManagerBasedRLEnv
    from scenarios.registry import load_go2_scenario

    policy = FlatPolicy(root, args.policy_device, checkpoint=args.checkpoint)
    report['policy'] = {'path': str(policy.checkpoint_path), 'sha256': policy.checkpoint_sha256,
                        'iteration': policy.iteration}
    print('GO2_POLICY_LOADED ' + json.dumps(report['policy']), flush=True)
    cfg = load_go2_scenario(args.scenario)
    cfg.sim.device = args.device
    cfg.seed = getattr(args, "seed", 42)
    if getattr(args, "initial_position", None) is not None:
        cfg.scene.robot.init_state.pos = tuple(args.initial_position)
    yaw = getattr(args, "initial_yaw", 0.0)
    cfg.scene.robot.init_state.rot = (math.cos(yaw / 2), 0., 0., math.sin(yaw / 2))
    cfg.commands.base_velocity.debug_vis = False
    cfg.commands.base_velocity.resampling_time_range = (1e9, 1e9)
    for name in ('lin_vel_x', 'lin_vel_y', 'ang_vel_z'):
        setattr(cfg.commands.base_velocity.ranges, name, (0.0, 0.0))
    # Continuous ROS operation must not teleport the robot every 20 seconds.
    cfg.terminations.time_out = None
    env = bridge = keyboard = None
    measurements = {}
    try:
        env = ManagerBasedRLEnv(cfg=cfg)
        env.reset()
        robot = env.scene['robot']
        observation = env.observation_manager.compute()['policy']
        controller = Go2Controller(env, policy)
        # Finish initial display/shader work before accepting short-lived commands.
        # render() leaves physics frozen; it is a no-op in the headless mode.
        print('GO2_ROS_PREPARING: warming display before accepting commands.', flush=True)
        for _ in range(10):
            env.sim.render()
        policy.actions(observation[0].tolist())
        keyboard = TerminalKeyboard(args)
        keyboard.__enter__()
        bridge = RobotRosBridge(args.cmd_vel_topic, args.command_timeout, accept_commands=not args.keyboard,
                                robot_id=getattr(args, "robot_id", "go2"),
                                namespace=getattr(args, "namespace", "unitree_go2"))
        report.update(robot_id=getattr(args, "robot_id", "go2"), namespace=bridge.namespace)
        report.update(control_steps=0, resets=0, stale_steps=0, command_history=[],
                      start_position=robot.data.root_pos_w[0].tolist(),
                      minimum_base_height_m=float(robot.data.root_pos_w[0, 2]),
                      command_observation_matches=True,
                      input_mode='keyboard' if args.keyboard else 'ros')
        previous = None
        segment_steps = 0
        step = 0
        started = time.monotonic()
        bridge.publish_clock(0.0)
        bridge.publish_robot_state(0.0, robot)
        print(f'GO2_ROS_READY: input={report["input_mode"]}; topic={args.cmd_vel_topic}; clock=/clock; rate=50Hz; '
              f'timeout={args.command_timeout}s; ROS_DOMAIN_ID={args.ros_domain_id}', flush=True)
        with torch.inference_mode():
            while app.is_running() and bridge.context.ok() and (not args.steps or step < args.steps):
                tick = time.monotonic()
                if args.keyboard:
                    command = keyboard.poll()
                    if keyboard.quit:
                        report['stopped_by_user'] = True
                        break
                    stale = keyboard.state.stale
                else:
                    command = bridge.poll()
                    stale = bridge.command.stale
                observation = controller.observation(command)
                if (command, stale) != previous:
                    entry = {'step': step, 'command': list(command), 'stale': stale,
                             'position': robot.data.root_pos_w[0].tolist()}
                    report['command_history'].append(entry)
                    # Bound memory for long-running control sessions.
                    report['command_history'] = report['command_history'][-1000:]
                    print('GO2_ROS_COMMAND ' + json.dumps(entry), flush=True)
                    previous = (command, stale)
                    segment_steps = 0
                actions = controller.actions(observation)
                _, _, terminated, truncated, _ = env.step(actions)
                step += 1
                report['control_steps'] = step
                reset_count = int((terminated | truncated).sum().item())
                report['resets'] += reset_count
                if reset_count:
                    bridge.notify_reset()
                report['stale_steps'] += int(stale)
                segment_steps += 1
                if not torch.isfinite(robot.data.root_state_w).all():
                    raise RuntimeError('Non-finite Go2 simulation state.')
                report['minimum_base_height_m'] = min(report['minimum_base_height_m'],
                                                     float(robot.data.root_pos_w[0, 2]))
                # Ignore the first 0.4s of each command when measuring its response.
                if segment_steps > 20:
                    key = (command, stale)
                    if key not in measurements and len(measurements) >= 256:
                        measurements.pop(next(iter(measurements)))
                    sample = measurements.setdefault(key, {'steps': 0, 'sum': [0., 0., 0.]})
                    velocity = [float(robot.data.root_lin_vel_b[0, 0]),
                                float(robot.data.root_lin_vel_b[0, 1]),
                                float(robot.data.root_ang_vel_b[0, 2])]
                    sample['steps'] += 1
                    sample['sum'] = [a + b for a, b in zip(sample['sum'], velocity)]
                bridge.publish_clock(step * env.step_dt)
                bridge.publish_robot_state(step * env.step_dt, robot)
                # Physics and policy continue at zero command when ROS input expires.
                time.sleep(max(0.0, env.step_dt - (time.monotonic() - tick)))
        report.update(passed=not args.steps or step == args.steps,
                      simulated_seconds=step * env.step_dt, elapsed_seconds=time.monotonic() - started)
    finally:
        if keyboard is not None:
            report['keyboard_events'] = keyboard.state.events
            keyboard.__exit__()
        if env is not None:
            report['command_responses'] = [
                {'command': list(command), 'stale': stale, 'measurement_steps': sample['steps'],
                 'mean_body_velocity': [value / sample['steps'] for value in sample['sum']]}
                for (command, stale), sample in measurements.items()]
        if bridge is not None:
            report.update(received_commands=bridge.command.received,
                          rejected_commands=bridge.command.rejected,
                          clamped_commands=bridge.command.clamped, clock_messages=bridge.clock_count,
                          state_messages=bridge.state_count)
            bridge.close()
        if env is not None:
            report['end_position'] = env.scene['robot'].data.root_pos_w[0].tolist()
            env.close()
