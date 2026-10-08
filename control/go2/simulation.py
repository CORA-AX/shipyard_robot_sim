"""Viewer-owned Go2 simulation controlled by a separate local policy process."""
import time

from control.control_channel import ControlServer, ProtocolError, vector
from .policy import CONTRACT, JOINT_NAMES


def run_controlled(app, args, report):
    import torch
    from isaaclab.envs import ManagerBasedRLEnv
    from scenarios.registry import load_go2_scenario

    # Bind before creating an environment so a duplicate controller port fails early.
    server = ControlServer(args.control_port, args.robot_id, timeout=args.control_timeout)
    env = None
    try:
        cfg = load_go2_scenario(args.scenario)
        cfg.sim.device = args.device
        if args.steps:
            cfg.episode_length_s = args.steps * cfg.sim.dt * cfg.decimation + 10.0
        env = ManagerBasedRLEnv(cfg=cfg)
        obs, _ = env.reset()
        # Same post-reset observation evaluation as the existing RSL-RL wrapper.
        obs = env.observation_manager.compute()['policy']
        robot = env.scene['robot']
        if (robot.joint_names != JOINT_NAMES or obs.shape != (1, 48)
                or env.action_manager.total_action_dim != 12 or env.step_dt != 0.02):
            raise RuntimeError('Go2 environment does not match the external controller contract.')
        report.update(robot_id=args.robot_id, scene_entities=list(env.scene.keys()),
                      control_port=server.port, control_steps=0, resets=0,
                      start_position=robot.data.root_pos_w[0].tolist(),
                      minimum_base_height_m=float(robot.data.root_pos_w[0, 2]),
                      paused_frames=0, pause_state_unchanged=True, transitions=[])
        def configure(command):
            vx = vector([command.get('vx')], 1)[0]
            if not 0.2 <= vx <= 0.8:
                raise ProtocolError('Go2 forward speed must be between 0.2 and 0.8 m/s.')
            term = env.command_manager.get_term('base_velocity')
            # Keep the target across command resampling and episode resets.
            term.cfg.ranges.lin_vel_x = (vx, vx)
            term.vel_command_b[:, 0] = vx
            term.vel_command_b[:, 1:] = 0.0
            report['commanded_forward_m_s'] = vx
            report.setdefault('command_history', []).append({'step': step, 'vx': vx})

        server.on_configure = configure
        step = episode = 0
        was_paused = None
        paused_state = None
        started = time.monotonic()
        last_connection_count = 0
        next_step_time = last_render_time = time.monotonic()
        print(f'ROBOT_VIEW_READY: {args.robot_id}; control=127.0.0.1:{server.port}; physics paused.', flush=True)
        with torch.inference_mode():
            while app.is_running() and (not args.steps or step < args.steps):
                tick = time.monotonic()
                state = {'position': robot.data.root_pos_w[0].tolist(),
                         'forward_m_s': float(robot.data.root_lin_vel_b[0, 0]),
                         'resets': report['resets']}
                message = {'step': step, 'episode': episode, 'contract': CONTRACT,
                           'action_size': 12, 'joint_names': JOINT_NAMES, 'step_dt': env.step_dt,
                           'observation': obs[0].tolist(), 'state': state}
                configured = server.configured
                action = server.poll(message) if tick >= next_step_time else None
                if server.configured and not configured:
                    obs = env.observation_manager.compute()['policy']
                    report['command_history'][-1]['policy_command'] = obs[0, 9:12].tolist()

                paused = action is None
                if server.connections != last_connection_count:
                    print(f'ROBOT_CONTROL_ATTACHED: {args.robot_id}; connection={server.connections}', flush=True)
                    last_connection_count = server.connections
                if paused:
                    if was_paused is False or was_paused is None:
                        paused_state = robot.data.root_state_w.clone()
                    # Refresh the screen while leaving physics and observations frozen.
                    if tick - last_render_time >= env.step_dt:
                        env.sim.render()
                        last_render_time = time.monotonic()
                        report['paused_frames'] += 1
                    report['pause_state_unchanged'] &= bool(torch.equal(paused_state, robot.data.root_state_w))
                    if not report['pause_state_unchanged']:
                        raise RuntimeError('Robot state changed while waiting for an action.')
                else:
                    if paused_state is not None:
                        if not torch.equal(paused_state, robot.data.root_state_w):
                            raise RuntimeError('Paused state changed before resuming.')
                        paused_state = None
                    obs_dict, _, terminated, truncated, _ = env.step(
                        torch.tensor([action], dtype=torch.float32, device=args.device))
                    obs = obs_dict['policy']
                    if not torch.isfinite(robot.data.root_state_w).all() or not torch.isfinite(obs).all():
                        raise RuntimeError('Non-finite Go2 state or observation.')
                    reset = int((terminated | truncated).sum().item())
                    episode += reset
                    report['resets'] += reset
                    step += 1
                    next_step_time = tick + env.step_dt
                    report['control_steps'] = step
                    report['minimum_base_height_m'] = min(report['minimum_base_height_m'], float(robot.data.root_pos_w[0, 2]))
                # Record connection-level state, not normal per-frame network waiting.
                connection_paused = server.peer is None
                if not report['transitions'] or report['transitions'][-1]['paused'] != connection_paused:
                    report['transitions'].append({'step': step, 'paused': connection_paused,
                                                   'reason': server.last_reason,
                                                   'elapsed_s': time.monotonic() - started})
                    report['transitions'] = report['transitions'][-20:]
                    print(f'ROBOT_CONTROL_STATE: paused={connection_paused}, step={step}, reason={server.last_reason}', flush=True)
                was_paused = paused
                # Keep UI responsive and poll replies without adding a full control-step delay.
                time.sleep(0.001)
        report['end_position'] = robot.data.root_pos_w[0].tolist()
        report['forward_displacement_m'] = report['end_position'][0] - report['start_position'][0]
        report['simulated_seconds'] = step * env.step_dt
        report['passed'] = (not args.steps or step == args.steps) and report['pause_state_unchanged']
        server.finish({'step': step, 'episode': episode, 'state': {
            'position': report['end_position'], 'resets': report['resets']}})
    finally:
        report.update(control_connections=server.connections, control_disconnects=server.disconnects,
                      rejected_messages=server.rejections)
        server.close()
        if env is not None:
            env.close()
