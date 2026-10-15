"""Lekiwi physical wheel control and ground-truth ROS output, one shared physics owner."""
import json
import time

from robot_control.lekiwi.controller import LekiwiController, load_parameters
from .ros_bridge import RobotRosBridge


def run_ros(app, args, root, report):
    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.scene import InteractiveScene
    from scenarios.registry import load_scene
    from .lekiwi_scene import spawn_lekiwi

    parameters = load_parameters(root)
    torch.manual_seed(args.seed)
    controller = LekiwiController(parameters)
    print(f'LEKIWI_ROS_PREPARING: loading {args.scenario} scene.', flush=True)
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(
        dt=.005, render_interval=4, device=args.device))
    scene_cfg, _, _ = load_scene(args.scenario)
    scene = InteractiveScene(scene_cfg)
    spawn = args.initial_position or [0., 0., .04]
    sim.set_camera_view(eye=[spawn[0] + 1.8, spawn[1] - 2.2, spawn[2] + 1.4],
                        target=[spawn[0], spawn[1], spawn[2] + .12])
    bridge = robot = None
    measurements = {}
    report.update(robot_type='lekiwi', robot_id=args.robot_id, namespace=args.namespace,
                  control_steps=0, resets=0, stale_steps=0, command_history=[])
    try:
        print('LEKIWI_ROS_PREPARING: loading mobile base and passive rollers.', flush=True)
        robot, asset = spawn_lekiwi(root, '/World/Robots/' + args.robot_id, parameters,
                                   args.initial_position or [0., 0., .04], args.initial_yaw)
        report['asset'] = asset
        print('LEKIWI_ROS_PREPARING: initializing physics and display.', flush=True)
        sim.reset()
        scene.reset()
        state = robot.data.default_root_state.clone()
        robot.write_root_pose_to_sim(state[:, :7])
        robot.write_root_velocity_to_sim(state[:, 7:])
        robot.write_joint_state_to_sim(robot.data.default_joint_pos, robot.data.default_joint_vel)
        robot.reset()
        robot.update(0.)
        wheel_ids = [robot.joint_names.index(name) for name in controller.joint_names]
        targets = torch.zeros((1, len(wheel_ids)), device=sim.device)
        report.update(joint_names=list(robot.joint_names), wheel_joints=controller.joint_names,
                      start_position=robot.data.root_pos_w[0].tolist(),
                      minimum_base_height_m=float(robot.data.root_pos_w[0, 2]))
        print('LEKIWI_ROS_PREPARING: warming display before accepting commands.', flush=True)
        # Warm physical contact and shaders before DDS discovery enables external command input.
        for warm_step in range(100):
            robot.set_joint_velocity_target(targets, joint_ids=wheel_ids)
            robot.write_data_to_sim()
            sim.step(render=False)
            robot.update(.005)
            if sim.has_gui() and warm_step % 4 == 3:
                sim.render()
        for _ in range(10):
            sim.render()
        if sim.has_gui():
            from isaacsim.core.simulation_manager import SimulationManager
            until = time.monotonic() + 300.
            while SimulationManager.assets_loading():
                if time.monotonic() >= until:
                    raise RuntimeError('Lekiwi display textures did not finish loading in 300 seconds.')
                sim.render()
        bridge = RobotRosBridge(args.cmd_vel_topic, args.command_timeout,
                                robot_id=args.robot_id, namespace=args.namespace)
        bridge.publish_clock(0.)
        bridge.publish_robot_state(0., robot)
        print(f'LEKIWI_ROS_READY: topic={args.cmd_vel_topic}; clock=/clock; rate=50Hz; '
              f'timeout={args.command_timeout}s; ROS_DOMAIN_ID={args.ros_domain_id}', flush=True)
        step, previous, segment_steps = 0, None, 0
        started = time.monotonic()
        physics_start = sim.current_time
        with torch.inference_mode():
            while app.is_running() and bridge.context.ok() and (not args.steps or step < args.steps):
                tick = time.monotonic()
                command = bridge.poll()
                stale = bridge.command.stale
                if (command, stale) != previous:
                    report['command_history'].append({'step': step, 'command': list(command),
                        'stale': stale, 'position': robot.data.root_pos_w[0].tolist()})
                    report['command_history'] = report['command_history'][-1000:]
                    previous, segment_steps = (command, stale), 0
                targets[0] = torch.tensor(controller.wheel_velocities(command), device=sim.device)
                robot.set_joint_velocity_target(targets, joint_ids=wheel_ids)
                for _ in range(4):
                    robot.write_data_to_sim()
                    sim.step(render=False)
                    robot.update(.005)
                if sim.has_gui():
                    sim.render()
                step += 1
                segment_steps += 1
                report['control_steps'] = step
                if step % 250 == 0:
                    print(f'LEKIWI_ROS_PROGRESS: step={step}; position={robot.data.root_pos_w[0].tolist()}', flush=True)
                report['stale_steps'] += int(stale)
                if not torch.isfinite(robot.data.root_state_w).all() or not torch.isfinite(robot.data.joint_pos).all():
                    raise RuntimeError('Non-finite Lekiwi physics state.')
                report['minimum_base_height_m'] = min(report['minimum_base_height_m'], float(robot.data.root_pos_w[0, 2]))
                if robot.data.projected_gravity_b[0, 2] > -.7:
                    raise RuntimeError('Lekiwi tipped over.')
                if segment_steps > 20:
                    key = (command, stale)
                    if key not in measurements and len(measurements) >= 256:
                        measurements.pop(next(iter(measurements)))
                    sample = measurements.setdefault(key, {'steps': 0, 'sum': [0., 0., 0.], 'joint_sum': [0.] * 3})
                    velocity = [float(robot.data.root_lin_vel_b[0, 0]), float(robot.data.root_lin_vel_b[0, 1]),
                                float(robot.data.root_ang_vel_b[0, 2])]
                    sample['steps'] += 1
                    sample['sum'] = [a + b for a, b in zip(sample['sum'], velocity)]
                    sample['joint_sum'] = [a + b for a, b in zip(sample['joint_sum'], robot.data.joint_vel[0, wheel_ids].tolist())]
                bridge.publish_clock(step * .02)
                bridge.publish_robot_state(step * .02, robot)
                time.sleep(max(0., .02 - (time.monotonic() - tick)))
        report.update(passed=not args.steps or step == args.steps,
                      simulated_seconds=step * .02, physics_elapsed_seconds=sim.current_time - physics_start,
                      elapsed_seconds=time.monotonic() - started, seed=args.seed)
    finally:
        report['command_responses'] = [dict(command=list(command), stale=stale,
            measurement_steps=sample['steps'],
            mean_body_velocity=[v / sample['steps'] for v in sample['sum']],
            mean_wheel_velocity=[v / sample['steps'] for v in sample['joint_sum']])
            for (command, stale), sample in measurements.items()]
        if bridge is not None:
            report.update(received_commands=bridge.command.received, rejected_commands=bridge.command.rejected,
                          clock_messages=bridge.clock_count, state_messages=bridge.state_count)
            bridge.close()
        if robot is not None and robot.is_initialized:
            report['end_position'] = robot.data.root_pos_w[0].tolist()
        # Unsubscribe the Isaac Lab pause/stop handler before app.close().
        # Stopping the timeline here would enter that handler's wait-for-play loop.
        sim.clear_all_callbacks()
        sim.clear_instance()
