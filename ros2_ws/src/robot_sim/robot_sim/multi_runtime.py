"""One physics loop, per-robot controllers/ROS/reset, and one simulation clock."""
import time

from .config import load_world_config
from .ros_bridge import RobotRosBridge, SimulationClock


class RobotInstance:
    def __init__(self, root, config, robot, asset, policy, device):
        import torch
        from robot_control.go2.controller import ArticulationGo2Controller
        from robot_control.lekiwi.controller import LekiwiController, load_parameters
        self.config, self.robot, self.asset = config, robot, asset
        self.bridge = None
        self.report = dict(robot_type=config['robot_type'], namespace=config['namespace'], asset=asset,
                           cmd_vel_topic='/' + config['namespace'] + '/cmd_vel',
                           resets=0, reset_history=[], command_history=[], stale_steps=0,
                           minimum_base_height_m=float(robot.data.root_pos_w[0, 2]))
        self.measurements, self.previous, self.segment_steps = {}, None, 0
        if config['robot_type'] == 'go2':
            self.controller = ArticulationGo2Controller(robot, policy)
        else:
            self.controller = LekiwiController(load_parameters(root))
            self.wheel_ids = [robot.joint_names.index(name) for name in self.controller.joint_names]
            self.targets = torch.zeros((1, 3), device=device)

    def apply(self, command):
        import torch
        if self.config['robot_type'] == 'go2':
            observation = self.controller.apply(command)
            if not torch.equal(observation[:, 9:12], torch.tensor([command], device=observation.device)):
                raise RuntimeError('Go2 command observation mismatch.')
        else:
            self.targets[0] = torch.tensor(self.controller.wheel_velocities(command), device=self.targets.device)
            self.robot.set_joint_velocity_target(self.targets, joint_ids=self.wheel_ids)

    def reset(self, step, reason):
        from .scene import reset_articulation
        reset_articulation(self.robot)
        if self.config['robot_type'] == 'go2':
            self.controller.reset()
        else:
            self.targets.zero_()
            self.robot.set_joint_velocity_target(self.targets, joint_ids=self.wheel_ids)
        if self.bridge is not None:
            self.bridge.notify_reset()
            self.bridge.reset_requested = False
        self.previous, self.segment_steps = None, 0
        if reason != 'initial':
            self.report['resets'] += 1
            self.report['reset_history'].append({'step': step, 'reason': reason,
                                                 'position': self.robot.data.root_pos_w[0].tolist()})
            self.report['reset_history'] = self.report['reset_history'][-1000:]

    def measure(self, command, stale, step):
        data = self.robot.data
        if (command, stale) != self.previous:
            self.report['command_history'].append({'step': step, 'command': list(command), 'stale': stale,
                                                  'position': data.root_pos_w[0].tolist()})
            self.report['command_history'] = self.report['command_history'][-1000:]
            self.previous, self.segment_steps = (command, stale), 0
        self.segment_steps += 1
        self.report['stale_steps'] += int(stale)
        self.report['minimum_base_height_m'] = min(self.report['minimum_base_height_m'], float(data.root_pos_w[0, 2]))
        if self.segment_steps > 20:
            key = (command, stale)
            if key not in self.measurements and len(self.measurements) >= 256:
                self.measurements.pop(next(iter(self.measurements)))
            sample = self.measurements.setdefault(key, {'steps': 0, 'sum': [0., 0., 0.]})
            velocity = [float(data.root_lin_vel_b[0, 0]), float(data.root_lin_vel_b[0, 1]),
                        float(data.root_ang_vel_b[0, 2])]
            sample['steps'] += 1
            sample['sum'] = [a + b for a, b in zip(sample['sum'], velocity)]

    def finish(self):
        self.report.update(end_position=self.robot.data.root_pos_w[0].tolist(),
            command_responses=[dict(command=list(cmd), stale=stale, measurement_steps=sample['steps'],
                                   mean_body_velocity=[v / sample['steps'] for v in sample['sum']])
                               for (cmd, stale), sample in self.measurements.items()])
        if self.bridge is not None:
            self.report.update(received_commands=self.bridge.command.received,
                               rejected_commands=self.bridge.command.rejected, state_messages=self.bridge.state_count)
            self.bridge.close()


def run_ros(app, args, root, report):
    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.scene import InteractiveScene
    from robot_control.go2.policy import FlatPolicy
    from scenarios.registry import load_scene
    from .scene import RobotContactMonitor, spawn_robot

    world = load_world_config(args.robots_file)
    torch.manual_seed(world['seed'])
    report.update(runtime='shared_world', scenario=world['scenario'], seed=world['seed'],
                  robots={}, control_steps=0, resets=0)
    report.pop('cmd_vel_topic', None)
    policy = None
    if any(config['robot_type'] == 'go2' for config in world['robots']):
        policy = FlatPolicy(root, args.policy_device, checkpoint=args.checkpoint)
        report['policy'] = dict(path=str(policy.checkpoint_path), sha256=policy.checkpoint_sha256,
                                iteration=policy.iteration)
    print('MULTI_ROS_PREPARING: loading shared world and robot articulations.', flush=True)
    scene_cfg, eye, target = load_scene(world['scenario'])
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(
        dt=.005, render_interval=4, device=args.device, physics_material=scene_cfg.terrain.physics_material))
    # Match ContactSensor's setup; this runtime creates tensor views directly.
    sim.set_setting('/physics/disableContactProcessing', False)
    scene = InteractiveScene(scene_cfg)
    sim.set_camera_view(eye=eye, target=target)
    instances, clock, contacts = [], None, None
    try:
        spawned = [(config, *spawn_robot(root, config)) for config in world['robots']]
        print('MULTI_ROS_PREPARING: initializing physics.', flush=True)
        sim.reset()
        scene.reset()
        for config, robot, asset in spawned:
            instance = RobotInstance(root, config, robot, asset, policy, sim.device)
            instances.append(instance)
            report['robots'][config['robot_id']] = instance.report
            instance.reset(0, 'initial')
            instance.report['start_position'] = robot.data.root_pos_w[0].tolist()
            instance.report['joint_names'] = list(robot.joint_names)
        contacts = RobotContactMonitor(world['robots'])
        print('MULTI_ROS_PREPARING: settling robots and preparing display.', flush=True)
        for warm_step in range(100):
            if warm_step % 4 == 0:
                for instance in instances:
                    instance.apply((0., 0., 0.))
            for instance in instances:
                instance.robot.write_data_to_sim()
            sim.step(render=False)
            for instance in instances:
                instance.robot.update(.005)
            if sim.has_gui() and warm_step % 4 == 3:
                sim.render()
        for _ in range(10):
            sim.render()
        if sim.has_gui():
            from isaacsim.core.simulation_manager import SimulationManager
            until = time.monotonic() + 300.
            while SimulationManager.assets_loading():
                if time.monotonic() >= until:
                    raise RuntimeError('Shared-world display textures did not finish loading.')
                sim.render()
        for instance in instances:
            config = instance.config
            instance.bridge = RobotRosBridge('/' + config['namespace'] + '/cmd_vel', args.command_timeout,
                robot_id=config['robot_id'], namespace=config['namespace'], publish_clock=False, enable_reset=True)
        clock = SimulationClock()
        clock.publish(0.)
        for instance in instances:
            instance.bridge.publish_robot_state(0., instance.robot)
        print('MULTI_ROS_READY: robots=' + ','.join(c['robot_id'] for c in world['robots'])
              + f'; single /clock; physics=200Hz; control=50Hz; ROS_DOMAIN_ID={args.ros_domain_id}', flush=True)
        step, started, physics_start = 0, time.monotonic(), sim.current_time
        with torch.inference_mode():
            while app.is_running() and all(i.bridge.context.ok() for i in instances) and (not args.steps or step < args.steps):
                tick = time.monotonic()
                commands = []
                for instance in instances:
                    command = instance.bridge.poll()
                    data = instance.robot.data
                    if not torch.isfinite(data.root_state_w).all() or not torch.isfinite(data.joint_pos).all():
                        raise RuntimeError(f'Non-finite state: {instance.config["robot_id"]}')
                    min_height = .12 if instance.config['robot_type'] == 'go2' else .015
                    fallen = float(data.projected_gravity_b[0, 2]) > -.5 or float(data.root_pos_w[0, 2]) < min_height
                    if instance.bridge.reset_requested or fallen:
                        instance.reset(step, 'service' if instance.bridge.reset_requested else 'fall')
                        command = (0., 0., 0.)
                    commands.append((command, instance.bridge.command.stale))
                    instance.apply(command)
                contacts.step = step
                for _ in range(4):
                    for instance in instances:
                        instance.robot.write_data_to_sim()
                    sim.step(render=False)
                    for instance in instances:
                        instance.robot.update(.005)
                    contacts.sample()
                if sim.has_gui():
                    sim.render()
                step += 1
                report['control_steps'] = step
                clock.publish(step * .02)
                for instance, (command, stale) in zip(instances, commands):
                    instance.measure(command, stale, step)
                    instance.bridge.publish_robot_state(step * .02, instance.robot)
                if step % 250 == 0:
                    print(f'MULTI_ROS_PROGRESS: step={step}', flush=True)
                time.sleep(max(0., .02 - (time.monotonic() - tick)))
        report.update(passed=not args.steps or step == args.steps, simulated_seconds=step * .02,
                      physics_elapsed_seconds=sim.current_time - physics_start,
                      elapsed_seconds=time.monotonic() - started)
    finally:
        for instance in instances:
            instance.finish()
        report['resets'] = sum(i.report['resets'] for i in instances)
        if contacts is not None:
            report['robot_contacts'] = list(contacts.pairs.values())
            report['contact_sample_steps'] = contacts.sample_steps
            report['contact_body_counts'] = contacts.body_counts
            contacts.close()
        if clock is not None:
            report['clock_messages'] = clock.count
            clock.close()
        sim.clear_all_callbacks()
        sim.clear_instance()
