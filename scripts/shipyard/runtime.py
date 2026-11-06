"""Additive shipyard orchestration using existing robot/controllers/ROS bridge."""
import time
from common import ROOT


def run(app,args,world,report):
    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.scene import InteractiveScene
    from robot_control.go2.policy import FlatPolicy
    from robot_sim.multi_runtime import RobotInstance
    from robot_sim.ros_bridge import RobotRosBridge, SimulationClock
    from robot_sim.scene import spawn_robot, RobotContactMonitor
    from scenarios.registry import load_scene

    torch.manual_seed(world['seed'])
    policy = FlatPolicy(ROOT,args.policy_device) if any(r['robot_type']=='go2' for r in world['robots']) else None
    cfg,_,_ = load_scene('shipyard')
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=.005,render_interval=4,
        device=args.device,physics_material=cfg.terrain.physics_material))
    sim.set_setting('/physics/disableContactProcessing',False)
    scene = InteractiveScene(cfg)
    sim.set_camera_view(eye=(-67,49,43),target=(-20,0,0))
    instances,clock,contacts = [],None,None
    report.update(robots={},runtime='shipyard_shared_world',control_steps=0)
    try:
        spawned = [(r,*spawn_robot(ROOT,r)) for r in world['robots']]
        sim.reset()
        scene.reset()
        for config,robot,asset in spawned:
            instance = RobotInstance(ROOT,config,robot,asset,policy,sim.device)
            instances.append(instance)
            report['robots'][config['robot_id']] = instance.report
            instance.reset(0,'initial')
            instance.report['start_position'] = robot.data.root_pos_w[0].tolist()
        contacts = RobotContactMonitor(world['robots'])
        for i in range(100):
            if i%4==0:
                for instance in instances:
                    instance.apply((0.,0.,0.))
            for instance in instances:
                instance.robot.write_data_to_sim()
            sim.step(render=False)
            for instance in instances:
                instance.robot.update(.005)
        for _ in range(10):
            sim.render()
        if sim.has_gui():
            from isaacsim.core.simulation_manager import SimulationManager
            deadline = time.monotonic()+300
            while SimulationManager.assets_loading():
                if time.monotonic()>deadline:
                    raise RuntimeError('Display assets did not finish loading.')
                sim.render()
        for instance in instances:
            r = instance.config
            instance.bridge = RobotRosBridge('/'+r['namespace']+'/cmd_vel',args.command_timeout,
                robot_id=r['robot_id'],namespace=r['namespace'],publish_clock=False,enable_reset=True)
        clock = SimulationClock()
        clock.publish(0.)
        for instance in instances:
            instance.bridge.publish_robot_state(0.,instance.robot)
        print('SHIPYARD_READY: one /clock; Go2 and Lekiwi share the saved shipyard world.',flush=True)
        step,started = 0,time.monotonic()
        with torch.inference_mode():
            while app.is_running() and all(i.bridge.context.ok() for i in instances) and (not args.steps or step<args.steps):
                if args.stop_requested:
                    report['stopped_by_user'] = True
                    break
                tick = time.monotonic()
                commands = []
                for instance in instances:
                    command = instance.bridge.poll()
                    data = instance.robot.data
                    if not torch.isfinite(data.root_state_w).all() or not torch.isfinite(data.joint_pos).all():
                        raise RuntimeError('Non-finite robot state: '+instance.config['robot_id'])
                    minimum = .12 if instance.config['robot_type']=='go2' else .015
                    fallen = float(data.projected_gravity_b[0,2])>-.5 or float(data.root_pos_w[0,2])<minimum
                    if instance.bridge.reset_requested or fallen:
                        instance.reset(step,'service' if instance.bridge.reset_requested else 'fall')
                        command = (0.,0.,0.)
                    commands.append((command,instance.bridge.command.stale))
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
                clock.publish(step*.02)
                for instance,(command,stale) in zip(instances,commands):
                    instance.measure(command,stale,step)
                    instance.bridge.publish_robot_state(step*.02,instance.robot)
                time.sleep(max(0.,.02-(time.monotonic()-tick)))
        report.update(passed=not args.steps or step==args.steps,simulated_seconds=step*.02,
                      elapsed_seconds=time.monotonic()-started)
    finally:
        for instance in instances:
            instance.finish()
        report['resets'] = sum(i.report['resets'] for i in instances)
        if contacts is not None:
            report['robot_contacts'] = list(contacts.pairs.values())
            contacts.close()
        if clock is not None:
            report['clock_messages'] = clock.count
            clock.close()
        sim.clear_all_callbacks()
        sim.clear_instance()
