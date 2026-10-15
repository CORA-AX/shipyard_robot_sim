"""One world and physics context with separately named robot articulations."""
import math


def spawn_robot(root, config):
    prim_path = '/World/Robots/' + config['robot_id']
    if config['robot_type'] == 'lekiwi':
        from robot_control.lekiwi.controller import load_parameters
        from .lekiwi_scene import spawn_lekiwi
        robot, asset = spawn_lekiwi(root, prim_path, load_parameters(root), config['position'], config['yaw'])
        # Place the complete authored assembly before physics parses it, including its passive rollers.
        # This avoids overlapping initial USD poses when multiple Lekiwi instances are configured.
        import omni.usd
        from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        transform = Gf.Matrix4d(1.)
        transform.SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(config['yaw'])))
        transform.SetTranslateOnly(Gf.Vec3d(*config['position']))
        UsdGeom.Xformable(prim).MakeMatrixXform().Set(transform)
        for child in Usd.PrimRange(prim):
            if child.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(child).CreateThresholdAttr(0.)
        return robot, asset
    from assets.go2.config.robot_cfg import GO2_CFG
    from isaaclab.assets import Articulation
    cfg = GO2_CFG.copy()
    cfg.prim_path = prim_path
    cfg.init_state.pos = tuple(config['position'])
    cfg.init_state.rot = (math.cos(config['yaw'] / 2), 0., 0., math.sin(config['yaw'] / 2))
    return Articulation(cfg), {'model': 'go2', 'action_scale': .25}


def reset_articulation(robot):
    """Only this articulation is reset; simulator time and other actors are untouched."""
    state = robot.data.default_root_state.clone()
    robot.write_root_pose_to_sim(state[:, :7])
    robot.write_root_velocity_to_sim(state[:, 7:])
    robot.write_joint_state_to_sim(robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone())
    robot.reset()
    robot.update(0.)


class RobotContactMonitor:
    """Read GPU pair forces with explicit sensor/filter paths, excluding the ground.

    Isaac Lab disables CPU omni.physx contact processing. The tensor API exposes
    the actual GPU solver impulses without depending on render-loop callbacks.
    """
    def __init__(self, robots):
        import omni.usd
        from isaacsim.core.simulation_manager import SimulationManager
        from pxr import Usd, UsdPhysics
        stage = omni.usd.get_context().get_stage()
        view = SimulationManager.get_physics_sim_view()
        bodies = {r['robot_id']: [str(p.GetPath()) for p in Usd.PrimRange(
            stage.GetPrimAtPath('/World/Robots/' + r['robot_id'])) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
            for r in robots}
        self.step, self.pairs, self.sample_steps = 0, {}, 0
        self.body_counts = {identity: len(paths) for identity, paths in bodies.items()}
        self.views = []
        ids = list(bodies)
        for index, first in enumerate(ids):
            for second in ids[index + 1:]:
                sensors, filters = bodies[first], bodies[second]
                contact_view = view.create_rigid_contact_view(sensors, [filters] * len(sensors))
                if contact_view.sensor_count != len(sensors) or contact_view.filter_count != len(filters):
                    raise RuntimeError(f'Contact view did not resolve {first}/{second} body paths.')
                self.views.append((first, second, contact_view))

    def sample(self):
        import torch
        self.sample_steps += 1
        for first, second, view in self.views:
            forces = view.get_contact_force_matrix(dt=.005)
            if not torch.isfinite(forces).all():
                raise RuntimeError('Non-finite robot contact forces.')
            impulse = float(forces.norm(dim=-1).sum()) * .005
            if impulse <= 1e-8:
                continue
            key = '|'.join(sorted((first, second)))
            entry = self.pairs.setdefault(key, {'robots': sorted((first, second)), 'events': 0,
                'impulse_sum': 0., 'first_step': self.step, 'last_step': self.step})
            entry.update(events=entry['events'] + 1, impulse_sum=entry['impulse_sum'] + impulse,
                         last_step=self.step)

    def close(self):
        self.views.clear()
