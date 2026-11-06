"""Load the user's saved Hunter USD and preserve its tuned actuator settings.

Only the default robot prim is referenced; the saved preview ground/render
prims are not spawned. All edits are stage overrides, never source-file saves.
"""
import math
from pathlib import Path


def prepare_hunter2_stage(stage, prim_path, parameters, position, yaw):
    """Validate and place an already referenced asset; also usable without Kit."""
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    from hunter.control.controller import required_rear_velocity_limit

    if len(position) != 3 or not all(math.isfinite(v) for v in (*position, yaw)):
        raise ValueError('Hunter spawn position/yaw must be finite.')
    root = stage.GetPrimAtPath(prim_path)
    if not root or not root.IsA(UsdGeom.Xform):
        raise ValueError(f'Missing Hunter Xform: {prim_path}')
    errors = [str(error) for p in Usd.PrimRange(root) for error in p.GetPrimIndex().localErrors]
    if errors:
        raise ValueError(f'Hunter USD composition errors: {errors}')
    base = stage.GetPrimAtPath(prim_path + '/' + parameters['base_link'])
    if not base or not base.HasAPI(UsdPhysics.RigidBodyAPI):
        raise ValueError('Hunter requires its configured base rigid body.')
    articulation_roots = [p for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if len(articulation_roots) != 1 or articulation_roots[0] != base:
        raise ValueError('Use the saved floating-base Hunter USD with articulation root on base_link.')
    for p in Usd.PrimRange(root):
        if p.IsA(UsdPhysics.FixedJoint):
            joint = UsdPhysics.Joint(p)
            if not joint.GetBody0Rel().GetTargets() or not joint.GetBody1Rel().GetTargets():
                raise ValueError(f'Hunter is fixed to the world: {p.GetPath()}')

    # Remove the author's preview-scene offset (currently z=-0.43027 m) and
    # place the robot above our own ground. Articulation init_state matches this.
    transform = Gf.Matrix4d(1.)
    transform.SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(yaw)))
    transform.SetTranslateOnly(Gf.Vec3d(*position))
    UsdGeom.Xformable(root).MakeMatrixXform().Set(transform)
    cache = UsdGeom.XformCache()
    base_inverse = cache.GetLocalToWorldTransform(base).GetInverse()
    details = {}
    expected_names = [entry['joint'] for group in ('steering', 'rear_wheels') for entry in parameters[group]]
    expected_names += parameters['passive_front_wheels']
    moving = {p.GetName(): p for p in Usd.PrimRange(root) if p.IsA(UsdPhysics.RevoluteJoint)}
    if len(moving) != len(expected_names) or set(moving) != set(expected_names):
        raise ValueError(f'Hunter must have the configured six moving joints; found {list(moving)}')
    for group in ('steering', 'rear_wheels'):
        for entry in parameters[group]:
            name, sign = entry['joint'], entry['sign']
            prim = moving[name]
            joint = UsdPhysics.RevoluteJoint(prim)
            parents, children = joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets()
            if parents != [base.GetPath()] or len(children) != 1 or not stage.GetPrimAtPath(children[0]):
                raise ValueError(f'{name}: expected base_link as parent and one valid child.')
            token = joint.GetAxisAttr().Get()
            if token not in ('X', 'Y', 'Z'):
                raise ValueError(f'{name}: invalid joint axis {token}')
            axis = Gf.Vec3d(*{'X': (1, 0, 0), 'Y': (0, 1, 0), 'Z': (0, 0, 1)}[token])
            frame = Gf.Matrix4d(1.)
            frame.SetRotate(Gf.Quatd(joint.GetLocalRot0Attr().Get()))
            frame.SetTranslateOnly(Gf.Vec3d(joint.GetLocalPos0Attr().Get()))
            parent_transform = cache.GetLocalToWorldTransform(base)
            joint_world = frame * parent_transform
            axis_base = (joint_world * base_inverse).TransformDir(axis).GetNormalized()
            expected = Gf.Vec3d(0, 0, sign) if group == 'steering' else Gf.Vec3d(0, sign, 0)
            if Gf.Dot(axis_base, expected) < .999:
                raise ValueError(f'{name}: configured sign disagrees with USD axis {axis_base}')
            child = stage.GetPrimAtPath(children[0])
            child_world = cache.GetLocalToWorldTransform(child)
            anchor0 = parent_transform.Transform(Gf.Vec3d(joint.GetLocalPos0Attr().Get()))
            anchor1 = child_world.Transform(Gf.Vec3d(joint.GetLocalPos1Attr().Get()))
            if (anchor0 - anchor1).GetLength() > 1e-4:
                raise ValueError(f'{name}: joint anchor positions do not coincide.')
            drive = UsdPhysics.DriveAPI(prim, 'angular')
            if not drive or drive.GetTypeAttr().Get() != 'force':
                raise ValueError(f'{name}: expected a force-type angular drive.')
            stiffness, damping = drive.GetStiffnessAttr().Get(), drive.GetDampingAttr().Get()
            effort = drive.GetMaxForceAttr().Get()
            velocity_attribute = prim.GetAttribute('physxJoint:maxJointVelocity')
            saved_velocity = velocity = velocity_attribute.Get()
            if (not all(math.isfinite(v) for v in (stiffness, damping, effort, velocity))
                    or min(damping, effort, velocity) <= 0 or stiffness < 0):
                raise ValueError(f'{name}: invalid saved drive gains or limits.')
            if group == 'rear_wheels':
                if stiffness != 0:
                    raise ValueError(f'{name}: rear velocity drive stiffness must be zero.')
                if parameters.get('rear_velocity_limit') is not None:
                    velocity = math.degrees(parameters['rear_velocity_limit'])
                if required_rear_velocity_limit(parameters) > math.radians(velocity):
                    raise ValueError(f'{name}: max_speed at full steering exceeds the joint velocity limit; '
                                     'lower max_speed or configure rear_velocity_limit in rad/s.')
                if parameters.get('rear_velocity_limit') is not None:
                    velocity_attribute.Set(velocity)
            elif stiffness <= 0 or not joint.GetLowerLimitAttr().Get() < 0 < joint.GetUpperLimitAttr().Get():
                raise ValueError(f'{name}: steering must hold zero within its joint limits.')
            if group == 'steering':
                if parameters['max_steering_rate'] > math.radians(velocity) + 1e-6:
                    raise ValueError(f'{name}: max_steering_rate exceeds the saved USD joint speed limit.')
                side = 1. if entry == parameters['steering'][0] else -1.
                for centre in (-parameters['max_steering_angle'], parameters['max_steering_angle']):
                    curvature = math.tan(centre) / parameters['wheelbase']
                    angle = sign * math.atan2(parameters['wheelbase'] * curvature,
                                             1. - side * parameters['front_track'] * curvature / 2.)
                    if not math.radians(joint.GetLowerLimitAttr().Get()) <= angle <= math.radians(joint.GetUpperLimitAttr().Get()):
                        raise ValueError(f'{name}: configured steering range exceeds saved USD joint limits.')
            drive.CreateTargetPositionAttr(0.)
            drive.CreateTargetVelocityAttr(0.)
            details[name] = dict(stiffness_usd=stiffness, damping_usd=damping,
                                 effort_limit_nm=effort, velocity_limit_deg_s=velocity,
                                 saved_velocity_limit_deg_s=saved_velocity,
                                 axis_base=list(axis_base), anchor_base=list(joint.GetLocalPos0Attr().Get()))
    front = [details[entry['joint']]['anchor_base'] for entry in parameters['steering']]
    rear = [details[entry['joint']]['anchor_base'] for entry in parameters['rear_wheels']]
    geometry = dict(wheelbase=sum(p[0] for p in front) / 2. - sum(p[0] for p in rear) / 2.,
                    front_track=front[0][1] - front[1][1], rear_track=rear[0][1] - rear[1][1])
    for name, measured in geometry.items():
        if abs(parameters[name] - measured) > .001:
            raise ValueError(f'Hunter {name} does not match saved USD joint anchors: {measured:.6f} m.')
    for anchors in (front, rear):
        if abs(anchors[0][0] - anchors[1][0]) > .001 or abs(sum(p[1] for p in anchors) / 2.) > .001:
            raise ValueError('Hunter requires centred, parallel front/rear axles.')
    for name in parameters['passive_front_wheels']:
        drive = UsdPhysics.DriveAPI.Apply(moving[name], 'angular')
        drive.CreateStiffnessAttr(0.)
        drive.CreateDampingAttr(0.)
        drive.CreateMaxForceAttr(0.)
        drive.CreateTargetVelocityAttr(0.)
    return dict(saved_drives=details, geometry_m=geometry,
                minimum_turning_radius_m=parameters['wheelbase'] / math.tan(parameters['max_steering_angle']),
                source_root_pose_replaced=True,
                passive_front_wheels=list(parameters['passive_front_wheels']))


def spawn_hunter2(root, prim_path, parameters, position=(0., 0., .37), yaw=0.):
    import isaaclab.sim as sim
    import omni.usd
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import Articulation, ArticulationCfg

    source = Path(parameters['usd_path'])
    source = source if source.is_absolute() else Path(root) / source
    if not source.is_file():
        raise ValueError(f'Hunter USD not found: {source}')
    cfg = sim.UsdFileCfg(usd_path=str(source))
    cfg.func(prim_path, cfg)
    details = prepare_hunter2_stage(omni.usd.get_context().get_stage(), prim_path, parameters, position, yaw)
    # None reads the composed stage through Isaac Lab's unit conversion,
    # including any explicit rear_velocity_limit override authored above.
    actuators = {
        group: ImplicitActuatorCfg(joint_names_expr=[entry['joint'] for entry in parameters[group]],
                                  stiffness=None, damping=None)
        for group in ('steering', 'rear_wheels')
    }
    actuators['passive_front_wheels'] = ImplicitActuatorCfg(
        joint_names_expr=parameters['passive_front_wheels'], stiffness=0., damping=0., effort_limit_sim=0.)
    articulation = Articulation(ArticulationCfg(
        prim_path=prim_path, spawn=None, actuators=actuators,
        init_state=ArticulationCfg.InitialStateCfg(
            pos=tuple(position), rot=(math.cos(yaw / 2), 0., 0., math.sin(yaw / 2)),
            joint_pos={'.*': 0.}, joint_vel={'.*': 0.})))
    return articulation, dict(source_usd=str(source.resolve()), **details)
