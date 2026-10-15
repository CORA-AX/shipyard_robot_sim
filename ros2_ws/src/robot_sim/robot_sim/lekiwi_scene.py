"""Reference Lekiwi visuals and author a mobile articulation with passive omni rollers.

All corrections are stage overrides. The imported USD files are never rewritten.
"""
import math


def spawn_lekiwi(root, prim_path, parameters, position, yaw):
    import isaaclab.sim as sim
    import omni.usd
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import Articulation, ArticulationCfg
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

    source = root / 'assets/Lekiwi/lekiwi_isaacsim/urdf/lekiwi/lekiwi.usd'
    config = sim.UsdFileCfg(usd_path=str(source))
    config.func(prim_path, config)
    stage = omni.usd.get_context().get_stage()
    errors = [str(error) for prim in Usd.PrimRange(stage.GetPrimAtPath(prim_path))
              for error in prim.GetPrimIndex().localErrors]
    if errors:
        raise RuntimeError(f'Lekiwi USD composition errors: {errors}')
    base = stage.GetPrimAtPath(prim_path + '/base_plate_layer1_v5')
    root_joint = stage.GetPrimAtPath(prim_path + '/root_joint')
    root_joint.SetActive(False)  # The imported asset is fixed to the world.
    articulation_api = UsdPhysics.ArticulationRootAPI.Apply(base)
    physx = PhysxSchema.PhysxArticulationAPI.Apply(articulation_api.GetPrim())
    physx.CreateEnabledSelfCollisionsAttr(False)
    physx.CreateSolverPositionIterationCountAttr(16)
    physx.CreateSolverVelocityIterationCountAttr(4)

    wheel_names = [wheel['joint'] for wheel in parameters['wheels']]
    arm_joints = []
    for prim in list(Usd.PrimRange(stage.GetPrimAtPath(prim_path))):
        if prim.IsA(UsdPhysics.RevoluteJoint) and prim.GetName() not in wheel_names:
            # Preserve the imported rest transforms while locking the arm in this base-only phase.
            arm_joints.append(prim.GetName())
            prim.SetTypeName('PhysicsFixedJoint')
            prim.RemoveAPI(UsdPhysics.DriveAPI, 'angular')
    material = UsdShade.Material.Define(stage, prim_path + '/roller_material')
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material.CreateStaticFrictionAttr(.9)
    physics_material.CreateDynamicFrictionAttr(.8)
    physics_material.CreateRestitutionAttr(0.)
    cache = UsdGeom.XformCache()
    radius, roller_radius = parameters['wheel_radius'], parameters['roller_radius']
    roller_count = parameters['rollers_per_wheel']
    axis_x = Gf.Vec3d(1, 0, 0)

    def quatf(quaternion):
        return Gf.Quatf(float(quaternion.GetReal()), Gf.Vec3f(quaternion.GetImaginary()))

    def make_joint(path, body0, body1, point, world_rotation, axis='X'):
        joint = UsdPhysics.RevoluteJoint.Define(stage, path)
        joint.CreateBody0Rel().SetTargets([body0.GetPath()])
        joint.CreateBody1Rel().SetTargets([body1.GetPath()])
        joint.CreateAxisAttr(axis)
        joint.CreateJointEnabledAttr(True)
        joint.CreateCollisionEnabledAttr(False)
        for index, body in enumerate((body0, body1)):
            transform = cache.GetLocalToWorldTransform(body)
            local_point = transform.GetInverse().Transform(point)
            local_rotation = transform.ExtractRotationQuat().GetInverse() * world_rotation
            getattr(joint, f'CreateLocalPos{index}Attr')(Gf.Vec3f(local_point))
            getattr(joint, f'CreateLocalRot{index}Attr')(quatf(local_rotation))
        return joint

    for index, wheel in enumerate(parameters['wheels']):
        body = stage.GetPrimAtPath(prim_path + '/' + wheel['body'])
        if not body.HasAPI(UsdPhysics.RigidBodyAPI):
            raise ValueError(f'Missing wheel rigid body: {body.GetPath()}')
        UsdPhysics.CollisionAPI(stage.GetPrimAtPath(str(body.GetPath()) + '/collisions')).CreateCollisionEnabledAttr(False)
        axle = Gf.Vec3d(*wheel['axle'], 0.).GetNormalized()
        pivot = Gf.Vec3d(*wheel['position'])
        center = pivot + axle * wheel['hub_offset']
        joint_rotation = Gf.Rotation(axis_x, axle).GetQuat()
        make_joint(prim_path + '/joints/' + wheel['joint'], base, body, pivot, joint_rotation)
        drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(prim_path + '/joints/' + wheel['joint']), 'angular')
        drive.CreateTypeAttr('force')
        tangent = Gf.Cross(axle, Gf.Vec3d(0, 0, 1)).GetNormalized()
        for number in range(roller_count):
            angle = 2 * math.pi * number / roller_count
            radial = Gf.Vec3d(0, 0, -1) * math.cos(angle) + tangent * math.sin(angle)
            roller_axis = Gf.Cross(axle, radial).GetNormalized()
            point = center + radial * (radius - roller_radius)
            rotation = Gf.Rotation(axis_x, roller_axis).GetQuat()
            name = f'wheel_{index}_roller_{number:02d}'
            capsule = UsdGeom.Capsule.Define(stage, prim_path + '/rollers/' + name)
            capsule.CreateAxisAttr('X')
            capsule.CreateRadiusAttr(roller_radius)
            capsule.CreateHeightAttr(.012)
            capsule.AddTranslateOp().Set(point)
            capsule.AddOrientOp().Set(quatf(rotation))
            capsule.CreateVisibilityAttr('invisible')
            prim = capsule.GetPrim()
            UsdPhysics.RigidBodyAPI.Apply(prim)
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(parameters['roller_mass'])
            UsdPhysics.CollisionAPI.Apply(prim)
            contact = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            contact.CreateContactOffsetAttr(.001)
            contact.CreateRestOffsetAttr(0.)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose='physics')
            make_joint(prim_path + '/joints/' + name, body, prim, point, rotation)

    cfg = ArticulationCfg(
        prim_path=prim_path, spawn=None,
        init_state=ArticulationCfg.InitialStateCfg(pos=tuple(position),
                    rot=(math.cos(yaw / 2), 0., 0., math.sin(yaw / 2))),
        actuators={
            'wheels': ImplicitActuatorCfg(joint_names_expr=wheel_names, stiffness=0.,
                      damping=parameters['wheel_damping'], effort_limit_sim=parameters['wheel_torque'],
                      velocity_limit_sim=500.),
            'rollers': ImplicitActuatorCfg(joint_names_expr=['wheel_.*_roller_.*'],
                       stiffness=0., damping=0., effort_limit_sim=0., velocity_limit_sim=1000.),
        })
    return Articulation(cfg), {'source_usd': str(source), 'contact_model': 'passive_capsule_rollers',
        'wheel_radius_m': radius, 'passive_rollers': len(parameters['wheels']) * roller_count,
        'locked_arm_joints': arm_joints, 'source_fixed_root_removed': True,
        'composition_errors': errors}
