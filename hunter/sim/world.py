"""Isaac-only world spawning; no ROS goals, map generation or robot ownership."""


def spawn_world(profile):
    import isaaclab.sim as sim
    if profile.usd_path is None:
        sim.GroundPlaneCfg().func('/World/ground', sim.GroundPlaneCfg())
        sim.DomeLightCfg(intensity=1500.).func('/World/light', sim.DomeLightCfg(intensity=1500.))
    else:
        cfg = sim.UsdFileCfg(usd_path=str(profile.usd_path))
        cfg.func('/World/ShipyardEnvironment', cfg)
        import omni.usd
        from pxr import Usd
        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath('/World/ShipyardEnvironment')
        errors = [str(error) for prim in Usd.PrimRange(root) for error in prim.GetPrimIndex().localErrors]
        if errors:
            raise RuntimeError(f'Shipyard USD composition errors: {errors[:5]}')
        if not stage.GetPrimAtPath('/World/ShipyardEnvironment/RoadNetwork'):
            raise RuntimeError('Shipyard service roads did not compose.')
