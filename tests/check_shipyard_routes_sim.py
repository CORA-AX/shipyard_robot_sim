#!/usr/bin/env python3
"""PhysX drop tests and RTX preview for the shipyard route addition."""
import json
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'outputs/shipyard'
OUT.mkdir(parents=True,exist_ok=True)
REPORT = OUT/'routes_simulation_report.json'
REPORT.write_text(json.dumps(dict(passed=False,status='starting'))+'\n')
sys.path[:0] = [str(ROOT),str(ROOT/'scripts')]
from visualization.runtime import prepare_environment, kit_arguments
prepare_environment(ROOT,'shipyard_routes_check')
from isaaclab.app import AppLauncher
app = AppLauncher(headless=True,enable_cameras=True,device='cuda:0',
                  kit_args=kit_arguments(ROOT,'shipyard_routes_check')).app
try:
    print('SHIPYARD_ROUTES: renderer initialized',flush=True)
    import numpy as np
    import omni.usd
    import omni.replicator.core as rep
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import DynamicCuboid
    from PIL import Image
    from pxr import Gf, UsdGeom
    from scenarios.registry import load_go2_scenario
    from scenarios.shipyard.scene_cfg import WORLD_USD, ROBOT_START
    assert tuple(load_go2_scenario('shipyard').scene.robot.init_state.pos)==ROBOT_START
    world = World(stage_units_in_meters=1,physics_dt=1/60,rendering_dt=1/60)
    stage = omni.usd.get_context().get_stage()
    UsdGeom.Xform.Define(stage,'/World/Environment').GetPrim().GetReferences().AddReference(str(WORLD_USD))
    print('SHIPYARD_ROUTES: saved world composed',flush=True)
    cases = dict(start=(-16,18,0),connection=(-18,18,0),bypass=(-25,0,0),
                 north_ramp=(-37,8,.6),south_ramp=(-37,-10,.6),deck=(-37,-1,1.2),
                 stair_low=(-31,-1,.15),stair_high=(-33.8,-1,1.2))
    cubes = {}
    for name,(x,y,z) in cases.items():
        cubes[name] = world.scene.add(DynamicCuboid(prim_path='/World/Probes/'+name,name=name,
            position=np.array([x,y,z+.4]),size=.1,mass=1))
    world.reset()
    print('SHIPYARD_ROUTES: physics initialized',flush=True)
    for _ in range(180):
        world.step(render=False)
    results = {}
    for name,cube in cubes.items():
        pos,_ = cube.get_world_pose()
        x,y,z = cases[name]
        if name=='north_ramp':
            z = (14-pos[1])*.1
        elif name=='south_ramp':
            z = (pos[1]+16)*.1
        assert np.isfinite(pos).all() and abs(pos[2]-z-.05)<.03,(name,pos,z)
        assert np.linalg.norm(pos[:2]-np.array([x,y]))<.25,(name,pos)
        results[name] = pos.tolist()
        UsdGeom.Imageable(stage.GetPrimAtPath('/World/Probes/'+name)).MakeInvisible()
    print('SHIPYARD_ROUTES: eight drop tests passed; rendering previews',flush=True)
    out = OUT
    cameras = [('/World/Environment/ServiceRoadOverview','routes_preview.png')]
    detail = UsdGeom.Camera.Define(stage,'/World/RouteDetail')
    detail.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(
        Gf.Vec3d(-16,14,12),Gf.Vec3d(-35,-1,.5),Gf.Vec3d(0,0,1)).GetInverse())
    detail.CreateFocalLengthAttr(24)
    cameras.append(('/World/RouteDetail','routes_detail.png'))
    for camera,filename in cameras:
        product = rep.create.render_product(camera,(1440,900))
        rgb = rep.AnnotatorRegistry.get_annotator('rgb')
        rgb.attach([product])
        for _ in range(30):
            world.step(render=True)
        pixels = rgb.get_data()
        assert pixels.shape[:2]==(900,1440)
        Image.fromarray(pixels).save(out/filename)
        print(f'SHIPYARD_ROUTES: saved {filename}',flush=True)
        rgb.detach()
    report = dict(passed=True,physics_steps=180,drop_tests=results,go2_spawn_preserved=True)
    REPORT.write_text(json.dumps(report,indent=2)+'\n')
    print('SHIPYARD_ROUTES_SIM_PASS',json.dumps(report),flush=True)
except BaseException:
    # Kit close may terminate the process before Python prints a pending error.
    traceback.print_exc()
    REPORT.write_text(json.dumps(dict(passed=False,error=traceback.format_exc()),indent=2)+'\n')
    app.app.post_quit(1)
    raise
finally:
    app.close()
