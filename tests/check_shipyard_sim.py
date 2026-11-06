#!/usr/bin/env python3
"""GPU smoke test: load shipyard, render a preview, and drop a cube on the apron."""
import json
import faulthandler
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'scripts')]
from visualization.runtime import prepare_environment, kit_arguments
prepare_environment(ROOT, 'shipyard_check')
faulthandler.dump_traceback_later(90, repeat=True)
from isaaclab.app import AppLauncher
app = AppLauncher(headless=True, enable_cameras=True, device='cuda:0',
                  kit_args=kit_arguments(ROOT, 'shipyard_check')).app
try:
    print('SHIPYARD_CHECK: renderer initialized', flush=True)
    import numpy as np
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import DynamicCuboid
    from pxr import UsdGeom
    from scenarios.registry import load_go2_scenario
    from scenarios.shipyard.scene_cfg import WORLD_USD, ROBOT_START
    cfg = load_go2_scenario('shipyard')
    print('SHIPYARD_CHECK: Go2 configuration loaded', flush=True)
    assert tuple(cfg.scene.robot.init_state.pos) == ROBOT_START
    world = World(stage_units_in_meters=1.0, physics_dt=1/60, rendering_dt=1/60)
    stage = omni.usd.get_context().get_stage()
    UsdGeom.Xform.Define(stage, '/World/Environment').GetPrim().GetReferences().AddReference(str(WORLD_USD))
    cube = world.scene.add(DynamicCuboid(prim_path='/World/DropTest', name='drop_test',
                                       position=np.array([*ROBOT_START[:2], 1.0]), size=.2, mass=1))
    world.reset()
    print('SHIPYARD_CHECK: physics initialized', flush=True)
    for _ in range(180):
        world.step(render=False)
    end, _ = cube.get_world_pose()
    assert np.isfinite(end).all() and .08 < end[2] < .14, end
    assert np.linalg.norm(end[:2] - np.array(ROBOT_START[:2])) < .1, end
    import omni.replicator.core as rep
    from PIL import Image
    out = ROOT / 'outputs/shipyard'
    out.mkdir(parents=True, exist_ok=True)
    product = rep.create.render_product('/World/Environment/Overview', (1280, 720))
    rgb = rep.AnnotatorRegistry.get_annotator('rgb')
    rgb.attach([product])
    for _ in range(30):
        world.step(render=True)
    pixels = rgb.get_data()
    assert pixels.shape[:2] == (720, 1280), pixels.shape
    Image.fromarray(pixels).save(out / 'preview.png')
    report = dict(passed=True, physics_steps=180, cube_end_position=end.tolist(), go2_config_loaded=True)
    (out / 'simulation_report.json').write_text(json.dumps(report, indent=2) + '\n')
    print('SHIPYARD_SIM_PASS', json.dumps(report), flush=True)
finally:
    faulthandler.cancel_dump_traceback_later()
    app.close()
