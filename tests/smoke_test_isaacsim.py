#!/usr/bin/env python3
"""Headless Kit/PhysX/importer smoke test; does not convert the HUNTER asset.

After conda activation, run: python tests/smoke_test_isaacsim.py.
On first launch the vendor's EULA prompt must be answered by the user.
"""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
from isaacsim import SimulationApp

app = SimulationApp({'headless':True,'active_gpu':0,'physics_gpu':0,'multi_gpu':False,
                     'extra_args':['--portable','--portable-root',str(ROOT/'.runtime/kit')]})
try:
    import numpy as np
    import omni.kit.commands
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import DynamicCuboid
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension('isaacsim.asset.importer.urdf')
    for _ in range(10):app.update()
    ok,config=omni.kit.commands.execute('URDFCreateImportConfig')
    assert ok and config is not None,'URDF importer command not available'
    world=World(stage_units_in_meters=1.0,physics_dt=1/60,rendering_dt=1/60)
    cube=world.scene.add(DynamicCuboid(prim_path='/World/InstallTestCube',name='install_test_cube',
                                     position=np.array([0.,0.,2.]),size=.2,mass=1.))
    world.reset()
    start=cube.get_world_pose()[0].copy()
    for _ in range(30):world.step(render=True)
    end=cube.get_world_pose()[0].copy()
    assert np.isfinite(end).all() and end[2] < start[2]-.2,(start,end)
    report={'simulation_app_started':True,'headless':True,'gpu_index':0,
            'urdf_importer_command':'pass','physics_gravity_test':'pass',
            'start_position':start.tolist(),'end_position':end.tolist(),
            'physics_steps':30,'dt':1/60,'hunter_imported':False,'usd_created':False}
    out=ROOT/'docs/isaacsim_install_evidence';out.mkdir(exist_ok=True)
    (out/'runtime_check.json').write_text(json.dumps(report,indent=2)+'\n')
    print('SHIPYARD_ISAAC_SMOKE_PASS '+json.dumps(report),flush=True)
finally:
    app.close()
