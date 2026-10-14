#!/usr/bin/env python3
"""Check the real warehouse USD, spawn clearance, collisions and repeatable Go2 resets."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'scripts'))
from visualization.runtime import kit_arguments, prepare_environment


def main():
    prepare_environment(ROOT, 'go2_ros')
    from isaaclab.app import AppLauncher
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, default=ROOT / '.runtime/warehouse/scene_check.json')
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    args.kit_args += ' ' + kit_arguments(ROOT, 'go2_ros')
    app = AppLauncher(args).app
    env = None
    result = {'passed': False}
    try:
        import carb
        import omni.usd
        from omni.physx import get_physx_scene_query_interface
        from pxr import Gf, UsdGeom, UsdPhysics
        import torch
        from isaaclab.envs import ManagerBasedRLEnv
        from scenarios.registry import DEFAULT_SCENARIO, load_go2_scenario, load_scene
        from scenarios.warehouse.scene_cfg import ROBOT_START, WAREHOUSE_PRIM_PATH, WAREHOUSE_URL

        assert DEFAULT_SCENARIO == 'warehouse'
        static_cfg, _, _ = load_scene(DEFAULT_SCENARIO)
        cfg = load_go2_scenario(DEFAULT_SCENARIO)
        assert static_cfg.terrain.usd_path == cfg.scene.terrain.usd_path == WAREHOUSE_URL
        cfg.sim.device = args.device
        cfg.sim.enable_scene_query_support = True
        cfg.commands.base_velocity.debug_vis = False
        env = ManagerBasedRLEnv(cfg)
        env.reset()
        stage = omni.usd.get_context().get_stage()
        warehouse = stage.GetPrimAtPath(WAREHOUSE_PRIM_PATH + '/terrain')
        assert warehouse.IsValid()
        composition_errors = [str(error) for prim in stage.Traverse()
                              for error in prim.GetPrimIndex().localErrors]
        assert not composition_errors, composition_errors
        assert UsdGeom.GetStageMetersPerUnit(stage) == 1.0
        assert UsdGeom.GetStageUpAxis(stage) == 'Z'
        collision_paths = [str(prim.GetPath()) for prim in stage.Traverse()
                           if str(prim.GetPath()).startswith(WAREHOUSE_PRIM_PATH)
                           and prim.HasAPI(UsdPhysics.CollisionAPI)
                           and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()]
        assert len(collision_paths) > 100, len(collision_paths)
        assert not stage.GetPrimAtPath('/World/ground').IsValid(), 'Duplicate ground plane'
        assert not any('CargoLeft' in str(p.GetPath()) for p in stage.Traverse())

        query = get_physx_scene_query_interface()
        overlaps = []
        def record_overlap(hit):
            path = str(hit.collision)
            if path.startswith(WAREHOUSE_PRIM_PATH):
                overlaps.append(path)
            return True
        query.overlap_box(Gf.Vec3f(.5, .3, .3), Gf.Vec3f(*ROBOT_START),
                          carb.Float4(0., 0., 0., 1.), record_overlap, False)
        assert not overlaps, f'Start position intersects warehouse: {overlaps}'

        def warehouse_ray(direction, distance):
            hits = []
            def record(hit):
                if str(hit.collision).startswith(WAREHOUSE_PRIM_PATH):
                    hits.append({'collision': str(hit.collision),
                                 'distance': float(hit.distance),
                                 'position': [hit.position.x, hit.position.y, hit.position.z]})
                return True
            query.raycast_all(Gf.Vec3f(0., 0., .5), Gf.Vec3f(*direction), distance, record)
            return min(hits, key=lambda hit: hit['distance']) if hits else None
        floor = warehouse_ray((0., 0., -1.), 2.)
        assert floor and abs(floor['position'][2]) < .01, floor
        walls = [warehouse_ray(direction, 100.) for direction in
                 ((1., 0., 0.), (-1., 0., 0.), (0., 1., 0.), (0., -1., 0.))]
        assert all(hit and hit['distance'] > .5 for hit in walls), walls
        robot = env.scene['robot']
        initial = robot.data.root_state_w.clone()
        for _ in range(2):
            env.step(torch.zeros((1, 12), device=env.device))
            env.reset()
            assert torch.equal(initial, robot.data.root_state_w), 'Reset changes spawn pose/velocity'
        assert torch.allclose(initial[0, :3], torch.tensor(ROBOT_START, device=env.device))
        result.update(passed=True, scenario=DEFAULT_SCENARIO, seed=cfg.seed,
                      usd=WAREHOUSE_URL, active_colliders=len(collision_paths),
                      start_position=initial[0, :3].tolist(), spawn_clear=True,
                      floor=floor, wall_rays=walls, repeatable_reset=True,
                      composition_errors=0, duplicate_ground=False)
        print('WAREHOUSE_SCENE_CHECK_PASS ' + json.dumps(result), flush=True)
    except Exception as exc:
        import traceback
        result['error'] = f'{type(exc).__name__}: {exc}'
        traceback.print_exc()
    finally:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(result, indent=2) + '\n')
        if env is not None:
            env.close()
        app.close()
    if not result['passed']:
        raise SystemExit('Warehouse scene check failed.')


if __name__ == '__main__':
    if '--worker' in sys.argv:
        sys.argv.remove('--worker')
        main()
    else:
        # Kit may terminate Python during shutdown. Inspect the fresh worker report
        # in the parent so a failed assertion cannot produce a successful exit.
        parser = argparse.ArgumentParser(add_help=False)
        parser.add_argument('--report', type=Path, default=ROOT / '.runtime/warehouse/scene_check.json')
        args, _ = parser.parse_known_args()
        if '-h' not in sys.argv and '--help' not in sys.argv:
            args.report.unlink(missing_ok=True)
        status = subprocess.run([sys.executable, '-u', str(Path(__file__).resolve()),
                                 *sys.argv[1:], '--worker']).returncode
        if '-h' in sys.argv or '--help' in sys.argv:
            raise SystemExit(status)
        passed = args.report.is_file() and json.loads(args.report.read_text()).get('passed')
        raise SystemExit(0 if status == 0 and passed else 1)
