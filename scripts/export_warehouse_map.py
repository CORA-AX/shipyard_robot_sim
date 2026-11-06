#!/usr/bin/env python3
"""Rasterize the warehouse's PhysX collisions, without spawning a robot.

Run with Isaac Python. The committed map is generated at 5 cm resolution over
the Go2 collision band (3–65 cm); navigation adds the robot footprint separately.
"""
import argparse
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'scripts'))


def main():
    from visualization.runtime import kit_arguments, prepare_environment
    sys.path.insert(0, str(ROOT / 'ros2_ws/src/robot_sim'))
    from robot_sim.environment import isaac_environment
    import os
    environment = isaac_environment(sys.executable, python_paths=[ROOT, ROOT / 'scripts'])
    for key in set(os.environ) - set(environment):
        os.environ.pop(key)
    os.environ.update(environment)
    prepare_environment(ROOT, 'go2_ros')
    from isaaclab.app import AppLauncher
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path,
                        default=ROOT / 'ros2_ws/src/robot_navigation/maps/warehouse.yaml')
    parser.add_argument('--resolution', type=float, default=.05)
    parser.add_argument('--min-height', type=float, default=.03)
    parser.add_argument('--max-height', type=float, default=.65)
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    if not .02 <= args.resolution <= .2 or not 0 < args.min_height < args.max_height:
        parser.error('Invalid map resolution or obstacle height band.')
    args.headless = True
    args.kit_args += ' ' + kit_arguments(ROOT, 'warehouse_map') + ' --/isaac/startup/ros_bridge_extension='
    app = AppLauncher(args).app
    sim = None
    try:
        import carb
        import numpy as np
        import omni.usd
        from omni.physx import get_physx_scene_query_interface
        from pxr import Gf, Usd, UsdGeom, UsdPhysics
        from isaaclab.sim import SimulationCfg, SimulationContext
        from isaaclab.terrains import TerrainImporter
        from scenarios.warehouse.scene_cfg import WarehouseSceneCfg, WAREHOUSE_PRIM_PATH, WAREHOUSE_URL

        sim = SimulationContext(SimulationCfg(device='cpu', dt=.005, enable_scene_query_support=True))
        terrain_cfg = WarehouseSceneCfg().terrain.copy()
        terrain_cfg.env_spacing, terrain_cfg.num_envs = 2.5, 1
        terrain = TerrainImporter(terrain_cfg)
        sim.reset()
        stage = omni.usd.get_context().get_stage()
        if UsdGeom.GetStageMetersPerUnit(stage) != 1. or UsdGeom.GetStageUpAxis(stage) != 'Z':
            raise RuntimeError('Warehouse map requires a meter/Z-up world.')
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render', 'proxy'])
        colliders, bounds = [], []
        for prim in stage.Traverse():
            if (str(prim.GetPath()).startswith(WAREHOUSE_PRIM_PATH)
                    and prim.HasAPI(UsdPhysics.CollisionAPI)
                    and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()):
                colliders.append(str(prim.GetPath()))
                box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
                lo, hi = box.GetMin(), box.GetMax()
                if (all(math.isfinite(v) and abs(v) < 1000 for v in (*lo, *hi))
                        and hi[2] > args.min_height and lo[2] < args.max_height
                        and 'GroundPlane' not in str(prim.GetPath())):
                    bounds.append((list(lo), list(hi)))
        if len(colliders) < 100 or not bounds:
            raise RuntimeError('Warehouse collision geometry failed to load.')
        errors = [str(error) for prim in stage.Traverse() for error in prim.GetPrimIndex().localErrors]
        if errors:
            raise RuntimeError(f'USD composition errors: {errors[:5]}')
        resolution = args.resolution
        lower = [math.floor((min(b[0][i] for b in bounds) - .5) / resolution) * resolution for i in (0, 1)]
        upper = [math.ceil((max(b[1][i] for b in bounds) + .5) / resolution) * resolution for i in (0, 1)]
        width, height = [round((b - a) / resolution) for a, b in zip(lower, upper)]
        print(f'Rasterizing {width}x{height}, bounds={lower}..{upper}, colliders={len(colliders)}', flush=True)
        grid = np.full((height, width), 205, dtype=np.uint8)
        query = get_physx_scene_query_interface()
        half = Gf.Vec3f(resolution / 2, resolution / 2, (args.max_height - args.min_height) / 2)
        rotation = carb.Float4(0, 0, 0, 1)
        occupied = False

        def hit(_):
            nonlocal occupied
            occupied = True
            return False

        for row in range(height):
            y = lower[1] + (row + .5) * resolution
            for col in range(width):
                x = lower[0] + (col + .5) * resolution
                occupied = False
                query.overlap_box(half, Gf.Vec3f(x, y, (args.min_height + args.max_height) / 2),
                                  rotation, hit, False)
                if occupied:
                    grid[row, col] = 0
                else:
                    floor = query.raycast_closest(Gf.Vec3f(x, y, .04), Gf.Vec3f(0, 0, -1), .08)
                    if floor['hit']:
                        grid[row, col] = 254
        # Only floor connected to the known spawn is navigable; the exterior and
        # enclosed rack interiors must not become alternative global routes.
        start = (int(-lower[1] / resolution), int(-lower[0] / resolution))
        if grid[start] != 254:
            raise RuntimeError('The known robot spawn is not free in the generated map.')
        reached = np.zeros(grid.shape, dtype=bool)
        queue = deque([start])
        reached[start] = True
        while queue:
            r, c = queue.popleft()
            for nr, nc in ((r-1,c), (r+1,c), (r,c-1), (r,c+1)):
                if 0 <= nr < height and 0 <= nc < width and not reached[nr,nc] and grid[nr,nc] == 254:
                    reached[nr,nc] = True
                    queue.append((nr,nc))
        grid[(grid == 254) & ~reached] = 205
        grid[[0, -1], :] = 205
        grid[:, [0, -1]] = 205
        output = args.output.resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        image = output.with_suffix('.pgm')
        image.write_bytes(f'P5\n{width} {height}\n255\n'.encode() + np.flipud(grid).tobytes())
        output.write_text(f'image: {image.name}\nmode: trinary\nresolution: {resolution}\n'
                          f'origin: [{lower[0]:.6f}, {lower[1]:.6f}, 0.0]\n'
                          'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        metadata = dict(source_url=WAREHOUSE_URL, source_lock=json.loads(
            (ROOT / 'assets/environments/warehouse/sources.lock.json').read_text()),
            generator='scripts/export_warehouse_map.py', isaac_sim='5.0.0',
            resolution=resolution, origin=[*lower, 0.], width=width, height=height,
            obstacle_height_band=[args.min_height, args.max_height], active_colliders=len(colliders),
            collider_paths_sha256=hashlib.sha256('\n'.join(sorted(colliders)).encode()).hexdigest(),
            image_sha256=hashlib.sha256(image.read_bytes()).hexdigest(),
            cells={name: int((grid == value).sum()) for name,value in
                   [('free',254),('occupied',0),('unknown',205)]}, floor_connected_to=[0.,0.],
            composition_errors=0)
        output.with_suffix('.json').write_text(json.dumps(metadata, indent=2) + '\n')
        print('WAREHOUSE_MAP_EXPORTED ' + json.dumps(metadata), flush=True)
    except BaseException:
        import traceback
        traceback.print_exc()
        sys.stderr.flush()
        raise
    finally:
        if sim is not None:
            # Release Isaac Lab's timeline callbacks before Kit's shutdown;
            # otherwise its pause-on-stop handler can leave this tool running.
            sim.clear_all_callbacks()
            sim.clear_instance()
        app.close()


if __name__ == '__main__':
    if '--worker' in sys.argv:
        sys.argv.remove('--worker')
        main()
    else:
        # Kit can terminate Python during app.close(), including on exceptions.
        # Require a fresh metadata file and matching image, never just exit code 0.
        import subprocess
        import time
        parser = argparse.ArgumentParser(add_help=False)
        parser.add_argument('--output', type=Path,
                            default=ROOT / 'ros2_ws/src/robot_navigation/maps/warehouse.yaml')
        arguments, _ = parser.parse_known_args()
        started = time.time_ns()
        sys.path.insert(0, str(ROOT / 'ros2_ws/src/robot_sim'))
        from robot_sim.environment import isaac_environment
        status = subprocess.run([sys.executable, '-u', str(Path(__file__).resolve()),
                                 *sys.argv[1:], '--worker'], env=isaac_environment(
                                     sys.executable, python_paths=[ROOT, ROOT / 'scripts'])).returncode
        if '-h' in sys.argv or '--help' in sys.argv:
            raise SystemExit(status)
        metadata = arguments.output.with_suffix('.json')
        fresh = metadata.exists() and metadata.stat().st_mtime_ns >= started
        correct = (fresh and arguments.output.is_file()
                   and json.loads(metadata.read_text())['image_sha256'] == hashlib.sha256(
                       arguments.output.with_suffix('.pgm').read_bytes()).hexdigest())
        raise SystemExit(0 if status == 0 and correct else 1)
