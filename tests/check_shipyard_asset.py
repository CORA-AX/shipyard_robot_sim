#!/usr/bin/env python3
"""CPU checks for the saved shipyard: geometry, collisions, relocation and persistence."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts/environments'))
from prepare_shipyard import ASSET, accessor, bootstrap_usd, read_glb


def main():
    bootstrap_usd()
    import numpy as np
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, UsdUtils
    source, binary = read_glb(ASSET / 'source/model.glb')
    # Regression for the quantized glTF positions that generic importers mishandle.
    first = accessor(source, binary, source['meshes'][0]['primitives'][0]['attributes']['POSITION'])
    assert np.isclose(np.abs(first).max(), 1.0)
    stage = Usd.Stage.Open(str(ASSET / 'world.usda'))
    assert stage.GetDefaultPrim().GetPath() == Sdf.Path('/World')
    assert UsdGeom.GetStageUpAxis(stage) == 'Z'
    assert UsdGeom.GetStageMetersPerUnit(stage) == 1
    bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render']).ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange()
    np.testing.assert_allclose(bounds.GetMin(), [-19, -30.30015, -6.20010], atol=.002)
    np.testing.assert_allclose(bounds.GetMax(), [22.0012, 26, 16.99994], atol=.002)
    cache = UsdGeom.XformCache()
    mesh_count = collider_count = water_count = 0
    triangles = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh_count += 1
        mesh = UsdGeom.Mesh(prim)
        points = np.asarray(mesh.GetPointsAttr().Get())
        assert np.isfinite(points).all()
        material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        assert material
        if 'basin_water' in str(prim.GetPath()):
            water_count += 1
            assert not prim.HasAPI(UsdPhysics.CollisionAPI)
        else:
            collider_count += 1
            assert UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
            assert UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() == 'none'
            world = np.c_[points, np.ones(len(points))] @ np.asarray(cache.GetLocalToWorldTransform(prim))
            triangles.extend(world[np.asarray(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3), :3])
    expected = sum(len(source['meshes'][node['mesh']]['primitives']) for node in source['nodes'] if 'mesh' in node)
    assert mesh_count == expected == 955
    assert (collider_count, water_count) == (930, 25)
    # Vertical intersections over the robot footprint must contain only level apron.
    t = np.asarray(triangles)
    a = t[:, 0, :2]; u = t[:, 1, :2] - a; v = t[:, 2, :2] - a
    det = u[:, 0] * v[:, 1] - u[:, 1] * v[:, 0]
    good = np.abs(det) > 1e-10
    for x in [-16.4, -16, -15.6]:
        for y in [17.7, 18, 18.3]:
            q = np.array([x, y]) - a
            b = np.zeros(len(t)); c = b.copy()
            b[good] = (q[good, 0] * v[good, 1] - q[good, 1] * v[good, 0]) / det[good]
            c[good] = (u[good, 0] * q[good, 1] - u[good, 1] * q[good, 0]) / det[good]
            hit = good & (b >= -1e-6) & (c >= -1e-6) & (b + c <= 1 + 1e-6)
            z = t[:, 0, 2] + b * (t[:, 1, 2] - t[:, 0, 2]) + c * (t[:, 2, 2] - t[:, 0, 2])
            assert hit.any() and abs(z[hit].max()) < .002, (x, y, z[hit])
    with tempfile.TemporaryDirectory() as directory:
        copied = Path(directory) / 'shipyard'
        shutil.copytree(ASSET, copied)
        layers, assets, unresolved = UsdUtils.ComputeAllDependencies(str(copied / 'world.usda'))
        assert not unresolved, unresolved
        for path in [*(layer.realPath for layer in layers), *assets]:
            assert Path(path).is_relative_to(copied), path
        relocated = Usd.Stage.Open(str(copied / 'world.usda'))
        assert sum(p.IsA(UsdGeom.Mesh) for p in relocated.Traverse()) == mesh_count
    editable = [ASSET / name for name in ('world.usda', 'layout.usda', 'physics.usda')]
    hashes = [hashlib.sha256(p.read_bytes()).hexdigest() for p in editable]
    subprocess.run([sys.executable, str(ROOT / 'scripts/environments/prepare_shipyard.py')], check=True)
    assert hashes == [hashlib.sha256(p.read_bytes()).hexdigest() for p in editable]
    report = dict(passed=True, meshes=mesh_count, colliders=collider_count, water_meshes=water_count,
                  bounds_min=list(bounds.GetMin()), bounds_max=list(bounds.GetMax()),
                  spawn_footprint_clear=True, portable_dependencies=True, editable_layers_preserved=True)
    out = ROOT / 'outputs/shipyard'
    out.mkdir(parents=True, exist_ok=True)
    (out / 'asset_report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
