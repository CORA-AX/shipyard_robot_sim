#!/usr/bin/env python3
"""Offline geometry/clearance regression for shipyard service routes only."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts/environments'))
from prepare_shipyard import ASSET, bootstrap_usd


def main():
    bootstrap_usd()
    import numpy as np
    import trimesh
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, UsdUtils

    # Read the actual entry point, but do not fetch the user's pre-existing remote
    # warehouse. The population mask affects only this in-memory test stage.
    mask = Usd.StagePopulationMask()
    for path in ['/World/Shipyard','/World/RoadNetwork','/World/PhysicsMaterial']:
        mask.Add(path)
    stage = Usd.Stage.OpenMasked(str(ASSET/'world.usda'), mask)
    assert './routes.usda' in stage.GetRootLayer().subLayerPaths
    assert stage.GetPrimAtPath('/World/RoadNetwork/Roads/NorthRamp')
    assert UsdGeom.GetStageUpAxis(stage) == 'Z'
    assert UsdGeom.GetStageMetersPerUnit(stage) == 1
    triangles = []
    cache = UsdGeom.XformCache()
    colliders = 0
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        assert UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
        if prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            assert set(mesh.GetFaceVertexCountsAttr().Get()) == {3}
            assert UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() == 'none'
            points = np.asarray(mesh.GetPointsAttr().Get())
            indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1,3)
        elif prim.IsA(UsdGeom.Cube):
            mesh = trimesh.creation.box(extents=[UsdGeom.Cube(prim).GetSizeAttr().Get()]*3)
            points, indices = mesh.vertices, mesh.faces
        elif prim.IsA(UsdGeom.Cylinder):
            cylinder = UsdGeom.Cylinder(prim)
            mesh = trimesh.creation.cylinder(radius=cylinder.GetRadiusAttr().Get(),
                                             height=cylinder.GetHeightAttr().Get(), sections=16)
            points, indices = mesh.vertices, mesh.faces
        else:
            raise AssertionError(prim.GetPath())
        world = np.c_[points,np.ones(len(points))] @ np.asarray(cache.GetLocalToWorldTransform(prim))
        triangles.extend(world[indices,:3])
        if str(prim.GetPath()).startswith('/World/RoadNetwork'):
            colliders += 1
            assert not prim.HasAPI(UsdPhysics.RigidBodyAPI)
            assert UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
            assert UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial(materialPurpose='physics')[0]
    for prim in Usd.PrimRange(stage.GetPrimAtPath('/World/RoadNetwork/Markings')):
        assert not prim.HasAPI(UsdPhysics.CollisionAPI), prim.GetPath()
    t = np.asarray(triangles)
    a = t[:,0,:2]; u = t[:,1,:2]-a; v = t[:,2,:2]-a
    det = u[:,0]*v[:,1]-u[:,1]*v[:,0]
    valid = np.abs(det)>1e-10
    t,a,u,v,det = [x[valid] for x in (t,a,u,v,det)]

    def heights(x,y):
        q = np.array([x,y])-a
        b = (q[:,0]*v[:,1]-q[:,1]*v[:,0])/det
        c = (u[:,0]*q[:,1]-u[:,1]*q[:,0])/det
        hit = (b>=-1e-7)&(c>=-1e-7)&(b+c<=1+1e-7)
        return (t[:,0,2]+b*(t[:,1,2]-t[:,0,2])+c*(t[:,2,2]-t[:,0,2]))[hit]

    hints = json.loads((ASSET/'route_hints.json').read_text())
    samples = 0
    for edge in hints['edges']:
        start,end = [np.array(hints['nodes'][edge[key]],dtype=float) for key in ('start','end')]
        delta = end-start
        distance = np.linalg.norm(delta[:2])
        side = np.array([-delta[1],delta[0],0])/distance
        if edge['kind']=='ramp':
            assert abs(abs(delta[2])/distance-.1)<1e-7
        if edge['kind']=='stairs':
            assert edge['requires']==['stair_climbing']
        for alpha in np.linspace(.001,.999,max(3,int(distance/.15)+1)):
            p = start+alpha*delta
            if edge['kind']=='stairs':
                p[2] = 0 if p[0]>-30.8 else min(1.2,(int((-30.8-p[0])/.4+1e-6)+1)*.15)
            for offset in (-.65,0,.65):
                q = p+offset*side
                z = heights(*q[:2])
                # Ground within 2 cm, and no obstacles in a 1.5 m tall corridor.
                assert np.any(abs(z-p[2])<.02), (edge,q,'missing support',z)
                assert not np.any((z>p[2]+.025)&(z<p[2]+1.5)), (edge,q,'obstruction',z)
                samples += 1
    # Every node and route edge is connected; the wheeled loop never needs stairs.
    pairs = {frozenset((e['start'],e['end'])):e for e in hints['edges']}
    for name, route in hints['suggested_routes'].items():
        for a,b in zip(route,route[1:]):
            edge = pairs[frozenset((a,b))]
            if name!='stair_access':
                assert edge['kind']!='stairs'
    # Reused public geometry is self-contained except for the existing local texture.
    layers, assets, unresolved = UsdUtils.ComputeAllDependencies(str(ASSET/'routes.usda'))
    assert not unresolved, unresolved
    assert all(Path(p).is_relative_to(ASSET) for p in [*(l.realPath for l in layers),*assets])
    provenance = json.loads((ASSET/'routes.sources.json').read_text())
    assert provenance['source_sha256']==hashlib.sha256((ASSET/'source/model.glb').read_bytes()).hexdigest()
    # Preserve user's original scene edits when the generator is rerun elsewhere.
    from build_shipyard_routes import build
    editable = [ASSET/name for name in ('world.usda','layout.usda','physics.usda','routes.usda')]
    before = [p.read_bytes() for p in editable]
    with tempfile.TemporaryDirectory() as directory:
        build(Path(directory))
        assert (Path(directory)/'routes.usda').read_bytes()==(ASSET/'routes.usda').read_bytes()
        for name in ('route_hints.json','routes.sources.json'):
            assert (Path(directory)/name).read_bytes()==(ASSET/name).read_bytes()
    assert before==[p.read_bytes() for p in editable]
    report = dict(passed=True,route_edges=len(hints['edges']),corridor_samples=samples,
                  sampled_corridor_width_m=1.3,clearance_height_m=1.5,
                  new_colliders=colliders,local_dependencies=True,reproducible=True,
                  original_layers_preserved=True)
    output = ROOT/'outputs/shipyard'
    output.mkdir(parents=True,exist_ok=True)
    (output/'routes_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
