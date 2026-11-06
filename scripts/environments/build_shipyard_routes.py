#!/usr/bin/env python3
"""Build only the shipyard service roads and route hints; never rewrite other layers.

Use --output-dir for a scratch build. Existing output requires --replace so editor
changes are not silently lost. Uses the pinned local CC0 shipyard props offline.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

from prepare_shipyard import ASSET, bootstrap_usd


def build(output):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    stage = Usd.Stage.CreateNew(str(output / 'routes.usda'))
    root = UsdGeom.Xform.Define(stage, '/World')
    stage.SetDefaultPrim(root.GetPrim())
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, 'Z')
    base = '/World/RoadNetwork'
    UsdGeom.Xform.Define(stage, base)
    materials = {}
    for name, color, metallic in [
        ('Asphalt', (.075, .09, .105), 0), ('Concrete', (.43, .46, .46), 0),
        ('Gravel', (.24, .25, .24), 0), ('Yellow', (.95, .65, .025), .15),
        ('White', (.85, .86, .80), 0), ('Steel', (.12, .17, .19), .75),
    ]:
        mat = UsdShade.Material.Define(stage, base + '/Looks/' + name)
        shader = UsdShade.Shader.Define(stage, str(mat.GetPath()) + '/Surface')
        shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.82)
        shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(metallic)
        shader.CreateOutput('surface', Sdf.ValueTypeNames.Token)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
        materials[name] = mat
    friction = UsdShade.Material.Define(stage, base + '/Looks/Traction')
    phys = UsdPhysics.MaterialAPI.Apply(friction.GetPrim())
    phys.CreateStaticFrictionAttr(.9)
    phys.CreateDynamicFrictionAttr(.8)
    phys.CreateRestitutionAttr(0)

    def finish(prim, material, collision):
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(materials[material])
        if collision:
            UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
            UsdShade.MaterialBindingAPI(prim).Bind(friction, materialPurpose='physics')
            if prim.IsA(UsdGeom.Mesh):
                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('none')

    def box(name, center, size, material='Concrete', collision=True):
        cube = UsdGeom.Cube.Define(stage, base + '/' + name)
        cube.CreateSizeAttr(1)
        cube.AddTranslateOp().Set(Gf.Vec3d(*center))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        finish(cube.GetPrim(), material, collision)
        return cube

    def slab(name, x0, x1, y0, y1, top=0, bottom=-.25, material='Asphalt'):
        return box(name, ((x0+x1)/2, (y0+y1)/2, (top+bottom)/2),
                   (x1-x0, y1-y0, top-bottom), material)

    def wedge(name, x0, x1, y0, y1, z0, z1, material='Concrete', collision=True):
        mesh = UsdGeom.Mesh.Define(stage, base + '/' + name)
        pts = [(x0,y0,-.25), (x1,y0,-.25), (x1,y1,-.25), (x0,y1,-.25),
               (x0,y0,z0), (x1,y0,z0), (x1,y1,z1), (x0,y1,z1)]
        mesh.CreatePointsAttr(pts)
        # Closed, outward-facing triangles: exact sloped surface, no box proxy.
        mesh.CreateFaceVertexCountsAttr([3]*12)
        mesh.CreateFaceVertexIndicesAttr([0,2,1,0,3,2,4,5,6,4,6,7,
                                         0,1,5,0,5,4,1,2,6,1,6,5,
                                         2,3,7,2,7,6,3,0,4,3,4,7])
        mesh.CreateSubdivisionSchemeAttr('none')
        mesh.CreateExtentAttr([(x0,y0,-.25),(x1,y1,max(z0,z1))])
        finish(mesh.GetPrim(), material, collision)

    def beam(name, a, b, width=.055):
        a, b = Gf.Vec3d(*a), Gf.Vec3d(*b)
        shape = UsdGeom.Cylinder.Define(stage, base + '/' + name)
        shape.CreateRadiusAttr(width/2)
        shape.CreateHeightAttr((b-a).GetLength())
        shape.AddTranslateOp().Set((a+b)/2)
        rotation = Gf.Rotation(Gf.Vec3d(0,0,1), b-a).GetQuat()
        shape.AddOrientOp().Set(Gf.Quatf(rotation.GetReal(), Gf.Vec3f(rotation.GetImaginary())))
        finish(shape.GetPrim(), 'Yellow', True)

    def rail(name, a, b):
        length = math.dist(a,b)
        count = math.ceil(length/2)
        for i in range(count+1):
            p = [a[j]+(b[j]-a[j])*i/count for j in range(3)]
            beam(f'Rails/{name}/Post{i}', p, (p[0],p[1],p[2]+1.1), .07)
        for i, h in enumerate((.55,1.1)):
            beam(f'Rails/{name}/Bar{i}', (a[0],a[1],a[2]+h), (b[0],b[1],b[2]+h))

    # Entirely west of the original dock. Connection meets the old apron at x=-18.
    slab('Ground/ServiceYard', -44,-18,-24,22, -.03,-.5,'Gravel')
    slab('Roads/NorthJunction', -40,-18,14,22)
    slab('Roads/SouthJunction', -40,-22,-24,-16)
    slab('Roads/LevelBypass', -28,-22,-16,14)
    wedge('Roads/NorthRamp', -40,-34,2,14,1.2,0)
    slab('Roads/LoadingDeck', -40,-34,-4,2,1.2,material='Concrete')
    wedge('Roads/SouthRamp', -40,-34,-16,-4,0,1.2)
    slab('Roads/StairApron', -30.8,-28,-3,1,material='Concrete')
    slab('Roads/EquipmentPad', -33.2,-28.5,4,12,material='Concrete')
    slab('Roads/ContainerPad', -33.2,-28.5,-14,-6,material='Concrete')
    for i in range(8):
        x1 = -30.8-i*.4
        slab(f'Stairs/Step{i+1:02}', x1-.4,x1,-2.2,.2,(i+1)*.15,material='Concrete')
        box(f'Markings/StairNosing{i+1:02}', (x1-.035,-1,(i+1)*.15+.003),
            (.07,2.4,.006),'Yellow',False)
    for x, label in [(-40,'West'),(-34,'East')]:
        rail(label+'North', (x,2,1.2),(x,14,0))
        rail(label+'South', (x,-16,0),(x,-4,1.2))
    rail('DeckWest',(-40,-4,1.2),(-40,2,1.2))
    rail('DeckEastSouth',(-34,-4,1.2),(-34,-2.2,1.2))
    rail('DeckEastNorth',(-34,.2,1.2),(-34,2,1.2))
    for y, label in [(-2.2,'South'),(.2,'North')]:
        rail('Stair'+label,(-30.8,y,.15),(-34,y,1.2))

    # Paint is visual only: it must not introduce wheel-catching collision lips.
    for x in (-27.7,-22.3):
        box(f'Markings/BypassEdge{abs(x)*10:.0f}',(x,-1,.003),(.10,30,.006),'White',False)
    for i,y in enumerate(range(-14,15,4)):
        box(f'Markings/BypassCenter{i}',(-25,y,.003),(.12,2,.006),'Yellow',False)
    for side,x in [('West',-39.7),('East',-34.3)]:
        wedge('Markings/'+side+'North',x-.05,x+.05,2,14,1.206,.006,'Yellow',False)
        wedge('Markings/'+side+'South',x-.05,x+.05,-16,-4,.006,1.206,'Yellow',False)
    for i,x in enumerate(range(-38,-18,4)):
        box(f'Markings/NorthCenter{i}',(x,18,.003),(2,.12,.006),'Yellow',False)
    for i,x in enumerate(range(-38,-22,4)):
        box(f'Markings/SouthCenter{i}',(x,-20,.003),(2,.12,.006),'Yellow',False)
    for i in range(7):
        box(f'Markings/Crossing{i}',(-27.5+i*.8,11,.004),(.4,2,.008),'White',False)
    # Low perimeter curbs; the connection at x=-18 is kept completely open.
    slab('Kerbs/West',-40.5,-40.2,-24,22,.15,-.25,'Concrete')
    slab('Kerbs/North',-40.2,-18,21.8,22,.15,-.25,'Concrete')
    slab('Kerbs/South',-40.2,-21.8,-24,-23.8,.15,-.25,'Concrete')
    slab('Kerbs/East',-22,-21.8,-23.8,13.8,.15,-.25,'Concrete')

    # Reuse a small subset of the existing CC0 pack, with materials and transforms.
    source = Usd.Stage.Open(str(ASSET/'models/shipyard.usd'))
    flat = source.Flatten()
    looks = base + '/Looks/Source'
    Sdf.CopySpec(flat, '/Shipyard/Looks', stage.GetRootLayer(), looks)
    geom = source.GetPrimAtPath('/Shipyard/Geometry').GetChildren()[0]
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default','render'])
    transforms = UsdGeom.XformCache()
    reused = []
    for suffix, name, destination in [
        ('_pipe_spool_rack','PipeRack',(-30.8,8,0)),
        ('_fire_point','FirePoint',(-29.1,5,0)),
        ('_store_container','StoreContainer',(-30.8,-10,0)),
        ('_floodlight_mast','YardLight',(-42,19,-.03)),
    ]:
        candidates = [p for p in geom.GetChildren() if p.GetName().endswith(suffix)]
        # Choose narrow-in-X container/rack variants already oriented along Y.
        old = min(candidates, key=lambda p: bbox.ComputeWorldBound(p).ComputeAlignedRange().GetSize()[0])
        bounds = bbox.ComputeWorldBound(old).ComputeAlignedRange()
        anchor = Gf.Vec3d(bounds.GetMidpoint()[0],bounds.GetMidpoint()[1],bounds.GetMin()[2])
        dst = base+'/Props/'+name
        UsdGeom.Xform.Define(stage, base+'/Props')
        Sdf.CopySpec(flat, old.GetPath(), stage.GetRootLayer(), dst)
        copied = stage.GetPrimAtPath(dst)
        xform = UsdGeom.Xformable(copied)
        xform.ClearXformOpOrder()
        matrix = transforms.GetLocalToWorldTransform(old)
        matrix.SetTranslateOnly(matrix.ExtractTranslation()+Gf.Vec3d(*destination)-anchor)
        xform.AddTransformOp().Set(matrix)
        for prim in Usd.PrimRange(copied):
            for rel in prim.GetRelationships():
                if rel.GetName().startswith('material:binding'):
                    rel.SetTargets([p.ReplacePrefix(Sdf.Path('/Shipyard/Looks'),Sdf.Path(looks))
                                    for p in rel.GetTargets()])
            if prim.IsA(UsdGeom.Mesh):
                UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('none')
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(friction, materialPurpose='physics')
        reused.append({'prim':dst, 'source_prim':str(old.GetPath())})
    # Flatten resolves texture paths; restore portable paths in copied materials.
    for prim in Usd.PrimRange(stage.GetPrimAtPath(looks)):
        for attr in prim.GetAttributes():
            if attr.GetTypeName() == Sdf.ValueTypeNames.Asset and attr.Get():
                attr.Set(Sdf.AssetPath('./textures/'+Path(attr.Get().path).name))
    camera = UsdGeom.Camera.Define(stage, '/World/ServiceRoadOverview')
    camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(
        Gf.Vec3d(-67,49,43),Gf.Vec3d(-20,0,0),Gf.Vec3d(0,0,1)).GetInverse())
    camera.CreateClippingRangeAttr((.1,500))
    camera.CreateFocalLengthAttr(24)
    stage.GetRootLayer().Save()

    nodes = {
        'yard_start':[-16,18,0], 'north_junction':[-25,18,0],
        'north_turn':[-37,18,0], 'north_toe':[-37,14,0],
        'deck_north':[-37,2,1.2], 'deck_access':[-37,-1,1.2],
        'deck_south':[-37,-4,1.2], 'south_toe':[-37,-16,0],
        'south_turn':[-37,-20,0], 'south_junction':[-25,-20,0],
        'bypass_access':[-25,-1,0], 'stair_toe':[-30.6,-1,0],
        'stair_top':[-34.2,-1,1.2],
    }
    for i in range(8):
        nodes[f'step_{i+1:02}'] = [round(-31-i*.4,2),-1,round((i+1)*.15,2)]
    edges = []
    def edge(a,b,kind='flat',width=6):
        edges.append(dict(start=a,end=b,bidirectional=True,kind=kind,width_m=width,
                          max_grade=None if kind=='stairs' else (.1 if kind=='ramp' else 0),
                          requires=['stair_climbing'] if kind=='stairs' else
                          (['slope_10_percent'] if kind=='ramp' else [])))
    loop = ['yard_start','north_junction','north_turn','north_toe','deck_north',
            'deck_access','deck_south','south_toe','south_turn','south_junction',
            'bypass_access','north_junction']
    for a,b in zip(loop,loop[1:]):
        edge(a,b,'ramp' if (a,b) in [('north_toe','deck_north'),('deck_south','south_toe')] else 'flat',
             4 if a=='yard_start' else 6)
    edge('bypass_access','stair_toe',width=2.4)
    stair_route = ['stair_toe']+[f'step_{i+1:02}' for i in range(8)]+['stair_top']
    for a,b in zip(stair_route,stair_route[1:]):
        edge(a,b,'stairs',2.4)
    edge('stair_top','deck_access',width=2.4)
    hints = dict(schema_version=1, units='meters', up_axis='Z',
                 coordinate_frame='shipyard world.usda /World local; apply environment transform when instanced',
                 z_semantics='support surface, not robot base or spawn height',
                 automatic_navigation_enabled=False,
                 notes=['Route hints only; validate footprint, turning radius, controller and grade capability per robot.',
                        'Stair segments are discrete treads, never interpolate as a smooth ramp.',
                        'No Nav2 map, controller, robot spawn or scenario registry is modified.'],
                 stairs=dict(riser_m=.15,tread_m=.4,count=8,width_m=2.4),
                 nodes=nodes,edges=edges,
                 suggested_routes=dict(service_loop=loop,
                     flat_only=['yard_start','north_junction','bypass_access','south_junction'],
                     stair_access=['bypass_access']+stair_route+['deck_access']))
    (output/'route_hints.json').write_text(json.dumps(hints,indent=2)+'\n')
    provenance = dict(
        geometry='Roads, ramps, stairs, rails and markings authored locally by build_shipyard_routes.py.',
        reused_pack='Shipyard and Dry Dock',
        source_url='https://3dassets.dev/packs/shipyard-and-dry-dock',
        upstream_license='CC0 1.0 Universal, as stated on pack page; checked 2026-11-02',
        license_url='https://creativecommons.org/publicdomain/zero/1.0/',
        source_sha256=hashlib.sha256((ASSET/'source/model.glb').read_bytes()).hexdigest(),
        reused_props=reused,
        official_alternative=dict(url='https://docs.omniverse.nvidia.com/usd/latest/usd_content_samples/downloadable_packs.html',
            pack='SimReady Warehouse 01',size='13.9 GB',imported=False,
            reason='Reuse the matching local CC0 pack and dimensioned local road geometry without new remote dependencies.'))
    (output/'routes.sources.json').write_text(json.dumps(provenance,indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,default=ASSET)
    parser.add_argument('--replace',action='store_true')
    args = parser.parse_args()
    bootstrap_usd()
    outputs = ['routes.usda','route_hints.json','routes.sources.json']
    if not args.replace and any((args.output_dir/p).exists() for p in outputs):
        parser.error('Output exists; use --output-dir for a scratch build or --replace to regenerate only route files.')
    args.output_dir.mkdir(parents=True,exist_ok=True)
    build(args.output_dir)
    print(f'Built shipyard routes in {args.output_dir}')


if __name__ == '__main__':
    main()
