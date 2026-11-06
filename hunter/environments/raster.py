"""Rasterize ground support and static obstacles from local USD, without Kit.

Flat-ground phase: five support samples per cell, near z=0 and slope <=2 deg;
triangle/cell overlap for obstacles in z=[0.03, 0.8]. No sensor observations.
"""
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import time

import numpy as np
import yaml


def collider_triangles(stage):
    from pxr import Usd, UsdGeom, UsdPhysics
    cache = UsdGeom.XformCache()
    paths, triangles = [], []
    box_faces = np.array([[0,1,3],[0,3,2],[4,6,7],[4,7,5], [0,4,5],[0,5,1],
                          [2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]])
    for prim in Usd.PrimRange(stage.GetDefaultPrim(), Usd.TraverseInstanceProxies()):
        if not prim.HasAPI(UsdPhysics.CollisionAPI) or not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():
            continue
        paths.append(str(prim.GetPath()))
        if prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            points = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
            indices, counts = mesh.GetFaceVertexIndicesAttr().Get(), mesh.GetFaceVertexCountsAttr().Get()
            faces, offset = [], 0
            for count in counts:
                face = indices[offset:offset+count]
                faces.extend((face[0], face[i], face[i+1]) for i in range(1, count-1))
                offset += count
            indices = np.asarray(faces, dtype=int)
        elif prim.IsA(UsdGeom.Cube):
            points = np.array([(x,y,z) for x in (-1.,1.) for y in (-1.,1.) for z in (-1.,1.)])
            points *= UsdGeom.Cube(prim).GetSizeAttr().Get()/2.
            indices = box_faces
        elif prim.IsA(UsdGeom.Cylinder):
            shape = UsdGeom.Cylinder(prim)
            radius, height = shape.GetRadiusAttr().Get(), shape.GetHeightAttr().Get()
            angles = np.arange(32)*2.*math.pi/32
            # Circumscribe the cylinder for conservative obstacle coverage.
            radius /= math.cos(math.pi/32)
            points = np.array([(radius*math.cos(a), radius*math.sin(a), z)
                               for z in (-height/2., height/2.) for a in angles])
            points = points[:, {'Z': (0,1,2), 'X': (2,0,1), 'Y': (0,2,1)}[shape.GetAxisAttr().Get()]]
            faces = []
            for i in range(32):
                j = (i+1)%32
                faces.extend(((i,j,j+32),(i,j+32,i+32)))
            faces.extend((0,i+1,i) for i in range(1,31))
            faces.extend((32,32+i,33+i) for i in range(1,31))
            indices = np.asarray(faces, dtype=int)
        else:
            raise ValueError(f'Unsupported shipyard collider: {prim.GetPath()} ({prim.GetTypeName()})')
        world = np.c_[points, np.ones(len(points))] @ np.asarray(cache.GetLocalToWorldTransform(prim))
        triangles.extend(world[indices, :3])
    result = np.asarray(triangles)
    if result.ndim != 3 or not np.isfinite(result).all():
        raise ValueError('Invalid or missing world collision geometry.')
    return result, paths


def clip_height(vertices, lower, upper):
    polygon = [np.asarray(v, dtype=float) for v in vertices]
    for level, sign in ((lower, 1), (upper, -1)):
        output = []
        for a, b in zip(polygon, polygon[1:]+polygon[:1]):
            inside_a, inside_b = sign*(a[2]-level) >= 0., sign*(b[2]-level) >= 0.
            if inside_a:
                output.append(a)
            if inside_a != inside_b:
                output.append(a+(b-a)*(level-a[2])/(b[2]-a[2]))
        polygon = output
        if not polygon:
            break
    return polygon


def tile(vertices, bounds, resolution, shape):
    xy = np.asarray(vertices)[:, :2]
    low = np.floor((xy.min(axis=0)-bounds[:2])/resolution).astype(int)-1
    high = np.floor((xy.max(axis=0)-bounds[:2])/resolution).astype(int)+1
    c0, r0 = np.maximum(low, (0,0))
    c1, r1 = np.minimum(high, (shape[1]-1,shape[0]-1))
    if c0 > c1 or r0 > r1:
        return None
    x = bounds[0]+(np.arange(c0,c1+1)+.5)*resolution
    y = bounds[1]+(np.arange(r0,r1+1)+.5)*resolution
    return (slice(r0,r1+1),slice(c0,c1+1)), x[None,:], y[:,None]


def polygon_cells(vertices, x, y, resolution):
    """Convex polygon versus cell squares, including degenerate vertical faces."""
    polygon = np.asarray(vertices)[:, :2]
    axes = [(1.,0.),(0.,1.)]
    for a,b in zip(polygon, np.roll(polygon,-1,axis=0)):
        delta = b-a
        if np.linalg.norm(delta) > 1e-12:
            axes.append((-delta[1],delta[0]))
    hit = np.ones((y.shape[0], x.shape[1]), dtype=bool)
    for nx,ny in axes:
        projection = polygon[:,0]*nx+polygon[:,1]*ny
        center, half = x*nx+y*ny, resolution/2.*(abs(nx)+abs(ny))
        hit &= (center+half >= projection.min()-1e-9)&(center-half <= projection.max()+1e-9)
    return hit


def rasterize(triangles, bounds, resolution, seed):
    bounds = np.asarray(bounds, dtype=float)
    width, height = np.rint((bounds[2:]-bounds[:2])/resolution).astype(int)
    support = np.zeros((height,width), dtype=np.uint8)
    obstacles = np.zeros_like(support, dtype=bool)
    samples = ((0.,0.),(-.5,-.5),(-.5,.5),(.5,-.5),(.5,.5))
    used = 0
    for triangle in triangles:
        if (triangle[:,0].max() < bounds[0] or triangle[:,0].min() > bounds[2]
                or triangle[:,1].max() < bounds[1] or triangle[:,1].min() > bounds[3]):
            continue
        normal = np.cross(triangle[1]-triangle[0], triangle[2]-triangle[0])
        norm = np.linalg.norm(normal)
        if (norm > 1e-12 and abs(normal[2])/norm >= math.cos(math.radians(2.))
                and triangle[:,2].min() <= .02 and triangle[:,2].max() >= -.02):
            entry = tile(triangle,bounds,resolution,support.shape)
            if entry is not None:
                region,x,y = entry
                a,b,c = triangle
                u,v = b[:2]-a[:2], c[:2]-a[:2]
                determinant = u[0]*v[1]-u[1]*v[0]
                for index,(dx,dy) in enumerate(samples):
                    px,py = x+dx*resolution-a[0], y+dy*resolution-a[1]
                    beta = (px*v[1]-py*v[0])/determinant
                    gamma = (u[0]*py-u[1]*px)/determinant
                    z = a[2]+beta*(b[2]-a[2])+gamma*(c[2]-a[2])
                    hit = (beta>=-1e-7)&(gamma>=-1e-7)&(beta+gamma<=1.+1e-7)&(abs(z)<=.02)
                    support[region] |= hit.astype(np.uint8)*(1<<index)
        if triangle[:,2].max() >= .03 and triangle[:,2].min() <= .8:
            polygon = clip_height(triangle,.03,.8)
            if polygon:
                entry = tile(polygon,bounds,resolution,support.shape)
                if entry is not None:
                    region,x,y = entry
                    obstacles[region] |= polygon_cells(polygon,x,y,resolution)
        used += 1
        if used%10000 == 0:
            print(f'HUNTER_MAP_PROGRESS triangles={used}', flush=True)
    grid = np.full((height,width),205,dtype=np.uint8)
    grid[support==31] = 254
    grid[obstacles] = 0
    start_col, start_row = np.floor((np.asarray(seed)-bounds[:2])/resolution).astype(int)
    if not (0 <= start_row < height and 0 <= start_col < width and grid[start_row,start_col] == 254):
        raise ValueError('Shipyard spawn has no clear, level support in the generated map.')
    reached = np.zeros_like(obstacles)
    queue = deque([(start_row,start_col)])
    reached[start_row,start_col] = True
    while queue:
        row,col = queue.popleft()
        for r,c in ((row-1,col),(row+1,col),(row,col-1),(row,col+1)):
            if 0 <= r < height and 0 <= c < width and not reached[r,c] and grid[r,c] == 254:
                reached[r,c] = True
                queue.append((r,c))
    grid[(grid==254)&~reached] = 205
    return grid


def export(root, profile):
    from pxr import Usd, UsdGeom
    root = Path(root)
    if profile.usd_path is None or profile.map_file is None:
        raise ValueError('Map export requires the shipyard environment.')
    started = time.monotonic()
    stage = Usd.Stage.Open(str(profile.usd_path))
    if UsdGeom.GetStageUpAxis(stage) != 'Z' or UsdGeom.GetStageMetersPerUnit(stage) != 1.:
        raise ValueError('Map export requires metre/Z-up geometry.')
    errors = [str(error) for prim in stage.Traverse() for error in prim.GetPrimIndex().localErrors]
    if errors:
        raise ValueError(f'USD composition errors: {errors[:5]}')
    warehouse = stage.GetPrimAtPath('/World/warehouse_reference')
    if warehouse and warehouse.IsActive():
        raise ValueError('Hunter map export must exclude the remote warehouse.')
    triangles, colliders = collider_triangles(stage)
    print(f'HUNTER_MAP_GEOMETRY colliders={len(colliders)} triangles={len(triangles)}', flush=True)
    grid = rasterize(triangles,profile.bounds,profile.resolution,profile.position[:2])
    path = profile.map_file
    path.parent.mkdir(parents=True,exist_ok=True)
    image = path.with_suffix('.pgm')
    content = f'P5\n{grid.shape[1]} {grid.shape[0]}\n255\n'.encode()+grid[::-1].tobytes()
    image.write_bytes(content)
    path.write_text(yaml.safe_dump(dict(image=image.name, resolution=profile.resolution,
        origin=[*profile.bounds[:2],0.], negate=0, occupied_thresh=.65, free_thresh=.196), sort_keys=False))
    sources = {Path(layer.realPath) for layer in stage.GetUsedLayers() if layer.realPath}
    sources.add(root/'config/environments'/f'{profile.name}.yaml')
    sources.add(Path(__file__).resolve())
    metadata = dict(environment=profile.name, generator='hunter/environments/raster.py',
        resolution=profile.resolution, bounds=profile.bounds, support_height_band=[-.02,.02],
        support_max_slope_deg=2., obstacle_height_band=[.03,.8], colliders=len(colliders),
        cells={name:int((grid==value).sum()) for name,value in [('free',254),('occupied',0),('unknown',205)]},
        image_sha256=hashlib.sha256(content).hexdigest(),
        sources=[dict(path=os.path.relpath(asset,root),sha256=hashlib.sha256(asset.read_bytes()).hexdigest())
                 for asset in sorted(sources)], flat_ground_only=True)
    path.with_suffix('.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print('HUNTER_MAP_READY '+json.dumps(dict(map=str(path),elapsed_wall_s=time.monotonic()-started,
                                             cells=metadata['cells'])),flush=True)
    return metadata
