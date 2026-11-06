#!/usr/bin/env python3
"""Convert the pinned shipyard GLB offline; preserve existing editable USD layers.

Run with .conda/isaacsim-5.0/bin/python. No Kit application or GPU is needed.
This importer deliberately supports the features used by this specific source,
not arbitrary glTF files. Glass transmission is approximated as opaque glass.
"""
import argparse
import hashlib
import io
import json
import os
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[2]
ASSET = ROOT / 'assets/environments/shipyard'


def bootstrap_usd():
    """Load the installed Isaac USD libraries without starting the renderer."""
    cache = Path(sys.prefix) / 'lib/python3.11/site-packages/isaacsim/extscache'
    libraries = sorted(cache.glob('omni.usd.libs-*'))
    if not libraries:
        raise SystemExit('Use the project .conda/isaacsim-5.0/bin/python.')
    library = libraries[0]
    if os.environ.get('SHIPYARD_USD_BOOTSTRAPPED') != str(library):
        env = dict(os.environ, SHIPYARD_USD_BOOTSTRAPPED=str(library))
        env['LD_LIBRARY_PATH'] = str(library / 'bin') + os.pathsep + env.get('LD_LIBRARY_PATH', '')
        env['PYTHONPATH'] = str(library) + os.pathsep + env.get('PYTHONPATH', '')
        os.execve(sys.executable, [sys.executable, *sys.argv], env)


def read_glb(path):
    blob = path.read_bytes()
    magic, version, length = struct.unpack_from('<III', blob)
    if (magic, version, length) != (0x46546C67, 2, len(blob)):
        raise ValueError('Invalid GLB 2 header')
    size, kind = struct.unpack_from('<II', blob, 12)
    assert kind == 0x4E4F534A
    doc = json.loads(blob[20:20 + size])
    start = 20 + size
    size, kind = struct.unpack_from('<II', blob, start)
    assert kind == 0x004E4942
    return doc, blob[start + 8:start + 8 + size]


def accessor(doc, binary, index):
    import numpy as np
    spec = doc['accessors'][index]
    if 'sparse' in spec:
        raise ValueError('Sparse accessors are not supported')
    view = doc['bufferViews'][spec['bufferView']]
    dtype = np.dtype({5120: 'i1', 5121: 'u1', 5122: '<i2', 5123: '<u2',
                      5125: '<u4', 5126: '<f4'}[spec['componentType']])
    width = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4}[spec['type']]
    values = np.ndarray((spec['count'], width), dtype=dtype, buffer=binary,
                        offset=view.get('byteOffset', 0) + spec.get('byteOffset', 0),
                        strides=(view.get('byteStride', width * dtype.itemsize), dtype.itemsize)).copy()
    if spec.get('normalized'):
        values = values.astype(np.float32) / np.iinfo(dtype).max
        if dtype.kind == 'i':
            values = np.maximum(values, -1.0)
    return values


def convert_model(doc, binary):
    import numpy as np
    from PIL import Image
    from pxr import Gf, Sdf, Tf, Usd, UsdGeom, UsdShade
    target = ASSET / 'models/shipyard.usd'
    target.parent.mkdir(parents=True, exist_ok=True)
    # Publish only a completed model so an interrupted build can be rerun safely.
    temporary = target.with_name('shipyard.build.usdc')
    stage = Usd.Stage.CreateNew(str(temporary))
    root = UsdGeom.Xform.Define(stage, '/Shipyard')
    stage.SetDefaultPrim(root.GetPrim())
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    geometry = UsdGeom.Xform.Define(stage, '/Shipyard/Geometry')
    # glTF Y-up -> simulation Z-up: (x, y, z) -> (x, -z, y).
    geometry.AddRotateXOp().Set(90)
    textures = ASSET / 'textures'
    textures.mkdir(exist_ok=True)
    for i, entry in enumerate(doc.get('images', [])):
        view = doc['bufferViews'][entry['bufferView']]
        offset = view.get('byteOffset', 0)
        Image.open(io.BytesIO(binary[offset:offset + view['byteLength']])).save(textures / f'image_{i}.png')
    materials = []
    for i, entry in enumerate(doc['materials']):
        mat = UsdShade.Material.Define(stage, f'/Shipyard/Looks/m{i}_{Tf.MakeValidIdentifier(entry["name"])}')
        shader = UsdShade.Shader.Define(stage, mat.GetPath().AppendChild('Surface'))
        shader.CreateIdAttr('UsdPreviewSurface')
        pbr = entry.get('pbrMetallicRoughness', {})
        color = pbr.get('baseColorFactor', [1, 1, 1, 1])
        shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color[:3]))
        shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(pbr.get('metallicFactor', 1))
        shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(pbr.get('roughnessFactor', 1))
        if entry.get('alphaMode', 'OPAQUE') != 'OPAQUE':
            raise ValueError('Unexpected alpha material')
        if 'baseColorTexture' in pbr:
            texture_info = pbr['baseColorTexture']
            assert texture_info.get('texCoord', 0) == 0
            texture = doc['textures'][texture_info['index']]
            source = texture.get('source', texture.get('extensions', {}).get('EXT_texture_webp', {}).get('source'))
            reader = UsdShade.Shader.Define(stage, mat.GetPath().AppendChild('UV'))
            reader.CreateIdAttr('UsdPrimvarReader_float2')
            reader.CreateInput('varname', Sdf.ValueTypeNames.Token).Set('st')
            reader.CreateOutput('result', Sdf.ValueTypeNames.Float2)
            tex = UsdShade.Shader.Define(stage, mat.GetPath().AppendChild('Texture'))
            tex.CreateIdAttr('UsdUVTexture')
            tex.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(f'../textures/image_{source}.png')
            tex.CreateInput('sourceColorSpace', Sdf.ValueTypeNames.Token).Set('sRGB')
            tex.CreateInput('wrapS', Sdf.ValueTypeNames.Token).Set('repeat')
            tex.CreateInput('wrapT', Sdf.ValueTypeNames.Token).Set('repeat')
            tex.CreateInput('scale', Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*color))
            tex.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), 'result')
            tex.CreateOutput('rgb', Sdf.ValueTypeNames.Float3)
            shader.GetInput('diffuseColor').ConnectToSource(tex.ConnectableAPI(), 'rgb')
        shader.CreateOutput('surface', Sdf.ValueTypeNames.Token)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
        materials.append(mat)

    mesh_count = 0
    def node(index, parent):
        nonlocal mesh_count
        entry = doc['nodes'][index]
        path = parent.AppendChild(f'n{index:04d}_{Tf.MakeValidIdentifier(entry.get("name", "node"))}')
        xform = UsdGeom.Xform.Define(stage, path)
        xform.GetPrim().SetDisplayName(entry.get('name', str(index)))
        if 'matrix' in entry:
            xform.AddTransformOp().Set(Gf.Matrix4d(*entry['matrix']))
        else:
            xform.AddTranslateOp().Set(Gf.Vec3d(*entry.get('translation', [0, 0, 0])))
            q = entry.get('rotation', [0, 0, 0, 1])
            xform.AddOrientOp().Set(Gf.Quatf(q[3], Gf.Vec3f(*q[:3])))
            xform.AddScaleOp().Set(Gf.Vec3f(*entry.get('scale', [1, 1, 1])))
        if 'mesh' in entry:
            for part, primitive in enumerate(doc['meshes'][entry['mesh']]['primitives']):
                if primitive.get('mode', 4) != 4:
                    raise ValueError('Only triangles are supported')
                attrs = primitive['attributes']
                points = accessor(doc, binary, attrs['POSITION']).astype(np.float32)
                indices = accessor(doc, binary, primitive['indices']).ravel().astype(np.int32)
                assert len(indices) % 3 == 0 and indices.min() >= 0 and indices.max() < len(points)
                mesh = UsdGeom.Mesh.Define(stage, path.AppendChild(f'mesh_{part}'))
                mesh.CreatePointsAttr(points)
                mesh.CreateFaceVertexIndicesAttr(indices)
                mesh.CreateFaceVertexCountsAttr([3] * (len(indices) // 3))
                mesh.CreateSubdivisionSchemeAttr('none')
                mesh.CreateExtentAttr([Gf.Vec3f(*points.min(axis=0).tolist()), Gf.Vec3f(*points.max(axis=0).tolist())])
                if 'NORMAL' in attrs:
                    normals = accessor(doc, binary, attrs['NORMAL']).astype(np.float32)
                    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
                    mesh.CreateNormalsAttr(normals)
                    mesh.SetNormalsInterpolation('vertex')
                if 'TEXCOORD_0' in attrs:
                    uv = accessor(doc, binary, attrs['TEXCOORD_0']).astype(np.float32)
                    uv[:, 1] = 1 - uv[:, 1]
                    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray, 'vertex').Set(uv)
                UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(materials[primitive['material']])
                mesh_count += 1
        for child in entry.get('children', []):
            node(child, path)
    for index in doc['scenes'][doc.get('scene', 0)]['nodes']:
        node(index, geometry.GetPath())
    stage.GetRootLayer().Save()
    temporary.replace(target)
    print(f'Converted {mesh_count} meshes: {target}')


def create_layers():
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade
    layout_path = ASSET / 'layout.usda'
    if not layout_path.exists():
        stage = Usd.Stage.CreateNew(str(layout_path))
        root = UsdGeom.Xform.Define(stage, '/World')
        stage.SetDefaultPrim(root.GetPrim())
        UsdGeom.SetStageUpAxis(stage, 'Z')
        UsdGeom.SetStageMetersPerUnit(stage, 1)
        shipyard = UsdGeom.Xform.Define(stage, '/World/Shipyard')
        shipyard.GetPrim().GetReferences().AddReference('./models/shipyard.usd')
        shipyard.AddTranslateOp().Set(Gf.Vec3d(0))
        shipyard.AddRotateXYZOp().Set(Gf.Vec3f(0))
        shipyard.AddScaleOp().Set(Gf.Vec3f(1))
        light = UsdLux.DomeLight.Define(stage, '/World/SkyLight')
        light.CreateIntensityAttr(750)
        camera = UsdGeom.Camera.Define(stage, '/World/Overview')
        view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(48, 58, 42), Gf.Vec3d(2, 0, 0), Gf.Vec3d(0, 0, 1))
        camera.AddTransformOp().Set(view.GetInverse())
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.1, 1000))
        camera.CreateFocalLengthAttr(24)
        stage.GetRootLayer().Save()
    physics_path = ASSET / 'physics.usda'
    if not physics_path.exists():
        stage = Usd.Stage.Open(str(layout_path))
        layer = Sdf.Layer.CreateNew(str(physics_path))
        stage.GetRootLayer().subLayerPaths.append('./physics.usda')
        stage.SetEditTarget(layer)
        material = UsdShade.Material.Define(stage, '/World/PhysicsMaterial')
        physical = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physical.CreateStaticFrictionAttr(1)
        physical.CreateDynamicFrictionAttr(1)
        physical.CreateRestitutionAttr(0)
        count = 0
        for prim in stage.Traverse():
            if prim.IsA(UsdGeom.Mesh) and 'basin_water' not in str(prim.GetPath()):
                UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('none')
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose='physics')
                count += 1
        layer.Save()
        # The temporary sublayer link is intentionally not saved in layout.
        print(f'Created {count} static triangle-mesh colliders (water excluded)')
    world_path = ASSET / 'world.usda'
    if not world_path.exists():
        stage = Usd.Stage.CreateNew(str(world_path))
        stage.GetRootLayer().subLayerPaths = ['./layout.usda', './physics.usda']
        stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
        UsdGeom.SetStageUpAxis(stage, 'Z')
        UsdGeom.SetStageMetersPerUnit(stage, 1)
        stage.GetRootLayer().Save()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rebuild-model', action='store_true', help='Replace generated model only; preserve layout/physics/world.')
    args = parser.parse_args()
    bootstrap_usd()
    source = ASSET / 'source/model.glb'
    lock = json.loads((ASSET / 'sources.lock.json').read_text())
    if hashlib.sha256(source.read_bytes()).hexdigest() != lock['sha256']:
        raise SystemExit('Source hash changed; review the new asset before updating the lock.')
    if args.rebuild_model or not (ASSET / 'models/shipyard.usd').exists():
        convert_model(*read_glb(source))
    create_layers()


if __name__ == '__main__':
    main()
