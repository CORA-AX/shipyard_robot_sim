#!/usr/bin/env python3
"""Structural URDF checks and physical review findings; no simulator required."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path, PurePosixPath
import struct
import sys
import xml.etree.ElementTree as ET

from prepare_hunter2_asset import ASSET, VENDOR, STEERING, STEERING_LINKS, WHEELS, config_text, derived_bytes, parameters, sha256, verify_sources


def require(condition, message):
    if not condition:
        raise ValueError(message)


def vector(text):
    result = [float(v) for v in text.split()]
    require(len(result) == 3 and all(math.isfinite(v) for v in result), f'Invalid vector: {text}')
    return result


def rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]


def matmul(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def apply(a, v):
    return [sum(row[k]*v[k] for k in range(3)) for row in a]


def unit(v):
    length = math.sqrt(sum(x*x for x in v))
    require(length > 0, 'Zero rotation axis')
    return [x/length for x in v]


def axis_rotation(axis, angle):
    x,y,z = unit(axis)
    c,s = math.cos(angle),math.sin(angle)
    t = 1-c
    return [[c+t*x*x,t*x*y-s*z,t*x*z+s*y],
            [t*x*y+s*z,c+t*y*y,t*y*z-s*x],
            [t*x*z-s*y,t*y*z+s*x,c+t*z*z]]


def check_steering_motion(edges, by_joint):
    """Independent joint probes with ancestor FK; no controller/mimic simulation."""
    def axes_at(positions):
        axes = {}
        def walk(link, parent_rotation):
            for child,joint in edges[link]:
                o = joint.find('origin')
                rpy = vector(o.get('rpy','0 0 0')) if o is not None else [0,0,0]
                rot = matmul(parent_rotation, rotation(rpy))
                name = joint.get('name')
                if joint.get('type') != 'fixed':
                    axis = unit(vector(joint.find('axis').get('xyz')))
                    axes[name] = apply(rot,axis)
                    rot = matmul(rot,axis_rotation(axis,positions.get(name,0.0)))
                walk(child,rot)
        walk('base_link',[[1,0,0],[0,1,0],[0,0,1]])
        return axes
    zero = axes_at({})
    for name in STEERING:
        require(abs(zero[name][2]-1.0) < 1e-6,
                f'{name}: steering axis must point up in base frame')
    for name in WHEELS:
        require(abs(zero[name][0]) < 1e-5 and abs(zero[name][2]) < 1e-5,
                f'{name}: wheel axle must be transverse at zero')
    probes = {}
    for index,name in enumerate(STEERING):
        limit = by_joint[name].find('limit')
        qmax = math.radians(10)
        require(float(limit.get('lower')) <= -qmax and float(limit.get('upper')) >= qmax,
                f'{name}: range must support +/-10 degree diagnostic probe')
        samples = []
        for q in (-qmax,0.0,qmax):
            axes = axes_at({name:q})
            headings = []
            for wheel in WHEELS[:2]:
                a = axes[wheel]
                d = [a[1],-a[0]]  # axle cross ground normal
                require(math.hypot(*d) > .99, f'{wheel}: wheel tilts out of rolling plane')
                if d[0] < 0: d = [-v for v in d]
                headings.append(math.atan2(d[1],d[0]))
            require(abs(headings[index]-q) < 1e-6,
                    f'{name}: rolling heading does not follow steering angle')
            require(abs(headings[1-index]) < 1e-6,
                    f'{name}: independent steering unexpectedly moves other wheel')
            samples.append({'joint_angle_rad':q,'rolling_heading_rad':headings[index],
                            'other_wheel_heading_rad':headings[1-index],
                            'wheel_axle_in_base':axes[WHEELS[index]]})
        probes[name] = samples
    return probes


def mesh_bounds(path):
    data = path.read_bytes()
    require(len(data) >= 84, f'Truncated STL: {path}')
    count = struct.unpack_from('<I', data, 80)[0]
    require(count > 0 and len(data) == 84 + count*50, f'Invalid binary STL: {path}')
    lower, upper = [math.inf]*3, [-math.inf]*3
    for triangle in struct.iter_unpack('<12fH', data[84:]):
        for offset in (3, 6, 9):
            for i in range(3):
                v = triangle[offset+i]
                require(math.isfinite(v), f'Nonfinite STL vertex: {path}')
                lower[i], upper[i] = min(lower[i], v), max(upper[i], v)
    return {'triangles': count, 'local_min': lower, 'local_max': upper}


def validate(path, check_provenance=False):
    path = Path(path).resolve()
    root = ET.parse(path).getroot()
    require(root.tag == 'robot', 'Expected robot root')
    links, joints = root.findall('link'), root.findall('joint')
    for label, elements in [('link', links), ('joint', joints)]:
        names = [e.get('name') for e in elements]
        require(all(names), f'Unnamed {label}')
        require(all(n == 1 for n in Counter(names).values()), f'Duplicate {label}')
    link_names = {e.get('name') for e in links}
    by_joint = {e.get('name'): e for e in joints}
    by_link = {e.get('name'): e for e in links}
    require('base_link' in link_names, 'Missing base_link')
    children, edges = set(), {name: [] for name in link_names}
    for joint in joints:
        name = joint.get('name')
        parent, child = joint.find('parent'), joint.find('child')
        require(parent is not None and child is not None, f'{name}: missing parent/child')
        a, b = parent.get('link'), child.get('link')
        require(a in link_names and b in link_names, f'{name}: unknown parent/child link')
        require(b not in children, f'{b}: multiple parents')
        children.add(b)
        edges[a].append((b, joint))
        require(joint.get('type') in ('fixed', 'continuous', 'revolute'), f'{name}: unsupported type')
        if joint.get('type') != 'fixed':
            axis = joint.find('axis')
            require(axis is not None, f'{name}: missing axis')
            require(sum(v*v for v in vector(axis.get('xyz', ''))) > 0, f'{name}: zero axis')
        if joint.get('type') == 'revolute':
            limit = joint.find('limit')
            require(limit is not None, f'{name}: missing limit')
            values = {k: float(limit.get(k, 'nan')) for k in ('lower', 'upper', 'effort', 'velocity')}
            require(all(math.isfinite(v) for v in values.values()), f'{name}: nonfinite/missing limit')
            require(values['lower'] < values['upper'], f'{name}: locked/reversed range')
            require(values['effort'] > 0 and values['velocity'] > 0, f'{name}: nonpositive effort/velocity')
    require(link_names-children == {'base_link'}, 'Expected one root: base_link')
    for name in STEERING:
        require(name in by_joint and by_joint[name].get('type') == 'revolute', f'{name}: expected revolute')
    for name in WHEELS:
        require(name in by_joint and by_joint[name].get('type') in ('continuous', 'revolute'), f'{name}: invalid wheel joint')
    visited, details = set(), []
    identity = [[1,0,0],[0,1,0],[0,0,1]]

    def walk(link, parent_rotation, parent_position):
        require(link not in visited, 'Cycle in joint graph')
        visited.add(link)
        for child, joint in edges[link]:
            origin = joint.find('origin')
            xyz = vector(origin.get('xyz', '0 0 0')) if origin is not None else [0,0,0]
            rpy = vector(origin.get('rpy', '0 0 0')) if origin is not None else [0,0,0]
            rot = matmul(parent_rotation, rotation(rpy))
            pos = [a+b for a,b in zip(parent_position, apply(parent_rotation, xyz))]
            axis = joint.find('axis')
            local = vector(axis.get('xyz')) if axis is not None else [0,0,0]
            details.append({'name': joint.get('name'), 'type': joint.get('type'), 'parent': link,
                            'child': child, 'axis_local': local, 'origin_rpy': rpy,
                            'origin_in_base': pos, 'axis_in_base_at_zero': apply(rot, local)})
            walk(child, rot, pos)
    walk('base_link', identity, [0,0,0])
    require(visited == link_names, 'Disconnected links/cycle')
    for index,name in enumerate(STEERING):
        joint = by_joint[name]
        link_name = STEERING_LINKS[index]
        require(joint.find('parent').get('link') == 'base_link' and
                joint.find('child').get('link') == link_name,
                f'{name}: unexpected steering topology')
        wheel = by_joint[WHEELS[index]]
        require(wheel.find('parent').get('link') == link_name,
                f'{WHEELS[index]}: front wheel must follow steering link')
        require(not by_link[link_name].findall('visual') and not by_link[link_name].findall('collision'),
                f'{link_name}: steering geometry must be absent (duplicate wheel ownership)')
    for name in STEERING+WHEELS:
        require(by_joint[name].find('mimic') is None, f'{name}: independent joint must not mimic')
    wheel_geometry = {}
    for name in WHEELS:
        child = by_joint[name].find('child').get('link')
        expected = f'meshes/{child}.STL'
        for kind in ('visual','collision'):
            shapes = by_link[child].findall(kind)
            require(len(shapes) == 1, f'{child}: expected one wheel {kind}')
            mesh = shapes[0].find('geometry/mesh')
            require(mesh is not None and mesh.get('filename') == expected,
                    f'{child}: incorrect wheel {kind} ownership')
        wheel_geometry[expected] = child
    for link in links:
        for mesh in link.findall('.//mesh'):
            owner = wheel_geometry.get(mesh.get('filename'))
            require(owner is None or link.get('name') == owner,
                    f'{link.get("name")}: wheel geometry belongs to {owner}')
    probes = check_steering_motion(edges, by_joint)
    for tag in ('gazebo', 'plugin', 'transmission'):
        require(root.find('.//' + tag) is None, f'Unexpected simulator/control tag: {tag}')
    meshes = {}
    refs = root.findall('.//mesh')
    require(refs, 'No meshes')
    for mesh in refs:
        uri = mesh.get('filename', '')
        rel = PurePosixPath(uri)
        require(uri and ':' not in uri and '\\' not in uri and not rel.is_absolute() and '..' not in rel.parts,
                f'Mesh is not a portable relative path: {uri}')
        require(rel.parts[0] == 'meshes', f'Mesh outside meshes/: {uri}')
        resolved = (path.parent / uri).resolve()
        require(resolved.is_relative_to(path.parent / 'meshes'), f'Mesh escapes source directory: {uri}')
        require(resolved.is_file(), f'Missing mesh: {uri}')
        if uri not in meshes:
            meshes[uri] = mesh_bounds(resolved)
    warnings = []
    masses = {}
    for link in links:
        mass = link.find('inertial/mass')
        require(mass is not None, f"{link.get('name')}: missing mass")
        value = float(mass.get('value', 'nan'))
        require(math.isfinite(value) and value >= 0, 'Invalid mass')
        masses[link.get('name')] = value
        if value == 0:
            warnings.append(f"{link.get('name')}: zero mass/inertia retained; review fixed-link handling before physics")
    for name in WHEELS[2:]:
        limit = by_joint[name].find('limit')
        value = float(limit.get('velocity','nan')) if limit is not None else float('nan')
        require(math.isfinite(value) and value > 0, f'{name}: invalid rear velocity limit')
        if value < 1.5/.165:
            warnings.append(f'{name}: velocity limit {value:g} rad/s below nominal straight-speed requirement 9.090909 rad/s.')
    warnings += ['Steering geometry was removed but vendor inertials are retained; mass distribution and omitted non-wheel geometry require review.',
                 'Driver-derived +/-0.58 rad is provisional; manual inner limit 33 degrees differs. Resolve before full-range driving.',
                 'Static independent steering probes do not validate Ackermann control or contact dynamics.',
                 'Relative mesh resolution by the selected Isaac Sim importer remains untested.']
    if check_provenance:
        verify_sources()
        p = parameters()
        original = (VENDOR / 'urdf/hunter2_base.urdf').read_bytes()
        require((ASSET / 'source/original/hunter2_base.urdf').read_bytes() == original, 'Snapshot changed')
        require(path.read_bytes() == derived_bytes(original, p['max_steer_angle']), 'Unexpected derived URDF changes')
        require((ASSET / 'config/hunter2.yaml').read_text() == config_text(p), 'Config differs from reviewed official parameters')
        vendor_meshes = {f.name for f in (VENDOR / 'meshes').glob('*.STL')}
        require({f.name for f in (path.parent / 'meshes').iterdir()} == vendor_meshes, 'Mesh inventory mismatch')
        for name in vendor_meshes:
            require(sha256(path.parent / 'meshes' / name) == sha256(VENDOR / 'meshes' / name), f'Mesh changed: {name}')
    return {'structural_validation': 'pass', 'source_kinematics_validation': 'pass',
            'steering_geometry_ownership': 'pass', 'steering_motion_probes': probes,
            'physics_validated': False, 'isaac_import_validated': False,
            'links': len(links), 'joints': len(joints), 'mesh_references': len(refs),
            'unique_meshes': len(meshes), 'joint_details': details, 'mesh_bounds': meshes,
            'link_masses_kg': masses, 'total_mass_kg': sum(masses.values()), 'warnings': warnings}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('urdf', nargs='?', type=Path, default=ASSET / 'source/hunter2_sim.urdf')
    parser.add_argument('--structural-only', action='store_true', help='Validate a relocated bundle without vendor/config comparisons')
    args = parser.parse_args()
    try:
        print(json.dumps(validate(args.urdf, not args.structural_only), indent=2))
    except (ValueError, OSError, ET.ParseError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        sys.exit(1)
