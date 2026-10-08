#!/usr/bin/env python3
"""Read-only kinematic review. Prints JSON; never imports a simulator or writes URDF/USD."""
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'assets/hunter2/source'
VENDOR = ROOT / 'third_party/ugv_gazebo_sim/hunter/hunter2_base'


def vec(text):
    return tuple(map(float, text.split()))


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def rotate(v, axis, angle):
    n = math.sqrt(dot(axis, axis))
    axis = tuple(x/n for x in axis)
    c, s = math.cos(angle), math.sin(angle)
    return tuple(v[i]*c + cross(axis, v)[i]*s + axis[i]*dot(axis, v)*(1-c) for i in range(3))


def origin_rotate(v, joint):
    r, p, y = vec(joint.find('origin').get('rpy'))
    return rotate(rotate(rotate(v, (1,0,0), r), (0,1,0), p), (0,0,1), y)


def steering_probe(root, side, angles):
    steer = root.find(f"joint[@name='front_steer_{side}_joint']")
    wheel = root.find(f"joint[@name='front_{side}_wheel_joint']")
    local_axis = vec(steer.find('axis').get('xyz'))
    axle_at_zero = origin_rotate(vec(wheel.find('axis').get('xyz')), wheel)
    result = {'axis_local': local_axis, 'axis_in_base': origin_rotate(local_axis, steer), 'samples': []}
    for q in angles:
        axle = origin_rotate(rotate(axle_at_zero, local_axis, q), steer)
        # Wheel-plane / ground-plane intersection, sign selected toward base +X.
        rolling = cross(axle, (0,0,1))
        if rolling[0] < 0:
            rolling = tuple(-v for v in rolling)
        heading = math.atan2(rolling[1], rolling[0])
        result['samples'].append({'joint_angle_rad': q, 'wheel_axle_in_base': axle,
                                  'rolling_heading_rad': heading})
    return result


def triangles(path):
    data = path.read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    if len(data) != 84 + 50*count:
        raise ValueError(f'Invalid binary STL: {path}')
    # Quantized to 1 micrometre assuming the URDF metre scale; normals ignored.
    return Counter(tuple(sorted(tuple(round(t[i+j], 6) for j in range(3))
                                for i in (3,6,9)))
                   for t in struct.iter_unpack('<12fH', data[84:]))


def main():
    paths = {'original': VENDOR / 'urdf/hunter2_base.urdf',
             'current': SOURCE / 'hunter2_sim.urdf',
             'gazebo': VENDOR / 'urdf/hunter2_base_gazebo.xacro'}
    roots = {k: ET.parse(p).getroot() for k, p in paths.items()}
    angles = [-0.58, -0.3, 0.0, 0.3, 0.58]
    probes = {k: {s: steering_probe(r, s, angles) for s in ('left', 'right')}
              for k, r in roots.items()}
    # Original probes outside zero are hypothetical: its limits forbid those positions.
    for side in ('left', 'right'):
        assert all(abs(s['rolling_heading_rad']) < 1e-9 for s in probes['original'][side]['samples'])
        assert all(abs(s['rolling_heading_rad']-s['joint_angle_rad']) < 1e-9
                   for s in probes['current'][side]['samples'])
        assert all(abs(s['rolling_heading_rad']-s['joint_angle_rad']) < 1e-9
                   for s in probes['gazebo'][side]['samples'])
    overlap = {}
    for side in ('left', 'right'):
        wheel_joint = roots['current'].find(f"joint[@name='front_{side}_wheel_joint']")
        assert vec(wheel_joint.find('origin').get('xyz')) == (0,0,0)
        assert vec(wheel_joint.find('origin').get('rpy')) == (0,0,0)
        # Raw vendor meshes are still on disk. Analyze their historic overlap
        # separately from active Current geometry, which no longer references S.
        for link in (f'front_steer_{side}_link', f'front_{side}_wheel_link'):
            for kind in ('visual', 'collision'):
                origin = roots['original'].find(f"link[@name='{link}']/{kind}/origin")
                assert vec(origin.get('xyz')) == (0,0,0) and vec(origin.get('rpy')) == (0,0,0)
        a = triangles(SOURCE / f'meshes/front_steer_{side}_link.STL')
        b = triangles(SOURCE / f'meshes/front_{side}_wheel_link.STL')
        steering_link = roots['current'].find(f"link[@name='front_steer_{side}_link']")
        assert not steering_link.findall('visual') and not steering_link.findall('collision')
        overlap[side] = {'original_steering_mesh_triangles': sum(a.values()), 'wheel_mesh_triangles': sum(b.values()),
                         'original_shared_triangles_at_zero_rounded_1um': sum((a & b).values()),
                         'current_steering_geometry_count': 0, 'current_shared_triangles': 0}
    masses = {k: sum(float(e.get('value')) for e in r.findall('link/inertial/mass')) for k,r in roots.items()}
    L, T, center, wheel_limit = 0.650, 0.605, 0.461, 0.58
    inner = math.atan(L/(L/math.tan(center)-T/2))
    assert inner < wheel_limit
    print(json.dumps({'scope': 'static source review, no USD import or physics execution',
                      'source_sha256': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths.values()},
                      'original_nonzero_angles_are_hypothetical': True,
                      'kinematic_probes': probes, 'mesh_shared_triangles': overlap,
                      'total_mass_kg': masses, 'nominal_center_0_461_inner_rad': inner,
                      'rear_limit_1_rad_s_nominal_speed_m_s': 0.165,
                      'wheel_speed_for_nominal_1_5_m_s_rad_s': 1.5/0.165,
                      'physics_validated': False}, indent=2))


if __name__ == '__main__':
    main()
