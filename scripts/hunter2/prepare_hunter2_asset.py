#!/usr/bin/env python3
"""Reproduce the reviewed Hunter asset offline, using only Python's stdlib."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
ASSET = ROOT / 'assets/hunter2'
VENDOR = ROOT / 'third_party/ugv_gazebo_sim/hunter/hunter2_base'
STEERING = ('front_steer_left_joint', 'front_steer_right_joint')
STEERING_LINKS = ('front_steer_left_link', 'front_steer_right_link')
WHEELS = ('front_left_wheel_joint', 'front_right_wheel_joint', 'left_rear_joint', 'right_rear_joint')
PREFIX = b'package://hunter2_base/'


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_sources():
    lock = json.loads((ASSET / 'config/sources.lock.json').read_text())
    for source in lock['sources']:
        repo = ROOT / source['path']
        commit = subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip()
        if commit != source['commit']:
            raise ValueError(f"Unreviewed vendor commit: {repo}: {commit}")
        for relative, expected in source['files'].items():
            if sha256(repo / relative) != expected:
                raise ValueError(f'Vendor input changed: {repo / relative}')
    return lock


def parameters():
    header = (ROOT / 'third_party/hunter_ros2_reference/hunter_base/include/hunter_base/hunter_params.hpp').read_text()
    body = re.search(r'struct HunterV2Params\s*\{(.*?)\};', header, re.S).group(1)
    body = re.sub(r'//[^\n]*', '', body)
    values = {k: float(v) for k, v in re.findall(r'static constexpr double\s+(\w+)\s*=\s*([\d.]+)\s*;', body)}
    expected = dict(track=0.605, wheelbase=0.650, wheel_radius=0.165,
                    max_steer_angle=0.58, max_steer_angle_central=0.461, max_linear_speed=1.5)
    if any(values.get(k) != v for k, v in expected.items()):
        raise ValueError('HunterV2Params changed; review sources before regenerating')
    return values


def derived_bytes(original, angle):
    """Reviewed edits only: URIs, steering limits/axes, steering geometry removal.

    Keep the vendor bytes/CRLF for everything else, including inertials and
    joint origins. Removed geometry stays available in the immutable meshes.
    """
    if original.count(PREFIX) != 18:
        raise ValueError('Unexpected package reference count')
    result = original.replace(PREFIX, b'')
    for name in STEERING:
        pattern = rb'<joint\s+name="' + name.encode() + rb'"\s+type="revolute">.*?</joint>'
        matches = list(re.finditer(pattern, result, re.S))
        if len(matches) != 1:
            raise ValueError(f'Unexpected steering joint: {name}')
        match = matches[0]
        block = match.group()
        block, count = re.subn(rb'(<axis\s+xyz=")0 0 -?1("\s*/>)',
                               rb'\g<1>0 1 0\2', block)
        if count != 1:
            raise ValueError(f'{name}: unexpected original steering axis')
        for key, value in (('lower', -angle), ('upper', angle)):
            block, count = re.subn(key.encode() + rb'="0"', f'{key}="{value:g}"'.encode(), block)
            if count != 1:
                raise ValueError(f'{name}: unexpected {key} limit')
        result = result[:match.start()] + block + result[match.end():]
    for name in STEERING_LINKS:
        pattern = rb'<link\s+name="' + name.encode() + rb'">.*?</link>'
        matches = list(re.finditer(pattern, result, re.S))
        if len(matches) != 1:
            raise ValueError(f'Unexpected steering link: {name}')
        match = matches[0]
        block = match.group()
        for tag in (b'visual', b'collision'):
            block, count = re.subn(rb'^[ \t]*<' + tag + rb'>.*?</' + tag + rb'>\r?\n',
                                   b'', block, flags=re.S | re.M)
            if count != 1:
                raise ValueError(f'{name}: expected one original {tag.decode()}')
        result = result[:match.start()] + block + result[match.end():]
    return result


def config_text(p):
    return f'''# SI units. Nominal official driver values, NOT measurements of this URDF.
# Evidence: sources.lock.json, HunterV2Params; conflicts and TODOs in ../README.md.
robot:
  name: hunter2
  type: ackermann
geometry:
  wheelbase: {p['wheelbase']:.3f}
  track_width: {p['track']:.3f}
  wheel_radius: {p['wheel_radius']:.3f}
limits:
  max_linear_velocity: {p['max_linear_speed']:g}
  max_wheel_steer_angle: {p['max_steer_angle']:g}
  max_center_steer_angle: {p['max_steer_angle_central']:g}
joints:
  steering:
    left: {STEERING[0]}
    right: {STEERING[1]}
  wheels:
    front_left: {WHEELS[0]}
    front_right: {WHEELS[1]}
    rear_left: {WHEELS[2]}
    rear_right: {WHEELS[3]}
coordinate:
  # Steering yaw checked mathematically; Isaac import is still untested.
  forward: "+X"
  left: "+Y"
  up: "+Z"
status:
  source_prepared: true
  steering_axis_corrected: true
  steering_geometry_removed: true
  source_kinematics_validated: true
  physics_validated: false
  isaac_import_validated: false
  mass_inertia_calibrated: false
  # Retain driver-derived +/-0.58 for this diagnostic source revision.
  # Manual inner limit is 33 deg (~0.575959); reconcile before full-range driving.
  steering_limit_policy: provisional_driver_bound
'''


def prepare():
    verify_sources()
    p = parameters()
    original = (VENDOR / 'urdf/hunter2_base.urdf').read_bytes()
    derived = derived_bytes(original, p['max_steer_angle'])
    source = ASSET / 'source'
    snapshot = source / 'original/hunter2_base.urdf'
    # An existing snapshot is immutable, including when vendor input changes.
    if snapshot.exists() and snapshot.read_bytes() != original:
        raise ValueError('Existing original snapshot differs; refusing to overwrite')
    meshes = sorted((VENDOR / 'meshes').glob('*.STL'))
    # Preflight before writing: do not overwrite unrecognized mesh edits.
    for mesh in meshes:
        target = source / 'meshes' / mesh.name
        if target.exists() and sha256(target) != sha256(mesh):
            raise ValueError(f'Copied mesh was modified: {target}')
    snapshot.parent.mkdir(parents=True, exist_ok=True)
    (source / 'meshes').mkdir(parents=True, exist_ok=True)
    if not snapshot.exists():
        snapshot.write_bytes(original)
    for mesh in meshes:
        target = source / 'meshes' / mesh.name
        if not target.exists():
            shutil.copyfile(mesh, target)
    (source / 'hunter2_sim.urdf').write_bytes(derived)
    (ASSET / 'config/hunter2.yaml').write_text(config_text(p), encoding='utf-8')
    from validate_hunter2_asset import validate
    report = validate(source / 'hunter2_sim.urdf', check_provenance=True)
    (ASSET / 'config/validation_report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(f"PASS: {report['links']} links, {report['joints']} joints, {report['mesh_references']} mesh references")
    for warning in report['warnings']:
        print(f'REVIEW: {warning}')


if __name__ == '__main__':
    argparse.ArgumentParser(description=__doc__).parse_args()
    try:
        prepare()
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        sys.exit(1)
