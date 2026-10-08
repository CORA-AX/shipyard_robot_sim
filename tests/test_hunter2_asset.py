"""Exercise asset preservation, relocation and rejection of malformed inputs."""
import copy
import math
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts' / 'hunter2'))
from prepare_hunter2_asset import ASSET, VENDOR, STEERING, STEERING_LINKS, derived_bytes
from validate_hunter2_asset import validate


class AssetChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.bundle = Path(cls.temp.name) / 'relocated'
        cls.bundle.mkdir()
        shutil.copytree(ASSET / 'source/meshes', cls.bundle / 'meshes')
        cls.path = cls.bundle / 'hunter2_sim.urdf'
        cls.good = (ASSET / 'source/hunter2_sim.urdf').read_bytes()

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def setUp(self):
        self.path.write_bytes(self.good)
        self.root = ET.fromstring(self.good)

    def reject(self, message):
        ET.ElementTree(self.root).write(self.path)
        with self.assertRaisesRegex(ValueError, message):
            validate(self.path)

    def test_relocated_bundle_and_physics_findings(self):
        result = validate(self.path)
        self.assertEqual((result['links'], result['joints'], result['unique_meshes']), (9, 8, 7))
        self.assertEqual(result['mesh_references'], 14)
        self.assertFalse(result['physics_validated'])
        self.assertEqual(result['source_kinematics_validation'], 'pass')
        self.assertEqual(result['steering_geometry_ownership'], 'pass')
        for samples in result['steering_motion_probes'].values():
            self.assertEqual(len(samples), 3)
            self.assertAlmostEqual(samples[2]['rolling_heading_rad'], math.radians(10))
            self.assertAlmostEqual(samples[0]['rolling_heading_rad'], -math.radians(10))
            self.assertEqual(samples[2]['other_wheel_heading_rad'], 0)
        self.assertEqual(sum('below nominal' in w for w in result['warnings']), 2)

    def test_snapshot_and_only_allowed_semantic_changes(self):
        original = (VENDOR / 'urdf/hunter2_base.urdf').read_bytes()
        self.assertEqual(original, (ASSET / 'source/original/hunter2_base.urdf').read_bytes())
        before, after = ET.fromstring(original), ET.fromstring(self.good)
        # Remove ONLY the reviewed visual/collision elements from expected XML.
        # All inertials, origins, wheel axes and other content must still match.
        for name in STEERING_LINKS:
            link = before.find(f"link[@name='{name}']")
            for tag in ('visual','collision'):
                self.assertEqual(len(link.findall(tag)), 1)
                link.remove(link.find(tag))
            # The removed blocks leave the closing link indentation after inertial.
            link.find('inertial').tail = '\n  '
        a, b = list(before.iter()), list(after.iter())
        self.assertEqual(len(a), len(b))
        changed = []
        for left, right in zip(a, b):
            self.assertEqual(left.tag, right.tag)
            self.assertEqual(left.text, right.text)
            self.assertEqual(left.tail, right.tail)
            self.assertEqual(set(left.attrib), set(right.attrib))
            for key in left.attrib:
                if left.get(key) != right.get(key):
                    changed.append((left.tag, key, left.get(key), right.get(key)))
        meshes = [c for c in changed if c[:2] == ('mesh', 'filename')]
        self.assertEqual(len(meshes), 14)
        for _, _, old, new in meshes:
            self.assertEqual(old, 'package://hunter2_base/' + new)
        limits = [c for c in changed if c[0] == 'limit']
        self.assertCountEqual(limits, [('limit', 'lower', '0', '-0.58')]*2 + [('limit', 'upper', '0', '0.58')]*2)
        axes = [c for c in changed if c[0] == 'axis']
        self.assertCountEqual(axes, [('axis','xyz','0 0 1','0 1 0'),
                                     ('axis','xyz','0 0 -1','0 1 0')])
        self.assertEqual(len(changed), 20)

    def test_generator_preserves_crlf_and_is_deterministic(self):
        original = (VENDOR / 'urdf/hunter2_base.urdf').read_bytes()
        result = derived_bytes(original, .58)
        self.assertEqual(result, self.good)
        self.assertEqual(result, derived_bytes(original, .58))
        self.assertEqual(result.count(b'\n'), result.count(b'\r\n'))

    def test_original_bad_axis_is_rejected(self):
        self.root.find("joint[@name='front_steer_left_joint']/axis").set('xyz','0 0 1')
        self.reject('steering axis must point up')

    def test_wrong_steering_sign_is_rejected(self):
        self.root.find("joint[@name='front_steer_right_joint']/axis").set('xyz','0 -1 0')
        self.reject('steering axis must point up')

    def test_wrong_wheel_axis_is_rejected(self):
        self.root.find("joint[@name='front_left_wheel_joint']/axis").set('xyz','0 1 0')
        self.reject('wheel axle must be transverse')

    def test_wheel_bypassing_steering_is_rejected(self):
        self.root.find("joint[@name='front_left_wheel_joint']/parent").set('link','base_link')
        self.reject('front wheel must follow steering link')

    def test_steering_collider_reintroduced_is_rejected(self):
        collider = self.root.find("link[@name='front_left_wheel_link']/collision")
        self.root.find("link[@name='front_steer_left_link']").append(copy.deepcopy(collider))
        self.reject('steering geometry must be absent')

    def test_missing_wheel_collider_is_rejected(self):
        wheel = self.root.find("link[@name='front_left_wheel_link']")
        wheel.remove(wheel.find('collision'))
        self.reject('expected one wheel collision')

    def test_wheel_collider_on_chassis_is_rejected(self):
        collider = self.root.find("link[@name='front_left_wheel_link']/collision")
        self.root.find("link[@name='base_link']").append(copy.deepcopy(collider))
        self.reject('wheel geometry belongs to')

    def test_mimic_is_rejected(self):
        joint = self.root.find("joint[@name='front_steer_right_joint']")
        ET.SubElement(joint,'mimic',joint='front_steer_left_joint',multiplier='1')
        self.reject('independent joint must not mimic')

    def test_duplicate_link(self):
        self.root.append(copy.deepcopy(self.root.find('link')))
        self.reject('Duplicate link')

    def test_duplicate_joint(self):
        self.root.append(copy.deepcopy(self.root.find('joint')))
        self.reject('Duplicate joint')

    def test_unknown_parent(self):
        self.root.find('joint/parent').set('link', 'missing')
        self.reject('unknown parent/child')

    def test_locked_steering(self):
        limit = self.root.find('joint/limit')
        limit.set('lower', '0')
        limit.set('upper', '0')
        self.reject('locked/reversed')

    def test_wrong_steering_type(self):
        self.root.find('joint').set('type', 'fixed')
        self.reject('expected revolute')

    def test_missing_mesh(self):
        self.root.find('.//mesh').set('filename', 'meshes/missing.STL')
        self.reject('Missing mesh')

    def test_ros_package_uri(self):
        self.root.find('.//mesh').set('filename', 'package://hunter2_base/meshes/base_link.STL')
        self.reject('not a portable relative path')

    def test_path_traversal(self):
        self.root.find('.//mesh').set('filename', 'meshes/../../outside.STL')
        self.reject('not a portable relative path')

    def test_disconnected_cycle(self):
        self.root.find("joint[@name='front_steer_left_joint']/parent").set('link', 'front_left_wheel_link')
        self.reject('Disconnected')

    def test_bad_xml(self):
        self.path.write_text('<robot>')
        with self.assertRaises(ET.ParseError):
            validate(self.path)


if __name__ == '__main__':
    unittest.main()
