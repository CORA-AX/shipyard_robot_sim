"""CPU USD composition checks; no Kit startup, source mutation or GPU needed."""
import copy
import hashlib
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent))
from hunter.control.controller import load_parameters
from hunter.sim.scene import prepare_hunter2_stage

try:
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
except ImportError:
    Usd = None


@unittest.skipIf(Usd is None, 'Requires the project Isaac Python with OpenUSD libraries configured.')
class HunterSceneTests(unittest.TestCase):
    def setUp(self):
        self.parameters = load_parameters(ROOT)
        self.source = ROOT / self.parameters['usd_path']
        self.stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.)
        self.path = '/World/Robots/hunter2_0'
        self.stage.DefinePrim(self.path, 'Xform').GetReferences().AddReference(str(self.source))
        self.source_files = [self.source, *self.source.parent.glob('configuration/*.usd')]
        self.hashes = [hashlib.sha256(p.read_bytes()).hexdigest() for p in self.source_files]

    def tearDown(self):
        self.assertEqual(self.hashes, [hashlib.sha256(p.read_bytes()).hexdigest() for p in self.source_files])

    def test_reference_omits_preview_scene_and_preserves_tuned_drives(self):
        rear = self.stage.GetPrimAtPath(self.path + '/joints/left_rear_joint')
        drive = UsdPhysics.DriveAPI(rear, 'angular')
        before = (drive.GetDampingAttr().Get(), drive.GetMaxForceAttr().Get())
        details = prepare_hunter2_stage(self.stage, self.path, self.parameters, (1., 2., .37), .4)
        self.assertEqual(before, (drive.GetDampingAttr().Get(), drive.GetMaxForceAttr().Get()))
        self.assertEqual(details['saved_drives']['left_rear_joint']['damping_usd'], before[0])
        self.assertEqual(len(details['saved_drives']), 4)
        self.assertAlmostEqual(details['geometry_m']['wheelbase'], self.parameters['wheelbase'], places=5)
        self.assertGreater(details['minimum_turning_radius_m'], 1.)
        self.assertFalse(self.stage.GetPrimAtPath('/FlatGrid'))
        self.assertFalse(self.stage.GetPrimAtPath('/Render'))
        root = self.stage.GetPrimAtPath(self.path)
        self.assertEqual(UsdGeom.Xformable(root).GetLocalTransformation().ExtractTranslation(), Gf.Vec3d(1, 2, .37))
        bodies = [p for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        self.assertEqual(len(bodies), 7)
        colliders = [p for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies())
                     if p.HasAPI(UsdPhysics.CollisionAPI)
                     and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()]
        self.assertEqual(len(colliders), 7)
        for name in self.parameters['passive_front_wheels']:
            passive = UsdPhysics.DriveAPI(self.stage.GetPrimAtPath(self.path + '/joints/' + name), 'angular')
            self.assertEqual(passive.GetMaxForceAttr().Get(), 0.)

    def test_saved_wheel_cylinders_replace_mesh_collisions_and_match_control_radius(self):
        prepare_hunter2_stage(self.stage, self.path, self.parameters, (0., 0., .37), 0.)
        rear_names = [entry['joint'] for entry in self.parameters['rear_wheels']]
        cache = UsdGeom.XformCache()
        for name in rear_names + self.parameters['passive_front_wheels']:
            with self.subTest(joint=name):
                joint = UsdPhysics.RevoluteJoint(self.stage.GetPrimAtPath(self.path + '/joints/' + name))
                body = self.stage.GetPrimAtPath(joint.GetBody1Rel().GetTargets()[0])
                collider = self.stage.GetPrimAtPath(str(body.GetPath()) + '/wheel_collider')
                self.assertTrue(collider.IsA(UsdGeom.Cylinder))
                self.assertFalse(collider.HasAPI(UsdPhysics.RigidBodyAPI))
                self.assertTrue(body.HasAPI(UsdPhysics.RigidBodyAPI))
                self.assertEqual(UsdGeom.Imageable(collider).ComputeVisibility(), 'invisible')
                cylinder = UsdGeom.Cylinder(collider)
                expected_radius = self.parameters['wheel_radius'] if name in rear_names else .164158
                self.assertAlmostEqual(cylinder.GetRadiusAttr().Get(), expected_radius, places=6)
                self.assertAlmostEqual(cylinder.GetHeightAttr().Get(), .141566, places=6)
                transform = UsdGeom.Xformable(collider).GetLocalTransformation()
                axes = {'X': Gf.Vec3d(1, 0, 0), 'Y': Gf.Vec3d(0, 1, 0), 'Z': Gf.Vec3d(0, 0, 1)}
                for axis in axes.values():
                    self.assertAlmostEqual(transform.TransformDir(axis).GetLength(), 1., places=6)
                cylinder_axis = cache.GetLocalToWorldTransform(collider).TransformDir(
                    axes[cylinder.GetAxisAttr().Get()]).GetNormalized()
                child_frame = Gf.Matrix4d(1.)
                child_frame.SetRotate(Gf.Quatd(joint.GetLocalRot1Attr().Get()))
                joint_axis = cache.GetLocalToWorldTransform(body).TransformDir(
                    child_frame.TransformDir(axes[joint.GetAxisAttr().Get()])).GetNormalized()
                self.assertAlmostEqual(abs(Gf.Dot(cylinder_axis, joint_axis)), 1., places=6)
                colliders = [p for p in Usd.PrimRange(body, Usd.TraverseInstanceProxies())
                             if p.HasAPI(UsdPhysics.CollisionAPI)]
                enabled = [p for p in colliders if UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()]
                self.assertEqual(enabled, [collider])
                old_meshes = [p for p in colliders if p.IsA(UsdGeom.Mesh)]
                self.assertTrue(old_meshes)
                self.assertTrue(all(not UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()
                                    for p in old_meshes))

    def test_original_urdf_left_rear_sign_is_rejected_for_this_usd(self):
        parameters = copy.deepcopy(self.parameters)
        parameters['rear_wheels'][0]['sign'] = -1
        with self.assertRaisesRegex(ValueError, 'configured sign disagrees'):
            prepare_hunter2_stage(self.stage, self.path, parameters, (0., 0., .37), 0.)

    def test_composed_model_layers_are_inside_hunter_assets(self):
        prepare_hunter2_stage(self.stage, self.path, self.parameters, (0., 0., .37), 0.)
        assets = (ROOT / 'assets').resolve()
        layers = [layer for layer in self.stage.GetUsedLayers() if not layer.anonymous]
        self.assertTrue(layers)
        for layer in layers:
            self.assertTrue(Path(layer.realPath).resolve().is_relative_to(assets), layer.identifier)

    def test_target_speed_above_saved_limit_is_rejected(self):
        parameters = copy.deepcopy(self.parameters)
        parameters['max_speed'] = 10.
        with self.assertRaisesRegex(ValueError, 'exceeds the joint velocity limit'):
            prepare_hunter2_stage(self.stage, self.path, parameters, (0., 0., .37), 0.)

    def test_rear_limit_override_allows_one_metre_per_second_without_changing_steering(self):
        details = prepare_hunter2_stage(self.stage, self.path, self.parameters, (0., 0., .37), 0.)
        for entry in self.parameters['rear_wheels']:
            joint = self.stage.GetPrimAtPath(self.path + '/joints/' + entry['joint'])
            limit = math.radians(joint.GetAttribute('physxJoint:maxJointVelocity').Get())
            self.assertAlmostEqual(limit, self.parameters['rear_velocity_limit'], places=6)
            self.assertGreater(limit, 1. / self.parameters['wheel_radius'])
        for entry in self.parameters['steering']:
            drive = details['saved_drives'][entry['joint']]
            self.assertEqual(drive['velocity_limit_deg_s'], drive['saved_velocity_limit_deg_s'])

    def test_omitting_override_preserves_saved_rear_speed_limit(self):
        parameters = copy.deepcopy(self.parameters)
        parameters.pop('rear_velocity_limit')
        parameters['max_speed'] = .3
        joint = self.stage.GetPrimAtPath(self.path + '/joints/left_rear_joint')
        before = joint.GetAttribute('physxJoint:maxJointVelocity').Get()
        prepare_hunter2_stage(self.stage, self.path, parameters, (0., 0., .37), 0.)
        self.assertEqual(joint.GetAttribute('physxJoint:maxJointVelocity').Get(), before)

    def test_wrong_geometry_and_excessive_steering_are_rejected(self):
        for field, value, message in (('front_track', .605, 'joint anchors'),
                                      ('wheelbase', .8, 'joint anchors'),
                                      ('max_steering_angle', .57, 'steering range'),
                                      ('max_steering_rate', 1.5, 'joint speed limit')):
            parameters = copy.deepcopy(self.parameters)
            parameters[field] = value
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, message):
                prepare_hunter2_stage(self.stage, self.path, parameters, (0., 0., .37), 0.)


if __name__ == '__main__':
    unittest.main()
