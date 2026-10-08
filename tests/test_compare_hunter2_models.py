"""Independent checks for the audit reader and coordinate calculations."""
import math
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'/'hunter2'))
from compare_hunter2_models import LCAS, RestrictedXacro, safe_expr, topology, point, turning, mesh_triangles


class ComparisonChecks(unittest.TestCase):
    def test_restricted_expression_rejects_code(self):
        with self.assertRaises(ValueError): safe_expr("__import__('os').getcwd()",{})
        with self.assertRaises(ValueError): safe_expr('pi.real',{'pi':math.pi})

    def test_restricted_expression_supported_math(self):
        self.assertAlmostEqual(safe_expr('-pi/2',{'pi':math.pi}),-math.pi/2)

    def test_expansion_includes_and_interfaces(self):
        r=RestrictedXacro().read(LCAS/'description/robot.urdf.xacro')
        self.assertEqual(len(r.findall('link')),8)
        self.assertEqual(len(r.findall('joint')),7)
        self.assertAlmostEqual(sum(float(e.get('value')) for e in r.findall('link/inertial/mass')),100.01)
        self.assertEqual(len(r.findall('ros2_control/joint/command_interface')),2)
        self.assertEqual(r.find("joint[@name='front_right_steering_joint']/mimic").get('multiplier'),'1.0')
        self.assertNotIn('${',ET.tostring(r,encoding='unicode'))

    def test_unknown_macro_fails(self):
        r=ET.fromstring('<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:if value="true"/></robot>')
        with self.assertRaises(ValueError): RestrictedXacro().expand(list(r),LCAS,{})

    def test_fk_accumulates_ancestor_rotation_and_translation(self):
        r=ET.fromstring(f'''<robot><link name="base_link"/><link name="body"/><link name="wheel"/>
        <joint name="fixed" type="fixed"><parent link="base_link"/><child link="body"/><origin xyz="1 2 3" rpy="0 0 {math.pi/2}"/></joint>
        <joint name="moving" type="continuous"><parent link="body"/><child link="wheel"/><origin xyz="1 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/></joint></robot>''')
        _,_,fn=topology(r);_,jf=fn()
        for a,b in zip(point(jf['moving'],[1,0,0],0),(0,1,0)):self.assertAlmostEqual(a,b)
        for a,b in zip(point(jf['moving'],[0,0,0]),(1,3,3)):self.assertAlmostEqual(a,b)

    def test_lcas_cad_units_and_triangle_indices(self):
        tris=mesh_triangles(LCAS/'meshes/front_left_wheel.dae')
        self.assertEqual(len(tris),10940)
        radius=max(math.hypot(p[0],p[1]) for t in tris for p in t)
        self.assertGreater(radius,.16);self.assertLess(radius,.17)

    def test_turning_relation_and_parallel_steering_failure(self):
        t=turning();s=t['manual_inner_33deg']
        self.assertAlmostEqual(1/math.tan(s['outer_rad'])-1/math.tan(s['inner_rad']),.605/.650)
        self.assertAlmostEqual(t['lcas_parallel_steer_0_461_icc_y']['left']-t['lcas_parallel_steer_0_461_icc_y']['right'],.585)
        self.assertAlmostEqual(t['required_straight_wheel_speed_rad_s']*.165,1.5)


if __name__=='__main__': unittest.main()
