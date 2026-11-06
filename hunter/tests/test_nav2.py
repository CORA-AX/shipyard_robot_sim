"""Vehicle constraints and independent Nav2 configuration; no ROS startup."""
import math
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent))
from hunter.control.controller import load_parameters
from hunter.navigation.nav2.config import configured_parameters, rviz_configuration, twist_to_drive
from hunter.navigation.custom.path import dubins
from hunter.navigation.state import Pose, wrap
from hunter.navigation.nav2.position_goal import heading_candidates, path_distance
from hunter.ros.nav2.rviz import rviz_environment


class HunterNav2Tests(unittest.TestCase):
    def setUp(self):
        self.robot = load_parameters(ROOT)

    def test_rear_twist_conversion_and_speed_limit_preserve_curvature(self):
        for speed, yaw_rate in ((.4, .2), (.4, -.2), (2., .3)):
            drive = twist_to_drive(speed, 0., yaw_rate, self.robot)
            self.assertAlmostEqual(drive.steering_angle, math.atan(self.robot['wheelbase']*yaw_rate/speed))
            self.assertLessEqual(drive.speed, self.robot['max_speed'])
        self.assertEqual(twist_to_drive(0., 0., 0., self.robot).speed, 0.)

    def test_impossible_or_invalid_commands_are_rejected(self):
        for values in ((0., 0., 1.), (-.1, 0., 0.), (.4, .1, 0.), (float('nan'), 0., 0.)):
            with self.assertRaises(ValueError):
                twist_to_drive(*values, self.robot)
        self.assertEqual(twist_to_drive(.1, 0., 5., self.robot).steering_angle,
                         self.robot['max_steering_angle'])

    def test_smac_radius_has_headroom_for_steering_actuators(self):
        parameters = configured_parameters(ROOT, self.robot, 'fleet/hunter', .5)
        planner = parameters['/fleet/hunter/planner_server']['ros__parameters']['GridBased']
        self.assertGreaterEqual(planner['minimum_turning_radius'],
                                1.35*self.robot['wheelbase']/math.tan(self.robot['max_steering_angle']))
        self.assertEqual(planner['motion_model_for_search'], 'DUBIN')
        self.assertFalse(planner['smooth_path'])
        controller = parameters['/fleet/hunter/controller_server']['ros__parameters']
        self.assertFalse(controller['FollowPath']['use_rotate_to_heading'])
        self.assertFalse(controller['FollowPath']['allow_reversing'])
        self.assertEqual(controller['position_goal_checker']['plugin'], 'nav2_controller::PositionGoalChecker')
        self.assertEqual(controller['FollowPath']['desired_linear_vel'], .5)
        for costmap in ('global_costmap', 'local_costmap'):
            cfg = parameters[f'/fleet/hunter/{costmap}/{costmap}']['ros__parameters']
            self.assertEqual(cfg['robot_base_frame'], 'fleet/hunter/base_link')
            self.assertNotIn('obstacle_layer', cfg['plugins'])
        tree = ET.parse(ROOT/'config/nav2_tree.xml')
        self.assertEqual([node.tag for node in tree.getroot().iter()
                          if node.tag in ('ComputePathToPose', 'FollowPath', 'Spin', 'BackUp')],
                         ['ComputePathToPose', 'FollowPath'])

    def test_configuration_rejects_invalid_speed_and_namespaces(self):
        for speed in (0., -1., 2., float('nan')):
            with self.assertRaises(ValueError):
                configured_parameters(ROOT, self.robot, 'hunter2_0', speed)
        with self.assertRaises(ValueError):
            rviz_configuration('invalid namespace')
        slow = configured_parameters(ROOT, self.robot, 'hunter2_0', .02)
        controller = slow['/hunter2_0/controller_server']['ros__parameters']['FollowPath']
        self.assertLessEqual(controller['min_approach_linear_velocity'], .02)
        self.assertLessEqual(controller['regulated_linear_scaling_min_speed'], .02)

    def test_rviz_uses_the_goal_and_state_topics_for_the_same_robot(self):
        manager = rviz_configuration('hunter2_0')['Visualization Manager']
        self.assertEqual(manager['Global Options']['Fixed Frame'], 'map')
        self.assertEqual(manager['Tools'][-1]['Topic']['Value'], '/hunter2_0/nav2/goal_pose')
        topics = [display.get('Topic', {}).get('Value') for display in manager['Displays']]
        self.assertIn('/hunter2_0/odom', topics)
        self.assertIn('/hunter2_0/nav2/selected_path', topics)
        point_tool = next(tool for tool in manager['Tools'] if tool['Class'].endswith('/PublishPoint'))
        self.assertEqual(point_tool['Topic']['Value'], '/hunter2_0/nav2/goal_point')

    def test_position_goal_headings_include_short_arrivals_and_are_unique(self):
        start = Pose(0., 0., 0.)
        for x, y in ((4., 0.), (3., 2.), (3., -2.), (-2., 1.)):
            headings = heading_candidates(start, x, y, 1.8)
            self.assertLessEqual(len(headings), 16)
            self.assertGreaterEqual(len(headings), 12)
            for i, angle in enumerate(headings):
                self.assertTrue(math.isfinite(angle))
                self.assertTrue(all(abs(wrap(angle-other)) > 1e-6 for other in headings[:i]))
            lengths = [sum(s.length for s in dubins(start, Pose(x, y, angle), 1.8)) for angle in headings]
            backwards = sum(s.length for s in dubins(start, Pose(x, y, math.pi), 1.8))
            self.assertLessEqual(min(lengths), backwards+1e-6)
        straight = [sum(s.length for s in dubins(start, Pose(4., 0., angle), 1.8))
                    for angle in heading_candidates(start, 4., 0., 1.8)]
        self.assertAlmostEqual(min(straight), 4.)

    def test_position_goal_candidates_reflect_left_right_geometry(self):
        left = heading_candidates(Pose(0., 0., 0.), 3., 2., 1.8)
        right = heading_candidates(Pose(0., 0., 0.), 3., -2., 1.8)
        self.assertEqual(len(left), len(right))
        for angle in left:
            self.assertTrue(any(abs(wrap(angle+other)) < 1e-6 for other in right))

    def test_planner_path_measurement_rejects_invalid_results(self):
        self.assertAlmostEqual(path_distance([(0., 0.), (3., 4.), (3., 6.)]), 7.)
        for points in ([], [(0., 0.)], [(0., 0.), (float('nan'), 1.)]):
            with self.assertRaises(ValueError):
                path_distance(points)

    def test_software_rendering_is_scoped_to_rviz_and_preserves_ros_environment(self):
        source = dict(ROS_DOMAIN_ID='0', LD_LIBRARY_PATH='/some/.conda/env/lib:/hunter/.runtime/nav2/lib:/opt/ros/humble/lib',
                      PATH='/usr/bin:/some/.conda/env/bin',QT_PLUGIN_PATH='/some/.conda/qt')
        original = dict(source)
        result = rviz_environment(source)
        self.assertEqual(source,original)
        self.assertEqual(result['ROS_DOMAIN_ID'],'0')
        self.assertIn('/hunter/.runtime/nav2/lib',result['LD_LIBRARY_PATH'])
        self.assertNotIn('.conda',result['LD_LIBRARY_PATH'])
        self.assertNotIn('QT_PLUGIN_PATH',result)
        self.assertEqual(result['LIBGL_ALWAYS_SOFTWARE'],'1')
        self.assertEqual(result['__GLX_VENDOR_LIBRARY_NAME'],'mesa')
        self.assertNotIn('LIBGL_ALWAYS_SOFTWARE',rviz_environment(source,'native'))


if __name__ == '__main__':
    unittest.main()
