"""Display routing, robot targeting and system GUI ABI boundaries."""
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'ros2_ws/src' / name) for name in ('robot_bringup', 'robot_sim')]
from robot_bringup.rviz_view import make_config, resolve_display, system_environment, validate_options
from robot_sim.config import load_world_config


class RvizTests(unittest.TestCase):
    def test_auto_and_explicit_display_choices_are_independent(self):
        self.assertEqual(resolve_display('auto', 'local'), 'local')
        self.assertEqual(resolve_display('auto', 'webrtc'), 'web')
        self.assertEqual(resolve_display('auto', 'headless'), 'off')
        self.assertEqual(resolve_display('web', 'headless'), 'web')
        self.assertEqual(resolve_display('off', 'webrtc'), 'off')
        with self.assertRaises(ValueError):
            resolve_display('webrtc')

    def test_shared_goal_input_and_namespaced_odometry_are_independent_of_initial_view(self):
        world = load_world_config(ROOT / 'ros2_ws/src/robot_bringup/config/go2_lekiwi.yaml')
        world['robots'][1]['namespace'] = 'fleet/mobile'
        config, topic = make_config(world, 'lekiwi_0',
            ROOT / 'ros2_ws/src/robot_bringup/config/robots.rviz', 1280, 800)
        self.assertEqual(topic, '/waypoint/goal_pose')
        manager = config['Visualization Manager']
        self.assertEqual(manager['Global Options']['Fixed Frame'], 'map')
        self.assertEqual(manager['Tools'][-1]['Topic']['Value'], topic)
        odometry = [display['Topic']['Value'] for display in manager['Displays']
                    if display['Class'].endswith('/Odometry')]
        self.assertEqual(odometry, ['/go2_0/odom', '/fleet/mobile/odom'])
        self.assertEqual(manager['Views']['Current']['Focal Point']['Y'], .8)
        with self.assertRaises(ValueError):
            make_config(world, 'missing', ROOT / 'ros2_ws/src/robot_bringup/config/robots.rviz', 1280, 800)

    def test_gui_environment_preserves_system_ros_and_removes_isaac_qt_and_conda_libraries(self):
        base = {'DISPLAY': ':8', 'AMENT_PREFIX_PATH': '/opt/ros/humble',
                'LD_LIBRARY_PATH': '/opt/ros/humble/lib:/project/.conda/isaacsim-5.0/lib',
                'PYTHONPATH': '/project/ros2_ws/install:/project/.conda/isaacsim-5.0/site-packages',
                'QT_PLUGIN_PATH': '/project/.conda/qt/plugins'}
        with patch.dict(os.environ, base, clear=True):
            environment = system_environment()
        self.assertEqual(environment['DISPLAY'], ':8')
        self.assertEqual(environment['LD_LIBRARY_PATH'], '/opt/ros/humble/lib')
        self.assertEqual(environment['PYTHONPATH'], '/project/ros2_ws/install')
        self.assertNotIn('QT_PLUGIN_PATH', environment)

    def test_invalid_ports_resolution_and_domain_fail_before_process_start(self):
        for resolution, port, domain in [('1280x800', 80, 0), ('1x1', 6080, 0),
                                           ('1440x900', 6080, 233)]:
            with self.subTest(resolution=resolution, port=port, domain=domain), self.assertRaises(ValueError):
                validate_options('web', resolution, port, 'auto', domain)


if __name__ == '__main__':
    unittest.main()
