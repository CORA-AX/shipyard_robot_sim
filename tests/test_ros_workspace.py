"""Prevent ROS Python ABI pollution and ambiguous robot configuration."""
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'ros2_ws/src/robot_sim'), str(ROOT / 'ros2_ws/src/robot_control')]
from robot_sim.config import load_robot_config, load_world_config
from robot_sim.environment import isaac_environment
import yaml


class WorkspaceTests(unittest.TestCase):
    def test_external_ros_libraries_and_python_overlay_are_removed(self):
        base = {'AMENT_PREFIX_PATH': '/opt/ros/humble:/tmp/external/install',
                'PYTHONPATH': '/opt/ros/humble/local/lib/python3.10/dist-packages',
                'LD_LIBRARY_PATH': '/opt/ros/humble/lib:/tmp/external/install/lib:/usr/local/cuda/lib64',
                'ROS_DOMAIN_ID': '89', 'DISPLAY': ':0'}
        env = isaac_environment(ROOT / '.conda/isaacsim-5.0/bin/python', base, [ROOT / 'ros2_ws/src/robot_sim'])
        self.assertNotIn('AMENT_PREFIX_PATH', env)
        self.assertNotIn('/opt/ros', env['PYTHONPATH'])
        self.assertNotIn('/opt/ros', env['LD_LIBRARY_PATH'])
        self.assertNotIn('/tmp/external', env['LD_LIBRARY_PATH'])
        self.assertIn('/usr/local/cuda/lib64', env['LD_LIBRARY_PATH'])
        self.assertEqual(env['ROS_DOMAIN_ID'], '89')
        self.assertEqual(env['DISPLAY'], ':0')

    def test_single_robot_pose_namespace_and_seed_survive_configuration(self):
        config = load_robot_config(ROOT / 'ros2_ws/src/robot_bringup/config/go2_single.yaml')
        self.assertEqual(config['namespace'], 'go2_0')
        self.assertEqual(config['position'], [0., 0., .4])
        self.assertEqual(config['seed'], 42)

    def test_unimplemented_robot_multiple_robots_and_invalid_spawn_are_rejected(self):
        invalid = [
            {'robots': [{'type': 'hunter2'}]},
            {'robots': [{'type': 'go2'}, {'type': 'go2'}]},
            {'robots': [{'type': 'go2', 'namespace': 'bad namespace'}]},
            {'robots': [{'type': 'go2', 'namespace': 123}]},
            {'robots': [{'type': 'go2', 'position': [0., 0., float('nan')]}]},
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'robots.yaml'
            for data in invalid:
                path.write_text(yaml.safe_dump(data))
                with self.subTest(data=data), self.assertRaises(ValueError):
                    load_robot_config(path)

    def test_lekiwi_configuration_selects_mobile_base_runtime(self):
        config = load_robot_config(ROOT / 'ros2_ws/src/robot_bringup/config/lekiwi_single.yaml')
        self.assertEqual(config['robot_type'], 'lekiwi')
        self.assertEqual(config['namespace'], 'lekiwi_0')
        self.assertEqual(config['position'], [0., 0., .04])

    def test_shared_world_has_unique_names_and_preserves_individual_poses(self):
        world = load_world_config(ROOT / 'ros2_ws/src/robot_bringup/config/go2_lekiwi.yaml')
        self.assertEqual([r['robot_type'] for r in world['robots']], ['go2', 'lekiwi'])
        self.assertEqual([r['position'][1] for r in world['robots']], [-.8, .8])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'robots.yaml'
            cases = [[], [{'type': 'go2'}, {'type': 'go2'}],
                     [{'type': 'go2', 'id': 'a', 'namespace': 'same'},
                      {'type': 'lekiwi', 'id': 'b', 'namespace': '/same/'}]]
            for robots in cases:
                path.write_text(yaml.safe_dump({'robots': robots}))
                with self.subTest(robots=robots), self.assertRaises(ValueError):
                    load_world_config(path)


if __name__ == '__main__':
    unittest.main()
