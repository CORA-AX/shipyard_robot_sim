"""Navigation contracts: collision envelope, asset identity and ROS isolation."""
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'ros2_ws/src'/p) for p in ('robot_navigation','robot_control','robot_sim')]
from robot_navigation.geometry import goal_is_free
try:
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.task import Future
    from rclpy.time import Time
    from robot_navigation.goal_selector import GoalSelector, yaw
except ImportError:
    GoalSelector = None  # Pure geometry/asset checks also run in Isaac Python.
from robot_navigation.launch_support import configured_parameters
from robot_sim.environment import isaac_environment


class NavigationTests(unittest.TestCase):
    @unittest.skipIf(GoalSelector is None, 'System Humble/Nav2 Python required.')
    def test_settling_compensation_is_bounded_and_rejects_occupied_subgoals(self):
        node = GoalSelector.__new__(GoalSelector)
        node.target, node.odom, node.map = PoseStamped(), Odometry(), Mock()
        node.target.pose.orientation.w = 1.
        node.odom.pose.pose.position.x, node.odom.pose.pose.position.y = .20, .10
        node.position_bias, node.heading_bias = (0., 0.), 0.
        with patch('robot_navigation.goal_selector.goal_is_free', return_value=False):
            goal = node.refinement_goal(math.hypot(.20, .10), 0.)
        self.assertEqual(goal.pose.position, node.target.pose.position)
        self.assertEqual(node.position_bias, (0., 0.))
        with patch('robot_navigation.goal_selector.goal_is_free', return_value=True) as free:
            goal = node.refinement_goal(math.hypot(.20, .10), 0.)
        self.assertAlmostEqual(math.hypot(goal.pose.position.x, goal.pose.position.y), .15)
        self.assertTrue(free.call_args.kwargs['any_heading'])
        node.odom.pose.pose.position.x = node.odom.pose.pose.position.y = 0.
        node.heading_bias = .29
        with patch('robot_navigation.goal_selector.goal_is_free', return_value=True):
            goal = node.refinement_goal(0., .30)
        self.assertAlmostEqual(yaw(goal.pose.orientation), .30)
        self.assertEqual(node.target.pose.position.x, 0.)
        self.assertEqual(node.target.pose.position.y, 0.)
        self.assertAlmostEqual(yaw(node.target.pose.orientation), 0.)

    @unittest.skipIf(GoalSelector is None, 'System Humble/Nav2 Python required.')
    def test_stopped_arrival_requires_heading_and_position_with_wrapped_angles(self):
        for current_yaw, target_yaw, x, refinements, expected in [
                (0., math.pi/2, 0., 5, 'FAILED'),
                (math.pi-.02, -math.pi+.02, 0., 0, 'REACHED'),
                (0., 0., .20, 5, 'FAILED'),
                (0., 0., .13, 0, 'STOPPING'),
                (.20, 0., 0., 0, 'STOPPING'),
                (0., math.pi/2, 0., 0, 'STOPPING')]:
            with self.subTest(current_yaw=current_yaw, target_yaw=target_yaw, x=x):
                node = GoalSelector.__new__(GoalSelector)
                node.prefix, node.phase = '/go2', 'SETTLING'
                node.count_publishers = lambda topic: 1
                node.state_future, node.state_poll = None, time.monotonic()
                node.state_client = Mock()
                node.state_client.service_is_ready.return_value = False
                node.odom_time = node.stopping_since = node.last_status = time.monotonic()
                node.pending = node.request = None
                node.refinements = refinements
                node.heading_bias = 0.
                node.position_bias = (0., 0.)
                node.map = OccupancyGrid()
                node.map.info.resolution = .1
                node.map.info.width = node.map.info.height = 40
                node.map.info.origin.position.x = node.map.info.origin.position.y = -2.
                node.map.data = [0]*1600
                node.raw, node.raw_time = Twist(), -math.inf
                node.stop_stamp, node.odom_stamp = 1., 2.
                node.odom, node.target = Odometry(), PoseStamped()
                node.odom.pose.pose.position.x = x
                for pose, angle in [(node.odom.pose.pose, current_yaw), (node.target.pose, target_yaw)]:
                    pose.orientation.z, pose.orientation.w = math.sin(angle/2), math.cos(angle/2)
                node.command_pub = Mock()
                node.set_phase = lambda phase, reason: setattr(node, 'phase', phase)
                node.tick()
                self.assertEqual(node.phase, expected)
                self.assertEqual(node.command_pub.publish.call_args.args[0].angular.z, 0.)
                if expected == 'STOPPING':
                    self.assertEqual(node.refinements, 1)
                    self.assertAlmostEqual(node.pending.pose.position.x, -.13 if x == .13 else 0.)
                    self.assertEqual(node.target.pose.position.x, 0.)
                    expected_bias = -.20 if current_yaw == .20 else 0.
                    self.assertAlmostEqual(node.heading_bias, expected_bias)
                    self.assertAlmostEqual(yaw(node.pending.pose.orientation), target_yaw+expected_bias)
                    self.assertAlmostEqual(yaw(node.target.pose.orientation), target_yaw)
                    self.assertIsNot(node.pending, node.target)
                    node.stop('CANCELED', 'User canceled pose refinement.')
                    self.assertIsNone(node.pending)
                    self.assertIsNone(node.target)

    @unittest.skipIf(GoalSelector is None, 'System Humble/Nav2 Python required for the action race test.')
    def test_reset_during_goal_acceptance_cannot_reopen_command_gate(self):
        # Deliberately delay the action acceptance until after a reset, the race
        # normal Nav2/DDS integration tests cannot schedule deterministically.
        node = GoalSelector.__new__(GoalSelector)
        node.prefix, node.pending, node.request = '/go2', PoseStamped(), None
        node._clock = Mock()
        node._clock.now.return_value = Time(seconds=10)
        node.command_pub = Mock()
        node.client = Mock()
        accepted, result = Future(), Future()
        node.client.send_goal_async.return_value = accepted
        node.set_phase = lambda phase, reason: setattr(node,'phase',phase)
        node.dispatch()
        self.assertEqual(node.phase,'SENDING')
        node.stop('RESET','actual reset')
        handle = Mock(accepted=True)
        handle.get_result_async.return_value = result
        accepted.set_result(handle)
        self.assertEqual(node.phase,'RESET')
        handle.cancel_goal_async.assert_called_once()
        command = Twist()
        command.linear.x = .5
        node.on_command(command)
        self.assertEqual(node.raw.linear.x,0.)
        result.set_result(Mock(status=5))  # STATUS_CANCELED
        self.assertIsNone(node.request)
        self.assertIsNone(node.pending)
        self.assertEqual(node.phase,'RESET')
        self.assertTrue(all(call.args[0].linear.x == 0. for call in node.command_pub.publish.call_args_list))

    def test_unknown_outside_and_rotated_footprint_are_conservative(self):
        grid = SimpleNamespace(info=SimpleNamespace(origin=SimpleNamespace(position=SimpleNamespace())))
        grid.info.resolution = .05
        grid.info.width = grid.info.height = 80
        grid.info.origin.position.x = grid.info.origin.position.y = -2.
        grid.data = [0]*6400
        self.assertTrue(goal_is_free(grid,0.,0.,0.))
        grid.data[40*80+47] = 100  # x=.375: inside length, outside width.
        self.assertFalse(goal_is_free(grid,0.,0.,0.))
        self.assertTrue(goal_is_free(grid,0.,0.,math.pi/2))
        self.assertFalse(goal_is_free(grid,0.,0.,math.pi/2,any_heading=True))
        grid.data[40*80+47] = -1
        self.assertFalse(goal_is_free(grid,0.,0.,0.))
        self.assertFalse(goal_is_free(grid,1.9,0.,0.))

    def test_nested_namespace_costmaps_and_topics_share_one_global_frame(self):
        parameters = configured_parameters(ROOT/'ros2_ws/src/robot_navigation/config/go2_nav2.yaml',
                                            'fleet/go2', '/tmp/map.yaml', '/tmp/navigate.xml')
        self.assertIn('/fleet/go2/local_costmap/local_costmap',parameters)
        local = parameters['/fleet/go2/local_costmap/local_costmap']['ros__parameters']
        self.assertEqual(local['robot_base_frame'],'fleet/go2/base_link')
        self.assertEqual(local['static_layer']['map_topic'],'/map')
        self.assertEqual(parameters['/fleet/go2/map_server']['ros__parameters']['yaml_filename'],'/tmp/map.yaml')
        self.assertEqual(parameters['/fleet/go2/bt_navigator']['ros__parameters']['odom_topic'],'/fleet/go2/odom')
        self.assertNotIn('obstacle_layer',local['plugins'])

    def test_private_navigation_libraries_never_enter_isaac(self):
        sysroot = str(ROOT/'.runtime/nav2/sysroot')
        environment = isaac_environment(ROOT/'.conda/isaacsim-5.0/bin/python', {
            'AMENT_PREFIX_PATH':sysroot+'/opt/ros/humble:/opt/ros/humble',
            'LD_LIBRARY_PATH':sysroot+'/usr/lib:'+sysroot+'/usr/lib/x86_64-linux-gnu:'+sysroot+'/opt/ros/humble/lib',
            'PYTHONPATH':sysroot+'/opt/ros/humble/local/lib/python3.10/dist-packages'})
        self.assertNotIn(sysroot,environment['LD_LIBRARY_PATH'])
        self.assertNotIn(sysroot,environment['PYTHONPATH'])

    def test_map_is_bound_to_warehouse_source_and_image(self):
        directory = ROOT/'ros2_ws/src/robot_navigation/maps'
        metadata = json.loads((directory/'warehouse.json').read_text())
        self.assertEqual(hashlib.sha256((directory/'warehouse.pgm').read_bytes()).hexdigest(),metadata['image_sha256'])
        self.assertEqual(metadata['source_lock'],json.loads((ROOT/'assets/environments/warehouse/sources.lock.json').read_text()))
        self.assertEqual(metadata['active_colliders'],757)
        self.assertEqual(metadata['obstacle_height_band'],[.03,.65])
        self.assertEqual(metadata['composition_errors'],0)


if __name__ == '__main__':
    unittest.main()
