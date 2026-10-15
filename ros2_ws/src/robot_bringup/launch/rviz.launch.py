"""Attach RViz to an already running simulation; do not start another Isaac instance."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from robot_bringup.launch_support import default_project_root
from robot_bringup.rviz_launch import launch_rviz, rviz_arguments


def generate_launch_description():
    share = get_package_share_directory('robot_bringup')
    arguments = {'project_root': default_project_root(),
                 'robots_file': share + '/config/go2_single.yaml',
                 'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0')}
    return LaunchDescription([*[DeclareLaunchArgument(key, default_value=value)
                                for key, value in arguments.items()],
                              *rviz_arguments(), OpaqueFunction(function=launch_rviz)])
