"""Launch configured robots with system Humble and isolated Isaac Python."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from robot_bringup.launch_support import default_project_root, launch_simulator
from robot_bringup.rviz_launch import rviz_arguments


def generate_launch_description():
    share = get_package_share_directory('robot_bringup')
    arguments = {
        'project_root': default_project_root(),
        'isaac_python': '',
        'robots_file': share + '/config/go2_single.yaml',
        'display': 'local',
        'device': 'cuda:0',
        'policy_device': 'cpu',
        'command_timeout': '0.5',
        'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
        'steps': '0',
        'report': '',
        'checkpoint': '',
        'server_ip': os.environ.get('PUBLIC_IP', ''),
    }
    return LaunchDescription([*[DeclareLaunchArgument(name, default_value=value)
                                for name, value in arguments.items()],
                              *rviz_arguments(),
                              OpaqueFunction(function=launch_simulator)])
