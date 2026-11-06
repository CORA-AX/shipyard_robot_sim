"""Go2 warehouse Nav2, with optional simulator and local/web RViz."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from robot_bringup.launch_support import default_project_root
from robot_bringup.rviz_launch import rviz_arguments
from robot_navigation.launch_support import launch_navigation


def generate_launch_description():
    bringup = Path(get_package_share_directory('robot_bringup'))
    defaults = {
        'project_root': default_project_root(), 'isaac_python': '',
        'robots_file': str(bringup / 'config/go2_single.yaml'),
        'start_simulator': 'true', 'navigation_robot': 'go2_0',
        'goal_selector': 'true', 'map': '', 'params_file': '',
        'display': 'local', 'device': 'cuda:0', 'policy_device': 'cpu',
        'command_timeout': '0.5', 'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
        'steps': '0', 'report': '', 'checkpoint': '',
        'server_ip': os.environ.get('PUBLIC_IP', ''),
    }
    simulation_keys = ['project_root', 'isaac_python', 'robots_file', 'display', 'device',
                       'policy_device', 'command_timeout', 'ros_domain_id', 'steps', 'report',
                       'checkpoint', 'server_ip', 'rviz_display', 'rviz_robot', 'rviz_port',
                       'rviz_resolution', 'rviz_rendering', 'rviz_goal_topic']
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value) for key,value in defaults.items()],
        *rviz_arguments(navigation=True),
        OpaqueFunction(function=launch_navigation),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup / 'launch/simulation.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_simulator')),
            launch_arguments={**{key: LaunchConfiguration(key) for key in simulation_keys},
                              'rviz_navigation': 'true', 'shared_world': 'true'}.items()),
    ])
