"""Isaac Humble environment isolated from a sourced system ROS workspace."""
import os
from pathlib import Path
import sys


def isaac_environment(python, environment=None, python_paths=()):
    env = dict(os.environ if environment is None else environment)
    prefix = Path(python).absolute().parent.parent
    bridge = prefix / 'lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble'
    if not (bridge / 'rclpy/rclpy').is_dir():
        raise ValueError(f'Isaac internal Humble libraries not found: {bridge}')
    sourced = [p for p in env.get('AMENT_PREFIX_PATH', '').split(os.pathsep) if p]
    sourced += [p for p in env.get('COLCON_PREFIX_PATH', '').split(os.pathsep) if p]
    sourced += ['/opt/ros']
    def external_ros(path):
        return any(path == p or path.startswith(p.rstrip('/') + '/') for p in sourced)
    libraries = [p for p in env.get('LD_LIBRARY_PATH', '').split(os.pathsep)
                 if p and not external_ros(p) and p != str(bridge / 'lib')]
    env['ROS_DISTRO'] = 'humble'
    env['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    env['PYTHONNOUSERSITE'] = '1'
    env['PYTHONUNBUFFERED'] = '1'
    env['PYTHONPATH'] = os.pathsep.join([str(bridge / 'rclpy'), *map(str, python_paths)])
    env['LD_LIBRARY_PATH'] = os.pathsep.join([str(bridge / 'lib'), *libraries])
    for key in ('AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'CMAKE_PREFIX_PATH',
                'OLD_PYTHONPATH', 'PYTHONHOME', 'VIRTUAL_ENV'):
        env.pop(key, None)
    return env


def ros_environment(environment=None):
    return isaac_environment(Path(sys.prefix) / 'bin/python', environment)
