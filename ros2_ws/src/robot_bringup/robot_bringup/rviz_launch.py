"""Keep simulator rendering and RViz display choices independent."""
from pathlib import Path

from .rviz_view import resolve_display, system_environment, validate_options, web_tools


def rviz_arguments():
    from launch.actions import DeclareLaunchArgument
    defaults = {'rviz_display': 'auto', 'rviz_robot': '', 'rviz_port': '6080',
                'rviz_resolution': '1440x900', 'rviz_rendering': 'auto',
                'rviz_goal_topic': '/waypoint/goal_pose'}
    return [DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()]


def launch_rviz(context, *, simulation_display=None):
    from launch.actions import ExecuteProcess, RegisterEventHandler
    from launch.event_handlers import OnProcessExit
    from launch.substitutions import LaunchConfiguration
    from robot_sim.config import load_world_config
    def value(name):
        return LaunchConfiguration(name).perform(context)
    mode = resolve_display(value('rviz_display'), simulation_display)
    if mode == 'off':
        return []
    root = Path(value('project_root')).resolve()
    world = load_world_config(value('robots_file'))
    if value('rviz_robot') and not any(robot['robot_id'] == value('rviz_robot') for robot in world['robots']):
        raise ValueError('rviz_robot must match an id from robots_file.')
    validate_options(mode, value('rviz_resolution'), int(value('rviz_port')),
                     value('rviz_rendering'), int(value('ros_domain_id')))
    if mode == 'web':
        web_tools(root)  # Fail before starting Isaac if remote tools are missing.
    environment = system_environment()
    if mode == 'local' and not (environment.get('DISPLAY') or environment.get('WAYLAND_DISPLAY')):
        raise ValueError('No local GUI display. In SSH use rviz_display:=web.')
    paths = [root / 'ros2_ws/src/robot_bringup', root / 'ros2_ws/src/robot_sim']
    environment['PYTHONPATH'] = ':'.join([*map(str, paths), environment.get('PYTHONPATH', '')])
    viewer = ExecuteProcess(
        cmd=['/usr/bin/python3', '-m', 'robot_bringup.rviz_view', '--project-root', str(root),
             '--robots-file', str(Path(value('robots_file')).resolve()), '--display', mode,
             '--robot', value('rviz_robot'), '--port', value('rviz_port'),
             '--goal-topic', value('rviz_goal_topic'),
             '--resolution', value('rviz_resolution'), '--rendering', value('rviz_rendering'),
             '--ros-domain-id', value('ros_domain_id')],
        cwd=str(root), env=environment, output='screen',
        sigterm_timeout='10', sigkill_timeout='5')
    def failed(event, context):
        if event.returncode:
            raise RuntimeError(f'RViz viewer failed with exit code {event.returncode}; see RVIZ_ERROR above.')
    return [RegisterEventHandler(OnProcessExit(target_action=viewer, on_exit=failed)), viewer]
