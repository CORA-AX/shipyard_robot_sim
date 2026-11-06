"""Create an explicit namespaced Nav2 stack using the existing global TF tree."""
from pathlib import Path
import tempfile

import yaml


def configured_parameters(source, namespace, map_path, tree_path):
    text = Path(source).read_text().replace('<ns>', namespace)
    values = yaml.safe_load(text)
    values['map_server']['ros__parameters']['yaml_filename'] = str(map_path)
    values['bt_navigator']['ros__parameters'].update(
        default_nav_to_pose_bt_xml=str(tree_path), default_nav_through_poses_bt_xml=str(tree_path))
    result = {}
    for key, value in values.items():
        if key in ('local_costmap', 'global_costmap'):
            result[f'/{namespace}/{key}/{key}'] = value[key]
        else:
            result[f'/{namespace}/{key}'] = value
    return result


def launch_navigation(context):
    from ament_index_python.packages import get_package_share_directory
    from launch.actions import EmitEvent, RegisterEventHandler, SetEnvironmentVariable
    from launch.event_handlers import OnProcessExit
    from launch.events import Shutdown
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node
    from robot_sim.config import load_world_config

    def value(name):
        return LaunchConfiguration(name).perform(context)
    root = Path(value('project_root')).resolve()
    world = load_world_config(value('robots_file'))
    robots = [robot for robot in world['robots'] if robot['robot_id'] == value('navigation_robot')]
    if len(robots) != 1 or robots[0]['robot_type'] != 'go2':
        raise ValueError('This navigation phase supports one Go2 selected by navigation_robot.')
    if world['scenario'] != 'warehouse':
        raise ValueError('The supplied static map is for warehouse; robots_file must use warehouse.')
    robot = robots[0]
    share = Path(get_package_share_directory('robot_navigation'))
    for package in ('nav2_map_server', 'nav2_planner', 'nav2_navfn_planner', 'nav2_controller',
                    'dwb_core', 'dwb_plugins', 'dwb_critics', 'nav2_bt_navigator',
                    'nav2_lifecycle_manager', 'nav2_velocity_smoother'):
        try:
            get_package_share_directory(package)
        except LookupError as exc:
            raise RuntimeError('Nav2 runtime missing. Run bash ros2_ws/setup.sh --only nav2, then '
                               'source ros2_ws/nav2_env.bash.') from exc
    map_path = Path(value('map') or str(share / 'maps/warehouse.yaml')).resolve()
    if not map_path.is_file():
        raise ValueError(f'Warehouse map missing: {map_path}; run scripts/export_warehouse_map.py.')
    params = configured_parameters(value('params_file') or share / 'config/go2_nav2.yaml',
                                   robot['namespace'], map_path, share / 'config/navigate.xml')
    directory = root / '.runtime/nav2/launches'
    directory.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', dir=directory, delete=False) as stream:
        yaml.safe_dump(params, stream, sort_keys=False)
        parameters_file = stream.name
    domain = int(value('ros_domain_id'))
    if not 0 <= domain <= 232:
        raise ValueError('ROS domain must be 0..232.')
    actions = [SetEnvironmentVariable('ROS_DOMAIN_ID', str(domain))]
    if value('start_simulator').lower() != 'true':
        from robot_bringup.rviz_launch import launch_rviz
        actions += launch_rviz(context, simulation_display=value('display'))
    lifecycle_names = ['map_server']
    nodes = [Node(package='nav2_map_server', executable='map_server', name='map_server', namespace=robot['namespace'],
                  parameters=[parameters_file], output='screen')]
    # /tf and /tf_static deliberately remain global. Each robot already has
    # unique frame ids; remapping these to relative tf would split the TF tree.
    for package, name, remappings in [
        ('nav2_planner', 'planner_server', []),
        ('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_velocity_smoother', 'velocity_smoother',
         [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel_smoothed')]),
        ('nav2_bt_navigator', 'bt_navigator', [('goal_pose', 'nav2_internal_goal_pose')]),
    ]:
        nodes.append(Node(package=package, executable=name, name=name, namespace=robot['namespace'],
                          parameters=[parameters_file], remappings=remappings, output='screen'))
        lifecycle_names.append(name)
    nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                      name='navigation_lifecycle_manager', namespace=robot['namespace'], parameters=[{
                          'use_sim_time': True, 'autostart': True,
                          'node_names': lifecycle_names, 'bond_timeout': 4.0}], output='screen'))
    if value('goal_selector').lower() == 'true':
        nodes.append(Node(package='robot_navigation', executable='goal_selector',
                          arguments=['--robots-file', str(Path(value('robots_file')).resolve()),
                                     '--robot', robot['robot_id'], '--goal-topic', value('rviz_goal_topic'),
                                     '--no-keyboard'], output='screen'))
    # A crashed lifecycle component must stop the rest, including the simulator
    # when it is owned by this launch. Normal Ctrl-C is already handled by launch.
    for node in nodes:
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=node, on_exit=[EmitEvent(event=Shutdown(reason='Navigation component exited'))])))
    return actions + nodes
