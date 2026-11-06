"""Hunter Nav2 configuration and Twist conversion, independent of ROS/Kit."""
import math
from pathlib import Path
import re

import yaml
from hunter.control.commands import DriveCommand


# Conservative saved-collider envelope, including front wheels at full steer,
# expressed at the rear axle midpoint; additional costmap padding is 0.02 m.
FOOTPRINT = [[.88, .46], [.88, -.46], [-.22, -.46], [-.22, .46]]

def planning_turning_radius(robot):
    return max(1.8, 1.35*robot['wheelbase']/math.tan(robot['max_steering_angle']))


def namespace_value(value):
    value = value.strip('/')
    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', value):
        raise ValueError('Use a valid namespace, such as hunter2_0.')
    return value


def twist_to_drive(speed, lateral, yaw_rate, robot):
    """Rear-centred v, omega -> virtual steering; reject non-car commands.

    Preserve requested curvature when limiting speed. Steering saturation is a
    final guard; Smac's larger planned radius provides tracking headroom.
    """
    if not all(math.isfinite(v) for v in (speed, lateral, yaw_rate)):
        raise ValueError('Non-finite Nav2 command.')
    if abs(lateral) > 1e-6 or speed < -1e-6:
        raise ValueError('This forward-only Hunter configuration cannot execute lateral/reverse commands.')
    if abs(speed) < 1e-5:
        if abs(yaw_rate) > 1e-4:
            raise ValueError('Hunter cannot rotate in place.')
        return DriveCommand()
    angle = math.atan(robot['wheelbase'] * yaw_rate / speed)
    angle = max(-robot['max_steering_angle'], min(robot['max_steering_angle'], angle))
    return DriveCommand(min(speed, robot['max_speed']), angle,
                        acceleration=min(.3, robot['max_acceleration']))


def configured_parameters(root, robot, namespace, speed=.4, source=None):
    root, namespace = Path(root), namespace_value(namespace)
    if not math.isfinite(speed) or not 0. < speed <= robot['max_speed']:
        raise ValueError('Navigation speed must be positive and within robot.yaml max_speed.')
    values = yaml.safe_load(Path(source or root/'config/nav2.yaml').read_text().replace('<ns>', namespace))
    planner = values['planner_server']['ros__parameters']['GridBased']
    planner['minimum_turning_radius'] = planning_turning_radius(robot)
    planner['analytic_expansion_max_length'] = max(10., 5.*planner['minimum_turning_radius'])
    controller = values['controller_server']['ros__parameters']['FollowPath']
    controller['desired_linear_vel'] = float(speed)
    controller['min_approach_linear_velocity'] = min(controller['min_approach_linear_velocity'], float(speed))
    controller['regulated_linear_scaling_min_speed'] = min(controller['regulated_linear_scaling_min_speed'], float(speed))
    controller['regulated_linear_scaling_min_radius'] = planner['minimum_turning_radius']
    bt = values['bt_navigator']['ros__parameters']
    bt['default_nav_to_pose_bt_xml'] = str((root/'config/nav2_tree.xml').resolve())
    bt['default_nav_through_poses_bt_xml'] = bt['default_nav_to_pose_bt_xml']
    result = {}
    for key, value in values.items():
        if key in ('local_costmap', 'global_costmap'):
            value[key]['ros__parameters']['footprint'] = str(FOOTPRINT)
            result[f'/{namespace}/{key}/{key}'] = value[key]
        else:
            result[f'/{namespace}/{key}'] = value
    return result


def rviz_configuration(namespace, world=None):
    namespace = namespace_value(namespace)
    prefix = '/'+namespace
    reliable = lambda topic: {'Value': topic, 'Reliability Policy': 'Reliable',
                              'Durability Policy': 'Volatile', 'Depth': 10}
    latched = lambda topic: dict(reliable(topic), **{'Durability Policy': 'Transient Local'})
    bounds = (-20.,-20.,20.,20.) if world is None else world.bounds
    center = ((bounds[0]+bounds[2])/2.,(bounds[1]+bounds[3])/2.)
    cells = math.ceil(max(bounds[2]-bounds[0],bounds[3]-bounds[1])/.5)
    return {
        'Panels': [{'Class': 'rviz_common/Displays', 'Name': 'Displays'},
                   {'Class': 'rviz_common/Tool Properties', 'Name': 'Tool Properties'},
                   {'Class': 'rviz_common/Views', 'Name': 'Views'}],
        'Visualization Manager': {
            'Class': '', 'Enabled': True,
            'Global Options': {'Fixed Frame': 'map', 'Frame Rate': 20,
                               'Background Color': '40; 44; 52'},
            'Displays': [
                {'Class': 'rviz_default_plugins/Grid', 'Name': 'Distance grid 0.5 m (not a driving boundary)',
                 'Enabled': True, 'Cell Size': .5, 'Plane Cell Count': cells, 'Alpha': .15,
                 'Color': '90; 100; 115', 'Offset': {'X':center[0],'Y':center[1],'Z':.01}},
                {'Class': 'rviz_default_plugins/Map', 'Name': 'Navigation map (static geometry)',
                 'Enabled': True, 'Topic': latched(prefix+'/nav2/map'), 'Alpha': 1.,
                 'Color Scheme': 'map', 'Draw Behind': False},
                {'Class': 'rviz_default_plugins/Odometry', 'Name': 'Rear axle position and heading',
                 'Enabled': True, 'Topic': reliable(prefix+'/odom'), 'Keep': 1,
                 'Shape': {'Value': 'Arrow', 'Color': '80; 200; 255',
                           'Shaft Length': .4, 'Shaft Radius': .035,
                           'Head Length': .15, 'Head Radius': .08},
                 'Covariance': {'Value': False}},
                {'Class': 'rviz_default_plugins/Path', 'Name': 'Nav2 planned path',
                 'Enabled': True, 'Topic': latched(prefix+'/nav2/selected_path'),
                 'Color': '80; 255; 120', 'Line Style': 'Billboards', 'Line Width': .04},
                {'Class': 'rviz_default_plugins/MarkerArray', 'Name': 'Hunter goal and state',
                 'Enabled': True, 'Topic': latched(prefix+'/nav2/markers')},
                {'Class': 'rviz_default_plugins/MarkerArray', 'Name': 'Stored waypoint order (not planned path)',
                 'Enabled': True, 'Topic': latched(prefix+'/patrol/markers')},
                {'Class': 'rviz_default_plugins/TF', 'Name': 'Coordinate frames',
                 'Enabled': False, 'Show Names': True, 'Show Axes': True}],
            'Tools': [{'Class': 'rviz_default_plugins/Interact'},
                      {'Class': 'rviz_default_plugins/MoveCamera'},
                      {'Class': 'rviz_default_plugins/Select'},
                      {'Class': 'rviz_default_plugins/PublishPoint',
                       'Topic': reliable(prefix+'/nav2/goal_point'), 'Single click': True},
                      {'Class': 'rviz_default_plugins/SetGoal',
                       'Topic': reliable(prefix+'/nav2/goal_pose')}],
            'Views': {'Current': {'Class': 'rviz_default_plugins/TopDownOrtho',
                                  'Name': 'Top view', 'Target Frame': 'map',
                                  'Scale': 70. if world is None else world.view_scale,
                                  'X': 1. if world is None else world.view_center[0],
                                  'Y': 0. if world is None else world.view_center[1]}}},
        'Window Geometry': {'Width': 1280, 'Height': 850}}
