"""Validate world and robot instances before starting Kit."""
import math
from pathlib import Path
import re

import yaml


def load_world_config(path):
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict):
        raise ValueError('robots_file must contain a YAML mapping.')
    world, robots = data.get('world', {}), data.get('robots', [])
    if not isinstance(world, dict) or not isinstance(robots, list) or not robots:
        raise ValueError('Provide a world mapping and at least one robot.')
    seed = world.get('seed', 42)
    if type(seed) is not int or seed < 0:
        raise ValueError('Seed must be a nonnegative integer.')
    scenario = world.get('scenario', 'warehouse')
    if scenario not in ('flat', 'warehouse'):
        raise ValueError('Scenario must be flat or warehouse.')
    normalized = [_robot_config(robot) for robot in robots]
    for key in ('robot_id', 'namespace'):
        if len({robot[key] for robot in normalized}) != len(normalized):
            raise ValueError(f'Each robot must have a unique {key}.')
    return {'scenario': scenario, 'seed': seed, 'robots': normalized}


def _robot_config(robot):
    if not isinstance(robot, dict) or robot.get('type') not in ('go2', 'lekiwi'):
        raise ValueError('Supported robot types: go2, lekiwi.')
    robot_type = robot['type']
    robot_id = robot.get('id', robot_type + '_0')
    namespace = robot.get('namespace', robot_id)
    if not isinstance(robot_id, str) or not isinstance(namespace, str):
        raise ValueError('Robot id and namespace must be strings.')
    namespace = namespace.strip('/')
    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', robot_id):
        raise ValueError('Robot id must be a valid ROS node name.')
    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', namespace):
        raise ValueError('Invalid robot namespace.')
    position = robot.get('position', [0., 0., .4 if robot_type == 'go2' else .04])
    if (not isinstance(position, list) or len(position) != 3
            or not all(type(v) in (int, float) and math.isfinite(v) for v in position)):
        raise ValueError('Position must contain three finite numbers.')
    yaw = robot.get('yaw', 0.)
    if type(yaw) not in (int, float) or not math.isfinite(yaw):
        raise ValueError('Yaw must be finite.')
    return {'robot_type': robot_type, 'robot_id': robot_id, 'namespace': namespace, 'position': position,
            'yaw': yaw}


def load_robot_config(path):
    """Compatibility view for callers that explicitly require one robot."""
    world = load_world_config(path)
    if len(world['robots']) != 1:
        raise ValueError('This caller requires exactly one robot; use load_world_config for a fleet.')
    return {**world['robots'][0], 'scenario': world['scenario'], 'seed': world['seed']}
