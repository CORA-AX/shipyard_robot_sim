"""Paths and validation for the additive shipyard tools."""
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT/'scripts')]
from control.workspace import add_ros_packages
add_ros_packages()

FLEET = Path(__file__).with_name('fleet.yaml')
HINTS = ROOT/'assets/environments/shipyard/route_hints.json'
PD_CONFIG = ROOT/'ros2_ws/src/robot_control/config/waypoint.yaml'


def load_fleet(path=FLEET):
    import yaml
    from robot_sim.config import _robot_config
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict) or not isinstance(data.get('world'), dict):
        raise ValueError('A world mapping is required.')
    if data['world'].get('scenario') != 'shipyard':
        raise ValueError('This entrypoint only supports shipyard.')
    seed = data['world'].get('seed',42)
    if type(seed) is not int or seed < 0:
        raise ValueError('Seed must be a nonnegative integer.')
    if not isinstance(data.get('robots'),list) or not data['robots']:
        raise ValueError('At least one robot is required.')
    robots = [_robot_config(r) for r in data['robots']]
    for key in ('robot_id','namespace'):
        if len({r[key] for r in robots}) != len(robots):
            raise ValueError(f'Duplicate {key}.')
    for i, first in enumerate(robots):
        for second in robots[i+1:]:
            if math.dist(first['position'][:2],second['position'][:2]) < 1.2:
                raise ValueError('Initial robot positions must be separated by at least 1.2 m.')
    return dict(scenario='shipyard',seed=seed,robots=robots)
