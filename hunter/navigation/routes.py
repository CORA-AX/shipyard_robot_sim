"""Stored flat shipyard checkpoints, independent of ROS and simulation."""
from dataclasses import dataclass
import copy
import math
from pathlib import Path

import yaml
from hunter.navigation.nav2.config import FOOTPRINT


@dataclass(frozen=True)
class Checkpoint:
    name: str
    x: float
    y: float


@dataclass(frozen=True)
class PatrolRoute:
    name: str
    start: tuple
    start_tolerance: float
    points: tuple

    def at_start(self, x, y):
        return math.hypot(x-self.start[0], y-self.start[1]) <= self.start_tolerance


def patrol_parameters(parameters, namespace):
    """Leave tracking room for forward-only turnaround arcs at junctions."""
    result = copy.deepcopy(parameters)
    for costmap in ('global_costmap', 'local_costmap'):
        values = result[f'/{namespace}/{costmap}/{costmap}']['ros__parameters']
        values['footprint_padding'] = .20
        values['inflation_layer'].update(inflate_around_unknown=True, inflation_radius=1.2)
    return result


def route_path_clear(grid, poses):
    """Require tracking room around every sampled rear-axle pose."""
    return all(grid.footprint_free(x, y, yaw, FOOTPRINT, padding=.20) for x,y,yaw in poses)


def load_route(root, name, grid):
    source = yaml.safe_load((Path(root)/'config/routes/shipyard.yaml').read_text())
    if source['frame'] != 'map' or name not in source['routes']:
        raise ValueError('Unknown shipyard route or unsupported coordinate frame.')

    def xy(value):
        if len(value) != 2 or not all(type(v) in (int, float) and math.isfinite(v) for v in value):
            raise ValueError('Route coordinates must be finite XY metres.')
        x, y = map(float, value)
        if not grid.inside(x, y, .2) or not grid.free(x, y):
            raise ValueError(f'Route point {(x,y)} is outside supported free ground.')
        return x, y

    start = xy(source['start'])
    tolerance = float(source['start_tolerance_m'])
    if not math.isfinite(tolerance) or not 0. < tolerance <= .5:
        raise ValueError('Start tolerance must be positive and at most 0.5 m.')
    points = tuple(Checkpoint(str(p['name']), *xy(p['xy'])) for p in source['routes'][name])
    if not points or len({p.name for p in points}) != len(points):
        raise ValueError('Provide a nonempty route with unique checkpoint names.')
    if math.hypot(points[-1].x-start[0], points[-1].y-start[1]) > 1e-6:
        raise ValueError('Patrol must return to the starting position.')
    return PatrolRoute(name, start, tolerance, points)
