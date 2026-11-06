"""One world profile shared by simulation, map generation and RViz/Nav2."""
from dataclasses import dataclass
import math
from pathlib import Path

import yaml


@dataclass(frozen=True)
class WorldProfile:
    name: str
    usd_path: Path | None
    position: tuple
    yaw: float
    camera_eye: tuple
    camera_target: tuple
    map_file: Path | None
    bounds: tuple
    resolution: float
    view_center: tuple
    view_scale: float


def load_world(root, name='flat'):
    root = Path(root)
    if name not in ('flat', 'shipyard'):
        raise ValueError('Environment must be flat or shipyard.')
    value = yaml.safe_load((root/'config/environments'/f'{name}.yaml').read_text())
    vector = lambda key, count: tuple(float(v) for v in value[key]) if len(value[key]) == count else ()
    position, eye, target = (vector(key, 3) for key in ('position', 'camera_eye', 'camera_target'))
    bounds, center = vector('bounds', 4), vector('view_center', 2)
    yaw, resolution, scale = (float(value[key]) for key in ('yaw', 'resolution', 'view_scale'))
    if (not all((position, eye, target, bounds, center))
            or not all(math.isfinite(v) for v in (*position, *eye, *target, *bounds, *center, yaw, resolution, scale))
            or resolution <= 0. or scale <= 0. or bounds[0] >= bounds[2] or bounds[1] >= bounds[3]):
        raise ValueError('Invalid world profile dimensions or finite coordinates.')
    resolve = lambda key: (root/value[key]).resolve() if value.get(key) else None
    usd = resolve('usd_path')
    if usd is not None and not usd.is_file():
        raise ValueError(f'Missing environment USD: {usd}')
    return WorldProfile(name, usd, position, yaw, eye, target, resolve('map_file'), bounds,
                        resolution, center, scale)
