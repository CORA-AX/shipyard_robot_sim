"""Ackermann targets for front steering and rear drive, using SI units.

Twist vx is longitudinal speed at the rear axle midpoint, wz is yaw rate.
This is inverse kinematics and target limiting, not a path-following controller.
"""
from dataclasses import dataclass
import math
from pathlib import Path

import yaml


def load_parameters(root, path=None):
    path = Path(path) if path is not None else Path(root) / 'config/robot.yaml'
    config = yaml.safe_load(path.read_text())
    if not isinstance(config, dict):
        raise ValueError('Hunter configuration must be a mapping.')
    for name in ('wheel_radius', 'wheelbase', 'front_track', 'rear_track',
                 'max_steering_angle', 'max_steering_rate', 'max_speed', 'max_acceleration'):
        number = config.get(name)
        if isinstance(number, bool) or not isinstance(number, (int, float)) or not math.isfinite(number) or number <= 0:
            raise ValueError(f'Hunter {name} must be finite and positive.')
    # Keep the turning centre outside both axles and all steering angles below pi/2.
    if (config['max_steering_angle'] >= math.pi / 2
            or math.tan(config['max_steering_angle']) / config['wheelbase']
            * max(config['front_track'], config['rear_track']) / 2 >= 1.):
        raise ValueError('Hunter max_steering_angle is too large for its axle geometry.')
    velocity_limit = config.get('rear_velocity_limit')
    if velocity_limit is not None:
        if (type(velocity_limit) not in (int, float) or not math.isfinite(velocity_limit)
                or velocity_limit <= 0):
            raise ValueError('Hunter rear_velocity_limit must be finite and positive in rad/s.')
        if required_rear_velocity_limit(config) > velocity_limit:
            raise ValueError('Hunter max_speed at full steering exceeds rear_velocity_limit.')
    if not isinstance(config.get('usd_path'), str) or not config['usd_path']:
        raise ValueError('Hunter usd_path must identify the saved USD asset.')
    if not isinstance(config.get('base_link'), str) or not config['base_link']:
        raise ValueError('Hunter base_link must be a nonempty name.')
    names = []
    for group in ('steering', 'rear_wheels'):
        entries = config.get(group)
        if not isinstance(entries, list) or len(entries) != 2:
            raise ValueError(f'Hunter {group} requires left and right joints, in that order.')
        for entry in entries:
            if (not isinstance(entry, dict) or not isinstance(entry.get('joint'), str)
                    or not entry['joint'] or type(entry.get('sign')) not in (int, float)
                    or entry['sign'] not in (-1, 1)):
                raise ValueError(f'Each {group} joint requires a name and sign of +1 or -1.')
            names.append(entry['joint'])
    passive = config.get('passive_front_wheels')
    if not isinstance(passive, list) or len(passive) != 2 or not all(isinstance(n, str) and n for n in passive):
        raise ValueError('Hunter requires two passive front wheel joint names.')
    if len(set(names + passive)) != 6:
        raise ValueError('Hunter joint names must be distinct.')
    return config


def required_rear_velocity_limit(config):
    """Largest outer-wheel speed over the complete allowed speed/steering range."""
    curvature = math.tan(config['max_steering_angle']) / config['wheelbase']
    return config['max_speed'] / config['wheel_radius'] * (1. + config['rear_track'] * curvature / 2.)


@dataclass(frozen=True)
class AckermannTargets:
    speed: float
    steering_angle: float
    yaw_rate: float
    steering_positions: tuple[float, float]
    rear_velocities: tuple[float, float]


class Hunter2Controller:
    """Rate-limited body speed and Ackermann steering with passive front wheels.

    Joint signs describe the *saved USD*, not the original URDF. This class
    never steps physics or writes USD files.
    """
    def __init__(self, config):
        self.config = config
        self.steering_names = [entry['joint'] for entry in config['steering']]
        self.rear_names = [entry['joint'] for entry in config['rear_wheels']]
        self.reset()

    def reset(self):
        self.speed = 0.
        self.inner_angle = 0.

    def update_command(self, command, dt):
        """Convert (vx, vy, wz) to joint targets; reject lateral motion and spin.

        Desired curvature comes from requested wz/vx rather than dividing by
        the ramped speed near zero. A stop holds steering while braking.
        """
        vx, vy, wz = command
        if not all(math.isfinite(v) for v in command) or vy != 0. or abs(vx) < 1e-6:
            return self._advance(0., self.inner_angle, dt)
        centre = max(-self.config['max_steering_angle'], min(self.config['max_steering_angle'],
                     math.atan(self.config['wheelbase'] * (wz / vx))))
        curvature = math.tan(centre) / self.config['wheelbase']
        inner = math.atan(self.config['wheelbase'] * curvature /
                          (1. - self.config['front_track'] * abs(curvature) / 2.))
        return self._advance(vx, inner, dt)

    def update(self, requested_speed, dt):
        """Compatibility entry for CLI straight driving."""
        if not math.isfinite(requested_speed) or not math.isfinite(dt) or dt <= 0:
            raise ValueError('Hunter speed must be finite and dt must be finite and positive.')
        return self._advance(requested_speed, 0., dt)

    def _advance(self, requested_speed, inner_angle, dt):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('Hunter dt must be finite and positive.')
        maximum = self.config['max_speed']
        requested_speed = max(-maximum, min(maximum, requested_speed))
        change = self.config['max_acceleration'] * dt
        self.speed += max(-change, min(change, requested_speed - self.speed))
        # Slew the signed inner-wheel angle. The outer angle is a monotonic,
        # shallower function of this angle, so both joints respect the rate
        # limit while sharing exactly the same instantaneous turning centre.
        change = self.config['max_steering_rate'] * dt
        self.inner_angle += max(-change, min(change, inner_angle - self.inner_angle))
        tangent = math.tan(self.inner_angle)
        length, front_half = self.config['wheelbase'], self.config['front_track'] / 2.
        curvature = tangent / (length + front_half * abs(tangent))
        angles = (math.atan2(length * curvature, 1. - front_half * curvature),
                  math.atan2(length * curvature, 1. + front_half * curvature))
        steering = tuple(entry['sign'] * angle for entry, angle in zip(self.config['steering'], angles))
        rear_half = self.config['rear_track'] / 2.
        velocities = tuple(entry['sign'] * self.speed * factor / self.config['wheel_radius']
                           for entry, factor in zip(self.config['rear_wheels'],
                               (1. - rear_half * curvature, 1. + rear_half * curvature)))
        return AckermannTargets(self.speed, math.atan(length * curvature),
                                self.speed * curvature, steering, velocities)
