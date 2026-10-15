"""Omni-wheel inverse kinematics; no ROS or simulator imports."""
import math
from pathlib import Path

import yaml


def load_parameters(root):
    path = Path(root) / 'ros2_ws/src/robot_control/config/lekiwi.yaml'
    config = yaml.safe_load(path.read_text())
    for name in ('wheel_radius', 'wheel_damping', 'wheel_torque', 'roller_radius', 'roller_mass'):
        if not math.isfinite(config[name]) or config[name] <= 0:
            raise ValueError(f'Lekiwi {name} must be finite and positive.')
    if type(config['rollers_per_wheel']) is not int or config['rollers_per_wheel'] < 8:
        raise ValueError('Use at least eight rollers per wheel.')
    if config['roller_radius'] >= config['wheel_radius']:
        raise ValueError('Roller radius must be smaller than wheel radius.')
    wheels = config.get('wheels', [])
    if len(wheels) != 3 or len({w['joint'] for w in wheels}) != 3 or len({w['body'] for w in wheels}) != 3:
        raise ValueError('Lekiwi requires three distinct wheel joints and bodies.')
    for wheel in wheels:
        if (len(wheel['position']) != 3 or len(wheel['axle']) != 2
                or not all(math.isfinite(v) for v in [*wheel['position'], *wheel['axle'], wheel['hub_offset']])
                or math.hypot(*wheel['axle']) < 1e-6):
            raise ValueError('Wheel position, axle and hub offset must be finite with a nonzero axle.')
    return config


class LekiwiController:
    """Map body Twist to wheel rad/s in configured joint order.

    Positive joint axis a=(ax, ay) gives rolling direction a cross +Z=(ay, -ax).
    Each wheel uses its actual contact center, including the hub's axle offset.
    """
    def __init__(self, config):
        self.config = config
        self.joint_names = [wheel['joint'] for wheel in config['wheels']]
        self.rows = []
        for wheel in config['wheels']:
            ax, ay = wheel['axle']
            norm = math.hypot(ax, ay)
            if not math.isfinite(norm) or norm < 1e-6:
                raise ValueError('Invalid Lekiwi wheel axle.')
            ax, ay = ax / norm, ay / norm
            x, y = wheel['position'][:2]
            x += ax * wheel['hub_offset']
            y += ay * wheel['hub_offset']
            self.rows.append((ay, -ax, -ax * x - ay * y))

    def wheel_velocities(self, command):
        if len(command) != 3 or not all(math.isfinite(v) for v in command):
            raise ValueError('Expected three finite body velocities.')
        return [sum(a * b for a, b in zip(row, command)) / self.config['wheel_radius']
                for row in self.rows]
