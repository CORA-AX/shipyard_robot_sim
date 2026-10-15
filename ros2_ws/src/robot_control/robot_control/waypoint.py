"""Planar waypoint PD math, independent of ROS, Isaac and terminal input."""
from dataclasses import dataclass, fields
import math
from pathlib import Path
import re

import yaml


ZERO = (0., 0., 0.)


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def quaternion(value):
    if len(value) != 4 or not all(math.isfinite(v) for v in value):
        raise ValueError('Quaternion must contain four finite values (x,y,z,w).')
    norm = math.sqrt(sum(v * v for v in value))
    if norm < 1e-8:
        raise ValueError('Quaternion has zero norm.')
    return tuple(v / norm for v in value)


def rotate(q, vector):
    x, y, z, w = quaternion(q)
    vx, vy, vz = vector
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


def yaw(q):
    x, y, z, w = quaternion(q)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def multiply(a, b):
    ax, ay, az, aw = quaternion(a)
    bx, by, bz, bw = quaternion(b)
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def clip_vector(x, y, limit):
    norm = math.hypot(x, y)
    scale = min(1., limit / norm) if norm else 1.
    return x * scale, y * scale


@dataclass(frozen=True)
class Parameters:
    kp_position: float
    kd_velocity: float
    kp_yaw: float
    kd_yaw_rate: float
    max_speed: float
    max_yaw_rate: float
    max_acceleration: float
    max_yaw_acceleration: float
    position_tolerance: float
    yaw_tolerance: float
    settle_speed: float
    settle_yaw_rate: float
    settle_time: float
    velocity_filter_tau: float
    goal_timeout: float

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            nonnegative = field.name in ('kd_velocity', 'kd_yaw_rate', 'settle_time', 'velocity_filter_tau')
            if type(value) not in (int, float) or not math.isfinite(value) or value < 0 or (not nonnegative and value == 0):
                raise ValueError(f'Invalid waypoint parameter {field.name}.')


@dataclass(frozen=True)
class State:
    time: float
    x: float
    y: float
    yaw: float
    vx: float  # world/map coordinates
    vy: float
    wz: float


def load_settings(path, robots_file=None):
    settings = yaml.safe_load(Path(path).read_text())
    if not isinstance(settings, dict):
        raise ValueError('Waypoint config must be a YAML mapping.')
    for key in ('rate', 'odom_timeout', 'goal_max_age', 'switch_timeout'):
        value = settings.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'Waypoint {key} must be finite and positive.')
    profiles = {name: Parameters(**values) for name, values in settings.get('profiles', {}).items()}
    robots = settings.get('robots')
    if robots_file:
        robots = yaml.safe_load(Path(robots_file).read_text()).get('robots')
    if not isinstance(robots, list) or not robots:
        raise ValueError('Provide at least one robot.')
    normalized = []
    for robot in robots:
        if not isinstance(robot, dict) or robot.get('type') not in ('go2', 'lekiwi'):
            raise ValueError('Waypoint supports Go2 and Lekiwi profiles.')
        identity = robot.get('id', robot['type'] + '_0')
        namespace = robot.get('namespace', identity)
        if (not isinstance(identity, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', identity)
                or not isinstance(namespace, str)
                or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', namespace.strip('/'))
                or robot['type'] not in profiles):
            raise ValueError('Invalid robot id, namespace or missing profile.')
        normalized.append(dict(id=identity, namespace=namespace.strip('/'), type=robot['type']))
    for key in ('id', 'namespace'):
        if len({r[key] for r in normalized}) != len(normalized):
            raise ValueError(f'Waypoint robots must have unique {key}.')
    return settings, profiles, normalized


class WaypointPD:
    def __init__(self, parameters):
        self.parameters = parameters
        self.state = None
        self.velocity = ZERO
        self.command = ZERO
        self.goal = None
        self.status = 'IDLE'
        self.started = self.settled_since = None
        self.last_step = None

    def update(self, state):
        if not all(math.isfinite(getattr(state, field.name)) for field in fields(State)):
            raise ValueError('Non-finite odometry.')
        previous = self.state
        if previous is not None and state.time <= previous.time:
            if state.time < previous.time:
                self.cancel('CLOCK_RESET')
                self.state = state
                self.velocity = (state.vx, state.vy, state.wz)
            return False
        self.state = state
        dt = state.time - previous.time if previous is not None else 0.
        alpha = dt / (self.parameters.velocity_filter_tau + dt) if dt else 1.
        self.velocity = tuple(a + alpha * (b - a) for a, b in
                              zip(self.velocity, (state.vx, state.vy, state.wz)))
        # Ground-truth teleports also cancel goals when an older simulator lacks reset_count.
        if previous is not None and (math.hypot(state.x - previous.x, state.y - previous.y)
                > .25 + self.parameters.max_speed * dt * 3
                or abs(wrap_angle(state.yaw - previous.yaw)) > .6 + self.parameters.max_yaw_rate * dt * 3):
            self.cancel('POSE_RESET')
        return True

    def set_goal(self, x, y, heading=None):
        if self.state is None or not all(math.isfinite(v) for v in (x, y)):
            raise ValueError('A finite goal and odometry are required.')
        if heading is not None and not math.isfinite(heading):
            raise ValueError('Invalid goal heading.')
        self.goal = (x, y, self.state.yaw if heading is None else heading, heading is not None)
        self.started = self.state.time
        self.settled_since = None
        self.last_step = self.state.time
        self.status = 'TRACKING'

    def cancel(self, reason='CANCELED'):
        self.goal = None
        self.command = ZERO
        self.last_step = self.settled_since = None
        self.status = reason

    def stopped(self):
        p = self.parameters
        return math.hypot(*self.velocity[:2]) <= p.settle_speed and abs(self.velocity[2]) <= p.settle_yaw_rate

    def step(self):
        if self.goal is None or self.state is None:
            self.command = ZERO
            return ZERO
        state, p = self.state, self.parameters
        if self.last_step is not None and state.time <= self.last_step:
            return self.command
        dt = min(.1, state.time - self.last_step) if self.last_step is not None else .02
        self.last_step = state.time
        if state.time - self.started > p.goal_timeout:
            self.cancel('GOAL_TIMEOUT')
            return ZERO
        gx, gy, heading, track_heading = self.goal
        ex, ey = gx - state.x, gy - state.y
        angle_error = wrap_angle(heading - state.yaw)
        near = math.hypot(ex, ey) <= p.position_tolerance
        aligned = not track_heading or abs(angle_error) <= p.yaw_tolerance
        if near and aligned and self.stopped():
            if self.settled_since is None:
                self.settled_since = state.time
            if state.time - self.settled_since >= p.settle_time:
                self.cancel('REACHED')
                return ZERO
        else:
            self.settled_since = None
        # D uses measured velocity, avoiding a derivative kick when replacing a goal.
        vx, vy = (0., 0.) if near else clip_vector(
            p.kp_position * ex - p.kd_velocity * self.velocity[0],
            p.kp_position * ey - p.kd_velocity * self.velocity[1], p.max_speed)
        c, s = math.cos(state.yaw), math.sin(state.yaw)
        body_x, body_y = c * vx + s * vy, -s * vx + c * vy
        wz = max(-p.max_yaw_rate, min(p.max_yaw_rate,
                 p.kp_yaw * angle_error - p.kd_yaw_rate * self.velocity[2]))
        dx, dy = clip_vector(body_x - self.command[0], body_y - self.command[1], p.max_acceleration * dt)
        dw = max(-p.max_yaw_acceleration * dt, min(p.max_yaw_acceleration * dt, wz - self.command[2]))
        self.command = (self.command[0] + dx, self.command[1] + dy, self.command[2] + dw)
        return self.command
