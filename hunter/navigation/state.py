"""Shared planar state and constant-curvature motion; no ROS or planner dependency."""
from dataclasses import dataclass
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.x, self.y, self.yaw)):
            raise ValueError('Waypoint pose must be finite.')


@dataclass(frozen=True)
class Segment:
    length: float
    curvature: float


def advance(pose, segment, distance=None):
    distance = segment.length if distance is None else distance
    k = segment.curvature
    yaw = pose.yaw + k * distance
    if abs(k) < 1e-12:
        return Pose(pose.x + distance * math.cos(pose.yaw),
                    pose.y + distance * math.sin(pose.yaw), pose.yaw)
    return Pose(pose.x + (math.sin(yaw) - math.sin(pose.yaw)) / k,
                pose.y + (math.cos(pose.yaw) - math.cos(yaw)) / k, wrap(yaw))


@dataclass(frozen=True)
class Observation:
    pose: Pose
    speed: float
    yaw_rate: float
    seconds: float

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.speed, self.yaw_rate, self.seconds)) or self.seconds < 0.:
            raise ValueError('Odometry requires finite state and nonnegative simulation time.')


