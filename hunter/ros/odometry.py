"""Rigid-body reference points, independent of ROS, Kit and tensor libraries.

Positions are in world coordinates. Quaternions use Isaac's (w, x, y, z)
order; linear/angular velocities and offsets use the link's axes.
"""
from dataclasses import dataclass
import math

from hunter.navigation.state import Observation, Pose


def vector(values, size=3):
    values = tuple(float(value) for value in values)
    if len(values) != size or not all(math.isfinite(value) for value in values):
        raise ValueError(f'Expected {size} finite coordinates.')
    return values


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


@dataclass(frozen=True)
class FrameState:
    position: tuple
    quaternion: tuple
    linear_velocity: tuple
    angular_velocity: tuple

    def __post_init__(self):
        for name in ('position', 'linear_velocity', 'angular_velocity'):
            object.__setattr__(self, name, vector(getattr(self, name)))
        quaternion = vector(self.quaternion, 4)
        norm = math.sqrt(sum(value * value for value in quaternion))
        if not math.isfinite(norm) or norm < 1e-12:
            raise ValueError('Orientation requires a nonzero finite quaternion.')
        object.__setattr__(self, 'quaternion', tuple(value / norm for value in quaternion))

    def at_offset(self, offset):
        """State of a fixed, parallel frame: v_point = v_link + omega x offset."""
        offset = vector(offset)
        w, *q = self.quaternion
        uv = cross(q, offset)
        uuv = cross(q, uv)
        world_offset = tuple(r + 2. * (w * u + uu) for r, u, uu in zip(offset, uv, uuv))
        velocity_offset = cross(self.angular_velocity, offset)
        return FrameState(tuple(p + r for p, r in zip(self.position, world_offset)), self.quaternion,
                          tuple(v + dv for v, dv in zip(self.linear_velocity, velocity_offset)),
                          self.angular_velocity)


def robot_frames(data, rear_axle_offset):
    """Use explicit IsaacLab link fields; root_lin_vel_b is a CoM alias."""
    base = FrameState(data.root_link_pos_w[0].tolist(), data.root_link_quat_w[0].tolist(),
                      data.root_link_lin_vel_b[0].tolist(), data.root_link_ang_vel_b[0].tolist())
    return base, base.at_offset(rear_axle_offset)


def read_odometry(message, namespace):
    """Decode rear-axle ROS odometry into the planner-independent state."""
    if message.header.frame_id != namespace+'/odom' or message.child_frame_id != namespace+'/base_link':
        raise ValueError('Expected rear-centred odometry with <namespace>/odom and <namespace>/base_link frames.')
    q = message.pose.pose.orientation
    values = (q.w, q.x, q.y, q.z)
    norm = math.sqrt(sum(v*v for v in values))
    if not all(math.isfinite(v) for v in values) or not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('Invalid odometry quaternion.')
    w, x, y, z = (v/norm for v in values)
    yaw = math.atan2(2.*(w*z+x*y), 1.-2.*(y*y+z*z))
    p, twist = message.pose.pose.position, message.twist.twist
    stamp = message.header.stamp
    return Observation(Pose(p.x, p.y, yaw), twist.linear.x, twist.angular.z,
                       stamp.sec + stamp.nanosec*1e-9)
