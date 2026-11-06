"""Geometric Dubins paths, no ROS/Isaac or external planning dependency.

Enumerate circle/line tangents (CSC) and radius-2R circle intersections (CCC).
Reference: LaValle, Planning Algorithms, section 15.3.1.
https://msl.cs.illinois.edu/planning/node821.html
"""
from bisect import bisect_right
import math

from hunter.navigation.state import Pose, Segment, advance, wrap


class Path:
    def __init__(self, start, segments, goal, spacing=.04):
        if not math.isfinite(spacing) or spacing <= 0.:
            raise ValueError('Path spacing must be positive.')
        self.start, self.goal = start, goal
        segments = tuple(segments)
        if any(not all(math.isfinite(v) for v in (s.length, s.curvature)) or s.length < 0. for s in segments):
            raise ValueError('Invalid path segment.')
        self.segments = tuple(s for s in segments if s.length > 1e-10)
        self.ends, self.origins = [], []
        pose, length = start, 0.
        for segment in self.segments:
            self.origins.append(pose)
            pose = advance(pose, segment)
            length += segment.length
            self.ends.append(length)
        self.length = length
        if math.hypot(pose.x-goal.x, pose.y-goal.y) > 1e-6 or abs(wrap(pose.yaw-goal.yaw)) > 1e-6:
            raise ValueError('Path does not terminate at the requested goal pose.')
        count = max(1, math.ceil(length / spacing))
        if count > 100000:
            raise ValueError('Goal is too far away for the configured path sampling.')
        self.samples = tuple((length*i/count, self.at(length*i/count)) for i in range(count+1))

    def at(self, distance):
        if not self.segments:
            return self.start
        distance = max(0., min(self.length, distance))
        index = min(bisect_right(self.ends, distance), len(self.segments)-1)
        beginning = self.ends[index-1] if index else 0.
        return advance(self.origins[index], self.segments[index], distance-beginning)

    def project(self, pose, progress, window=1.):
        """Project locally; prevent jumping to another branch of a looping path."""
        if self.length == 0.:
            return 0., math.hypot(pose.x-self.goal.x, pose.y-self.goal.y)
        low, high = max(0., progress-.1), min(self.length, progress+window)
        best = (float('inf'), progress)
        for (s0, a), (s1, b) in zip(self.samples, self.samples[1:]):
            if s1 < low or s0 > high:
                continue
            dx, dy = b.x-a.x, b.y-a.y
            square = dx*dx + dy*dy
            t = max(0., min(1., ((pose.x-a.x)*dx + (pose.y-a.y)*dy)/square)) if square else 0.
            distance = math.hypot(pose.x-a.x-t*dx, pose.y-a.y-t*dy)
            best = min(best, (distance, s0+t*(s1-s0)))
        return max(progress, best[1]), best[0]


def _arc(angle):
    result = angle % (2.*math.pi)
    return 0. if result < 1e-10 or 2.*math.pi-result < 1e-10 else result


def _centre(pose, sign, radius):
    return (pose.x-sign*radius*math.sin(pose.yaw), pose.y+sign*radius*math.cos(pose.yaw))


def dubins(start, goal, radius):
    """Shortest of the CSC/CCC geometric candidates at the specified radius."""
    if not math.isfinite(radius) or radius <= 0.:
        raise ValueError('Turning radius must be positive.')
    candidates = []
    for first in (1, -1):
        a = _centre(start, first, radius)
        for last in (1, -1):
            b = _centre(goal, last, radius)
            dx, dy = b[0]-a[0], b[1]-a[1]
            d = math.hypot(dx, dy)
            if d < 1e-10:
                if first == last:
                    candidates.append((Segment(radius*_arc(first*(goal.yaw-start.yaw)), first/radius),))
                continue
            ratio = (last-first)*radius/d
            if abs(ratio) > 1.+1e-12:
                continue
            heading = math.atan2(dy, dx)-math.asin(max(-1., min(1., ratio)))
            candidates.append((Segment(radius*_arc(first*(heading-start.yaw)), first/radius),
                               Segment(math.sqrt(max(0., d*d-((last-first)*radius)**2)), 0.),
                               Segment(radius*_arc(last*(goal.yaw-heading)), last/radius)))
        # Equal outer turn directions; the middle circle is 2R from each centre.
        b = _centre(goal, first, radius)
        dx, dy = b[0]-a[0], b[1]-a[1]
        d = math.hypot(dx, dy)
        if not 1e-10 < d <= 4.*radius:
            continue
        height = math.sqrt(max(0., 4.*radius*radius-d*d/4.))
        for side in (1, -1):
            middle = ((a[0]+b[0])/2.-side*dy*height/d,
                      (a[1]+b[1])/2.+side*dx*height/d)
            heading1 = math.atan2(middle[1]-a[1], middle[0]-a[0])+first*math.pi/2.
            heading2 = math.atan2(middle[1]-b[1], middle[0]-b[0])+first*math.pi/2.
            candidates.append((Segment(radius*_arc(first*(heading1-start.yaw)), first/radius),
                               Segment(radius*_arc(-first*(heading2-heading1)), -first/radius),
                               Segment(radius*_arc(first*(goal.yaw-heading2)), first/radius)))
    if not candidates:
        raise ValueError('No forward Dubins path found.')
    return min(candidates, key=lambda segments: sum(s.length for s in segments))


def plan(start, goal, radius, approach=.8):
    """End with a straight approach; this is a tracking baseline, not a time optimum."""
    if not math.isfinite(approach) or approach < 0.:
        raise ValueError('Approach distance must be nonnegative.')
    dx, dy = goal.x-start.x, goal.y-start.y
    forward = dx*math.cos(start.yaw) + dy*math.sin(start.yaw)
    lateral = -dx*math.sin(start.yaw) + dy*math.cos(start.yaw)
    if abs(lateral) < 1e-8 and forward >= 0. and abs(wrap(goal.yaw-start.yaw)) < 1e-8:
        return Path(start, (Segment(forward, 0.),), goal)
    separation = math.hypot(dx, dy)
    if separation > 1e-6:
        approach = min(approach, separation/2.)
    entry = Pose(goal.x-approach*math.cos(goal.yaw), goal.y-approach*math.sin(goal.yaw), goal.yaw)
    segments = (*dubins(start, entry, radius), Segment(approach, 0.))
    return Path(start, segments, goal)
