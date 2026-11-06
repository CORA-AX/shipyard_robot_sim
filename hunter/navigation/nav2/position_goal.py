"""Finite terminal-heading search for position-only Nav2 goals.

Only candidate headings and path measurements live here. Smac still generates
every executed path. Finite sampling does not guarantee a continuous optimum.
"""
import math

from hunter.navigation.state import wrap


def heading_candidates(start, x, y, radius, bins=12):
    """Uniform headings plus bearing, current heading and circle tangents.

    Tangents from the initial left/right turning circles to the target often
    give useful arc-then-straight arrivals in the current empty test world.
    They are candidates, not a substitute for Nav2 planning/collision checks.
    """
    if (not all(math.isfinite(v) for v in (x, y, radius)) or radius <= 0.
            or type(bins) is not int or bins < 4):
        raise ValueError('Use finite goal coordinates, positive radius and at least four heading bins.')
    headings = [math.atan2(y-start.y, x-start.x), start.yaw]
    for sign in (1, -1):
        cx = start.x-sign*radius*math.sin(start.yaw)
        cy = start.y+sign*radius*math.cos(start.yaw)
        dx, dy = x-cx, y-cy
        distance = math.hypot(dx, dy)
        if distance >= radius:
            headings.append(math.atan2(dy, dx)+math.asin(sign*radius/distance))
    headings.extend(2.*math.pi*i/bins for i in range(bins))
    result = []
    for value in headings:
        value = wrap(value)
        if not any(abs(wrap(value-other)) < 1e-6 for other in result):
            result.append(value)
    return result


def path_distance(points):
    """Polyline distance for finite XY pairs; reject empty planner results."""
    points = tuple(points)
    if len(points) < 2 or any(not all(math.isfinite(v) for v in point) for point in points):
        raise ValueError('Nav2 must return at least two finite path points.')
    return sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(points, points[1:]))
