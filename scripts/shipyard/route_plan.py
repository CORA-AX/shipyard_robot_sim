"""Deterministic surface route planning; no ROS/Isaac dependencies."""
import json
import math
from pathlib import Path


def distance_to_segment(point, start, end):
    dx,dy = end[0]-start[0],end[1]-start[1]
    length2 = dx*dx+dy*dy
    t = max(0.,min(1.,((point[0]-start[0])*dx+(point[1]-start[1])*dy)/length2)) if length2 else 0.
    return math.hypot(point[0]-start[0]-t*dx,point[1]-start[1]-t*dy)


def build_plan(hints_path, robot, route='flat_only', allow_ramps=False, spacing=2.):
    if not math.isfinite(spacing) or not .25 <= spacing <= 3:
        raise ValueError('Waypoint spacing must be between 0.25 and 3 m.')
    hints = json.loads(Path(hints_path).read_text())
    if hints.get('schema_version') != 1 or hints.get('units') != 'meters' or hints.get('up_axis') != 'Z':
        raise ValueError('Expected shipyard route hints v1, meters and Z-up.')
    nodes = hints['nodes']
    if any(len(p)!=3 or not all(type(v) in (int,float) and math.isfinite(v) for v in p)
           for p in nodes.values()):
        raise ValueError('Route nodes must have finite XYZ coordinates.')
    edges = {}
    for edge in hints['edges']:
        edges[(edge['start'],edge['end'])] = edge
        if edge['bidirectional']:
            edges[(edge['end'],edge['start'])] = edge
    if route == 'smoke':
        names = ['yard_start','north_junction'] if robot['robot_type']=='go2' else ['south_junction','bypass_access']
    elif route in hints['suggested_routes']:
        names = list(hints['suggested_routes'][route])
        if robot['robot_type']=='lekiwi':
            if route != 'flat_only':
                raise ValueError('Lekiwi currently supports flat_only and smoke; ramps are not validated.')
            names = list(reversed(names[1:]))  # Stay away from the parked Go2 at yard_start.
    else:
        raise ValueError(f'Unknown route: {route}')
    round_trip_names = names+list(reversed(names[:-1]))
    for a,b in zip(round_trip_names,round_trip_names[1:]):
        edge = edges.get((a,b))
        if edge is None:
            raise ValueError(f'No directed connection: {a} -> {b}')
        if edge['kind']=='stairs' or 'stair_climbing' in edge['requires']:
            raise ValueError('Stairs are disabled: the existing controller is a flat locomotion policy.')
        if edge['kind'] not in ('flat','ramp'):
            raise ValueError('Unsupported terrain kind.')
        if edge['kind']=='ramp' and not allow_ramps:
            raise ValueError('Ramp route requires explicit --allow-ramps; it is experimental.')
        if edge['width_m'] < 1.3:
            raise ValueError('Route is narrower than the validated corridor.')
    points = [list(nodes[name]) for name in names]
    if route=='smoke':
        a,b = points
        length = math.dist(a,b)
        points = [a,[a[i]+(b[i]-a[i])/length for i in range(3)]]
    # Surface coordinates: each robot's base-height offset belongs to the simulator.
    parking = [*robot['position'][:2],0.]
    if math.dist(parking,points[0]) > 2.:
        raise ValueError('Parking must be within 2 m of the route entry; no free-space planner is provided.')
    # Explicit round trip keeps the inactive robot outside the other robot's path.
    vertices = [parking]+points+list(reversed(points[:-1]))+[parking]
    compact = []
    for point in vertices:
        if not compact or math.dist(compact[-1],point)>1e-6:
            compact.append(point)
    result = [compact[0]]
    for a,b in zip(compact,compact[1:]):
        count = max(1,math.ceil(math.dist(a,b)/spacing))
        result.extend([[a[j]+(b[j]-a[j])*i/count for j in range(3)] for i in range(1,count+1)])
    return dict(robot=robot['robot_id'],route=route,frame='map',round_trip=True,
                experimental_ramps=allow_ramps,points=result,
                length_m=sum(math.dist(a,b) for a,b in zip(result,result[1:])))
