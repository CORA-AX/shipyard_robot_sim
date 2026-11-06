"""Conservative Go2 collision-envelope checks over ROS-style occupancy grids."""
import math


def goal_is_free(grid, x, y, yaw, *, any_heading=False, pad_cells=True):
    """Check the padded robot envelope against occupied/unknown grid cells.

    Goal acceptance also expands by half a cell diagonal. Recorded trajectory
    checks can use cell centers, matching Nav2's discrete footprint evaluation.
    """
    resolution = grid.info.resolution
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    radius = math.hypot(.44, .29) + resolution
    margin = resolution*.71 if pad_cells else 0.
    cosine, sine = math.cos(yaw), math.sin(yaw)
    for row in range(math.floor((y-radius-oy)/resolution), math.ceil((y+radius-oy)/resolution)+1):
        for col in range(math.floor((x-radius-ox)/resolution), math.ceil((x+radius-ox)/resolution)+1):
            dx, dy = ox+(col+.5)*resolution-x, oy+(row+.5)*resolution-y
            bx, by = cosine*dx+sine*dy, -sine*dx+cosine*dy
            outside = (math.hypot(dx,dy) > math.hypot(.44,.29)+margin if any_heading
                       else abs(bx) > .44+margin or abs(by) > .29+margin)
            if outside:
                continue
            if not (0 <= row < grid.info.height and 0 <= col < grid.info.width):
                return False
            if grid.data[row*grid.info.width+col] != 0:
                return False
    return True
