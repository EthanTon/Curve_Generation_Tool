import numpy as np


def double_step_line(x0, y0, x1, y1):
    """
    Python implementation of Zingl's AA line algorithm.
    Returns a list of (x, y, intensity) tuples.
    """
    points = []

    dx = np.abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = np.abs(y1 - y0)
    sy = 1 if y0 < y1 else -1

    err = dx - dy
    # Euclidean distance for intensity scaling
    ed = 1 if (dx + dy == 0) else np.sqrt(dx**2 + dy**2)

    while True:
        # Main pixel plotting
        # The intensity is calculated based on the distance from the ideal line

        points.append((x0, y0))

        e2 = err
        x2 = x0

        # Horizontal step
        if 2 * e2 >= -dx:
            if x0 == x1:
                break
            if e2 + dy < ed:
                points.append((x0, y0 + sy))
            err -= dy
            x0 += sx

        # Vertical step
        if 2 * e2 <= dy:
            if y0 == y1:
                break
            if dx - e2 < ed:
                points.append((x2 + sx, y0))
            err += dx
            y0 += sy

    return points


def step_line(x0, y0, x1, y1):
    """Generates points on a line where every pixel touches by a face (4-connectivity)"""
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while True:
        points.append((x0, y0))

        if x0 == x1 and y0 == y1:
            break

        e2 = 2 * err

        # KEY MODIFICATION:
        # If both conditions are met, a diagonal jump is about to happen.
        # We insert an intermediate pixel to ensure face-to-face contact.
        if e2 > -dy and e2 < dx:
            # You can choose to move in X or Y first.
            # Moving in X first is the standard convention here.
            points.append((x0 + sx, y0))

        if e2 > -dy:
            err -= dy
            x0 += sx

        if e2 < dx:
            err += dx
            y0 += sy

    return points


def bresenham_line(x0, y0, x1, y1):
    """Generates points on a line from (x0, y0) to (x1, y1) using Bresenham's algorithm"""
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        err2 = err * 2
        if err2 > -dy:
            err -= dy
            x0 += sx
        if err2 < dx:
            err += dx
            y0 += sy

    return points


"""
1 (xc - x, yc + y)
2 (xc + y, yc - x)
3 (xc - y, yc - x)
4 (xc + x, yc + y)
5 (xc + x, yc - y)
6 (xc - y, yc + x)
7 (xc + y, yc + x)
8 (xc - x, yc - y)
"""


def bresenham_arc_4connected(xc, yc, r, start_point, end_point, counterclockwise=True):
    """
    Generates points on an arc with 4-connectivity (no diagonal jumps).
    """
    # Initialize variables for the circle algorithm
    x, y, err = -r, 0, 2 - 2 * r
    points = []

    start_angle = determine_angle((xc, yc), start_point)
    end_angle = determine_angle((xc, yc), end_point)

    # 8 lists to store points for each octant
    octal_points = [[] for _ in range(8)]

    def process_step(cur_x, cur_y):
        """Helper to assign current points to their respective octants if they fall in the arc."""
        candidates = [
            (xc - cur_x, yc + cur_y),
            (xc + cur_y, yc - cur_x),
            (xc - cur_y, yc - cur_x),
            (xc + cur_x, yc + cur_y),
            (xc + cur_x, yc - cur_y),
            (xc - cur_y, yc + cur_x),
            (xc + cur_y, yc + cur_x),
            (xc - cur_x, yc - cur_y),
        ]
        for n, (px, py) in enumerate(candidates):
            point_angle = determine_angle((xc, yc), (px, py))
            if angle_in_arc(point_angle, start_angle, end_angle, counterclockwise):
                # We maintain directionality within each octant list
                if counterclockwise == (n % 2 == 0):
                    octal_points[n].append((px, py))
                else:
                    octal_points[n].insert(0, (px, py))

    # Main Algorithm Loop
    while abs(x) >= y:
        process_step(x, y)

        prev_err = err
        # Step in Y
        if prev_err <= y:
            y += 1
            err += y * 2 + 1
            # If a diagonal move is happening (X will also step),
            # add intermediate point for 4-connectivity
            if prev_err > x or err > y:
                process_step(x, y)

        # Step in X
        if prev_err > x or err > y:
            x += 1
            err += x * 2 + 1

        # Ensure we don't skip the last point in the loop
        if abs(x) < y:
            process_step(x, y)

    # --- Octant Stitching Logic ---
    step = 1 if counterclockwise else -1
    index = int(start_angle * 4 // np.pi)
    end = int(end_angle * 4 // np.pi)
    n = len(octal_points)

    # Traverse octants in order to build the final continuous path
    while True:
        curr_idx = index % n

        if octal_points[curr_idx]:
            # Avoid duplicate points where octants meet
            if points and points[-1] == octal_points[curr_idx][0]:
                points.extend(octal_points[curr_idx][1:])
            else:
                points.extend(octal_points[curr_idx])

        if curr_idx == end % n:
            break
        index += step

    return points


def bresenham_arc(xc, yc, r, start_point, end_point, counterclockwise=True):
    """Generate arc points using Bresenham's algorithm"""
    x, y, err = -r, 0, 2 - 2 * r
    points = []

    start_angle = determine_angle((xc, yc), start_point)
    end_angle = determine_angle((xc, yc), end_point)

    octal_points = [[] for _ in range(8)]

    while abs(x) >= y:
        # All 8 octants of the circle
        candidates = [
            (xc - x, yc + y),
            (xc + y, yc - x),
            (xc - y, yc - x),
            (xc + x, yc + y),
            (xc + x, yc - y),
            (xc - y, yc + x),
            (xc + y, yc + x),
            (xc - x, yc - y),
        ]
        for n in range(0, len(candidates)):
            px, py = candidates[n]
            point_angle = determine_angle((xc, yc), (px, py))
            if angle_in_arc(point_angle, start_angle, end_angle, counterclockwise):
                if counterclockwise == (n % 2 == 0):
                    octal_points[n].append((px, py))
                else:
                    octal_points[n].insert(0, (px, py))

        r = err
        if r <= y:
            y += 1
            err += y * 2 + 1
        if r > x or err > y:
            x += 1
            err += x * 2 + 1

    step = 1 if counterclockwise else -1
    index = int(start_angle * 4 // np.pi)
    end = (
        int(end_angle * 4 // np.pi) if counterclockwise else int(end_angle * 4 // np.pi)
    )
    n = len(octal_points)

    while True:
        prev_idx = (index - step) % n
        curr_idx = index % n

        if octal_points[prev_idx] and octal_points[curr_idx]:
            if octal_points[curr_idx][0] == octal_points[prev_idx][-1]:
                octal_points[prev_idx].pop()

        if octal_points[curr_idx]:
            points.extend(octal_points[curr_idx])

        if index % n == end % n:
            break

        index += step

    return points


def bresenham_filled_arc_4connected(
    xc, yc, r, start_point, end_point, counterclockwise=True
):
    boundary_points = []

    # A. Use 4-connected lines for the radial edges
    boundary_points.extend(step_line(xc, yc, start_point[0], start_point[1]))
    boundary_points.extend(step_line(xc, yc, end_point[0], end_point[1]))

    # B. Generate 4-connected perimeter
    x, y, err = -r, 0, 2 - 2 * r
    start_angle = determine_angle((xc, yc), start_point)
    end_angle = determine_angle((xc, yc), end_point)

    while abs(x) >= y:

        def add_perimeter(cur_x, cur_y):
            candidates = [
                (xc - cur_x, yc + cur_y),
                (xc + cur_y, yc - cur_x),
                (xc - cur_y, yc - cur_x),
                (xc + cur_x, yc + cur_y),
                (xc + cur_x, yc - cur_y),
                (xc - cur_y, yc + cur_x),
                (xc + cur_y, yc + cur_x),
                (xc - cur_x, yc - cur_y),
            ]
            for px, py in candidates:
                if angle_in_arc(
                    determine_angle((xc, yc), (px, py)),
                    start_angle,
                    end_angle,
                    counterclockwise,
                ):
                    boundary_points.append((px, py))

        add_perimeter(x, y)

        curr_err = err
        if curr_err <= y:
            y += 1
            err += y * 2 + 1
            if curr_err > x or err > y:  # Intermediate point for 4-connectivity
                add_perimeter(x, y)
        if curr_err > x or err > y:
            x += 1
            err += x * 2 + 1

    # C. Group and Fill Scanlines (Logic remains identical, but input is now watertight)
    y_spans = {}
    for px, py in boundary_points:
        if py not in y_spans:
            y_spans[py] = [px, px]
        else:
            y_spans[py][0] = min(y_spans[py][0], px)
            y_spans[py][1] = max(y_spans[py][1], px)

    final_points = []
    for py, (min_x, max_x) in y_spans.items():
        for px in range(min_x, max_x + 1):
            final_points.append((px, py))

    return final_points


def bresenham_filled_arc_stepped(
    xc, yc, r, start_point, end_point, counterclockwise=True
):
    """
    Fills an arc where every boundary pixel has all faces covered by an
    additional neighbor pixel, similar to double_step_line.

    Instead of post-hoc dilation, the arc perimeter algorithm emits extra
    adjacent pixels whenever a diagonal step occurs — one offset along each
    axis of change — so that no face of any boundary pixel is left exposed.
    Radial edges use double_step_line for the same coverage guarantee.
    """
    if r <= 0:
        return [(xc, yc)]

    boundary_points = []

    # A. Radial edges with double-step coverage
    boundary_points.extend(double_step_line(xc, yc, start_point[0], start_point[1]))
    boundary_points.extend(double_step_line(xc, yc, end_point[0], end_point[1]))

    # B. Arc perimeter with double-step neighbor pixels on diagonal moves
    x, y, err = -r, 0, 2 - 2 * r
    start_angle = determine_angle((xc, yc), start_point)
    end_angle = determine_angle((xc, yc), end_point)

    def add_perimeter(cur_x, cur_y):
        """Emit the 8 symmetric candidates filtered to the arc range."""
        candidates = [
            (xc - cur_x, yc + cur_y),
            (xc + cur_y, yc - cur_x),
            (xc - cur_y, yc - cur_x),
            (xc + cur_x, yc + cur_y),
            (xc + cur_x, yc - cur_y),
            (xc - cur_y, yc + cur_x),
            (xc + cur_y, yc + cur_x),
            (xc - cur_x, yc - cur_y),
        ]
        for px, py in candidates:
            if angle_in_arc(
                determine_angle((xc, yc), (px, py)),
                start_angle,
                end_angle,
                counterclockwise,
            ):
                boundary_points.append((px, py))

    def add_double_step_neighbors(prev_x, prev_y, new_x, new_y):
        """
        On a diagonal step, emit extra pixels for both intermediate positions
        (only-Y-changed and only-X-changed), filtered to the arc range.
        Analogous to double_step_line's extra neighbor emissions.
        """
        # Intermediate: Y changed, X stayed at prev
        intermediates_y = [
            (xc - prev_x, yc + new_y),
            (xc + new_y, yc - prev_x),
            (xc - new_y, yc - prev_x),
            (xc + prev_x, yc + new_y),
            (xc + prev_x, yc - new_y),
            (xc - new_y, yc + prev_x),
            (xc + new_y, yc + prev_x),
            (xc - prev_x, yc - new_y),
        ]
        # Intermediate: X changed, Y stayed at prev
        intermediates_x = [
            (xc - new_x, yc + prev_y),
            (xc + prev_y, yc - new_x),
            (xc - prev_y, yc - new_x),
            (xc + new_x, yc + prev_y),
            (xc + new_x, yc - prev_y),
            (xc - prev_y, yc + new_x),
            (xc + prev_y, yc + new_x),
            (xc - new_x, yc - prev_y),
        ]
        for px, py in intermediates_y + intermediates_x:
            if angle_in_arc(
                determine_angle((xc, yc), (px, py)),
                start_angle,
                end_angle,
                counterclockwise,
            ):
                boundary_points.append((px, py))

    while abs(x) >= y:
        add_perimeter(x, y)

        prev_x, prev_y = x, y
        curr_err = err

        stepped_y = False
        stepped_x = False

        if curr_err <= y:
            y += 1
            err += y * 2 + 1
            stepped_y = True

        if curr_err > prev_x or err > y:
            x += 1
            err += x * 2 + 1
            stepped_x = True

        # Diagonal step — add double-step neighbor pixels for face coverage
        if stepped_x and stepped_y:
            add_double_step_neighbors(prev_x, prev_y, x, y)

        add_perimeter(x, y)

    # C. Build scanline spans from boundary and fill
    y_spans = {}
    for px, py in boundary_points:
        if py not in y_spans:
            y_spans[py] = [px, px]
        else:
            if px < y_spans[py][0]:
                y_spans[py][0] = px
            if px > y_spans[py][1]:
                y_spans[py][1] = px

    final_points = []
    for py, (min_x, max_x) in y_spans.items():
        for px in range(min_x, max_x + 1):
            final_points.append((px, py))

    return final_points


def bresenham_filled_arc(xc, yc, r, start_point, end_point, counterclockwise=True):
    """Fills an arc by defining the boundary and filling horizontal spans (scanlines)."""

    # 1. Collect all boundary points of the wedge
    boundary_points = []

    # A. Add the two radial lines (the straight edges of the slice)
    boundary_points.extend(bresenham_line(xc, yc, start_point[0], start_point[1]))
    boundary_points.extend(bresenham_line(xc, yc, end_point[0], end_point[1]))

    # B. Add the arc perimeter points
    x, y, err = -r, 0, 2 - 2 * r
    start_angle = determine_angle((xc, yc), start_point)
    end_angle = determine_angle((xc, yc), end_point)

    while abs(x) >= y:
        candidates = [
            (xc - x, yc + y),
            (xc + y, yc - x),
            (xc - y, yc - x),
            (xc + x, yc + y),
            (xc + x, yc - y),
            (xc - y, yc + x),
            (xc + y, yc + x),
            (xc - x, yc - y),
        ]
        for px, py in candidates:
            point_angle = determine_angle((xc, yc), (px, py))
            if angle_in_arc(point_angle, start_angle, end_angle, counterclockwise):
                boundary_points.append((px, py))

        curr_err = err
        if curr_err <= y:
            y += 1
            err += y * 2 + 1
        if curr_err > x or err > y:
            x += 1
            err += x * 2 + 1

    # 2. Group boundary points by Y-coordinate to fill spans
    y_spans = {}
    for px, py in boundary_points:
        if py not in y_spans:
            y_spans[py] = [px, px]
        else:
            y_spans[py][0] = min(y_spans[py][0], px)
            y_spans[py][1] = max(y_spans[py][1], px)

    # 3. Create the final filled point list
    final_points = []
    for py, (min_x, max_x) in y_spans.items():
        for px in range(min_x, max_x + 1):
            final_points.append((px, py))

    return final_points


def normalize_angle(angle):
    """Normalize angle to [0, 2π)"""
    while angle < 0:
        angle += 2 * np.pi
    while angle >= 2 * np.pi:
        angle -= 2 * np.pi
    return angle


def angle_in_arc(angle, start_angle, end_angle, counterclockwise=True):
    """Check if angle is within arc from start_angle to end_angle"""
    angle = normalize_angle(angle)
    start = normalize_angle(start_angle)
    end = normalize_angle(end_angle)

    if counterclockwise:
        # Arc goes counterclockwise from start to end
        if start <= end:
            return start <= angle <= end
        else:
            # Arc crosses 0
            return angle >= start or angle <= end
    else:
        # Arc goes clockwise from start to end
        if start >= end:
            return end <= angle <= start
        else:
            # Arc crosses 0
            return angle <= start or angle >= end


def determine_angle(center, point):
    angle = np.arctan2(point[1] - center[1], point[0] - center[0])
    return np.where(angle < 0, angle + 2 * np.pi, angle)


def bresenham_circle_4connected(xc, yc, r):
    """
    Generates points on a circle with 4-connectivity (face-to-face).
    Intermediate steps are added when both x and y change to avoid gaps.
    """
    x = 0
    y = r
    d = 2 - 2 * r
    points = []

    def add_octants(cx, cy, x, y):
        # Adds all 8 symmetrical points
        points.extend(
            [
                (cx + x, cy + y),
                (cx - x, cy + y),
                (cx + x, cy - y),
                (cx - x, cy - y),
                (cx + y, cy + x),
                (cx - y, cy + x),
                (cx + y, cy - x),
                (cx - y, cy - x),
            ]
        )

    add_octants(xc, yc, x, y)

    while y >= x:
        x += 1

        if d > 0:
            # A diagonal move is about to happen (x increases AND y decreases)
            # To maintain 4-connectivity, we add the point before y decreases.
            # We add points for the horizontal step before the vertical one.
            add_octants(xc, yc, x, y)  # Intermediate point
            y -= 1
            d = d + 4 * (x - y) + 10
        else:
            d = d + 4 * x + 6

        add_octants(xc, yc, x, y)

    return list(
        dict.fromkeys(points)
    )  # Use set to remove duplicates at octant boundaries


"""
1 (xc - x, yc + y)
2 (xc + y, yc - x)
3 (xc - y, yc - x)
4 (xc + x, yc + y)
5 (xc + x, yc - y)
6 (xc - y, yc + x)
7 (xc + y, yc + x)
8 (xc - x, yc - y)
"""


def bresenham_circle(xc, yc, r):
    """Generates points on a circle centered at (xc, yc) with radius r using Bresenham's circle algorithm"""

    x, y, err = -r, 0, 2 - 2 * r
    points = []

    while abs(x) >= y:

        points.append((xc - x, yc + y))
        points.append((xc + y, yc - x))
        points.append((xc - y, yc - x))
        points.append((xc + x, yc + y))
        points.append((xc + x, yc - y))
        points.append((xc - y, yc + x))
        points.append((xc + y, yc + x))
        points.append((xc - x, yc - y))

        r = err
        if r <= y:
            y += 1
            err += y * 2 + 1
        if r > x or err > y:
            x += 1
            err += x * 2 + 1

    return list(dict.fromkeys(points))


def bresenham_filled_circle(xc, yc, r, thick=False):
    x, y, err = -r, 0, 2 - 2 * r
    points = []

    while abs(x) >= y:

        points.extend(bresenham_line(xc - x, yc + y, xc - y, yc - x))  # 180 -> 225
        points.extend(bresenham_line(xc + x, yc - y, xc + y, yc + x))  # 0 -> 45

        if r == 1:
            break

        if thick and r == 2:
            points.extend(
                bresenham_line(xc - x, yc + y + 1, xc - x, yc + y - 1)
            )  # 180 -> 225
            points.extend(
                bresenham_line(xc - y + 1, yc - x, xc - y - 1, yc - x)
            )  # 90 -> 135
            points.extend(
                bresenham_line(xc + x, yc - y + 1, xc + x, yc - y - 1)
            )  # 0 -> 45
            points.extend(
                bresenham_line(xc + y + 1, yc + x, xc + y - 1, yc + x)
            )  # 270 -> 315

            break

        if y != 0:
            points.extend(bresenham_line(xc + x, yc + y, xc - x, yc - y))
            points.extend(bresenham_line(xc + y, yc - x, xc - y, yc + x))

        r = err
        if r <= y:
            y += 1
            err += y * 2 + 1
        if r > x or err > y:
            x += 1
            err += x * 2 + 1

    return points


def bresenham_filled_circle_stepped(xc, yc, r):
    """
    Generates a filled circle where every boundary pixel has all faces covered
    by an additional neighbor pixel, similar to double_step_line.

    Instead of post-hoc dilation, the circle algorithm itself emits extra
    adjacent pixels whenever a diagonal step occurs — one offset along each
    axis of change — so that no face of any boundary pixel is left exposed.
    """
    if r <= 0:
        return [(xc, yc)]

    x, y, err = -r, 0, 2 - 2 * r
    boundary = set()

    def add_octants(cx, cy):
        """Emit the 8 symmetric boundary points for the current (x, y)."""
        boundary.update([
            (xc - cx, yc + cy),
            (xc + cy, yc - cx),
            (xc - cy, yc - cx),
            (xc + cx, yc + cy),
            (xc + cx, yc - cy),
            (xc - cy, yc + cx),
            (xc + cy, yc + cx),
            (xc - cx, yc - cy),
        ])

    def add_double_step_neighbors(prev_x, prev_y, new_x, new_y):
        """
        When both x and y change (diagonal step), emit extra pixels to cover
        all exposed faces — analogous to double_step_line emitting
        (x0, y0+sy) on horizontal steps and (x2+sx, y0) on vertical steps.

        For each octant reflection we add the pixel that shares a face along
        the axis that *didn't* move yet (the intermediate positions).
        """
        # Intermediate with only-Y-changed (x stays at prev)
        boundary.update([
            (xc - prev_x, yc + new_y),
            (xc + new_y, yc - prev_x),
            (xc - new_y, yc - prev_x),
            (xc + prev_x, yc + new_y),
            (xc + prev_x, yc - new_y),
            (xc - new_y, yc + prev_x),
            (xc + new_y, yc + prev_x),
            (xc - prev_x, yc - new_y),
        ])
        # Intermediate with only-X-changed (y stays at prev)
        boundary.update([
            (xc - new_x, yc + prev_y),
            (xc + prev_y, yc - new_x),
            (xc - prev_y, yc - new_x),
            (xc + new_x, yc + prev_y),
            (xc + new_x, yc - prev_y),
            (xc - prev_y, yc + new_x),
            (xc + prev_y, yc + new_x),
            (xc - new_x, yc - prev_y),
        ])

    while abs(x) >= y:
        add_octants(x, y)

        prev_x, prev_y = x, y
        curr_err = err

        stepped_y = False
        stepped_x = False

        if curr_err <= y:
            y += 1
            err += y * 2 + 1
            stepped_y = True

        if curr_err > prev_x or err > y:
            x += 1
            err += x * 2 + 1
            stepped_x = True

        # Diagonal step detected — add double-step neighbor pixels
        if stepped_x and stepped_y:
            add_double_step_neighbors(prev_x, prev_y, x, y)

        add_octants(x, y)

    # Fill scanlines from boundary
    y_spans = {}
    for px, py in boundary:
        if py not in y_spans:
            y_spans[py] = [px, px]
        else:
            if px < y_spans[py][0]:
                y_spans[py][0] = px
            if px > y_spans[py][1]:
                y_spans[py][1] = px

    points = []
    for py, (min_x, max_x) in y_spans.items():
        for px in range(min_x, max_x + 1):
            points.append((px, py))
    return points


def square(xc, yc, r):
    points = []
    for x in range(xc - r, xc + r + 1):
        points.append((x, yc - r))  # top edge
        points.append((x, yc + r))  # bottom edge
    for y in range(yc - r + 1, yc + r):
        points.append((xc - r, y))  # left edge
        points.append((xc + r, y))  # right edge
    return points


def filled_square(xc, yc, r):
    points = []
    for x in range(xc - r, xc + r + 1):
        for y in range(yc - r, yc + r + 1):
            points.append((x, y))
    return points