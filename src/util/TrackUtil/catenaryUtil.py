from ..CoreUtil.shapeUtil import (
    bresenham_filled_circle,
    bresenham_line,
    bresenham_filled_circle_stepped,
    step_line,
    double_step_line
)

from ..CoreUtil.pathUtil import weighted_pca_orthogonal
import numpy as np


def generate_catenary(
    path, track1, track2, base, base_width, track_width, pantograph_interval, offset=0
):
    # --- Derived geometry ---
    base_edge = determine_base_edge(path, base_width // 2 - 1, base)
    cantilever_length = base_width // 2 - track_width // 2 + 1

    # Build fast-lookup sets for each track
    tc1 = set()
    for part in track1:
        tc1.update(part)

    tc2 = set()
    for part in track2:
        tc2.update(part)

    # --- Determine pole positions along the path ---
    pole_indices = determine_catenary_indices(path, pantograph_interval, offset)

    # --- For each pole index, compute poles, intersections, and cantilevers ---
    t1_catenary_poles = []
    t1_cantilevers = []
    t1_intersections = []

    t2_catenary_poles = []
    t2_cantilevers = []
    t2_intersections = []

    for idx in pole_indices:
        pole_a, pole_b = determine_catenary_pole_position(
            path, idx, base_width, base_edge
        )
        if pole_a is None or pole_b is None:
            continue

        # Walk a line between the two poles and find where it crosses each track
        cross_line = step_line(pole_a[0], pole_a[1], pole_b[0], pole_b[1])
        t1_int, t2_int = determine_track_intersection(cross_line, tc1, tc2)

        # Skip this pole if either intersection missed the track entirely
        if t1_int is None or t2_int is None:
            continue

        # Assign each pole to the nearer track
        dist_a_t1 = (pole_a[0] - t1_int[0]) ** 2 + (pole_a[1] - t1_int[1]) ** 2
        dist_b_t1 = (pole_b[0] - t1_int[0]) ** 2 + (pole_b[1] - t1_int[1]) ** 2

        if dist_a_t1 <= dist_b_t1:
            t1_pole, t2_pole = pole_a, pole_b
        else:
            t1_pole, t2_pole = pole_b, pole_a

        t1_catenary_poles.append(t1_pole)
        t2_catenary_poles.append(t2_pole)

        t1_intersections.append(t1_int)
        t2_intersections.append(t2_int)

        t1_cantilevers.append(determine_cantilever(t1_pole, t1_int, cantilever_length))
        t2_cantilevers.append(determine_cantilever(t2_pole, t2_int, cantilever_length))

    return [
        [t1_catenary_poles, t1_cantilevers, t1_intersections],
        [t2_catenary_poles, t2_cantilevers, t2_intersections],
    ]


def determine_cantilever(pole, intersection, cantilever_length):
    dx = intersection[0] - pole[0]
    dy = intersection[1] - pole[1]
    dist = (dx**2 + dy**2) ** 0.5

    # Try bresenham first; if shorter than cantilever_length, use step_line
    line = bresenham_line(pole[0], pole[1], intersection[0], intersection[1])
    if len(line) < cantilever_length:
        line = step_line(pole[0], pole[1], intersection[0], intersection[1])

    # Build cantilever: (cantilever_length - 1) points from pole + intersection at the end
    if len(line) >= cantilever_length:
        cantilever = list(line[: cantilever_length - 1]) + [intersection]
    elif len(line) >= 2:
        cantilever = list(line)
        # Pad by interpolating between last point and intersection
        while len(cantilever) < cantilever_length - 1:
            last = cantilever[-1]
            mx = (last[0] + intersection[0]) // 2
            my = (last[1] + intersection[1]) // 2
            cantilever.append((mx, my))
        cantilever = cantilever[: cantilever_length - 1] + [intersection]
    else:
        nx = round(dx / max(dist, 1e-9))
        ny = round(dy / max(dist, 1e-9))
        cantilever = [
            (pole[0] + i * nx, pole[1] + i * ny) for i in range(cantilever_length - 1)
        ] + [intersection]

    return cantilever[:cantilever_length]


def determine_catenary_indices(path, pantograph_interval, offset):
    catenary_indices = [offset]
    bound = set([tuple(pt) for pt in path])

    reference_x, reference_y = path[offset]

    for i in range(offset, len(path)):
        x, y = path[i]

        dist = ((x - reference_x) ** 2 + (y - reference_y) ** 2) ** 0.5

        if dist < pantograph_interval:
            continue
        else:
            reference_x, reference_y = x, y

        if line_leaves_bound(step_line(x, y, reference_x, reference_y), bound):
            determine_valid_midpoint(path, catenary_indices, bound, i)
        else:
            catenary_indices.append(i)

    return catenary_indices


def determine_valid_midpoint(
    path, catenary_indices, bound, current_idx, max_iterations=4
):
    last_idx = catenary_indices[-1]
    prev_x, prev_y = path[last_idx]
    low, high = last_idx, current_idx

    for _ in range(max_iterations):
        mid = (low + high) // 2
        mid_x, mid_y = path[mid]

        if line_leaves_bound(step_line(mid_x, mid_y, prev_x, prev_y), bound):
            high = mid
        else:
            catenary_indices.append(mid)
            return

    catenary_indices.append(current_idx)


def determine_catenary_pole_position(path, index, base_width, base_edge):
    sigma = 1.0
    xp, yp = path[index]
    scalar = base_width // 2

    normal = weighted_pca_orthogonal(path, index, sigma)
    vector1 = (round(normal[0] * scalar + xp), round(normal[1] * scalar + yp))
    vector2 = (round(-normal[0] * scalar + xp), round(-normal[1] * scalar + yp))

    # Snap vector1 to base_edge
    if vector1 not in base_edge:
        best = None
        best_dist = float("inf")
        for pt in bresenham_filled_circle(vector1[0], vector1[1], 3):
            if pt in base_edge:
                d = (pt[0] - vector1[0]) ** 2 + (pt[1] - vector1[1]) ** 2
                if d < best_dist:
                    best_dist = d
                    best = pt
        if best is not None:
            vector1 = best
        else:
            return (None, None)

    # Snap vector2 to base_edge
    if vector2 not in base_edge:
        best = None
        best_dist = float("inf")
        for pt in bresenham_filled_circle(vector2[0], vector2[1], 3):
            if pt in base_edge:
                d = (pt[0] - vector2[0]) ** 2 + (pt[1] - vector2[1]) ** 2
                if d < best_dist:
                    best_dist = d
                    best = pt
        if best is not None:
            vector2 = best
        else:
            return (None, None)

    return (vector1, vector2)


def determine_track_intersection(cross_line, tc1, tc2):
    tc1_int_points = [pt for pt in cross_line if pt in tc1]
    tc2_int_points = [pt for pt in cross_line if pt in tc2]

    return (
        determine_average_point(tc1_int_points),
        determine_average_point(tc2_int_points),
    )


def determine_average_point(points):
    if not points:
        return None
    if len(points) == 1:
        return points[0]

    x_sum = sum(pt[0] for pt in points)
    y_sum = sum(pt[1] for pt in points)

    return (round(x_sum / len(points)), round(y_sum / len(points)))


def determine_bound(path, radius):
    bound = set()
    for pt in path:
        bound.update(bresenham_filled_circle(int(pt[0]), int(pt[1]), radius))
    return bound


def line_leaves_bound(line, bound):
    for pt in line:
        if pt not in bound:
            return True
    return False


def determine_base_edge(path, width, base):
    edge = set()
    for x, y in base:
        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            if (x + dx, y + dy) not in base:
                edge.add((x, y))
                break

    filter_set = set()
    for x, y in path:
        filter_set.update(
            bresenham_filled_circle_stepped(int(x), int(y), width // 2 - 1)
        )
    return edge - filter_set
