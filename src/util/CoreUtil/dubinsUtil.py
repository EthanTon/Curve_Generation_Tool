# Adapted from https://github.com/FelicienC/RRT-Dubins/blob/master/code/dubins.py
# Create pixelated Dubins path implementation in Python

import numpy as np
from .shapeUtil import bresenham_arc, bresenham_line
from .pathUtil import dist, ortho


def all_options(start, end, radius, sort=False):
    center_0_left = find_center(start, "L", radius)
    center_0_right = find_center(start, "R", radius)
    center_2_left = find_center(end, "L", radius)
    center_2_right = find_center(end, "R", radius)
    options = [
        lsl(start, end, center_0_left, center_2_left, radius),
        rsr(start, end, center_0_right, center_2_right, radius),
        rsl(start, end, center_0_right, center_2_left, radius),
        lsr(start, end, center_0_left, center_2_right, radius),
        rlr(start, end, center_0_right, center_2_right, radius),
        lrl(start, end, center_0_left, center_2_left, radius),
    ]
    if sort:
        options.sort(key=lambda x: x[0])
    return options


def int_dubins_path(start, end, radius):
    options = all_options(start, end, radius)
    dubins_path, straight = min(options, key=lambda x: x[0])[1:]
    return int_generate_points(start, end, dubins_path, straight, radius)


def dubins_path(start, end, radius, point_separation=0.1):
    options = all_options(start, end, radius)
    dubins_path, straight = min(options, key=lambda x: x[0])[1:]
    return generate_points(start, end, dubins_path, straight, radius, point_separation)


def int_generate_points(start, end, dubins_path, straight, radius):

    if straight:
        return int_generate_points_straight(start, end, dubins_path, radius)
    return int_generate_points_curve(start, end, dubins_path, radius)


def generate_points(start, end, dubins_path, straight, radius, point_separation):
    if straight:
        return generate_points_straight(
            start, end, dubins_path, radius, point_separation
        )
    return generate_points_curve(start, end, dubins_path, radius, point_separation)


def lsl(start, end, center_0, center_2, radius):
    straight_dist = dist(center_0, center_2)
    alpha = np.arctan2((center_2 - center_0)[1], (center_2 - center_0)[0])
    beta_2 = (end[2] - alpha) % (2 * np.pi)
    beta_0 = (alpha - start[2]) % (2 * np.pi)
    total_len = radius * (beta_2 + beta_0) + straight_dist
    return (total_len, (beta_0, beta_2, straight_dist), True)


def rsr(start, end, center_0, center_2, radius):
    straight_dist = dist(center_0, center_2)
    alpha = np.arctan2((center_2 - center_0)[1], (center_2 - center_0)[0])
    beta_2 = (-end[2] + alpha) % (2 * np.pi)
    beta_0 = (-alpha + start[2]) % (2 * np.pi)
    total_len = radius * (beta_2 + beta_0) + straight_dist
    return (total_len, (-beta_0, -beta_2, straight_dist), True)


def rsl(start, end, center_0, center_2, radius):
    median_point = (center_2 - center_0) / 2
    psia = np.arctan2(median_point[1], median_point[0])
    half_intercenter = np.linalg.norm(median_point)
    if half_intercenter < radius:
        return (float("inf"), (0, 0, 0), True)
    alpha = np.arccos(radius / half_intercenter)
    beta_0 = -(psia + alpha - start[2] - np.pi / 2) % (2 * np.pi)
    beta_2 = (np.pi + end[2] - np.pi / 2 - alpha - psia) % (2 * np.pi)
    straight_dist = 2 * (half_intercenter**2 - radius**2) ** 0.5
    total_len = radius * (beta_2 + beta_0) + straight_dist
    return (total_len, (-beta_0, beta_2, straight_dist), True)


def lsr(start, end, center_0, center_2, radius):
    median_point = (center_2 - center_0) / 2
    psia = np.arctan2(median_point[1], median_point[0])
    half_intercenter = np.linalg.norm(median_point)
    if half_intercenter < radius:
        return (float("inf"), (0, 0, 0), True)
    alpha = np.arccos(radius / half_intercenter)
    beta_0 = (psia - alpha - start[2] + np.pi / 2) % (2 * np.pi)
    beta_2 = (0.5 * np.pi - end[2] - alpha + psia) % (2 * np.pi)
    straight_dist = 2 * (half_intercenter**2 - radius**2) ** 0.5
    total_len = radius * (beta_2 + beta_0) + straight_dist
    return (total_len, (beta_0, -beta_2, straight_dist), True)


def lrl(start, end, center_0, center_2, radius):
    dist_intercenter = dist(center_0, center_2)
    intercenter = (center_2 - center_0) / 2
    psia = np.arctan2(intercenter[1], intercenter[0])
    if not (2 * radius <= dist_intercenter <= 4 * radius):
        return (float("inf"), (0, 0, 0), False)
    gamma = 2 * np.arcsin(dist_intercenter / (4 * radius))
    beta_0 = (psia - start[2] + np.pi / 2 + (np.pi - gamma) / 2) % (2 * np.pi)
    beta_1 = (-psia + np.pi / 2 + end[2] + (np.pi - gamma) / 2) % (2 * np.pi)
    total_len = (2 * np.pi - gamma + abs(beta_0) + abs(beta_1)) * radius
    return (total_len, (beta_0, beta_1, 2 * np.pi - gamma), False)


def rlr(start, end, center_0, center_2, radius):
    dist_intercenter = dist(center_0, center_2)
    intercenter = (center_2 - center_0) / 2
    psia = np.arctan2(intercenter[1], intercenter[0])
    if not (2 * radius <= dist_intercenter <= 4 * radius):
        return (float("inf"), (0, 0, 0), False)
    gamma = 2 * np.arcsin(dist_intercenter / (4 * radius))
    beta_0 = -((-psia + (start[2] + np.pi / 2) + (np.pi - gamma) / 2) % (2 * np.pi))
    beta_1 = -((psia + np.pi / 2 - end[2] + (np.pi - gamma) / 2) % (2 * np.pi))
    total_len = (2 * np.pi - gamma + abs(beta_0) + abs(beta_1)) * radius
    return (total_len, (beta_0, beta_1, 2 * np.pi - gamma), False)


def find_center(point, side, radius):
    assert side in "LR"
    angle = point[2] + (np.pi / 2 if side == "L" else -np.pi / 2)
    return np.array(
        (point[0] + np.cos(angle) * radius, point[1] + np.sin(angle) * radius)
    )


def _to_pt(x, y=None):
    """Convert to plain (int, int) tuple consistently."""
    if y is None:
        return (int(round(x[0])), int(round(x[1])))
    return (int(round(x)), int(round(y)))


def _trim_to_junction(points, junction, at_end=True):
    """Trim a segment's points so it stops at the junction point.
    If at_end=True, finds the LAST occurrence of junction and drops everything after.
    If at_end=False, finds the FIRST occurrence and drops everything before."""
    if not points:
        return points
    if at_end:
        # Find last occurrence of junction
        for i in range(len(points) - 1, -1, -1):
            if points[i] == junction:
                return points[: i + 1]
        # Junction not found - append it
        points.append(junction)
        return points
    else:
        # Find first occurrence of junction
        for i in range(len(points)):
            if points[i] == junction:
                return points[i:]
        # Junction not found - prepend it
        return [junction] + points


def int_generate_points_straight(start, end, path, radius):
    EPS = 1e-6
    center_0 = find_center(start, "L" if path[0] > 0 else "R", radius)
    center_2 = find_center(end, "L" if path[1] > 0 else "R", radius)

    start_pt = _to_pt(start[:2])
    end_pt = _to_pt(end[:2])

    # Find where the straight segment starts
    if abs(path[0]) > EPS:
        angle = start[2] + (abs(path[0]) - np.pi / 2) * np.sign(path[0])
        ini = center_0 + radius * np.array([np.cos(angle), np.sin(angle)])
    else:
        ini = np.array(start[:2], dtype=float)

    # Find where the straight segment ends
    if abs(path[1]) > EPS:
        angle = end[2] + (-abs(path[1]) - np.pi / 2) * np.sign(path[1])
        fin = center_2 + radius * np.array([np.cos(angle), np.sin(angle)])
    else:
        fin = np.array(end[:2], dtype=float)

    ini_pt = _to_pt(ini)
    fin_pt = _to_pt(fin)

    # Build segments, trimming each to its boundary
    points = []

    # First arc
    if abs(path[0]) > EPS:
        arc0 = bresenham_arc(
            round(center_0[0]),
            round(center_0[1]),
            radius,
            start_pt,
            ini_pt,
            (path[0] > 0),
        )
        arc0 = _trim_to_junction(arc0, ini_pt, at_end=True)
        points.extend(arc0)

    # Straight segment - skip first point if it duplicates arc0's last
    line = bresenham_line(ini_pt[0], ini_pt[1], fin_pt[0], fin_pt[1])
    if points and line and line[0] == points[-1]:
        line = line[1:]
    points.extend(line)

    # Last arc - skip first point if it duplicates line's last
    if abs(path[1]) > EPS:
        arc2 = bresenham_arc(
            round(center_2[0]),
            round(center_2[1]),
            radius,
            fin_pt,
            end_pt,
            (path[1] > 0),
        )
        arc2 = _trim_to_junction(arc2, fin_pt, at_end=False)
        if points and arc2 and arc2[0] == points[-1]:
            arc2 = arc2[1:]
        points.extend(arc2)

    # Ensure start is first and end is last
    if not points or points[0] != start_pt:
        points.insert(0, start_pt)
    if points[-1] != end_pt:
        points.append(end_pt)

    return np.array(points)


def generate_points_straight(start, end, path, radius, point_separation=0.1):
    total = radius * (abs(path[1]) + abs(path[0])) + path[2]  # Path length
    center_0 = find_center(start, "L" if path[0] > 0 else "R", radius)
    center_2 = find_center(end, "L" if path[1] > 0 else "R", radius)

    # We first need to find the points where the straight segment starts
    if abs(path[0]) > 0:
        angle = start[2] + (abs(path[0]) - np.pi / 2) * np.sign(path[0])
        ini = center_0 + radius * np.array([np.cos(angle), np.sin(angle)])
    else:
        ini = np.array(start[:2])
    # We then identify its end
    if abs(path[1]) > 0:
        angle = end[2] + (-abs(path[1]) - np.pi / 2) * np.sign(path[1])
        fin = center_2 + radius * np.array([np.cos(angle), np.sin(angle)])
    else:
        fin = np.array(end[:2])
    dist_straight = dist(ini, fin)

    # We can now generate all the points with the desired precision
    points = []
    for x in np.arange(0, total, point_separation):
        if x < abs(path[0]) * radius:  # First turn
            points.append(circle_arc(start, path[0], center_0, radius, x))
        elif x > total - abs(path[1]) * radius:  # Last turn
            points.append(circle_arc(end, path[1], center_2, radius, x - total))
        else:  # Straight segment
            coeff = (x - abs(path[0]) * radius) / dist_straight
            points.append(coeff * fin + (1 - coeff) * ini)
    points.append(end[:2])
    return points


def int_generate_points_curve(start, end, path, radius):
    EPS = 1e-6
    center_0 = find_center(start, "L" if path[0] > 0 else "R", radius)
    center_2 = find_center(end, "L" if path[1] > 0 else "R", radius)
    intercenter = dist(center_0, center_2)
    center_1 = (center_0 + center_2) / 2 + np.sign(path[0]) * ortho(
        (center_2 - center_0) / intercenter
    ) * (4 * radius**2 - (intercenter / 2) ** 2) ** 0.5

    ini = center_1 + radius * (center_0 - center_1) / dist(center_0, center_1)
    fin = center_1 + radius * (center_2 - center_1) / dist(center_2, center_1)

    start_pt = _to_pt(start[:2])
    end_pt = _to_pt(end[:2])
    ini_pt = _to_pt(ini)
    fin_pt = _to_pt(fin)

    points = []

    # First arc
    if abs(path[0]) > EPS:
        arc0 = bresenham_arc(
            round(center_0[0]),
            round(center_0[1]),
            radius,
            start_pt,
            ini_pt,
            path[0] > 0,
        )
        arc0 = _trim_to_junction(arc0, ini_pt, at_end=True)
        points.extend(arc0)

    # Middle arc - skip first point if it duplicates arc0's last
    if abs(path[2]) > EPS:
        arc1 = bresenham_arc(
            round(center_1[0]),
            round(center_1[1]),
            radius,
            ini_pt,
            fin_pt,
            path[0] < 0,
        )
        arc1 = _trim_to_junction(arc1, ini_pt, at_end=False)
        arc1 = _trim_to_junction(arc1, fin_pt, at_end=True)
        if points and arc1 and arc1[0] == points[-1]:
            arc1 = arc1[1:]
        points.extend(arc1)

    # Last arc - skip first point if it duplicates arc1's last
    if abs(path[1]) > EPS:
        arc2 = bresenham_arc(
            round(center_2[0]),
            round(center_2[1]),
            radius,
            fin_pt,
            end_pt,
            path[1] > 0,
        )
        arc2 = _trim_to_junction(arc2, fin_pt, at_end=False)
        if points and arc2 and arc2[0] == points[-1]:
            arc2 = arc2[1:]
        points.extend(arc2)

    # Ensure start is first and end is last
    if not points or points[0] != start_pt:
        points.insert(0, start_pt)
    if points[-1] != end_pt:
        points.append(end_pt)

    return np.array(points)


def generate_points_curve(start, end, path, radius, point_separation):
    total = radius * (abs(path[1]) + abs(path[0]) + abs(path[2]))
    center_0 = find_center(start, "L" if path[0] > 0 else "R", radius)
    center_2 = find_center(end, "L" if path[1] > 0 else "R", radius)
    intercenter = dist(center_0, center_2)
    center_1 = (center_0 + center_2) / 2 + np.sign(path[0]) * ortho(
        (center_2 - center_0) / intercenter
    ) * (4 * radius**2 - (intercenter / 2) ** 2) ** 0.5
    psi_0 = np.arctan2((center_1 - center_0)[1], (center_1 - center_0)[0]) - np.pi

    points = []
    for x in np.arange(0, total, point_separation):
        if x < abs(path[0]) * radius:  # First turn
            points.append(circle_arc(start, path[0], center_0, radius, x))
        elif x > total - abs(path[1]) * radius:  # Last turn
            points.append(circle_arc(end, path[1], center_2, radius, x - total))
        else:  # Middle Turn
            angle = psi_0 - np.sign(path[0]) * (x / radius - abs(path[0]))
            vect = np.array([np.cos(angle), np.sin(angle)])
            points.append(center_1 + radius * vect)
    points.append(end[:2])
    return points


def circle_arc(reference, beta, center, radius, x):

    angle = reference[2] + ((x / radius) - np.pi / 2) * np.sign(beta)
    vect = np.array([np.cos(angle), np.sin(angle)])
    return center + radius * vect
