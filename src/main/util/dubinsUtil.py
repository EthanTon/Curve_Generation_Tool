# Adapted from https://github.com/FelicienC/RRT-Dubins/blob/master/code/dubins.py
# Create pixelated Dubins path implementation in Python

import numpy as np
from util.shapeUtil import bresenham_arc, bresenham_line
from util.pathUtil import dist, ortho


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


def int_generate_points_straight(start, end, path, radius):
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

    # Generate all the points using Bresenham algorithms
    points = bresenham_arc(
        np.rint(center_0[0]),
        np.rint(center_0[1]),
        radius,
        np.rint(start[:2]),
        np.rint(ini),
        (path[0] > 0),
    )
    points.extend(
        bresenham_line(
            np.rint(ini[0]), np.rint(ini[1]), np.rint(fin[0]), np.rint(fin[1])
        )
    )
    points.extend(
        bresenham_arc(
            np.rint(center_2[0]),
            np.rint(center_2[1]),
            radius,
            np.rint(fin),
            np.rint(end[:2]),
            (path[1] > 0),
        )
    )
    return np.array(list(dict.fromkeys(points)))


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
    center_0 = find_center(start, "L" if path[0] > 0 else "R", radius)
    center_2 = find_center(end, "L" if path[1] > 0 else "R", radius)
    intercenter = dist(center_0, center_2)
    center_1 = (center_0 + center_2) / 2 + np.sign(path[0]) * ortho(
        (center_2 - center_0) / intercenter
    ) * (4 * radius**2 - (intercenter / 2) ** 2) ** 0.5

    ini = center_1 + radius * (center_0 - center_1) / dist(center_0, center_1)

    fin = center_1 + radius * (center_2 - center_1) / dist(center_2, center_1)

    psi_0 = np.arctan2((center_1 - center_0)[1], (center_1 - center_0)[0]) - np.pi

    points = bresenham_arc(
        np.rint(center_0[0]),
        np.rint(center_0[1]),
        radius,
        np.rint(start[:2]),
        np.rint(ini),
        path[0] > 0,
    )

    points.extend(
        bresenham_arc(
            np.rint(center_1[0]),
            np.rint(center_1[1]),
            radius,
            np.rint(ini[:2]),
            np.rint(fin[:2]),
            path[0] < 0,
        )
    )

    points.extend(
        bresenham_arc(
            np.rint(center_2[0]),
            np.rint(center_2[1]),
            radius,
            np.rint(fin[:2]),
            np.rint(end[:2]),
            path[1] > 0,
        )
    )
    return np.array(list(dict.fromkeys(points)))


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