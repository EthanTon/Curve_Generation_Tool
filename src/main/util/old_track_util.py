import numpy as np
from main.util.shapeUtil import (
    bresenham_line,
    bresenham_circle,
    bresenham_filled_circle,
    bresenham_arc,
    filled_square,
)

TRACK_ELEVATION_OFFSET = 1


def track(path, start, end, track_distance, edge_distance, slope=0):

    track_edge = generate_orthogonal_path(path, start, end, edge_distance)
    track_base = generate_track_base(path, track_edge, edge_distance)

    track_center = generate_track_center(path, start, end, track_distance, track_base)
    track_outline = generate_track_outline(track_center, track_base)

    print("done")

    return (track_outline, track_center, list(track_base))


# def generate_elevation_mask(start, end, base):


def generate_track_base(path, edge, base_distance):
    if base_distance <= 0:
        return None

    base_filter = set()
    for pt in path:
        base_filter.update(bresenham_filled_circle(pt[0], pt[1], base_distance, True))

    base_reference = set()

    for n in range(1, len(path)-1):
        base_reference.update(generate_orthogonal_line(path[n], edge, base_distance))

    base = set()

    for p in base_reference:
        temp = filled_square(int(p[0]), int(p[1]), 1)

        for a in temp:
            if a in base_filter:
                base.add(a)

    return base


def generate_orthogonal_path(path, start, end, track_distance):
    if track_distance < 0:
        raise ValueError

    track_points = []

    for n in range(0, track_distance):
        track_points.append(set())

    for pt in path:
        for n in range(0, track_distance):
            track_points[n].update(
                bresenham_circle(pt[0], pt[1], n + 1, track_distance == 2)
            )

        for n in range(0, track_distance - 1):
            for p in track_points[n]:
                if p in track_points[-1]:
                    track_points[-1].remove(p)

    if track_distance <= 2:
        for p in path:
            if tuple(p.tolist()) in track_points[-1]:
                track_points[-1].remove(tuple(p.tolist()))

    return sort_by_adjacency(
        list(clean_ends(track_points[-1], start, end, track_distance))
    )


def clean_ends(path, start, end, track_distance):
    ortho_start_1 = (
        np.rint(-np.sin(start[2]) * track_distance + start[0]),
        np.rint(np.cos(start[2]) * track_distance + start[1]),
    )
    ortho_start_2 = (
        np.rint(np.sin(start[2]) * track_distance + start[0]),
        np.rint(-np.cos(start[2]) * track_distance + start[1]),
    )

    ortho_end_1 = (
        np.rint(np.sin(end[2]) * track_distance + end[0]),
        np.rint(-np.cos(end[2]) * track_distance + end[1]),
    )
    ortho_end_2 = (
        np.rint(-np.sin(end[2]) * track_distance + end[0]),
        np.rint(np.cos(end[2]) * track_distance + end[1]),
    )

    filter = set()
    filter.update(
        bresenham_arc(
            start[0], start[1], track_distance, ortho_start_1, ortho_start_2, True
        )
    )

    filter.update(
        bresenham_arc(end[0], end[1], track_distance, ortho_end_1, ortho_end_2, True)
    )

    for pt in filter:
        if (
            pt in path
            and not pt
            in [
                ortho_start_1,
                ortho_start_2,
                ortho_end_1,
                ortho_end_2,
            ]
            and ((dist(pt, start) < track_distance))
            or (dist(pt, end) < track_distance)
        ):
            path.remove(pt)

    return path


def generate_orthogonal_line(p_r, path_r, distance):
    ortho_point_0, ortho_point_1 = determine_orthogonal_intersection_points(
        p_r, path_r, distance
    )

    return bresenham_line(
        ortho_point_0[0], ortho_point_0[1], ortho_point_1[0], ortho_point_1[1]
    )


def determine_orthogonal_intersection_points(p_r, path_r, distance):
    starting_point = (p_r[0], p_r[1] + distance)
    ending_point = (p_r[0], p_r[1] - distance)

    right_reference_circle = bresenham_arc(
        p_r[0], p_r[1], distance, starting_point, ending_point, True
    )
    left_reference_circle = bresenham_arc(
        p_r[0], p_r[1], distance, starting_point, ending_point, False
    )

    right_intersection = determine_intersection_points(right_reference_circle, path_r)
    left_intersection = determine_intersection_points(left_reference_circle, path_r)

    starting_point = (p_r[0] + distance, p_r[1])
    ending_point = (p_r[0] - distance, p_r[1])

    top_reference_circle = bresenham_arc(
        p_r[0], p_r[1], distance, starting_point, ending_point, True
    )
    bottom_reference_circle = bresenham_arc(
        p_r[0], p_r[1], distance, starting_point, ending_point, False
    )

    top_intersection = determine_intersection_points(top_reference_circle, path_r)
    bottom_intersection = determine_intersection_points(bottom_reference_circle, path_r)

    avg_right = average_point(right_intersection)
    avg_left = average_point(left_intersection)

    avg_top = average_point(top_intersection)
    avg_bottom = average_point(bottom_intersection)

    if dist(avg_right, avg_left) > distance * np.sqrt(2):
        return avg_right, avg_left

    return avg_top, avg_bottom


def determine_intersection_points(reference_shape, reference_path):
    intersection = set()

    for p in reference_shape:
        if p in reference_path:
            intersection.add(p)

    return intersection


def average_point(points):
    x_avg = 0
    y_avg = 0

    num_of_points = len(points)

    for pts in points:
        x_avg += pts[0]
        y_avg += pts[1]

    if num_of_points == 0:
        return x_avg, y_avg

    return (np.rint(x_avg / num_of_points), np.rint(y_avg / num_of_points))


def validate_point(points, track_base):
    filter = set()
    for pt in points:
        if pt not in track_base:
            filter.add(pt)

    for pt in filter:
        points.remove(pt)

    return points


def generate_track_center(path, start, end, track_distance, track_base):
    track_center = generate_orthogonal_path(path, start, end, track_distance)

    return validate_point(track_center, track_base)


def generate_track_outline(track_center, track_base, outline_distance=1):

    track_outline = set()

    for pt in track_center:
        track_outline.update(bresenham_circle(pt[0], pt[1], outline_distance))

    for p in track_center:
        if p in track_outline:
            track_outline.remove(p)

    return validate_point(sort_by_adjacency(list(track_outline)), track_base)


def dist(pt_a, pt_b):
    """Euclidian distance between two (x, y) points"""
    return ((pt_a[0] - pt_b[0]) ** 2 + (pt_a[1] - pt_b[1]) ** 2) ** 0.5


def sort_by_adjacency(positions, prepend=True):
    """
    Sort a list of 2D positions based on adjacency.

    Starting from the first position, each subsequent position is the closest
    unvisited neighbor to the current position (greedy nearest-neighbor approach).

    Args:
        positions: List of (x, y) tuples or lists representing 2D positions
        prepend: If True, also consider adding points to the front of the chain
                 when they're closer to the start than to the end

    Returns:
        List of positions sorted by adjacency
    """
    if not positions:
        return []

    if len(positions) == 1:
        return list(positions)

    positions = [tuple(p) for p in positions]
    remaining = set(positions)

    current = positions[0]
    remaining.remove(current)
    sorted_positions = [current]

    if prepend:
        while remaining:
            start = sorted_positions[0]
            end = sorted_positions[-1]

            nearest_to_end = min(
                remaining, key=lambda p: (p[0] - end[0]) ** 2 + (p[1] - end[1]) ** 2
            )
            dist_to_end = (nearest_to_end[0] - end[0]) ** 2 + (
                nearest_to_end[1] - end[1]
            ) ** 2

            nearest_to_start = min(
                remaining, key=lambda p: (p[0] - start[0]) ** 2 + (p[1] - start[1]) ** 2
            )
            dist_to_start = (nearest_to_start[0] - start[0]) ** 2 + (
                nearest_to_start[1] - start[1]
            ) ** 2

            if dist_to_start < dist_to_end:
                remaining.remove(nearest_to_start)
                sorted_positions.insert(0, nearest_to_start)
            else:
                remaining.remove(nearest_to_end)
                sorted_positions.append(nearest_to_end)
    else:
        while remaining:
            nearest = min(
                remaining,
                key=lambda p: (p[0] - sorted_positions[-1][0]) ** 2
                + (p[1] - sorted_positions[-1][1]) ** 2,
            )
            remaining.remove(nearest)
            sorted_positions.append(nearest)

    return sorted_positions
