import math

from util.CoreUtil.shapeUtil import (
    bresenham_filled_circle,
    bresenham_line,
    step_line,
)

from util.CoreUtil.pathUtil import weighted_pca_orthogonal
from util.CoreUtil.crossSectionUtil import flip_cross_section


def _walk_toward(x0, z0, x1, z1, n):
    bres = bresenham_line(x0, z0, x1, z1)
    stepped = step_line(x0, z0, x1, z1)

    if not bres and not stepped:
        return [(x0, z0)] * n

    diff_bres = abs(len(bres) - n) if bres else float("inf")
    diff_step = abs(len(stepped) - n) if stepped else float("inf")
    line = stepped if diff_step <= diff_bres else bres

    if len(line) >= n:
        return line[len(line) - n :]

    if len(line) >= 2:
        dx = line[-1][0] - line[-2][0]
        dz = line[-1][1] - line[-2][1]
    else:
        dx = x1 - x0
        dz = z1 - z0
        dist = max((dx**2 + dz**2) ** 0.5, 1e-9)
        dx = round(dx / dist)
        dz = round(dz / dist)

    result = list(line)
    while len(result) < n:
        last = result[-1]
        result.append((last[0] + dx, last[1] + dz))
    return result


def assemble(
    path: list,
    path_silhouette: set,
    elevation_lut: dict,
    base_width: int,
    track1: set,
    track2: set,
    track_width: int,
    catenary_cross_section: list,
    catenary_interval: int,
    offset=0,
):
    base = determine_base(path, base_width, path_silhouette)

    tracks_data, center_points, tangent_angles, valid_pole_indices = generate_catenary(
        path=path,
        track1=track1,
        track2=track2,
        base=base,
        base_width=base_width,
        track_width=track_width,
        catenary_interval=catenary_interval,
        offset=offset,
    )

    curve = {}
    n_slices = len(catenary_cross_section)
    flipped_cross_section = [flip_cross_section(s) for s in catenary_cross_section]

    for track_idx, track_data in enumerate(tracks_data):
        poles, cantilevers, intersections = track_data
        slices = flipped_cross_section if track_idx == 1 else catenary_cross_section

        for cantilever, center in zip(cantilevers, center_points):
            center_y = elevation_lut.get(center, 0)
            for i in range(min(len(cantilever), n_slices)):
                section = slices[i]
                cx, cz = cantilever[i]
                for block, offsets in section.items():
                    for rx, ry, rz in offsets:
                        wx, wz = round(cx), round(cz)
                        wy = elevation_lut.get(center, center_y) + ry
                        curve.setdefault(block, set()).add((wx, wy, wz))

    t1_intersections = list(tracks_data[0][2])
    t2_intersections = list(tracks_data[1][2])
    p_indices = list(valid_pole_indices)

    last_path_idx = len(path) - 1

    if not p_indices or p_indices[-1] < last_path_idx:
        base_edge = determine_base_edge(path, base_width // 2 - 1, base)
        pole_a, pole_b = determine_catenary_pole_position(
            path, last_path_idx, base_width, base_edge
        )

        if pole_a and pole_b:
            cross_line = step_line(pole_a[0], pole_a[1], pole_b[0], pole_b[1])
            t1_int, t2_int = determine_track_intersection(
                cross_line, set(track1), set(track2)
            )

            if t1_int and t2_int:
                t1_intersections.append(t1_int)
                t2_intersections.append(t2_int)
                p_indices.append(last_path_idx)

    intersection_info = {
        "t1_intersections": t1_intersections,
        "t2_intersections": t2_intersections,
        "pole_indices": p_indices,
    }

    return curve, intersection_info


def generate_catenary(
    path, track1, track2, base, base_width, track_width, catenary_interval, offset=0
):
    base_edge = determine_base_edge(path, base_width // 2 - 1, base)
    cantilever_length = base_width // 2 - track_width // 2 + 1

    tc1 = set(track1)
    tc2 = set(track2)

    pole_indices = determine_catenary_indices(path, catenary_interval, offset)

    t1_catenary_poles = []
    t1_cantilevers = []
    t1_intersections = []

    t2_catenary_poles = []
    t2_cantilevers = []
    t2_intersections = []

    center_points = []
    tangent_angles = []
    valid_pole_indices = []

    for idx in pole_indices:
        pole_a, pole_b = determine_catenary_pole_position(
            path, idx, base_width, base_edge
        )
        if pole_a is None or pole_b is None:
            continue

        cross_line = step_line(pole_a[0], pole_a[1], pole_b[0], pole_b[1])
        t1_int, t2_int = determine_track_intersection(cross_line, tc1, tc2)

        if t1_int is None or t2_int is None:
            continue

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

        center_points.append(tuple(path[idx]))
        valid_pole_indices.append(idx)

        if idx > 0 and idx < len(path) - 1:
            tx = path[idx + 1][0] - path[idx - 1][0]
            tz = path[idx + 1][1] - path[idx - 1][1]
        elif idx < len(path) - 1:
            tx = path[idx + 1][0] - path[idx][0]
            tz = path[idx + 1][1] - path[idx][1]
        else:
            tx = path[idx][0] - path[idx - 1][0]
            tz = path[idx][1] - path[idx - 1][1]
        tangent_angles.append(math.atan2(tz, tx))

    return (
        [
            [t1_catenary_poles, t1_cantilevers, t1_intersections],
            [t2_catenary_poles, t2_cantilevers, t2_intersections],
        ],
        center_points,
        tangent_angles,
        valid_pole_indices,
    )


def determine_cantilever(pole, intersection, cantilever_length):
    return _walk_toward(
        pole[0], pole[1], intersection[0], intersection[1], cantilever_length
    )


def determine_catenary_indices(path, catenary_interval, offset):
    catenary_indices = [offset]
    bound = set([tuple(pt) for pt in path])

    reference_x, reference_y = path[offset]

    for i in range(offset, len(path)):
        x, y = path[i]

        dist = ((x - reference_x) ** 2 + (y - reference_y) ** 2) ** 0.5

        if dist < catenary_interval:
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
    added = 0

    for _ in range(max_iterations):
        mid = (low + high) // 2
        mid_x, mid_y = path[mid]

        if line_leaves_bound(step_line(mid_x, mid_y, prev_x, prev_y), bound):
            high = mid
        else:
            catenary_indices.append(mid)
            added += 1
            prev_x, prev_y = mid_x, mid_y
            low = mid

    if added == 0:
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
        for pt in bresenham_filled_circle(vector1[0], vector1[1], 2):
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
        for pt in bresenham_filled_circle(vector2[0], vector2[1], 2):
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


def determine_base(path, width, silhouette):
    base = set()
    radius = width // 2

    for x, y in path:
        circle_points = bresenham_filled_circle(x, y, radius)
        base.update(circle_points)

    return base.intersection(silhouette)


def determine_base_edge(path, width, base):
    edge = set()
    for x, y in base:
        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            if (x + dx, y + dy) not in base:
                edge.add((x, y))
                break

    filter_set = set()
    for x, y in path:
        filter_set.update(bresenham_filled_circle(int(x), int(y), width // 2 - 1))
    return edge - filter_set
