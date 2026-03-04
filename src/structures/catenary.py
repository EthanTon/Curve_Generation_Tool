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
    min_y_lut=None,
    skip_trailing_pole=False,
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

    if not skip_trailing_pole and (not p_indices or p_indices[-1] < last_path_idx):
        base_edge = determine_base_edge(path, base_width // 2 - 1, base, track_width=track_width)
        all_tracks = set(track1) | set(track2)
        pole_a, pole_b = determine_catenary_pole_position(
            path, last_path_idx, base_width, base_edge, track_positions=all_tracks
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

    # Apply minimum y level filter
    if min_y_lut and curve:
        for block in list(curve.keys()):
            filtered = set()
            for pos in curve[block]:
                x, y, z = pos
                min_y = min_y_lut.get((x, z))
                if min_y is not None and y < min_y:
                    continue
                filtered.add(pos)
            if filtered:
                curve[block] = filtered
            else:
                del curve[block]

    return curve, intersection_info


def generate_catenary(
    path, track1, track2, base, base_width, track_width, catenary_interval, offset=0
):
    base_edge = determine_base_edge(path, base_width // 2 - 1, base, track_width=track_width)
    cantilever_length = base_width // 2 - track_width // 2 + 1

    tc1 = set(track1)
    tc2 = set(track2)
    all_tracks = tc1 | tc2

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
            path, idx, base_width, base_edge, track_positions=all_tracks
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
    """Generate up to max_iterations equally-spaced poles between the last
    accepted index and *current_idx*.  Each candidate is checked for
    line-of-sight to the previous accepted point; as soon as one fails
    the remaining candidates are skipped so we never leave a gap."""
    last_idx = catenary_indices[-1]
    span = current_idx - last_idx
    if span <= 0:
        catenary_indices.append(current_idx)
        return

    # Build up to max_iterations equally-spaced candidate indices
    n_poles = min(max_iterations, span)
    candidates = []
    for k in range(1, n_poles + 1):
        idx = last_idx + round(k * span / (n_poles + 1))
        idx = max(last_idx + 1, min(idx, current_idx - 1))
        if not candidates or idx > candidates[-1]:
            candidates.append(idx)

    added = 0
    prev_x, prev_y = path[last_idx]
    for idx in candidates:
        mid_x, mid_y = path[idx]
        if line_leaves_bound(step_line(mid_x, mid_y, prev_x, prev_y), bound):
            break
        catenary_indices.append(idx)
        added += 1
        prev_x, prev_y = mid_x, mid_y

    if added == 0:
        catenary_indices.append(current_idx)


def determine_catenary_pole_position(path, index, base_width, base_edge, track_positions=None):
    """Determine two pole positions on opposite edges of the base, orthogonal
    to the path at *index*.

    *track_positions* is an optional set of (x, z) coordinates that belong to
    the track area.  When provided, any candidate that lands inside the track
    is rejected so that poles always sit on the true outer edge.

    The search always runs: even if the initial normal-projected point is in
    base_edge, we verify it isn't overlapping the track and pick the best
    candidate from the search area.
    """
    sigma = 1.0
    xp, yp = path[index]
    scalar = base_width // 2

    normal = weighted_pca_orthogonal(path, index, sigma)
    vector1 = (round(normal[0] * scalar + xp), round(normal[1] * scalar + yp))
    vector2 = (round(-normal[0] * scalar + xp), round(-normal[1] * scalar + yp))

    snap_radius = max(3, base_width // 4)

    def _snap_to_edge(vec):
        """Search for the best base_edge point near *vec* that is not on a
        track.  Always runs the full search — never short-circuits — so that
        we guarantee the result is on the true outer edge even when the
        initial projection looks valid.

        Among candidates, prefer the one closest to *vec* (stays on the
        normal).  Ties broken by distance from path center (farther = more
        outer = better)."""
        best = None
        best_dist = float("inf")
        best_center_dist = -1
        for pt in bresenham_filled_circle(vec[0], vec[1], snap_radius):
            if pt not in base_edge:
                continue
            if track_positions and pt in track_positions:
                continue
            d = (pt[0] - vec[0]) ** 2 + (pt[1] - vec[1]) ** 2
            cd = (pt[0] - xp) ** 2 + (pt[1] - yp) ** 2
            if d < best_dist or (d == best_dist and cd > best_center_dist):
                best_dist = d
                best_center_dist = cd
                best = pt
        return best

    vector1 = _snap_to_edge(vector1)
    if vector1 is None:
        return (None, None)

    vector2 = _snap_to_edge(vector2)
    if vector2 is None:
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


def determine_base_edge(path, width, base, track_width=None):
    edge = set()
    for x, y in base:
        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            if (x + dx, y + dy) not in base:
                edge.add((x, y))
                break

    # Use track_width to determine the exclusion radius so poles never
    # land inside the track area.  Fall back to the old (narrow) radius
    # when track_width is not provided for backward compatibility.
    if track_width is not None:
        filter_radius = track_width // 2 + 1
    else:
        filter_radius = width // 2 - 1

    filter_set = set()
    for x, y in path:
        filter_set.update(bresenham_filled_circle(int(x), int(y), filter_radius))
    return edge - filter_set