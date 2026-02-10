import numpy as np

from util.shapeUtil import (
    bresenham_arc,
    bresenham_circle,
    bresenham_circle_4connected,
    bresenham_filled_arc,
    bresenham_filled_circle,
    bresenham_line,
    step_line,
)


def track(path, start, end, base_width, track_width, step_size=1):
    base = generate_base(path, start, end, base_width)
    base_edge = determine_base_edge(path, base_width, base)

    track_center = generate_track_center_double(path, track_width, base)
    rail_0 = generate_rail(track_center[0], 2, base)
    rail_1 = generate_rail(track_center[1], 2, base)
    brim = generate_brim(path, base_width, base)
    return [base, brim], [track_center, rail_0, rail_1]


def generate_brim(path, base_width, base):
    brim = set()
    valid_area = set(map(tuple, base)) - set(map(tuple, path))
    for pt in path:
        brim.update(
            set(bresenham_filled_circle(int(pt[0]), int(pt[1]), (base_width // 2) - 1))
            & valid_area
        )
    return brim


def _add_directions_to_points(points):
    if len(points) == 0:
        return []
    if len(points) == 1:
        return [(points[0][0], points[0][1], 1.0, 0.0)]
    result = []
    points_arr = np.asarray(points, dtype=float)
    for i in range(len(points)):
        if i == 0:
            direction = points_arr[1] - points_arr[0]
        elif i == len(points) - 1:
            direction = points_arr[-1] - points_arr[-2]
        else:
            direction = points_arr[i + 1] - points_arr[i - 1]
        length = np.linalg.norm(direction)
        if length > 0:
            direction /= length
        else:
            direction = np.array([1.0, 0.0])
        result.append(
            (points[i][0], points[i][1], float(direction[0]), float(direction[1]))
        )
    return result


def generate_rail(track_center, rail_width, base):
    if len(track_center) < 2:
        return [], []
    track_center_xy = [(pt[0], pt[1]) for pt in track_center]
    track_center_set = set(track_center_xy)
    rail_points = set()
    for pt in track_center_xy:
        for n in bresenham_circle(pt[0], pt[1], rail_width // 2):
            if n not in track_center_set and n in base:
                rail_points.add(n)
    if not rail_points:
        return [], []
    track_arr = np.asarray(track_center_xy, dtype=float)
    directions = np.empty_like(track_arr)
    directions[0] = track_arr[1] - track_arr[0]
    directions[-1] = track_arr[-1] - track_arr[-2]
    if len(track_arr) > 2:
        directions[1:-1] = track_arr[2:] - track_arr[:-2]
    lengths = np.linalg.norm(directions, axis=1, keepdims=True)
    lengths[lengths == 0] = 1
    directions /= lengths
    left_normals = np.column_stack([-directions[:, 1], directions[:, 0]])
    inner_points = []
    outer_points = []
    for pt in rail_points:
        diffs = track_arr - np.array([pt[0], pt[1]])
        dists_sq = np.sum(diffs**2, axis=1)
        closest_idx = np.argmin(dists_sq)
        px, py = track_arr[closest_idx]
        nx, ny = left_normals[closest_idx]
        dx, dy = pt[0] - px, pt[1] - py
        if dx * nx + dy * ny > 0:
            outer_points.append((closest_idx, pt))
        else:
            inner_points.append((closest_idx, pt))
    inner_points.sort(key=lambda x: x[0])
    outer_points.sort(key=lambda x: x[0])
    inner_rail_xy = [pt for _, pt in inner_points]
    outer_rail_xy = [pt for _, pt in outer_points]
    inner_rail = _add_directions_to_points(inner_rail_xy)
    outer_rail = _add_directions_to_points(outer_rail_xy)
    return inner_rail, outer_rail


def generate_track_center_double(path, track_width, base):
    path = np.asarray(path)
    if len(path) < 2:
        return [], []
    radius_outer = track_width // 2
    radius_inner = radius_outer - 1
    filter_set = set()
    boundary_set = set()
    for n in path:
        x, y = int(n[0]), int(n[1])
        filter_set.update(bresenham_filled_circle(x, y, radius_inner))
    for n in path:
        x, y = int(n[0]), int(n[1])
        for pt in bresenham_circle_4connected(x, y, radius_outer):
            if pt not in filter_set and pt in base:
                boundary_set.add(pt)
    if not boundary_set:
        return [], []
    path_float = path.astype(float)
    directions = np.empty_like(path_float)
    directions[0] = path_float[1] - path_float[0]
    directions[-1] = path_float[-1] - path_float[-2]
    if len(path) > 2:
        directions[1:-1] = path_float[2:] - path_float[:-2]
    lengths = np.linalg.norm(directions, axis=1, keepdims=True)
    lengths[lengths == 0] = 1
    directions /= lengths
    left_normals = np.column_stack([-directions[:, 1], directions[:, 0]])
    search_radius_sq = (radius_outer + 2) ** 2
    left_set = set()
    right_set = set()
    remaining = boundary_set.copy()
    for i, (px, py) in enumerate(path_float):
        if not remaining:
            break
        nx, ny = left_normals[i]
        to_remove = []
        for pt in remaining:
            dx, dy = pt[0] - px, pt[1] - py
            if dx * dx + dy * dy <= search_radius_sq:
                if dx * nx + dy * ny > 0:
                    left_set.add(pt)
                else:
                    right_set.add(pt)
                to_remove.append(pt)
        for pt in to_remove:
            remaining.remove(pt)
    for pt in remaining:
        if len(left_set) <= len(right_set):
            left_set.add(pt)
        else:
            right_set.add(pt)

    def order_points_manhattan(pt_set):
        if len(pt_set) <= 1:
            return list(pt_set)
        neighbors = {}
        endpoints = []
        for pt in pt_set:
            x, y = pt
            adj = []
            for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                if (nx, ny) in pt_set:
                    adj.append((nx, ny))
            neighbors[pt] = adj
            if len(adj) == 1:
                endpoints.append(pt)
        start = endpoints[0] if endpoints else next(iter(pt_set))
        ordered = [start]
        visited = {start}
        current = start
        n = len(pt_set)
        while len(ordered) < n:
            next_pt = None
            for neighbor in neighbors[current]:
                if neighbor not in visited:
                    next_pt = neighbor
                    break
            if next_pt is None:
                cx, cy = current
                min_dist = float("inf")
                for pt in pt_set:
                    if pt not in visited:
                        d = abs(pt[0] - cx) + abs(pt[1] - cy)
                        if d < min_dist:
                            min_dist = d
                            next_pt = pt
                if next_pt is None:
                    break
            ordered.append(next_pt)
            visited.add(next_pt)
            current = next_pt
        return ordered

    left_ordered = order_points_manhattan(left_set)
    right_ordered = order_points_manhattan(right_set)
    return _add_directions_to_points(left_ordered), _add_directions_to_points(
        right_ordered
    )


def generate_track_center_single(path, track_width, base):
    track_center = []
    seen = set()
    for i in range(len(path) - 1):
        p1 = path[i]
        p2 = path[i + 1]
        segment = step_line(p1[0], p1[1], p2[0], p2[1])
        for pt in segment:
            if pt in seen:
                continue
            if pt in base:
                seen.add(pt)
                track_center.append(pt)
    return _add_directions_to_points(track_center)


def generate_base(path, start, end, width):
    base = set()
    for n in path:
        base.update(bresenham_filled_circle(n[0], n[1], width // 2))
    clean_end(base, start, end, width // 2)
    return base


def clean_end(base, start, end, distance):
    s_sin, s_cos = np.sin(start[2]) * distance, np.cos(start[2]) * distance
    e_sin, e_cos = np.sin(end[2]) * distance, np.cos(end[2]) * distance
    ortho_start_1 = np.rint([start[0] - s_sin, start[1] + s_cos])
    ortho_start_2 = np.rint([start[0] + s_sin, start[1] - s_cos])
    ortho_end_1 = np.rint([end[0] + e_sin, end[1] - e_cos])
    ortho_end_2 = np.rint([end[0] - e_sin, end[1] + e_cos])
    filter = set(
        bresenham_filled_arc(
            int(start[0]), int(start[1]), distance, ortho_start_1, ortho_start_2, True
        )
        + bresenham_filled_arc(
            int(end[0]), int(end[1]), distance, ortho_end_1, ortho_end_2, True
        )
    )
    protected = set(
        bresenham_line(
            ortho_start_1[0], ortho_start_1[1], ortho_start_2[0], ortho_start_2[1]
        )
        + bresenham_line(ortho_end_1[0], ortho_end_1[1], ortho_end_2[0], ortho_end_2[1])
    )
    for pt in filter:
        if pt in base and pt not in protected:
            base.remove(pt)
    return


def determine_base_edge(path, width, base):
    filter = set()
    for x, y in path:
        filter.update(bresenham_filled_circle(x, y, width // 2 - 1))
    return set(base - filter)


def determine_orthogonal_intersection_points(p_r, width, path_r):
    x, y = p_r
    distance = width // 2

    def get_avg_intersection(start, end, clockwise):
        arc = bresenham_arc(x, y, distance, start, end, clockwise)
        return average_point(set(arc) & set(path_r))

    s_v, e_v = (x, y + distance), (x, y - distance)
    avg_right = get_avg_intersection(s_v, e_v, True)
    avg_left = get_avg_intersection(s_v, e_v, False)
    if np.linalg.norm(np.array(avg_right) - np.array(avg_left)) > distance:
        return avg_right, avg_left
    s_h, e_h = (x + distance, y), (x - distance, y)
    avg_top = get_avg_intersection(s_h, e_h, True)
    avg_bottom = get_avg_intersection(s_h, e_h, False)
    return avg_top, avg_bottom


def average_point(points):
    if not points:
        return (0, 0)
    pts = list(points)
    x_avg = np.rint(sum(p[0] for p in pts) / len(pts))
    y_avg = np.rint(sum(p[1] for p in pts) / len(pts))
    return (x_avg, y_avg)


def weighted_pca_orthogonal(path, index, sigma=2):
    pts = np.array(path, dtype=float)
    center = pts[index]

    # Gaussian weights based on index distance
    indices = np.arange(len(path))
    weights = np.exp(-0.5 * ((indices - index) / sigma) ** 2)

    weighted_center = np.average(pts, axis=0, weights=weights)
    centered = pts - weighted_center

    # Weighted covariance
    W = np.diag(weights)
    cov = (centered.T @ W @ centered) / weights.sum()

    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    tangent = eigenvectors[:, np.argmax(eigenvalues)]
    normal = (-tangent[1], tangent[0])
    return normal
