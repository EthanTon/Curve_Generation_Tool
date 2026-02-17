import numpy as np

from ..CoreUtil.shapeUtil import (
    bresenham_circle,
    bresenham_circle_4connected,
    bresenham_filled_arc,
    bresenham_filled_circle,
    bresenham_line,
    bresenham_filled_circle_stepped,
    bresenham_filled_arc_stepped,
)

from ..CoreUtil.pathUtil import add_directions_to_points


def track(path, start, end, base_width, track_width):
    base = generate_base(path, start, end, base_width)
    track_center = generate_track_center_double(path, track_width, base)
    rail_0 = generate_rail(track_center[0], 2, base, path)
    rail_1 = generate_rail(track_center[1], 2, base, path)
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


def generate_rail(track_center, rail_width, base, center_path=None):
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

    path_xy = [(p[0], p[1]) for p in center_path] if center_path is not None else None
    inner_rail = add_directions_to_points(inner_rail_xy, center_path=path_xy)
    outer_rail = add_directions_to_points(outer_rail_xy, center_path=path_xy)
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

    path_xy = [(float(p[0]), float(p[1])) for p in path]
    return (
        add_directions_to_points(left_ordered, center_path=path_xy),
        add_directions_to_points(right_ordered, center_path=path_xy),
    )


def generate_base(path, start, end, width):
    base = set()
    for n in path:
        base.update(bresenham_filled_circle(int(n[0]), int(n[1]), width // 2))
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
