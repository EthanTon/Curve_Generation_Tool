import numpy as np
from util.CoreUtil.dubinsUtil import int_dubins_path
from util.CoreUtil.pathUtil import add_directions_to_points
from util.CoreUtil.shapeUtil import (
    bresenham_filled_circle,
    bresenham_filled_arc,
    bresenham_line,
    bresenham_circle,
)


def draw_path(control_points, radius):
    if len(control_points) < 2:
        raise ValueError("At least two control points are required to generate a path.")
    path = []
    for i in range(len(control_points) - 1):
        start = control_points[i]
        end = control_points[i + 1]
        segment = int_dubins_path(start, end, radius)
        if i > 0 and path and len(segment) > 0:
            if np.array_equal(np.asarray(path[-1])[:2], np.asarray(segment[0])[:2]):
                segment = segment[1:]
        path.extend(segment)

    # Ensure the final control point is represented as the last path point
    if path and not np.array_equal(
        np.asarray(path[-1])[:2], np.asarray(control_points[-1])[:2]
    ):
        # Match the dimensionality of existing path points
        pt = np.asarray(control_points[-1])
        target_dim = len(path[-1])
        path.append(tuple(pt[:target_dim]))

    return path, control_points[0], control_points[-1]


def draw_path_silhouette(path, start, end, width):
    radius = width // 2

    silhouette = set()
    for x, y in path:
        silhouette.update(bresenham_filled_circle(x, y, radius))

    _clean_end_(silhouette, start, end, width)

    return silhouette


def draw_path_outline(path, width, silhouette):
    radius = width // 2 - 1

    filter = set()
    for x, y in path:
        filter.update(bresenham_filled_circle(x, y, radius))

    return filter - silhouette


def draw_offset_path(path, distance, start=None, end=None):
    if len(path) < 2:
        return []
    width = abs(distance) * 2

    if start is None:
        sx, sy = path[0]
        start_angle = np.arctan2(path[1][1] - sy, path[1][0] - sx)
        start = (sx, sy, start_angle)
    if end is None:
        ex, ey = path[-1]
        end_angle = np.arctan2(ey - path[-2][1], ex - path[-2][0])
        end = (ex, ey, end_angle)

    silhouette = draw_path_silhouette(path, start, end, width)
    candidates = draw_path_outline(path, width, silhouette)

    if not candidates:
        return []

    source = np.asarray(path, dtype=float)
    tangents = np.empty_like(source)
    tangents[0] = source[1] - source[0]
    tangents[-1] = source[-1] - source[-2]
    if len(source) > 2:
        tangents[1:-1] = source[2:] - source[:-2]
    magnitudes = np.linalg.norm(tangents, axis=1, keepdims=True)
    magnitudes[magnitudes == 0] = 1
    tangents /= magnitudes
    normals = np.column_stack([-tangents[:, 1], tangents[:, 0]])

    result = []
    for candidate in candidates:
        offsets = source - np.array([candidate[0], candidate[1]])
        dists_sq = np.sum(offsets**2, axis=1)
        nearest = np.argmin(dists_sq)
        cx, cy = source[nearest]
        nx, ny = normals[nearest]
        dx, dy = candidate[0] - cx, candidate[1] - cy
        side = dx * nx + dy * ny
        if (distance > 0 and side > 0) or (distance < 0 and side <= 0):
            tx, ty = tangents[nearest]
            position = nearest + (dx * tx + dy * ty)
            result.append((position, candidate))

    result.sort(key=lambda x: x[0])
    return [pt for _, pt in result]


def _clean_end_(silhouette, start, end, distance):
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
        if pt in silhouette and pt not in protected:
            silhouette.remove(pt)
    return
