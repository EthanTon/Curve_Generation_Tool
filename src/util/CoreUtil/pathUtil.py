import numpy as np


def path_boundaries(path, step_size, start_idx, end_idx):
    if end_idx == -1:
        end_idx = len(path) - 1

    boundaries = [start_idx] if start_idx != 0 else []
    slope = step_size * np.sqrt(2)
    reference = path[start_idx]

    for n in range(start_idx, end_idx + 1):
        if np.linalg.norm(reference - path[n]) > slope:
            reference = path[n]
            boundaries.append(n)

    return boundaries


def path_tangent(pts, index, window=3):
    """Return tangent angle (radians) at index via finite difference over a window."""
    lo = max(index - window, 0)
    hi = min(index + window, len(pts) - 1)
    dx = pts[hi][0] - pts[lo][0]
    dy = pts[hi][1] - pts[lo][1]
    if dx == 0 and dy == 0:
        return 0.0
    return float(np.arctan2(dy, dx))


def weighted_pca_orthogonal(path, index, sigma=2):
    pts = np.array(path, dtype=float)
    indices = np.arange(len(path))
    weights = np.exp(-0.5 * ((indices - index) / sigma) ** 2)
    weighted_center = np.average(pts, axis=0, weights=weights)
    centered = pts - weighted_center
    W = np.diag(weights)
    cov = (centered.T @ W @ centered) / weights.sum()
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    tangent = eigenvectors[:, np.argmax(eigenvalues)]
    return (-tangent[1], tangent[0])


def add_directions_to_points(points, center_path=None):
    """
    Assigns a unit tangent direction to each point.

    If *center_path* is provided, directions are derived from the nearest
    point on that path (smooth, stable).  Otherwise a simple finite-
    difference fallback is used on *points* itself.
    """
    if len(points) == 0:
        return []

    pts = np.asarray(points, dtype=float)

    if center_path is not None and len(center_path) >= 2:
        angles = center_path_tangents(center_path)
        cp = np.asarray(center_path, dtype=float)

        chunk = max(1, int(25_000_000 / max(len(cp), 1)))
        nearest_idx = np.empty(len(pts), dtype=int)
        for lo in range(0, len(pts), chunk):
            hi = min(lo + chunk, len(pts))
            diffs = pts[lo:hi, np.newaxis, :] - cp[np.newaxis, :, :]
            dists_sq = np.sum(diffs**2, axis=2)
            nearest_idx[lo:hi] = np.argmin(dists_sq, axis=1)

        result = []
        for i in range(len(pts)):
            a = angles[nearest_idx[i]]
            result.append(
                (
                    float(pts[i][0]),
                    float(pts[i][1]),
                    float(np.cos(a)),
                    float(np.sin(a)),
                )
            )
        return result

    if len(points) == 1:
        return [(float(pts[0][0]), float(pts[0][1]), 1.0, 0.0)]

    directions = np.empty_like(pts)
    directions[0] = pts[1] - pts[0]
    directions[-1] = pts[-1] - pts[-2]
    if len(pts) > 2:
        directions[1:-1] = pts[2:] - pts[:-2]
    lengths = np.linalg.norm(directions, axis=1, keepdims=True)
    lengths[lengths == 0] = 1
    directions /= lengths

    return [
        (
            float(pts[i][0]),
            float(pts[i][1]),
            float(directions[i][0]),
            float(directions[i][1]),
        )
        for i in range(len(pts))
    ]


def center_path_tangents(center_path, window=3):
    """Return list of tangent angles (radians) for every point on the path."""
    n = len(center_path)
    angles = [0.0] * n
    for i in range(n):
        lo = max(i - window, 0)
        hi = min(i + window, n - 1)
        dx = center_path[hi][0] - center_path[lo][0]
        dy = center_path[hi][1] - center_path[lo][1]
        angles[i] = float(np.arctan2(dy, dx)) if (dx != 0 or dy != 0) else 0.0
    return angles


def optimal_tangent_window(radius, min_window=1, max_window=None):
    import math

    if radius <= 0:
        raise ValueError("radius must be positive")

    w_opt = math.sqrt(2.0 * radius)

    if max_window is None:

        max_window = max(int(2 * math.ceil(w_opt)), 10)

    window = int(round(w_opt))
    window = max(min_window, min(window, max_window))
    return window


def ortho(vect2d):
    return np.array((-vect2d[1], vect2d[0]))


def dist(pt_a, pt_b):
    return ((pt_a[0] - pt_b[0]) ** 2 + (pt_a[1] - pt_b[1]) ** 2) ** 0.5
