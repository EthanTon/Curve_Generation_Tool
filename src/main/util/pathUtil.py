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
    lo = max(index - window, 0)
    hi = min(index + window, len(pts) - 1)
    tangent = pts[hi] - pts[lo]

    length = np.linalg.norm(tangent)
    if length > 0:
        tangent = tangent / length
    else:
        tangent = np.array([1.0, 0.0])

    return tangent


def ortho(vect2d):
    return np.array((-vect2d[1], vect2d[0]))


def dist(pt_a, pt_b):
    return ((pt_a[0] - pt_b[0]) ** 2 + (pt_a[1] - pt_b[1]) ** 2) ** 0.5

