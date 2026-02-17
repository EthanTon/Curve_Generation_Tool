import numpy as np

from .pathUtil import path_tangent

def mask_all(path, boundaries, base_width, base):
    pts = np.asarray(path, dtype=float)
    P = len(pts)
    base_list = list(base)
    base_arr = np.asarray(base_list, dtype=float)
    B = len(base_arr)
    n_boundaries = len(boundaries)
    n_segments = n_boundaries + 1

    if n_boundaries == 0:
        return [set(base)]

    # --- Pass 1: Voronoi assignment ------------------------------------
    assignment = _voronoi_assign(base_arr, pts, B, P, boundaries)

    # --- Pass 2: Orthogonal refinement ---------------------------------
    _refine_boundaries(assignment, base_arr, pts, boundaries, base_width)

    # --- Collect results ------------------------------------------------
    return [
        {base_list[i] for i in np.where(assignment == seg)[0]}
        for seg in range(n_segments)
    ]


def mask(path, start_idx, end_idx, base_width, base):
    boundaries = []
    if start_idx is not None and start_idx > 0:
        boundaries.append(start_idx)
    if end_idx is not None and end_idx < len(path):
        boundaries.append(end_idx)

    masks = mask_all(path, boundaries, base_width, base)

    if start_idx is None or start_idx == 0:
        return masks[0]

    seg = boundaries.index(start_idx) + 1
    return masks[seg]


def _voronoi_assign(base_arr, pts, B, P, boundaries):
    """Assign each base point to the nearest path index, then bucket."""
    chunk = max(1, int(25_000_000 / max(P, 1)))
    nearest_idx = np.empty(B, dtype=int)

    for lo in range(0, B, chunk):
        hi = min(lo + chunk, B)
        diffs = base_arr[lo:hi, np.newaxis, :] - pts[np.newaxis, :, :]
        dists_sq = np.sum(diffs ** 2, axis=2)
        nearest_idx[lo:hi] = np.argmin(dists_sq, axis=1)

    seg_edges = np.array(boundaries, dtype=int)
    return np.searchsorted(seg_edges, nearest_idx, side="right")


def _refine_boundaries(assignment, base_arr, pts, boundaries, base_width):
    """Re-split points near each boundary using the orthogonal plane."""
    radius = base_width // 2 + 1
    radius_sq = radius * radius

    for j, b_idx in enumerate(boundaries):
        tangent = path_tangent(pts, b_idx)
        origin = pts[b_idx]

        seg_before = j
        seg_after = j + 1

        candidates = np.where(
            (assignment == seg_before) | (assignment == seg_after)
        )[0]
        if len(candidates) == 0:
            continue

        diff = base_arr[candidates] - origin
        dist_sq = diff[:, 0] ** 2 + diff[:, 1] ** 2
        local = candidates[dist_sq <= radius_sq]
        if len(local) == 0:
            continue

        local_diff = base_arr[local] - origin
        proj = local_diff[:, 0] * tangent[0] + local_diff[:, 1] * tangent[1]

        assignment[local[proj >= 0]] = seg_after
        assignment[local[proj < 0]] = seg_before