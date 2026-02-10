import numpy as np


def generate_elevation_mask(
    path, base_width, base, step_size=1, start_idx=0, end_idx=-1, z_start=0
):
    boundaries = _path_boundaries(path, step_size, start_idx, end_idx)
    masks = mask_all(path, boundaries, base_width, base)

    lut = {}
    for i, point_set in enumerate(masks):
        lut[i] = {pt: z_start for pt in point_set}

    return lut


def _path_boundaries(path, step_size, start_idx, end_idx):
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


def _path_tangent(pts, index, window=3):
    lo = max(index - window, 0)
    hi = min(index + window, len(pts) - 1)
    tangent = pts[hi] - pts[lo]

    length = np.linalg.norm(tangent)
    if length > 0:
        tangent = tangent / length
    else:
        tangent = np.array([1.0, 0.0])

    return tangent


def mask_all(path, boundaries, base_width, base):
    pts = np.asarray(path, dtype=float)  # (P, 2)
    P = len(pts)
    base_list = list(base)
    base_arr = np.asarray(base_list, dtype=float)  # (B, 2)
    B = len(base_arr)
    n_boundaries = len(boundaries)
    n_segments = n_boundaries + 1

    if n_boundaries == 0:
        return [set(base)]

    # ------------------------------------------------------------------
    # Pass 1: Voronoi — assign each base point to nearest path index
    # ------------------------------------------------------------------
    chunk = max(1, int(25_000_000 / max(P, 1)))
    nearest_idx = np.empty(B, dtype=int)

    for lo in range(0, B, chunk):
        hi = min(lo + chunk, B)
        diffs = base_arr[lo:hi, np.newaxis, :] - pts[np.newaxis, :, :]
        dists_sq = np.sum(diffs**2, axis=2)  # (hi-lo, P)
        nearest_idx[lo:hi] = np.argmin(dists_sq, axis=1)

    # Map nearest path index → preliminary segment
    # Segment edges: (-inf, boundaries[0]), [boundaries[0], boundaries[1]), …
    seg_edges = np.array(boundaries, dtype=int)
    assignment = np.searchsorted(seg_edges, nearest_idx, side="right")
    # assignment[i] is in [0, n_segments-1]

    # ------------------------------------------------------------------
    # Pass 2: Orthogonal refinement at each boundary
    # Only re-split points within base_width/2 of the boundary point;
    # distant points keep their Voronoi assignment.
    # ------------------------------------------------------------------
    radius = base_width // 2 + 1  # +1 to ensure we cover all points within base_width/2
    radius_sq = radius * radius

    for j, b_idx in enumerate(boundaries):
        tangent = _path_tangent(pts, b_idx)
        origin = pts[b_idx]

        seg_before = j  # segment index on the "backward" side
        seg_after = j + 1  # segment index on the "forward" side

        # Select points currently in either neighbouring segment
        candidates = np.where((assignment == seg_before) | (assignment == seg_after))[0]

        if len(candidates) == 0:
            continue

        # Distance filter: only refine points within base_width/2 of origin
        diff = base_arr[candidates] - origin  # (C, 2)
        dist_sq = diff[:, 0] ** 2 + diff[:, 1] ** 2
        within = dist_sq <= radius_sq
        local = candidates[within]

        if len(local) == 0:
            continue

        # Signed projection: positive = forward of orthogonal line
        local_diff = base_arr[local] - origin
        proj = local_diff[:, 0] * tangent[0] + local_diff[:, 1] * tangent[1]

        assignment[local[proj >= 0]] = seg_after
        assignment[local[proj < 0]] = seg_before

    masks = []
    for seg in range(n_segments):
        masks.append({base_list[i] for i in np.where(assignment == seg)[0]})
    return masks


def mask(path, start_idx, end_idx, base_width, base):
    boundaries = []
    if start_idx is not None and start_idx > 0:
        boundaries.append(start_idx)
    if end_idx is not None and end_idx < len(path):
        boundaries.append(end_idx)

    masks = mask_all(path, boundaries, base_width, base)

    # Determine which segment corresponds to our request
    if start_idx is None or start_idx == 0:
        return masks[0]
    else:
        # Our segment is after the start boundary
        seg = boundaries.index(start_idx) + 1
        return masks[seg]
