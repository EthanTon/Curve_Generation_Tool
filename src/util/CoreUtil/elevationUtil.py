import numpy as np

from ..CoreUtil.pathUtil import path_boundaries
from ..CoreUtil.maskingUtil import mask_all


def _points_match(a, b):
    """Compare two points by their first 2 coordinates (x, y), ignoring z."""
    return np.array_equal(np.asarray(a)[:2], np.asarray(b)[:2])


def _nearest_path_index(path, point, tolerance=None):
    """Return the index of the path point closest to `point` (2D).

    If *tolerance* is given the match must be within that distance,
    otherwise the closest point is returned unconditionally.
    """
    target = np.asarray(point, dtype=float)[:2]
    best_idx = None
    best_dist = np.inf
    for idx, pt in enumerate(path):
        d = np.linalg.norm(np.asarray(pt, dtype=float)[:2] - target)
        if d < best_dist:
            best_dist = d
            best_idx = idx
    if tolerance is not None and best_dist > tolerance:
        return None
    return best_idx


def generate_elevation_lookup(
    path, base_width, base, step_size, elevation_control_points
):
    lut = {}

    for i in range(len(elevation_control_points) - 1):
        start_pt = elevation_control_points[i]
        end_pt = elevation_control_points[i + 1]

        # Find the indices in the path closest to the start and end control points
        start_idx = _nearest_path_index(path, start_pt)
        end_idx = _nearest_path_index(path, end_pt)

        if start_idx is None:
            raise ValueError(
                f"Invalid control point: {start_pt} was not found in the path."
            )
        if end_idx is None:
            raise ValueError(
                f"Invalid control point: {end_pt} was not found in the path."
            )

        # Determine the z levels for the start and end control points
        z_start = start_pt[2]
        z_end = end_pt[2]

        # Generate the elevation mask for this segment of the path
        segment_lut = generate_elevation_mask(
            path,
            base_width,
            base,
            step_size,
            min_step=step_size,
            start_idx=start_idx,
            end_idx=end_idx,
            z_start=z_start,
            z_end=z_end,
        )

        # Merge the segment LUT into the overall LUT
        lut.update(segment_lut)

    return lut


def generate_elevation_mask(
    path,
    base_width,
    base,
    step_size=1,
    min_step=1,
    start_idx=0,
    end_idx=-1,
    z_start=0,
    z_end=None,
):
    direction = -1 if (z_end is not None and z_end < z_start) else 1
    elevation_change = abs(z_end - z_start) if z_end is not None else 0

    np_path = [np.asarray(pt, dtype=float) for pt in path]

    # Slice path to only the segment of interest so mask_all doesn't
    # assign base points from other segments via Voronoi spill-over.
    sub_path = np_path[start_idx : end_idx + 1]

    # Filter base to a bounding box around the sub-path to prevent
    # distant points from being claimed by edge segments.
    sub_arr = np.asarray(sub_path)
    margin = base_width + 1
    bbox_min = sub_arr.min(axis=0) - margin
    bbox_max = sub_arr.max(axis=0) + margin
    filtered_base = {
        pt
        for pt in base
        if bbox_min[0] <= pt[0] <= bbox_max[0] and bbox_min[1] <= pt[1] <= bbox_max[1]
    }

    # Generate boundaries relative to the sub-path (0-indexed).
    effective_step = max(min_step, step_size)
    boundaries = path_boundaries(sub_path, effective_step, 0, len(sub_path) - 1)
    masks = mask_all(sub_path, boundaries, base_width, filtered_base)
    num_masks = len(masks)

    if num_masks == 0:
        return {}

    if elevation_change > 0 and num_masks < elevation_change:
        segment_length = len(sub_path)
        max_step_for_change = segment_length // elevation_change
        raise ValueError(
            f"Cannot reach elevation change from y={z_start} to y={z_end} "
            f"(Δ{elevation_change}) with step_size={effective_step}. "
            f"The path segment (indices {start_idx}–{end_idx}, length {segment_length}) "
            f"produced {num_masks} mask layers but at least {elevation_change} are required. "
            f"Reduce step_size to {max_step_for_change} or less, shorten the elevation "
            f"change, or lengthen the path segment."
        )

    lut: dict[tuple, float] = {}

    # Seed the segment endpoints so path points at the control-point
    # locations are always present, even if path_boundaries skips them.
    start_key = tuple(int(c) for c in path[start_idx][:2])
    lut[start_key] = z_start
    if z_end is not None:
        end_key = tuple(int(c) for c in path[end_idx][:2])
        lut[end_key] = z_end

    if elevation_change == 0 or z_end is None:
        # Flat segment – every mask layer shares the same y level.
        for point_set in masks:
            for pt in point_set:
                lut[pt] = z_start
    else:
        # Distribute the elevation change evenly across all mask layers,
        # using the *largest* interval possible between successive y-level
        # changes.  We linearly map the mask index to the elevation range
        # so that the first mask is at z_start and the last at z_end.
        for i, point_set in enumerate(masks):
            if num_masks > 1:
                level = round(elevation_change * i / (num_masks - 1))
            else:
                level = elevation_change
            y_val = z_start + level * direction
            for pt in point_set:
                lut[pt] = y_val

    return lut
