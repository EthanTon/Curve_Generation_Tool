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

    # Number of distinct elevation levels (inclusive of both endpoints)
    required_steps = abs(z_end - z_start) + 1 if z_end is not None else None

    boundaries = []
    if required_steps is not None:
        # Determine the largest step size that still produces enough mask layers
        # to cover the required elevation change, while staying >= min_step
        path_length = end_idx - start_idx
        # Floor division gives the largest step that yields enough segments
        elevation_step = path_length // (required_steps + 2)
        # Use the largest step possible, but no smaller than min_step
        effective_step = max(min_step, elevation_step)
        boundaries = path_boundaries(path, effective_step, start_idx, end_idx)
    else:
        boundaries = path_boundaries(path, step_size, start_idx, end_idx)

    masks = mask_all(path, boundaries, base_width, base)

    num_steps = len(masks)

    if required_steps is not None and num_steps < required_steps:
        raise ValueError(
            f"Invalid control point: the elevation change from z={z_start} to z={z_end} "
            f"requires {required_steps} mask layers, but only {num_steps} were generated "
            f"for the given path segment (indices {start_idx} to {end_idx}). "
            f"Adjust the control point positions or elevation values."
        )

    lut: dict[tuple, float] = {}
    for i, point_set in enumerate(masks):
        y_val = z_start + i * direction
        if z_end is not None:
            y_val = min(y_val, z_end) if direction >= 0 else max(y_val, z_end)
        for pt in point_set:
            lut[pt] = y_val
    return lut
