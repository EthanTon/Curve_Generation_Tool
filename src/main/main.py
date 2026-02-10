import util.dubinsUtil as dubinUtil
import util.trackUtil as trackUtil
import util.elevationUtil as elevUtil
import util.exportUtil as exportUtil
import numpy as np


def compute_slope_for_segment(path_length, y_start, y_end, slope_min, offset=0):
    effective_y_start = y_start - offset  # actual elevation with deficit
    total_dz = y_end - effective_y_start  # signed total change needed
    abs_dz = abs(total_dz)

    if abs_dz == 0:
        return None, y_end, None, 0, effective_y_start

    half_length = path_length / 2

    # 1) Try to achieve the full change in the first half of the segment
    half_slope = half_length / abs_dz
    if half_slope >= slope_min:
        return half_slope, y_end, None, 0, effective_y_start

    # 2) Half isn't enough – try using the full segment length
    full_slope = path_length / abs_dz
    if full_slope >= slope_min:
        return full_slope, y_end, None, 0, effective_y_start

    # 3) Cannot reach target even at steepest allowed slope
    max_dz = int(path_length // slope_min)
    if total_dz > 0:
        realised_y_end = effective_y_start + max_dz
    else:
        realised_y_end = effective_y_start - max_dz

    remaining_offset = y_end - realised_y_end  # deficit to carry forward

    direction = "up" if total_dz > 0 else "down"
    err = (
        f"Cannot reach elevation {y_end} from effective start {effective_y_start} "
        f"({direction} {abs_dz}m) over path length {path_length}. "
        f"At steepest slope ({slope_min} steps/m), max change is {max_dz}m. "
        f"Carrying offset of {remaining_offset} to next segment."
    )
    return slope_min, realised_y_end, err, remaining_offset, effective_y_start


def parse_points(points, radius=30.0, base_width=13, track_width=7, slope_min=35):
    """
    Process an ordered list of points and generate segments from consecutive pairs.

    Each point is:
        ((x, z, angle), y)

    where (x, z, angle) is the pose and y is the elevation.

    Segments are formed from consecutive points:
        points[0] -> points[1], points[1] -> points[2], ...

    If a previous segment could not fully reach its target elevation,
    the deficit is carried forward as an offset into subsequent segments.

    Returns:
        (results, errors)
    """
    if len(points) < 2:
        raise ValueError("Need at least 2 points to form a segment.")

    results = []
    errors = []
    offset = 0  # uncompleted elevation carried from previous segments

    for idx in range(len(points) - 1):
        start_pose, start_y = points[idx]
        end_pose, end_y = points[idx + 1]

        # --- Dubins path --------------------------------------------------
        center_track = dubinUtil.int_dubins_path(start_pose, end_pose, radius)
        path_length = len(center_track)

        # --- Slope calculation (with offset from prior segments) ----------
        actual_slope, realised_y_end, err_msg, offset, effective_y_start = (
            compute_slope_for_segment(
                path_length, start_y, end_y, slope_min, offset
            )
        )

        if err_msg is not None:
            errors.append((idx, err_msg))

        # --- Geometry -----------------------------------------------------
        base, track = trackUtil.track(
            center_track, start_pose, end_pose, base_width, track_width
        )

        slope_to_use = actual_slope if actual_slope is not None else path_length + 1

        elevation_mask = elevUtil.generate_elevation_mask(
            center_track,
            base_width,
            list(base[0]),
            slope_to_use,
            0,
            -1,
            effective_y_start,
        )

        results.append(
            {
                "index": idx,
                "start": points[idx],
                "end": points[idx + 1],
                "y_start": start_y,
                "y_start_effective": effective_y_start,
                "y_end_target": end_y,
                "y_end_realised": realised_y_end,
                "offset_remaining": offset,
                "slope_used": slope_to_use,
                "slope_min": slope_min,
                "path_length": path_length,
                "center_track": center_track,
                "base": base,
                "track": track,
                "elevation_mask": elevation_mask,
            }
        )

    return results, errors


def main():
    # Define the path as an ordered list of points: ((x, z, angle), y)
    points = [
        ((0, 0, 0), 9),
        ((0, 100, np.pi / 3), 11),
        ((170, 170, np.pi / 2), 13),
    ]

    results, errors = parse_points(
        points, radius=50.0, base_width=13, track_width=7, slope_min=35
    )

    # ---- Report per-segment diagnostics ----------------------------------
    for res in results:
        status = "OK"
        if res["y_end_realised"] != res["y_end_target"]:
            status = (
                f"ADJUSTED (target={res['y_end_target']}, "
                f"realised={res['y_end_realised']}, "
                f"offset_remaining={res['offset_remaining']})"
            )
        eff = ""
        if res["y_start_effective"] != res["y_start"]:
            eff = f" (effective={res['y_start_effective']})"
        print(
            f"Segment {res['index']}: "
            f"path_len={res['path_length']}, "
            f"y {res['y_start']}{eff}→{res['y_end_realised']} "
            f"(slope={res['slope_used']:.1f}), "
            f"status={status}"
        )

    if errors:
        print("\n⚠  Elevation errors:")
        for seg_idx, msg in errors:
            print(f"  Segment {seg_idx}: {msg}")

    # ---- Merge all segments – elevation mask keyed by y-level ------------
    combined_base = set()
    combined_brim = set()
    combined_elevation = {}
    combined_track_center = []
    combined_rail0 = []
    combined_rail1 = []

    for res in results:
        base_set, brim_set = res["base"]
        combined_base.update(map(tuple, base_set))
        combined_brim.update(map(tuple, brim_set))

        # track[0] is a list of sub-lists — flatten into individual tuples
        track = res["track"]
        for sublist in track[0]:
            if isinstance(sublist, (list, tuple)) and len(sublist) > 0 and isinstance(sublist[0], (list, tuple)):
                combined_track_center.extend(sublist)
            else:
                combined_track_center.append(sublist)

        combined_rail0.extend(track[1][0])
        combined_rail0.extend(track[1][1])

        combined_rail1.extend(track[2][0])
        combined_rail1.extend(track[2][1])

        y_level = res["y_start_effective"]
        if y_level not in combined_elevation:
            combined_elevation[y_level] = {}

        for seg_idx, point_map in res["elevation_mask"].items():
            for pt, y in point_map.items():
                combined_elevation[y_level][pt] = y

    exportUtil.export_full_track(
        combined_base,
        combined_brim,
        combined_track_center,
        combined_rail0,
        combined_rail1,
        combined_elevation,
        filename="track.schem",
    )

    print(f"\nExported {len(combined_base)} blocks to track.schem")


if __name__ == "__main__":
    main()