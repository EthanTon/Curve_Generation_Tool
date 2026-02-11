import util.dubinsUtil as dubinUtil
import util.trackUtil as trackUtil
import util.pantographUtil as pantographUtil
import util.elevationUtil as elevUtil
import util.exportUtil as exportUtil
import numpy as np


def _nearest_elevation(pt, elevation_map, default=0):
    """Spiral search for the nearest known ground elevation."""
    for r in range(1, 20):
        for dx in range(-r, r + 1):
            for dz in range(-r, r + 1):
                if abs(dx) != r and abs(dz) != r:
                    continue
                neighbour = (pt[0] + dx, pt[1] + dz)
                if neighbour in elevation_map:
                    return elevation_map[neighbour]
    return default


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


def parse_points(points, radius=30.0, base_width=13, track_width=7, slope_min=35,
                 pantograph_interval=45):
    """
    Process an ordered list of points and generate segments from consecutive pairs.

    Each point is:
        ((x, z, angle), y)

    where (x, z, angle) is the pose and y is the elevation.

    Returns:
        (results, errors)
    """
    if len(points) < 2:
        raise ValueError("Need at least 2 points to form a segment.")

    results = []
    errors = []
    offset = 0

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

        # --- Geometry (no pantograph – applied later on combined curve) ----
        base, track = trackUtil.track(
            center_track, start_pose, end_pose, base_width, track_width,
        )

        # --- Elevation mask -----------------------------------------------
        abs_dz = abs(realised_y_end - effective_y_start)
        if abs_dz > 0:
            pts = np.asarray(center_track, dtype=float)
            total_dist = np.sum(
                np.linalg.norm(np.diff(pts, axis=0), axis=1)
            ) if len(pts) > 1 else 1.0
            step_size = total_dist / ((abs_dz + 1) * np.sqrt(2))
        else:
            step_size = len(center_track) + 1

        elevation_mask = elevUtil.generate_elevation_mask(
            center_track,
            base_width,
            list(base[0]),
            step_size=step_size,
            start_idx=0,
            end_idx=-1,
            z_start=effective_y_start,
            z_end=realised_y_end,
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
        ((0, 0, 0), 8),
        ((40, 100, np.pi / 3), 11),
        ((170, 170, np.pi / 2), 13),
        ((170, 250, np.pi / 2), 13)
    ]

    results, errors = parse_points(
        points, radius=100.0, base_width=13, track_width=7, slope_min=35,
        pantograph_interval=35,
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
            f"y {res['y_start']}{eff} → {res['y_end_realised']}, "
            f"status={status}"
        )

    if errors:
        print("\n⚠  Elevation errors:")
        for seg_idx, msg in errors:
            print(f"  Segment {seg_idx}: {msg}")

    # ---- Merge all segments ----------------------------------------------
    combined_base = set()
    combined_brim = set()
    combined_elevation = {}
    combined_center_track = []
    combined_track_center = []          # mixed (for export rail rendering)
    combined_track_center_left = []     # left track centre  (for pantograph)
    combined_track_center_right = []    # right track centre (for pantograph)
    combined_rail0 = []
    combined_rail1 = []

    for res in results:
        base_set, brim_set = res["base"]
        combined_base.update(map(tuple, base_set))
        combined_brim.update(map(tuple, brim_set))

        # Accumulate the raw centre-line for the pantograph pass
        combined_center_track.extend(res["center_track"])

        track = res["track"]
        # track[0] is (left_ordered_with_dirs, right_ordered_with_dirs)
        track_center_pair = track[0]

        # Separate left / right track centres for pantograph intersection
        if (isinstance(track_center_pair, (list, tuple))
                and len(track_center_pair) == 2):
            left_tc, right_tc = track_center_pair
            if isinstance(left_tc, (list, tuple)):
                combined_track_center_left.extend(left_tc)
            if isinstance(right_tc, (list, tuple)):
                combined_track_center_right.extend(right_tc)

        # Also build the mixed list expected by the export rail renderer
        for sublist in track[0]:
            if (
                isinstance(sublist, (list, tuple))
                and len(sublist) > 0
                and isinstance(sublist[0], (list, tuple))
            ):
                combined_track_center.extend(sublist)
            else:
                combined_track_center.append(sublist)

        combined_rail0.extend(track[1][0])
        combined_rail0.extend(track[1][1])

        combined_rail1.extend(track[2][0])
        combined_rail1.extend(track[2][1])

        # Merge elevation mask into a flat dict
        for mask_idx, point_map in res["elevation_mask"].items():
            for pt, y in point_map.items():
                combined_elevation[pt] = y

    # ---- Pantograph (applied once on the full combined curve) -------------
    base_width = 13  # must match the value used during segment generation
    (
        combined_poles,
        pole_lines,
        cross_line_pixels,
        overhead_wire_pixels,
        tc1_intersections,
        tc2_intersections,
    ) = pantographUtil.apply_pantograph(
        combined_center_track,
        combined_base,
        base_width,
        combined_elevation,
        track_center1=combined_track_center_left,
        track_center2=combined_track_center_right,
        rail0=combined_rail0,
        rail1=combined_rail1,
        pantograph_interval=45,
    )

    # Ensure every pole position has a ground elevation entry.
    # Poles sit at the base edge and may not have one yet — look up the
    # nearest known elevation so the export places the pole column at the
    # correct Y.
    for left, right in combined_poles:
        for p in (left, right):
            key = (int(round(p[0])), int(round(p[1])))
            if key not in combined_elevation:
                combined_elevation[key] = _nearest_elevation(
                    key, combined_elevation
                )

    # Re-wrap elevation into the {0: {...}} format expected by export
    # NOTE: pole_lines is passed separately as pantograph_elevation so
    # overhead structures get their own interpolated Y.  Do NOT merge
    # pole_lines into combined_elevation — that would overwrite the
    # base/brim/track ground elevations with wire elevations.
    combined_elevation_wrapped = {0: combined_elevation}

    exportUtil.export_full_track(
        combined_base,
        combined_brim,
        combined_track_center,
        combined_rail0,
        combined_rail1,
        combined_elevation_wrapped,
        filename="track.schem",
        poles=combined_poles,
        cross_line_pixels=cross_line_pixels,
        overhead_wire_pixels=overhead_wire_pixels,
        pantograph_elevation=pole_lines,
    )

    print(f"\nExported {len(combined_base)} blocks to track.schem")
    print(f"Total pantograph poles: {len(combined_poles)} pairs")
    print(f"Cross-line pixels: {len(cross_line_pixels)}")
    print(f"Overhead wire pixels: {len(overhead_wire_pixels)}")
    print(f"Track 1 intersections: {len(tc1_intersections)}")
    print(f"Track 2 intersections: {len(tc2_intersections)}")


if __name__ == "__main__":
    main()