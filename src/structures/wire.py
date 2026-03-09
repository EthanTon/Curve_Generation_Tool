from util.CoreUtil.shapeUtil import step_line, bresenham_line
from util.CoreUtil.blockUtil import resolve_block_connections
from structures.catenary import determine_intersections


def _nearest_elevation(x, z, elevation_lut, default=0):
    ix, iz = int(round(x)), int(round(z))
    if (ix, iz) in elevation_lut:
        return elevation_lut[(ix, iz)]
    for r in range(1, 30):
        for dx in range(-r, r + 1):
            for dz in range(-r, r + 1):
                if abs(dx) != r and abs(dz) != r:
                    continue
                neighbor = (ix + dx, iz + dz)
                if neighbor in elevation_lut:
                    return elevation_lut[neighbor]
    return default


def _draw_lines(points, elevation_lut, use_step_line):
    line_func = step_line if use_step_line else bresenham_line
    result = []

    for i in range(len(points) - 1):
        x0, z0 = int(round(points[i][0])), int(round(points[i][1]))
        x1, z1 = int(round(points[i + 1][0])), int(round(points[i + 1][1]))

        y_start = _nearest_elevation(x0, z0, elevation_lut)
        y_end = _nearest_elevation(x1, z1, elevation_lut)

        segment = line_func(x0, z0, x1, z1)
        seg_len = len(segment)

        for j, (x, z) in enumerate(segment):
            # Avoid duplicating connecting points between segments
            if i > 0 and j == 0:
                continue
            t = j / (seg_len - 1) if seg_len > 1 else 0.0
            y = round(y_start + t * (y_end - y_start))
            result.append((x, y, z))

    return result


def _stamp_cross_section(base_points, wire_cross_section):
    blocks = {}
    for bx, by, bz in base_points:
        for block, offsets in wire_cross_section.items():
            for rx, ry, rz in offsets:
                pos = (bx + rx, by + ry, bz + rz)
                blocks.setdefault(block, set()).add(pos)
    return blocks


def _filter_valid_points(intersections):
    """Return only non-None intersection points."""
    return [pt for pt in intersections if pt is not None]


def _over_track_points(
    path, tangents, track1, track2, track_width, indices, elevation_lut, use_step_line
):
    if not indices:
        return []

    t1_ints, t2_ints = determine_intersections(
        path, tangents, track1, track2, track_width, indices
    )

    base_points = []

    t1_pts = _filter_valid_points(t1_ints)
    if len(t1_pts) >= 2:
        base_points.extend(_draw_lines(t1_pts, elevation_lut, use_step_line))

    t2_pts = _filter_valid_points(t2_ints)
    if len(t2_pts) >= 2:
        base_points.extend(_draw_lines(t2_pts, elevation_lut, use_step_line))

    return base_points


def _build_excess_segment(
    path,
    tangents,
    track1,
    track2,
    track_width,
    excess_indices,
    pole_t1_endpoint,
    pole_t2_endpoint,
    elevation_lut,
    use_step_line,
    append_pole,
):
    if not excess_indices:
        return []

    t1_ints, t2_ints = determine_intersections(
        path, tangents, track1, track2, track_width, excess_indices
    )

    base_points = []

    for track_ints, pole_pt in [
        (t1_ints, pole_t1_endpoint),
        (t2_ints, pole_t2_endpoint),
    ]:
        pts = _filter_valid_points(track_ints)

        # Connect the excess to the pole intersection point
        if pole_pt is not None:
            if append_pole:
                pts.append(pole_pt)
            else:
                pts.insert(0, pole_pt)

        if len(pts) >= 2:
            base_points.extend(_draw_lines(pts, elevation_lut, use_step_line))

    return base_points


def assemble(
    path,
    tangents,
    elevation_lut,
    track1,
    track2,
    track_width,
    wires,
    catenary_intersection_data,
    subpath_ranges,
    resolve_blocks=True,
    protected_coords=None,
):
    all_blocks = {}

    if not subpath_ranges:
        return all_blocks

    # Build the set of all path indices covered by any range
    covered = set()
    for s, e in subpath_ranges:
        covered.update(range(s, e + 1))

    # Merge all catenary pole data that falls within any covered index
    merged_poles = []
    for info in catenary_intersection_data.values():
        pole_indices = info["pole_indices"]
        t1_ints = info["t1_intersections"]
        t2_ints = info["t2_intersections"]
        for i, idx in enumerate(pole_indices):
            if idx in covered:
                merged_poles.append((idx, t1_ints[i], t2_ints[i]))

    # Sort and deduplicate by path index
    merged_poles.sort(key=lambda x: x[0])
    seen = set()
    unique_poles = []
    for idx, t1, t2 in merged_poles:
        if idx not in seen:
            seen.add(idx)
            unique_poles.append((idx, t1, t2))

    pole_indices = [p[0] for p in unique_poles]
    pole_t1 = [p[1] for p in unique_poles]
    pole_t2 = [p[2] for p in unique_poles]

    for wire_cfg in wires:
        wire_cross_section = wire_cfg["wire_cross_section"]
        use_step_line = wire_cfg.get("use_step_line", False)
        use_lines = wire_cfg.get("use_lines", True)

        base_points = []

        if use_lines and len(pole_indices) >= 2:
            t1_pts = _filter_valid_points(pole_t1)
            t2_pts = _filter_valid_points(pole_t2)

            if len(t1_pts) >= 2:
                base_points.extend(_draw_lines(t1_pts, elevation_lut, use_step_line))
            if len(t2_pts) >= 2:
                base_points.extend(_draw_lines(t2_pts, elevation_lut, use_step_line))
            excess_before = sorted(i for i in covered if i < pole_indices[0])
            if excess_before:
                base_points.extend(
                    _build_excess_segment(
                        path,
                        tangents,
                        track1,
                        track2,
                        track_width,
                        excess_before,
                        pole_t1[0],
                        pole_t2[0],
                        elevation_lut,
                        use_step_line,
                        append_pole=True,
                    )
                )

            # --- Excess after last pole: covered indices after last pole ---
            excess_after = sorted(i for i in covered if i > pole_indices[-1])
            if excess_after:
                base_points.extend(
                    _build_excess_segment(
                        path,
                        tangents,
                        track1,
                        track2,
                        track_width,
                        excess_after,
                        pole_t1[-1],
                        pole_t2[-1],
                        elevation_lut,
                        use_step_line,
                        append_pole=False,
                    )
                )

        else:
            all_indices = sorted(covered)
            base_points.extend(
                _over_track_points(
                    path,
                    tangents,
                    track1,
                    track2,
                    track_width,
                    all_indices,
                    elevation_lut,
                    use_step_line,
                )
            )

        if not base_points:
            continue

        blocks = _stamp_cross_section(base_points, wire_cross_section)

        if resolve_blocks and blocks:
            blocks_list = {b: list(pts) for b, pts in blocks.items()}
            blocks_list = resolve_block_connections(blocks_list)
            blocks = {b: set(pts) for b, pts in blocks_list.items()}

        # Remove wire blocks that overlap with protected catenary coordinates
        if protected_coords:
            for block in list(blocks.keys()):
                blocks[block] -= protected_coords
                if not blocks[block]:
                    del blocks[block]

        for block, pts in blocks.items():
            all_blocks.setdefault(block, set()).update(pts)

    return all_blocks