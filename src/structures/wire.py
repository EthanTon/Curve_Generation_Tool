from util.CoreUtil.shapeUtil import step_line, bresenham_line
from util.CoreUtil.blockUtil import resolve_block_connections
from util.CoreUtil.maskingUtil import mask
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


def _draw_line_segments(points, elevation_lut, use_step_line):
    """Draw lines between consecutive non-None points, splitting at None gaps."""
    result = []
    segment = []
    for pt in points:
        if pt is not None:
            segment.append(pt)
        else:
            if len(segment) >= 2:
                result.extend(_draw_lines(segment, elevation_lut, use_step_line))
            segment = []
    if len(segment) >= 2:
        result.extend(_draw_lines(segment, elevation_lut, use_step_line))
    return result


def _filter_valid_points(intersections):
    """Return only non-None intersection points."""
    return [pt for pt in intersections if pt is not None]


def _over_track_points(
    path, track1, track2, track_width, indices, silhouette, elevation_lut
):
    if not indices:
        return []

    coverage_map = set()
    masked_segment = mask(path, indices[0], indices[-1], track_width, silhouette)
    coverage_map.update(masked_segment)

    base_points = []
    for pt in track1:
        if pt in coverage_map:                
            y = elevation_lut[pt]
            base_points.append((pt[0], y, pt[1]))
    for pt in track2:
        if pt in coverage_map:                
            y = elevation_lut[pt]
            base_points.append((pt[0], y, pt[1]))
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


def _merge_contiguous_ranges(ranges):
    """Merge overlapping or adjacent (s,e) ranges into contiguous blocks."""
    if not ranges:
        return []
    sorted_ranges = sorted(ranges)
    merged = [list(sorted_ranges[0])]
    for s, e in sorted_ranges[1:]:
        if s <= merged[-1][1] + 1:
            merged[-1][1] = max(merged[-1][1], e)
        else:
            merged.append([s, e])
    return [(s, e) for s, e in merged]


def assemble(
    path,
    tangents,
    elevation_lut,
    track1,
    track2,
    track_width,
    cross_section_silhouettes,
    wires,
    catenary_intersection_data,
    subpath_ranges,
    resolve_blocks=True,
    protected_coords=None,
):
    all_blocks = {}

    if not subpath_ranges:
        return all_blocks

    # Merge overlapping/adjacent ranges into contiguous blocks
    contiguous_blocks = _merge_contiguous_ranges(subpath_ranges)

    # Build the full covered set (for the non-line fallback path)
    covered = set()
    for s, e in contiguous_blocks:
        covered.update(range(s, e + 1))

    silhouette = set()
    #Rebuild silhouette
    for points in cross_section_silhouettes.values():
        silhouette.update(points)

    # Collect all catenary pole data that falls within any covered index,
    # sorted and deduplicated by path index
    merged_poles = []
    for info in catenary_intersection_data.values():
        pole_indices = info["pole_indices"]
        t1_ints = info["t1_intersections"]
        t2_ints = info["t2_intersections"]
        for i, idx in enumerate(pole_indices):
            if idx in covered:
                merged_poles.append((idx, t1_ints[i], t2_ints[i]))

    merged_poles.sort(key=lambda x: x[0])
    seen = set()
    unique_poles = []
    for idx, t1, t2 in merged_poles:
        if idx not in seen:
            seen.add(idx)
            unique_poles.append((idx, t1, t2))

    for wire_cfg in wires:
        wire_cross_section = wire_cfg["wire_cross_section"]
        use_step_line = wire_cfg.get("use_step_line", False)
        use_lines = wire_cfg.get("use_lines", True)

        base_points = []

        if use_lines:
            # Process each contiguous block independently so wires never
            # span across gaps between non-consecutive cross-sections.
            for blk_s, blk_e in contiguous_blocks:
                blk_covered = set(range(blk_s, blk_e + 1))

                # Poles belonging to this contiguous block
                blk_poles = [
                    (idx, t1, t2) for idx, t1, t2 in unique_poles if idx in blk_covered
                ]

                if len(blk_poles) < 2:
                    # Not enough poles in this block – fall back to over-track
                    blk_indices = sorted(blk_covered)
                    base_points.extend(
                        _over_track_points(
                            path,
                            track1,
                            track2,
                            track_width,
                            blk_indices,
                            silhouette,
                            elevation_lut,
                        )
                    )
                    continue

                blk_pole_indices = [p[0] for p in blk_poles]
                blk_t1 = [p[1] for p in blk_poles]
                blk_t2 = [p[2] for p in blk_poles]

                base_points.extend(
                    _draw_line_segments(blk_t1, elevation_lut, use_step_line)
                )
                base_points.extend(
                    _draw_line_segments(blk_t2, elevation_lut, use_step_line)
                )

                # Excess before first pole in this block
                excess_before = sorted(
                    i for i in blk_covered if i < blk_pole_indices[0]
                )
                if excess_before:
                    base_points.extend(
                        _build_excess_segment(
                            path,
                            tangents,
                            track1,
                            track2,
                            track_width,
                            excess_before,
                            blk_t1[0],
                            blk_t2[0],
                            elevation_lut,
                            use_step_line,
                            append_pole=True,
                        )
                    )

                # Excess after last pole in this block
                excess_after = sorted(
                    i for i in blk_covered if i > blk_pole_indices[-1]
                )
                if excess_after:
                    base_points.extend(
                        _build_excess_segment(
                            path,
                            tangents,
                            track1,
                            track2,
                            track_width,
                            excess_after,
                            blk_t1[-1],
                            blk_t2[-1],
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
                    track1,
                    track2,
                    track_width,
                    all_indices,
                    silhouette,
                    elevation_lut,
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
