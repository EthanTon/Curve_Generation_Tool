from util.CoreUtil.shapeUtil import step_line, bresenham_line
from util.CoreUtil.blockUtil import resolve_block_connections


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


def assemble(
    wire_cross_section,
    elevation_lut,
    t1_intersections=None,
    t2_intersections=None,
    pole_indices=None,
    mask_ranges=None,  # Kept in signature to prevent breaking external calls
    track_rail_coords=None,
    use_step_line=True,
    resolve_blocks=True,
    override_catenary=False,
    catenary_positions=None,
    t1_path_start=None,
    t2_path_start=None,
    path_start_index=None,
    t1_path_end=None,
    t2_path_end=None,
    path_end_index=None,
    min_y_lut=None,
):
    has_poles = bool(t1_intersections and t2_intersections and pole_indices)
    can_prepend = bool(t1_path_start and t2_path_start)
    can_extend = bool(t1_path_end and t2_path_end)

    base_points = []

    if has_poles and len(t1_intersections) >= 2:
        t1_pts = list(t1_intersections)
        t2_pts = list(t2_intersections)
        p_indices = list(pole_indices)

        if can_prepend:
            t1_pts.insert(0, t1_path_start)
            t2_pts.insert(0, t2_path_start)
            idx = path_start_index if path_start_index is not None else max(0, p_indices[0] - 1)
            p_indices.insert(0, idx)

        if can_extend:
            t1_pts.append(t1_path_end)
            t2_pts.append(t2_path_end)
            idx = path_end_index if path_end_index is not None else (p_indices[-1] + 1)
            p_indices.append(idx)

        # Draw lines continuously across all valid poles, skipping the mask filter
        base_points.extend(_draw_lines(t1_pts, elevation_lut, use_step_line))
        base_points.extend(_draw_lines(t2_pts, elevation_lut, use_step_line))

    elif track_rail_coords:
        # Fallback to drawing over the line/tracks if no intersections exist
        base_points.extend(list(track_rail_coords))

    if not base_points:
        return {}

    blocks = _stamp_cross_section(base_points, wire_cross_section)

    if resolve_blocks and blocks:
        blocks_list = {b: list(pts) for b, pts in blocks.items()}
        blocks_list = resolve_block_connections(blocks_list)
        blocks = {b: set(pts) for b, pts in blocks_list.items()}

    if not override_catenary and catenary_positions:
        for block in list(blocks.keys()):
            blocks[block] -= catenary_positions
            if not blocks[block]:
                del blocks[block]

    # Apply minimum y level filter
    if min_y_lut and blocks:
        for block in list(blocks.keys()):
            filtered = set()
            for pos in blocks[block]:
                x, y, z = pos
                min_y = min_y_lut.get((x, z))
                if min_y is not None and y < min_y:
                    continue
                filtered.add(pos)
            if filtered:
                blocks[block] = filtered
            else:
                del blocks[block]

    return blocks