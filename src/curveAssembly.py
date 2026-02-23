import math

from util.CoreUtil.blockUtil import (
    rotate_block_state,
    is_rail_block,
    mirror_block_state,
)
from util.CoreUtil.curveUtil import draw_path, draw_path_silhouette
from util.CoreUtil.elevationUtil import generate_elevation_lookup
from util.CoreUtil.pathUtil import optimal_tangent_window, center_path_tangents
from util.CoreUtil.shapeUtil import bresenham_line


def _norm(v):
    return int(round(float(v)))


def _flip_cross_section(cross_section):
    """Mirror cross-section across the path centre (the paste origin).

    Because cross-section offsets are already expressed relative to the paste
    origin (0, 0, 0), flipping simply negates the X and Z components while
    also mirroring any directional block states.
    """
    flipped = {}
    for block, offsets in cross_section.items():
        m_block = mirror_block_state(block,"xz")
        new_offsets = [(-ox, oy, -oz) for ox, oy, oz in offsets]
        flipped.setdefault(m_block, []).extend(new_offsets)
    return flipped


def _extend_path(path, start_angle, end_angle, dept):
    """Extend path at both ends for cross-section coverage. Returns (full_path, raw_start, raw_end)."""
    ext = 2 * dept
    sx, sz = path[0]
    a0 = start_angle + math.pi
    start_line = bresenham_line(
        _norm(sx + ext * math.cos(start_angle)), _norm(sz + ext * math.sin(start_angle)), sx, sz
    )
    ex, ez = path[-1]
    end_line = bresenham_line(
        ex, ez, _norm(ex + ext * math.cos(end_angle)), _norm(ez + ext * math.sin(end_angle))
    )
    pre = start_line[:-1]
    post = end_line[1:]
    
    return pre + path + post, len(pre), len(pre) + len(path)


def _precompute_sections(cross_section):
    """Precompute rotated cross-sections for 4 angles × 2 flip states."""
    sections = {}
    for angle_deg in (0, 90, 180, 270):
        angle_rad = math.radians(angle_deg)
        steps = angle_deg // 90
        cos_a, sin_a = math.cos(angle_rad), math.sin(angle_rad)
        for flipped in (False, True):
            rotated = {}
            for block, offsets in cross_section.items():
                if flipped:
                    r_block = rotate_block_state(mirror_block_state(block), steps)
                else:
                    r_block = rotate_block_state(block, steps)
                for ox, oy, oz in offsets:
                    fox = -ox if flipped else ox
                    rx = _norm(cos_a * fox - sin_a * oz)
                    rz = _norm(sin_a * fox + cos_a * oz)
                    rotated.setdefault(r_block, []).append((rx, oy, rz))
            sections[(angle_deg, flipped)] = rotated
    return sections


def _stamp(xc, zc, section, silhouette, elev_lut, coord_map, curve, rail_coords=None):
    """Place cross-section at (xc, zc), resolving conflicts via coord_map."""
    center_y = elev_lut.get((xc, zc), 0)
    for block, offsets in section.items():
        is_rail = rail_coords is not None and is_rail_block(block)
        for rx, ry, rz in offsets:
            wx, wz = _norm(xc + rx), _norm(zc + rz)
            if (wx, wz) not in silhouette:
                continue
            wy = elev_lut.get((wx, wz), center_y) + ry
            coord = (wx, wy, wz)
            if coord in coord_map and coord_map[coord] != block:
                curve[coord_map[coord]].discard(coord)
            coord_map[coord] = block
            curve.setdefault(block, set()).add(coord)
            if is_rail:
                rail_coords.add(coord)


def _stamp_halves(xc, zc, halves, key, silhouette, elev_lut, curve):
    """Stamp all cross-section halves at (xc, zc) with orientation key."""
    for sections, coord_map, rc in halves:
        _stamp(xc, zc, sections[key], silhouette, elev_lut, coord_map, curve, rc)


def _calculate_backtrack_dist(width, radius, dept):
    return max(1, int(math.ceil(width // 2 - dept + 2)))


def _walk_path(
    path,
    tangents,
    raw_start,
    raw_end,
    symmetrical,
    halves,
    silhouette,
    elev_lut,
    curve,
    width,
    radius,
    dept,
):
    """Walk the path, handling sector transitions with dynamic backtracking."""
    prev_sector = None
    flipped = False
    angle_90 = 0

    # Calculate how many indices to move backwards on a flip
    backtrack_steps = _calculate_backtrack_dist(width, radius, dept)

    idx = 0
    while idx < len(path):
        xc, zc = path[idx]
        deg = math.degrees(float(tangents[idx])) % 360
        sector = int(deg // 45) % 8
        in_ext = idx < raw_start or idx >= raw_end

        if prev_sector is None:
            angle_90 = int(round(deg / 90.0) * 90) % 360
        elif not in_ext and sector != prev_sector:
            delta = (sector - prev_sector + 4) % 8 - 4
            boundary = sector if delta > 0 else prev_sector
            old_key = (angle_90, flipped)

            # ODD BOUNDARY (45, 135, 225, 315)
            if boundary % 2 == 1:
                flipped = not flipped
                angle_90 = (angle_90 + (90 if delta > 0 else -90)) % 360
                new_key = (angle_90, flipped)

                _stamp_halves(xc, zc, halves, old_key, silhouette, elev_lut, curve)

                back_start = max(raw_start, idx - backtrack_steps)
                for b_idx in range(back_start, idx):
                    b_xc, b_zc = path[b_idx]
                    _stamp_halves(
                        b_xc, b_zc, halves, new_key, silhouette, elev_lut, curve
                    )

            else:
                # EVEN BOUNDARY (90, 180, 270, 0)
                flipped = not flipped
                new_key = (angle_90, flipped)
                _stamp_halves(xc, zc, halves, old_key, silhouette, elev_lut, curve)

                back_start = max(raw_start, idx - dept)
                for b_idx in range(back_start, idx):
                    b_xc, b_zc = path[b_idx]
                    _stamp_halves(
                        b_xc, b_zc, halves, new_key, silhouette, elev_lut, curve
                    )

        if not in_ext:
            prev_sector = sector

        # Stamp the current position and increment safely
        _stamp_halves(xc, zc, halves, (angle_90, flipped), silhouette, elev_lut, curve)
        idx += 1


def rotate_cross_section_point(block_string, offset, tangent_angle):
    raw = math.degrees(float(tangent_angle)) % 360
    angle_deg = int(round(raw / 90.0) * 90) % 360
    angle_rad = math.radians(angle_deg)
    cos_a, sin_a = math.cos(angle_rad), math.sin(angle_rad)
    ox, oy, oz = offset
    pos = (_norm(cos_a * ox - sin_a * oz), oy, _norm(sin_a * ox + cos_a * oz))
    return rotate_block_state(block_string, angle_deg // 90), pos


def assemble_curve_path(
    control_points, radius, elevation_control_points=[], step_size=1
):
    """Build a single-block-wide path and return *(blocks_dict, path_origin)*.

    *path_origin* is the (x, y, z) of the first control point on the curve
    and is used at export time to set the schematic offset.
    """
    path, _, _ = draw_path(control_points, radius)
    norm_path = [(_norm(pt[0]), _norm(pt[1])) for pt in path]
    elev_lut = generate_elevation_lookup(
        norm_path, 1, set(norm_path), step_size, elevation_control_points
    )
    blocks = {}
    for x, z in norm_path:
        blocks.setdefault("minecraft:stone", []).append((x, elev_lut.get((x, z), 0), z))

    # Path origin = first control point position (used as export origin)
    cp0_xz = norm_path[0]
    cp0_y = elev_lut.get(cp0_xz, 0)
    path_origin = (cp0_xz[0], cp0_y, cp0_xz[1])

    return blocks, path_origin


def assemble_curve(
    control_points,
    radius,
    cross_section,
    cross_section_width,
    dept,
    elevation_control_points=[],
    step_size=1,
    symmetrical=False,
    resolve_rails=False,
    side="left",
):
    start_angle = control_points[0][-1]
    end_angle = control_points[-1][-1]

    raw_path, _, _ = draw_path(control_points, radius)
    raw_path = [(_norm(pt[0]), _norm(pt[1])) for pt in raw_path]
    path, raw_start, raw_end = _extend_path(
        raw_path, start_angle, end_angle, dept
    )

    silhouette = draw_path_silhouette(
        raw_path, control_points[0], control_points[-1], cross_section_width
    )
    tangents = center_path_tangents(path, optimal_tangent_window(radius))
    elev_lut = generate_elevation_lookup(
        path, cross_section_width, silhouette, step_size, elevation_control_points
    )

    curve = {}

    if symmetrical:
        # Mirror around the paste origin (0, 0, 0) – no external origin needed.
        cs_mirror = _flip_cross_section(cross_section)
        shared_coord_map = {}

        rc_a = set() if resolve_rails else None
        rc_b = set() if resolve_rails else None
        halves = [
            (_precompute_sections(cross_section), shared_coord_map, rc_a),
            (_precompute_sections(cs_mirror), shared_coord_map, rc_b),
        ]
    else:
        # Single half with its own coord_map
        coord_map = {}
        rc = set() if resolve_rails else None
        halves = [(_precompute_sections(cross_section), coord_map, rc)]

    _walk_path(
        path,
        tangents,
        raw_start,
        raw_end,
        symmetrical,
        halves,
        silhouette,
        elev_lut,
        curve,
        cross_section_width,
        radius,
        dept,
    )

    result = {block: list(pts) for block, pts in curve.items() if pts}

    if resolve_rails:
        from util.CoreUtil.blockUtil import resolve_rail_shapes

        rail_groups = [rc for _, _, rc in halves if rc]
        if rail_groups:
            result = resolve_rail_shapes(result, rail_groups)

    # Path origin = first control point position (used as export origin)
    cp0_xz = raw_path[0]
    cp0_y = elev_lut.get(cp0_xz, 0)
    path_origin = (cp0_xz[0], cp0_y, cp0_xz[1])

    return result, path_origin
