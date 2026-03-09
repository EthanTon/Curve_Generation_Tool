import math

from util.CoreUtil.blockUtil import is_rail_block
from util.CoreUtil.curveUtil import draw_path, draw_path_silhouette
from util.CoreUtil.elevationUtil import generate_elevation_lookup
from util.CoreUtil.pathUtil import optimal_tangent_window, center_path_tangents
from util.CoreUtil.crossSectionUtil import flip_cross_section, precompute_sections
from util.CoreUtil.maskingUtil import mask

from curveAssembly import (
    _norm,
    _extend_path,
    _calculate_backtrack_dist,
)

from structures.assembleStructure import assemble_structures


def _get_index_from_point(path, point):
    pt = (int(point[0]), int(point[1]))
    try:
        return path.index(pt)
    except ValueError:
        best_idx = 0
        best_dist = float("inf")
        px, pz = pt
        for i, (x, z) in enumerate(path):
            d = (x - px) ** 2 + (z - pz) ** 2
            if d < best_dist:
                best_dist = d
                best_idx = i
        return best_idx


def _determine_index_pairs(path, cross_section_points):
    index_pairs = []
    for pt1, pt2 in cross_section_points:
        i1 = _get_index_from_point(path, pt1)
        i2 = _get_index_from_point(path, pt2)
        if i1 > i2:
            i1, i2 = i2, i1
        index_pairs.append((i1, i2))
    return index_pairs


def _cross_section_silhouettes(
    path, silhouette, cross_sections, cross_sections_index_pairs
):

    cs_silhouettes = {}
    for name, cs_def in cross_sections.items():
        cs_width = cs_def["width"]
        cs_mask = set()
        for s_idx, e_idx in cross_sections_index_pairs[name]:
            cs_mask = cs_mask.union(mask(path, s_idx, e_idx, cs_width, silhouette))
        cs_silhouettes[name] = cs_mask
    return cs_silhouettes


def _build_segment_lut(cross_sections_index_pairs, path_len):

    lut = [set() for _ in range(path_len)]
    for name, pairs in cross_sections_index_pairs.items():
        for s, e in pairs:
            for i in range(s, min(e + 1, path_len)):
                lut[i].add(name)
    return lut


def _propagate_cross_section_bounds(
    cross_sections_index_pairs, cross_sections, path_len, radius
):
    adjusted = {}
    for name, pairs in cross_sections_index_pairs.items():
        cs_def = cross_sections[name]
        offset = max(1, cs_def["width"] // 2 - cs_def["dept"] + 1)
        new_pairs = []
        for s, e in pairs:
            new_s = max(0, s - offset)
            new_e = min(path_len - 1, e + offset)
            if new_s <= new_e:
                new_pairs.append((new_s, new_e))
        adjusted[name] = new_pairs
    return adjusted


def _prepare_halves(cross_section, symmetrical, resolve_rails):
    rc_a = set() if resolve_rails else None
    if symmetrical:
        cs_mirror = flip_cross_section(cross_section)
        shared = {}
        rc_b = set() if resolve_rails else None
        return [
            (precompute_sections(cross_section), shared, rc_a),
            (precompute_sections(cs_mirror), shared, rc_b),
        ]
    coord_map = {}
    return [(precompute_sections(cross_section), coord_map, rc_a)]


def _stamp(xc, zc, section, silhouette, elev_lut, coord_map, curve, rail_coords=None):
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


def _stamp_halves(xc, zc, halves, key, silhouette, elev_lut, curve_halves):
    for i, (sections, coord_map, rc) in enumerate(halves):
        _stamp(
            xc, zc, sections[key], silhouette, elev_lut, coord_map, curve_halves[i], rc
        )


def _walk_path(
    path,
    tangents,
    raw_start,
    raw_end,
    all_halves,
    cross_sections,
    cross_section_silhouettes,
    cross_section_lut,
    elev_lut,
    curve_halves,
    radius,
):
    prev_sector = None
    flipped = False
    angle_90 = 0

    idx = 0
    while idx < len(path):
        xc, zc = path[idx]
        deg = math.degrees(float(tangents[idx])) % 360
        sector = int(deg // 45) % 8
        in_ext = idx < raw_start or idx >= raw_end

        # Skip indices that have no cross-section coverage
        if not cross_section_lut[idx]:
            if not in_ext:
                prev_sector = sector
            idx += 1
            continue

        # Update shared orientation state based on the overall loop
        if prev_sector is None:
            angle_90 = int(round(deg / 90.0) * 90) % 360
        elif not in_ext and sector != prev_sector:
            delta = (sector - prev_sector + 4) % 8 - 4
            direction = 1 if delta > 0 else -1
            abs_delta = abs(delta)
            old_key = (angle_90, flipped)

            # Walk through each intermediate boundary sequentially
            had_odd = False
            current = prev_sector
            for _ in range(abs_delta):
                next_s = (current + direction) % 8
                boundary = next_s if direction > 0 else current

                if boundary % 2 == 1:
                    # ODD BOUNDARY (45, 135, 225, 315)
                    flipped = not flipped
                    angle_90 = (angle_90 + 90 * direction) % 360
                    had_odd = True
                else:
                    # EVEN BOUNDARY (90, 180, 270, 0)
                    flipped = not flipped

                current = next_s

            new_key = (angle_90, flipped)

            # Stamp and backtrack for each active cross-section
            for cs_name in cross_section_lut[idx]:
                cs_sil = cross_section_silhouettes.get(cs_name)
                if cs_sil is None:
                    continue

                backtrack_steps = _calculate_backtrack_dist(
                    cross_sections[cs_name]["width"],
                    radius,
                    cross_sections[cs_name]["dept"],
                )
                _stamp_halves(
                    xc,
                    zc,
                    all_halves[cs_name],
                    old_key,
                    cs_sil,
                    elev_lut,
                    curve_halves[cs_name],
                )

                bt = backtrack_steps if had_odd else cross_sections[cs_name]["dept"]
                back_start = max(raw_start, idx - bt)
                for b_idx in range(back_start, idx):
                    bx, bz = path[b_idx]
                    _stamp_halves(
                        bx,
                        bz,
                        all_halves[cs_name],
                        new_key,
                        cs_sil,
                        elev_lut,
                        curve_halves[cs_name],
                    )

        if not in_ext:
            prev_sector = sector

        # Stamp current position for each active cross-section
        key = (angle_90, flipped)
        for cs_name in cross_section_lut[idx]:
            cs_sil = cross_section_silhouettes.get(cs_name)
            if cs_sil is None:
                continue
            _stamp_halves(
                xc,
                zc,
                all_halves[cs_name],
                key,
                cs_sil,
                elev_lut,
                curve_halves[cs_name],
            )

        idx += 1


def _merge_curve_halves(curve_halves):
    merged = {}
    for cs_name, halves in curve_halves.items():
        for half_curve in halves:
            for block, pts in half_curve.items():
                merged.setdefault(block, set()).update(pts)
    return merged


def _cs_needs_symmetry(structures, cs_name):
    for s in structures:
        if cs_name not in s.get("cross_sections", []):
            continue
        if s.get("type") in ("catenary", "wire"):
            return True
    return False


def assemble_advance_curve(
    control_points,
    radius,
    cross_sections,
    structures=None,
    elevation_control_points=[],
    step_size=1,
    symmetrical=False,
    resolve_rails=False,
    min_y_lut=None,
):
    if structures is None:
        structures = []

    start_angle = control_points[0][-1]
    end_angle = control_points[-1][-1]

    max_dept = max(cs_def["dept"] for cs_def in cross_sections.values())
    max_width = max(cs_def["width"] for cs_def in cross_sections.values())

    raw_path, _, _ = draw_path(control_points, radius)
    raw_path = [(_norm(pt[0]), _norm(pt[1])) for pt in raw_path]
    path, raw_start, raw_end = _extend_path(raw_path, start_angle, end_angle, max_dept)

    silhouette = draw_path_silhouette(
        raw_path, control_points[0], control_points[-1], max_width
    )

    tangents = center_path_tangents(path, optimal_tangent_window(radius))
    elev_lut = generate_elevation_lookup(
        path, max_width, silhouette, step_size, elevation_control_points
    )

    cross_sections_index_pairs = {}
    for cs_name, cs_def in cross_sections.items():
        cross_sections_index_pairs[cs_name] = _determine_index_pairs(
            path, cs_def["point_pairs"]
        )

    cross_section_silhouettes = _cross_section_silhouettes(
        path, silhouette, cross_sections, cross_sections_index_pairs
    )

    propagated = _propagate_cross_section_bounds(
        cross_sections_index_pairs, cross_sections, len(path), radius
    )

    cross_section_lut = _build_segment_lut(propagated, len(path))

    all_halves = {}
    for cs_name, cs_def in cross_sections.items():
        cs_symmetrical = symmetrical or _cs_needs_symmetry(structures, cs_name)
        all_halves[cs_name] = _prepare_halves(
            cs_def["cross_section"], cs_symmetrical, resolve_rails
        )

    curve_halves = {}
    for cs_name in cross_sections:
        num_halves = len(all_halves[cs_name])
        curve_halves[cs_name] = [{} for _ in range(num_halves)]

    _walk_path(
        path,
        tangents,
        raw_start,
        raw_end,
        all_halves,
        cross_sections,
        cross_section_silhouettes,
        cross_section_lut,
        elev_lut,
        curve_halves,
        radius,
    )

    curve = _merge_curve_halves(curve_halves)

    # Remap index pairs from extended-path indices to raw_path indices
    raw_index_pairs = {}
    for cs_name, pairs in cross_sections_index_pairs.items():
        remapped = []
        for s, e in pairs:
            s2 = max(0, s - raw_start)
            e2 = min(len(raw_path) - 1, e - raw_start)
            if s2 <= e2:
                remapped.append((s2, e2))
        raw_index_pairs[cs_name] = remapped

    # Slice tangents to match raw_path
    raw_tangents = tangents[raw_start:raw_end]

    assemble_structures(
        path=raw_path,
        tangents=raw_tangents,
        elev_lut=elev_lut,
        structures=structures,
        cross_sections=cross_sections,
        cross_sections_index_pairs=raw_index_pairs,
        cross_section_silhouettes=cross_section_silhouettes,
        curve_halves=curve_halves,
        curve=curve,
        min_y_lut=min_y_lut,
    )

    result = {block: list(pts) for block, pts in curve.items() if pts}

    # Apply minimum y level filter to the entire result
    if min_y_lut and result:
        for block in list(result.keys()):
            filtered = [
                pos
                for pos in result[block]
                if min_y_lut.get((pos[0], pos[2])) is None
                or pos[1] >= min_y_lut[(pos[0], pos[2])]
            ]
            if filtered:
                result[block] = filtered
            else:
                del result[block]

    if resolve_rails:
        from util.CoreUtil.blockUtil import resolve_rail_shapes

        all_rail_coords = set()
        for cs_halves in all_halves.values():
            for _, _, rc in cs_halves:
                if rc:
                    all_rail_coords.update(rc)
        if all_rail_coords:
            result = resolve_rail_shapes(result, [all_rail_coords])

    cp0_xz = raw_path[0]
    cp0_y = elev_lut.get(cp0_xz, 0)
    path_origin = (cp0_xz[0], cp0_y, cp0_xz[1])

    return result, path_origin
