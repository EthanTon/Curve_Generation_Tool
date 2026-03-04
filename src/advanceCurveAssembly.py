import math

from util.CoreUtil.blockUtil import is_rail_block
from util.CoreUtil.curveUtil import draw_path, draw_path_silhouette
from util.CoreUtil.elevationUtil import generate_elevation_lookup
from util.CoreUtil.pathUtil import optimal_tangent_window, center_path_tangents
from util.CoreUtil.crossSectionUtil import flip_cross_section, precompute_sections
from util.CoreUtil.maskingUtil import mask
from util.CoreUtil.shapeUtil import bresenham_line

from curveAssembly import (
    _norm,
    _extend_path,
    _calculate_backtrack_dist,
)

from structures.pillar import assemble as assemble_pillars
from structures.catenary import assemble as assemble_catenary
from structures.wire import assemble as assemble_wire


def _get_index_from_point(path, point):
    return path.index(point)


def _determine_index_pairs(path, cross_section_points):
    index_pairs = []
    for pt1, pt2 in cross_section_points:
        index_pairs.append(
            (_get_index_from_point(path, pt1), _get_index_from_point(path, pt2))
        )
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


def _structures_for_cs(structures, cs_name, struct_type):
    for s in structures:
        if s.get("type") == struct_type and cs_name in s.get("cross_sections", []):
            yield s


def _cs_needs_symmetry(structures, cs_name):
    for s in structures:
        if cs_name not in s.get("cross_sections", []):
            continue
        if s.get("type") in ("catenary", "wire"):
            return True
    return False


def _generate_pillars(
    path,
    tangents,
    elev_lut,
    structures,
    cross_sections,
    cross_sections_index_pairs,
    cross_section_silhouettes,
    curve,
):
    pillar_coords = {}

    for cs_name in cross_sections:
        index_ranges = cross_sections_index_pairs[cs_name]
        cs_mask = cross_section_silhouettes.get(cs_name, set())

        for struct in _structures_for_cs(structures, cs_name, "pillar"):
            for s, e in index_ranges:
                e = min(e, len(path) - 1)
                if s > e:
                    continue
                sub_path = path[s : e + 1]
                sub_tangents = tangents[s : e + 1]

                result, _ = assemble_pillars(
                    path=sub_path,
                    tangents=sub_tangents,
                    elev_lut=elev_lut,
                    pillar_sections=struct["pillar_sections"],
                    pillar_distance=struct["distance"],
                    mask=cs_mask,
                )

                for block, pts in result.items():
                    for coord in pts:
                        pillar_coords[coord] = block

    # Purge pillar coordinates from existing curve blocks
    for coord in pillar_coords:
        for existing_pts in curve.values():
            existing_pts.discard(coord)

    # Merge pillar blocks into curve
    for coord, block in pillar_coords.items():
        curve.setdefault(block, set()).add(coord)



def _generate_catenaries(
    path,
    structures,
    cross_sections,
    cross_sections_index_pairs,
    cross_section_silhouettes,
    elev_lut,
    curve_halves,
    curve,
    min_y_lut=None,
):
    catenary_positions = set()
    catenary_intersection_data = {}

    # ----------------------------------------------------------------
    # 1. Collect all catenary structures with their index ranges so
    #    we can detect adjacency and order them along the path.
    # ----------------------------------------------------------------
    catenary_entries = []  # (cs_name, struct, global_start, global_end)
    for cs_name in cross_sections:
        halves = curve_halves.get(cs_name, [])
        if len(halves) < 2:
            continue
        for struct in _structures_for_cs(structures, cs_name, "catenary"):
            for s, e in cross_sections_index_pairs[cs_name]:
                e = min(e, len(path) - 1)
                if s <= e:
                    catenary_entries.append((cs_name, struct, s, e))

    # Sort by start index so we process them along the path
    catenary_entries.sort(key=lambda x: x[2])

    # ----------------------------------------------------------------
    # 2. Build adjacency: entry[i] is "followed by" entry[j] if
    #    entry[i].end == entry[j].start.
    # ----------------------------------------------------------------
    has_successor = set()   # indices into catenary_entries that have a successor
    has_predecessor = set()  # indices that have a predecessor
    for i in range(len(catenary_entries)):
        for j in range(len(catenary_entries)):
            if i == j:
                continue
            if catenary_entries[i][3] == catenary_entries[j][2]:
                has_successor.add(i)
                has_predecessor.add(j)

    # ----------------------------------------------------------------
    # 3. Generate catenaries in order, propagating offset across
    #    adjacent sections.
    # ----------------------------------------------------------------
    # Track the offset that should carry forward from one section
    # to the next.  Keyed by the global boundary index.
    boundary_offsets = {}  # boundary_index -> offset for next section

    for entry_idx, (cs_name, struct, s, e) in enumerate(catenary_entries):
        halves = curve_halves.get(cs_name, [])
        cs_mask = cross_section_silhouettes.get(cs_name, set())

        catenary_cross_section = struct.get("catenary_cross_section", [])
        if not catenary_cross_section:
            continue

        # Collect only rail XZ footprints from each half for this segment
        track1 = set()
        track2 = set()
        for block, pts in halves[0].items():
            if is_rail_block(block):
                for coord in pts:
                    track1.add((coord[0], coord[2]))
        for block, pts in halves[1].items():
            if is_rail_block(block):
                for coord in pts:
                    track2.add((coord[0], coord[2]))

        # Determine offset: if a predecessor ended at our start index,
        # use the propagated offset.  Otherwise use the struct's own.
        if entry_idx in has_predecessor and s in boundary_offsets:
            effective_offset = boundary_offsets[s]
        else:
            effective_offset = struct.get("offset", 0)

        # If this section is followed by an adjacent catenary, skip the
        # trailing pole (the next section will start near that boundary).
        skip_trailing = entry_idx in has_successor

        sub_path = path[s : e + 1]

        result, intersection_info = assemble_catenary(
            path=sub_path,
            path_silhouette=cs_mask,
            elevation_lut=elev_lut,
            base_width=struct["base_width"],
            track1=track1,
            track2=track2,
            track_width=struct["track_width"],
            catenary_cross_section=catenary_cross_section,
            catenary_interval=struct["catenary_interval"],
            offset=effective_offset,
            min_y_lut=min_y_lut,
            skip_trailing_pole=skip_trailing,
        )

        # Merge catenary blocks into the combined curve, OVERRIDING existing blocks
        for block, pts in result.items():
            for existing_block in list(curve.keys()):
                if existing_block != block:
                    curve[existing_block].difference_update(pts)
                    if not curve[existing_block]:
                        del curve[existing_block]
            curve.setdefault(block, set()).update(pts)
            catenary_positions.update(pts)

        # Accumulate intersection data, offsetting local indices to global
        cs_data = catenary_intersection_data.setdefault(
            cs_name,
            {
                "t1_intersections": [],
                "t2_intersections": [],
                "pole_indices": [],
            },
        )
        cs_data["t1_intersections"].extend(
            intersection_info["t1_intersections"]
        )
        cs_data["t2_intersections"].extend(
            intersection_info["t2_intersections"]
        )
        cs_data["pole_indices"].extend(
            s + pi for pi in intersection_info["pole_indices"]
        )

        # Propagate offset to the next adjacent section: the offset is
        # the distance from the last generated pole to the end of this
        # sub-path, so the next section continues the spacing.
        local_poles = intersection_info["pole_indices"]
        if local_poles:
            last_local = max(local_poles)
            remaining = (e - s) - last_local
            boundary_offsets[e] = remaining

    return catenary_positions, catenary_intersection_data


def _generate_wires(
    path,
    structures,
    cross_sections,
    cross_sections_index_pairs,
    cross_section_silhouettes,
    elev_lut,
    curve_halves,
    catenary_positions,
    catenary_intersection_data,
    curve,
    min_y_lut=None,
):
    # ------------------------------------------------------------------
    # Build a lookup: for each cross-section, does a catenary exist?
    # ------------------------------------------------------------------
    cs_has_catenary = set(catenary_intersection_data.keys())

    for struct in structures:
        if struct.get("type") != "wire":
            continue

        wire_cross_section = struct.get("wire_cross_section")
        if not wire_cross_section:
            continue

        active_cs_names = struct.get("cross_sections", [])
        if not active_cs_names:
            continue

        t1_int_global = []
        t2_int_global = []
        pole_idx_global = []
        track_rail_coords_global = set()
        cs_sil_global = set()
        default_mask_ranges = []

        for cs_name in active_cs_names:
            if cs_name not in cross_sections:
                continue

            cs_int = catenary_intersection_data.get(cs_name)
            if cs_int:
                t1_int_global.extend(cs_int["t1_intersections"])
                t2_int_global.extend(cs_int["t2_intersections"])
                pole_idx_global.extend(cs_int["pole_indices"])

            halves = curve_halves.get(cs_name, [])
            for half in halves:
                for block, pts in half.items():
                    if is_rail_block(block):
                        track_rail_coords_global.update(pts)

            cs_sil_global.update(cross_section_silhouettes.get(cs_name, set()))
            default_mask_ranges.extend(cross_sections_index_pairs.get(cs_name, []))

        if pole_idx_global:
            unique_poles = {}
            for t1, t2, pi in zip(t1_int_global, t2_int_global, pole_idx_global):
                if pi not in unique_poles:
                    unique_poles[pi] = (t1, t2)

            pole_idx_global = sorted(unique_poles.keys())
            t1_int_global = [unique_poles[pi][0] for pi in pole_idx_global]
            t2_int_global = [unique_poles[pi][1] for pi in pole_idx_global]

        # --------------------------------------------------------------
        # Determine which end of the wire's span has a catenary and
        # which doesn't, so we can extend / trim correctly.
        # --------------------------------------------------------------
        wire_point_pairs = struct.get("point_pairs")
        wire_ranges = (
            _determine_index_pairs(path, wire_point_pairs) if wire_point_pairs else None
        )
        if wire_ranges is None:
            wire_ranges = default_mask_ranges

        # Overall span covered by this wire
        if wire_ranges:
            wire_start_idx = min(s for s, _ in wire_ranges)
            wire_end_idx = max(e for _, e in wire_ranges)
        else:
            wire_start_idx = 0
            wire_end_idx = len(path) - 1

        # Check if the first / last cross-section in this wire has catenary
        has_cat_at_start = active_cs_names[0] in cs_has_catenary if active_cs_names else False
        has_cat_at_end = active_cs_names[-1] in cs_has_catenary if active_cs_names else False

        # For adjacent catenary sections sharing a boundary, deduplicate
        # poles that appear at the same index (both sections generated one)
        if pole_idx_global:
            deduped = {}
            for pi, t1, t2 in zip(pole_idx_global, t1_int_global, t2_int_global):
                if pi not in deduped:
                    deduped[pi] = (t1, t2)
            pole_idx_global = sorted(deduped.keys())
            t1_int_global = [deduped[pi][0] for pi in pole_idx_global]
            t2_int_global = [deduped[pi][1] for pi in pole_idx_global]

        # --------------------------------------------------------------
        # Wire extension logic — the wire draws between ALL poles.
        # We never remove poles from the list.
        #
        # The only extension needed is at the end: if the last cross
        # section has no catenary, the wire extends from the last pole
        # to the path end.  The end-anchor uses the last intersection
        # so the wire stays on the track, not the center line.
        # --------------------------------------------------------------
        t1_path_end = None
        t2_path_end = None
        path_end_index = None

        if pole_idx_global and len(t1_int_global) >= 2:
            if not has_cat_at_end:
                t1_path_end = t1_int_global[-1]
                t2_path_end = t2_int_global[-1]
                path_end_index = wire_end_idx

        raw_mask_pairs = struct.get("mask_point_pairs")
        mask_ranges = None
        if raw_mask_pairs:
            mask_ranges = _determine_index_pairs(path, raw_mask_pairs)
        else:
            mask_ranges = default_mask_ranges

        if (
            (not pole_idx_global or len(t1_int_global) < 2)
            and mask_ranges
            and track_rail_coords_global
        ):
            wire_silhouette = set()
            max_width = max(
                [
                    cross_sections[cs]["width"]
                    for cs in active_cs_names
                    if cs in cross_sections
                ]
                + [0]
            )
            for ms, me in mask_ranges:
                wire_silhouette |= mask(path, ms, me, max_width, cs_sil_global)
            track_rail_coords_global = {
                c for c in track_rail_coords_global if (c[0], c[2]) in wire_silhouette
            }


        if not pole_idx_global and not track_rail_coords_global:
            continue

        result = assemble_wire(
            wire_cross_section=wire_cross_section,
            elevation_lut=elev_lut,
            t1_intersections=t1_int_global if t1_int_global else None,
            t2_intersections=t2_int_global if t2_int_global else None,
            pole_indices=pole_idx_global if pole_idx_global else None,
            mask_ranges=mask_ranges,
            track_rail_coords=(
                track_rail_coords_global
                if (not pole_idx_global or len(t1_int_global) < 2)
                else None
            ),
            use_step_line=struct.get("use_step_line", True),
            resolve_blocks=struct.get("resolve_blocks", True),
            override_catenary=struct.get("override_catenary", False),
            catenary_positions=catenary_positions,
            t1_path_end=t1_path_end,
            t2_path_end=t2_path_end,
            path_end_index=path_end_index,
            min_y_lut=min_y_lut,
        )

        for block, pts in result.items():
            curve.setdefault(block, set()).update(pts)


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

    catenary_positions, catenary_intersection_data = _generate_catenaries(
        path,
        structures,
        cross_sections,
        cross_sections_index_pairs,
        cross_section_silhouettes,
        elev_lut,
        curve_halves,
        curve,
        min_y_lut=min_y_lut,
    )

    _generate_wires(
        path,
        structures,
        cross_sections,
        cross_sections_index_pairs,
        cross_section_silhouettes,
        elev_lut,
        curve_halves,
        catenary_positions,
        catenary_intersection_data,
        curve,
        min_y_lut=min_y_lut,
    )

    _generate_pillars(
        path,
        tangents,
        elev_lut,
        structures,
        cross_sections,
        cross_sections_index_pairs,
        cross_section_silhouettes,
        curve,
    )

    result = {block: list(pts) for block, pts in curve.items() if pts}

    # Apply minimum y level filter to the entire result
    if min_y_lut and result:
        for block in list(result.keys()):
            filtered = [
                pos for pos in result[block]
                if min_y_lut.get((pos[0], pos[2])) is None
                or pos[1] >= min_y_lut[(pos[0], pos[2])]
            ]
            if filtered:
                result[block] = filtered
            else:
                del result[block]

    if resolve_rails:
        from util.CoreUtil.blockUtil import resolve_rail_shapes

        rail_groups = [
            rc for cs_halves in all_halves.values() for _, _, rc in cs_halves if rc
        ]
        if rail_groups:
            result = resolve_rail_shapes(result, rail_groups)

    cp0_xz = raw_path[0]
    cp0_y = elev_lut.get(cp0_xz, 0)
    path_origin = (cp0_xz[0], cp0_y, cp0_xz[1])

    return result, path_origin