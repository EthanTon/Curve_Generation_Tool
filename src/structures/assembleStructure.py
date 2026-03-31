from util.CoreUtil.blockUtil import is_rail_block

from structures.pillar import assemble as assemble_pillars
from structures.catenary import assemble as assemble_catenary, determine_intersections
from structures.wire import assemble as assemble_wire


def _structures_for_cs(structures, cs_name, struct_type):
    for s in structures:
        if s.get("type") == struct_type and cs_name in s.get("cross_sections", []):
            yield s


def _extract_track_halves_for_cs(curve_halves, cs_names):
    track1 = set()
    track2 = set()
    for cs_name in cs_names:
        halves = curve_halves.get(cs_name)
        if halves is None or len(halves) < 2:
            continue
        for block, pts in halves[0].items():
            if is_rail_block(block):
                for x, y, z in pts:
                    track1.add((x, z))
        for block, pts in halves[1].items():
            if is_rail_block(block):
                for x, y, z in pts:
                    track2.add((x, z))
    return track1, track2


def assemble_structures(
    path,
    tangents,
    elev_lut,
    structures,
    cross_sections,
    cross_sections_index_pairs,
    cross_section_silhouettes,
    curve_halves,
    curve,
    surface_lut=None,
):
    if not structures:
        return

    _generate_pillars(
        path,
        tangents,
        elev_lut,
        structures,
        cross_sections,
        cross_sections_index_pairs,
        cross_section_silhouettes,
        curve,
        surface_lut=surface_lut,
    )

    all_cs_names = list(curve_halves.keys())
    global_track1, global_track2 = _extract_track_halves_for_cs(
        curve_halves, all_cs_names
    )

    # --- Catenaries ---
    _, catenary_intersection_data, catenary_protected_coords = _generate_catenaries(
        path,
        tangents,
        structures,
        cross_sections,
        cross_sections_index_pairs,
        elev_lut,
        curve_halves,
        (global_track1, global_track2),
        curve,
    )

    _generate_wires(
        path,
        tangents,
        elev_lut,
        structures,
        cross_sections,
        cross_sections_index_pairs,
        (global_track1, global_track2),
        catenary_intersection_data,
        catenary_protected_coords,
        curve,
    )


def _generate_pillars(
    path,
    tangents,
    elev_lut,
    structures,
    cross_sections,
    cross_sections_index_pairs,
    cross_section_silhouettes,
    curve,
    surface_lut=None,
):
    pillar_coords = {}

    for cs_name in cross_sections:
        index_ranges = cross_sections_index_pairs[cs_name]
        cs_mask = cross_section_silhouettes.get(cs_name, set())

        for struct in _structures_for_cs(structures, cs_name, "pillar"):
            min_y = surface_lut if surface_lut and struct.get("use_y_min") else None
            max_y = surface_lut if surface_lut and struct.get("use_y_max") else None

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
                    min_y_lut=min_y,
                    max_y_lut=max_y,
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
    tangents,
    structures,
    cross_sections,
    cross_sections_index_pairs,
    elev_lut,
    curve_halves,
    global_tracks,
    curve,
):
    catenary_positions = {}
    catenary_protected_coords = set()
    catenary_intersection_data = {}

    global_track1, global_track2 = global_tracks

    for struct in structures:
        if struct.get("type") != "catenary":
            continue

        struct_cs_names = struct.get("cross_sections", [])
        catenary_interval = struct["distance"]
        track_width = struct["track_width"]
        catenaries = struct["catenaries"]
        offset = struct.get("offset", 0)
        override = struct.get("override", True)

        track1, track2 = _extract_track_halves_for_cs(curve_halves, struct_cs_names)

        intersection_track1, intersection_track2 = global_tracks

        all_ranges = []
        catenary_lut = {}
        min_covered_index = len(path)
        for cs_name in struct_cs_names:
            if cs_name not in cross_sections:
                continue
            index_ranges = cross_sections_index_pairs.get(cs_name, [])
            for s, e in index_ranges:
                e = min(e, len(path) - 1)
                if s > e:
                    continue
                all_ranges.append((s, e))
                if s < min_covered_index:
                    min_covered_index = s
                for i in range(s, e + 1):
                    x, z = path[i]
                    catenary_lut[(x, z)] = cs_name

        if not catenary_lut:
            continue

        # Build a set of all valid path indices covered by cross-sections
        all_ranges.sort()
        covered_indices = set()
        for s, e in all_ranges:
            covered_indices.update(range(s, e + 1))

        effective_offset = min_covered_index + offset

        result, path_origin, intersection_info = assemble_catenary(
            path=path,
            tangents=tangents,
            elev_lut=elev_lut,
            track1=track1,
            track2=track2,
            track_width=track_width,
            catenary_lut=catenary_lut,
            catenaries=catenaries,
            catenary_interval=catenary_interval,
            offset=effective_offset,
            covered_indices=covered_indices,
        )

        for block, pts in result.items():
            for coord in pts:
                catenary_positions[coord] = block
                if not override:
                    catenary_protected_coords.add(coord)

        pole_indices = intersection_info["pole_indices"]
        global_t1, global_t2 = determine_intersections(
            path,
            tangents,
            global_track1,
            global_track2,
            track_width,
            pole_indices,
        )
        catenary_intersection_data[id(struct)] = {
            "pole_indices": pole_indices,
            "t1_intersections": global_t1,
            "t2_intersections": global_t2,
        }

    # Purge catenary coordinates from existing curve blocks (skip override=false)
    for coord in catenary_positions:
        if coord in catenary_protected_coords:
            continue
        for existing_pts in curve.values():
            existing_pts.discard(coord)

    # Merge catenary blocks into curve
    for coord, block in catenary_positions.items():
        curve.setdefault(block, set()).add(coord)

    return catenary_positions, catenary_intersection_data, catenary_protected_coords


def _generate_wires(
    path,
    tangents,
    elev_lut,
    structures,
    cross_sections,
    cross_sections_index_pairs,
    global_tracks,
    catenary_intersection_data,
    catenary_protected_coords,
    curve,
):
    wire_positions = {}

    # Wires follow the rails which may span multiple cross-sections.
    track1, track2 = global_tracks

    for struct in structures:
        if struct.get("type") != "wire":
            continue

        struct_cs_names = struct.get("cross_sections", [])
        track_width = struct["track_width"]
        wires = struct["wires"]

        all_ranges = []
        for cs_name in struct_cs_names:
            if cs_name not in cross_sections:
                continue
            for s, e in cross_sections_index_pairs.get(cs_name, []):
                e = min(e, len(path) - 1)
                if s <= e:
                    all_ranges.append((s, e))

        if not all_ranges:
            continue

        all_ranges.sort()

        result = assemble_wire(
            path=path,
            tangents=tangents,
            elevation_lut=elev_lut,
            track1=track1,
            track2=track2,
            track_width=track_width,
            wires=wires,
            catenary_intersection_data=catenary_intersection_data,
            subpath_ranges=all_ranges,
            protected_coords=catenary_protected_coords,
        )

        for block, pts in result.items():
            for coord in pts:
                wire_positions[coord] = block

    # Purge wire coordinates from existing curve blocks
    for coord in wire_positions:
        for existing_pts in curve.values():
            existing_pts.discard(coord)

    # Merge wire blocks into curve
    for coord, block in wire_positions.items():
        curve.setdefault(block, set()).add(coord)
