import math
from util.CoreUtil.shapeUtil import bresenham_filled_circle, step_line, bresenham_line
from util.CoreUtil.blockUtil import rotate_block_state, mirror_block_state
from curveAssembly import _norm


def _stamp(xi, zi, section, elev_lut, coord_map, catenary):
    intersection_y = elev_lut.get((xi, zi), 0)
    for block, offsets in section.items():
        for rx, ry, rz in offsets:
            wx, wz = _norm(xi + rx), _norm(zi + rz)
            wy = intersection_y + ry
            coord = (wx, wy, wz)
            if coord in coord_map and coord_map[coord] != block:
                catenary[coord_map[coord]].discard(coord)
            coord_map[coord] = block
            catenary.setdefault(block, set()).add(coord)


def _precompute_catenary_sections(catenary_sections):
    sections = {}
    for i in range(16):
        angle = i * 22.5
        within_quadrant = angle % 90
        quadrant_steps = int(angle // 90)  # number of 90° rotations

        if within_quadrant <= 45:
            base_angle = within_quadrant
            flipped = False
        else:
            base_angle = 90 - within_quadrant
            flipped = True

        base_section = catenary_sections[base_angle]
        rot_rad = math.radians(quadrant_steps * 90)
        cos_a = round(math.cos(rot_rad))
        sin_a = round(math.sin(rot_rad))

        rotated = {}
        for block, offsets in base_section.items():
            if flipped:
                r_block = rotate_block_state(
                    mirror_block_state(block), quadrant_steps + 1
                )
            else:
                r_block = rotate_block_state(block, quadrant_steps)

            for ox, oy, oz in offsets:
                if flipped:
                    # Mirror across 45° diagonal: swap X and Z, negate to face outward
                    fox, foz = -oz, -ox
                else:
                    fox, foz = ox, oz
                rx = _norm(cos_a * fox - sin_a * foz)
                rz = _norm(sin_a * fox + cos_a * foz)
                rotated.setdefault(r_block, []).append((rx, oy, rz))

        sections[angle] = rotated

    return sections


# Catenary_lut is a subset of cross_section_lut that only contains cross sections with catenary poles
def assemble(
    path: list,
    tangents: list,
    elev_lut: dict,
    track1: set,
    track2: set,
    track_width: int,
    catenary_lut: dict,
    catenaries: dict,
    catenary_interval: int,
    offset=0,
    covered_indices=None,
):
    sections = {}
    for cs_name in catenaries.keys():
        catenary_sections = catenaries[cs_name]
        sections[cs_name] = _precompute_catenary_sections(catenary_sections)

    catenary_indices = determine_catenary_indices(
        path, catenary_interval, offset, covered_indices
    )

    t1_intersections, t2_intersections = determine_intersections(
        path, tangents, track1, track2, track_width, catenary_indices
    )

    # filtered_indices = []
    # filtered_t1 = []
    # filtered_t2 = []

    # for i, idx in enumerate(catenary_indices):
    #     t1 = t1_intersections[i]
    #     t2 = t2_intersections[i]
    #     if t1 is None and t2 is None:
    #         continue
    #     filtered_indices.append(idx)
    #     filtered_t1.append(t1)
    #     filtered_t2.append(t2)

    # catenary_indices = filtered_indices
    # t1_intersections = filtered_t1
    # t2_intersections = filtered_t2

    all_blocks = {}
    origins = []

    for i, idx in enumerate(catenary_indices):
        x, z = path[idx]
        cs_name = catenary_lut.get((x, z))

        if cs_name is None:
            continue

        angle = float(tangents[idx])
        coord_map = {}
        stamp_blocks = {}

        path_pt = path[idx]

        track_points = [t1_intersections[i], t2_intersections[i]]

        for t_pt in track_points:
            if not t_pt:
                continue

            side = determine_track_side(path_pt, t_pt, angle)

            key = round(math.degrees(angle) / 22.5) * 22.5 % 360

            # Flip to the opposite section for the right side
            if side == "right":
                key = (key + 180) % 360

            point = (_norm(t_pt[0]), _norm(t_pt[1]))

            _stamp(
                point[0],
                point[1],
                sections[cs_name][key],
                elev_lut,
                coord_map,
                stamp_blocks,
            )

        for block, pts in stamp_blocks.items():
            all_blocks.setdefault(block, set()).update(pts)

        # Keep the path origin for reference tracking
        cp0_y = elev_lut.get((_norm(path_pt[0]), _norm(path_pt[1])), 0)
        origins.append((_norm(path_pt[0]), cp0_y, _norm(path_pt[1])))

    result = {block: list(pts) for block, pts in all_blocks.items() if pts}
    path_origin = origins[0] if origins else (0, 0, 0)

    intersection_info = {
        "t1_intersections": t1_intersections,
        "t2_intersections": t2_intersections,
        "pole_indices": catenary_indices,
    }

    return result, path_origin, intersection_info


def determine_catenary_indices(path, catenary_interval, offset=0, covered_indices=None):
    if not path or offset >= len(path):
        return []
    catenary_indices = [offset]
    bound = determine_bound(path, 2)

    reference_x, reference_y = path[offset]

    for i in range(offset, len(path)):
        x, y = path[i]

        dist = ((x - reference_x) ** 2 + (y - reference_y) ** 2) ** 0.5

        if line_leaves_bound(step_line(x, y, reference_x, reference_y), bound):
            determine_valid_midpoint(path, catenary_indices, bound, i)
        elif dist < catenary_interval:
            continue
        else:
            catenary_indices.append(i)

        last = catenary_indices[-1]
        reference_x, reference_y = path[last]

    # if covered_indices is not None:
    #     covered_set = set(covered_indices)
    #     catenary_indices = [i for i in catenary_indices if i in covered_set]

    return catenary_indices


def determine_valid_midpoint(
    path, catenary_indices, bound, current_idx, max_iterations=4
):
    last_idx = catenary_indices[-1]
    span = current_idx - last_idx
    if span <= 0:
        catenary_indices.append(current_idx)
        return

    n_poles = min(max_iterations, span)
    candidates = []
    for k in range(1, n_poles + 1):
        idx = last_idx + round(k * span / (n_poles + 1))
        idx = max(last_idx + 1, min(idx, current_idx - 1))
        if not candidates or idx > candidates[-1]:
            candidates.append(idx)

    added = 0
    prev_x, prev_y = path[last_idx]
    for idx in candidates:
        mid_x, mid_y = path[idx]
        if line_leaves_bound(bresenham_line(mid_x, mid_y, prev_x, prev_y), bound):
            break
        catenary_indices.append(idx)
        added += 1
        prev_x, prev_y = mid_x, mid_y

    if added == 0:
        catenary_indices.append(current_idx)


def determine_intersections(
    path, tangents, track1, track2, track_width, catenary_indices
):
    t1_intersections = []
    t2_intersections = []

    radius = track_width + 2

    for idx in catenary_indices:
        angle = float(tangents[idx])

        dx = math.cos(angle + math.pi / 2)
        dz = math.sin(angle + math.pi / 2)

        x, z = path[idx]

        pt1_x, pt1_z = _norm(x - radius * dx), _norm(z - radius * dz)
        pt2_x, pt2_z = _norm(x + radius * dx), _norm(z + radius * dz)

        cross_line = step_line(pt1_x, pt1_z, pt2_x, pt2_z)

        t1i, t2i = _track_intersections(cross_line, track1, track2)

        if t1i is None or t2i is None:
            wider = radius + track_width
            w1x, w1z = _norm(x - wider * dx), _norm(z - wider * dz)
            w2x, w2z = _norm(x + wider * dx), _norm(z + wider * dz)
            wider_line = step_line(w1x, w1z, w2x, w2z)
            wt1i, wt2i = _track_intersections(wider_line, track1, track2)
            if t1i is None:
                t1i = wt1i
            if t2i is None:
                t2i = wt2i

        t1_intersections.append(t1i)
        t2_intersections.append(t2i)

    return t1_intersections, t2_intersections


def _track_intersections(cross_line, tc1, tc2):
    tc1_int_points = [pt for pt in cross_line if pt in tc1]
    tc2_int_points = [pt for pt in cross_line if pt in tc2]

    return (
        determine_average_point(tc1_int_points),
        determine_average_point(tc2_int_points),
    )


def determine_track_side(path_pt, track_pt, angle):
    if not track_pt:
        return None

    vx = math.cos(angle)
    vz = math.sin(angle)

    ux = track_pt[0] - path_pt[0]
    uz = track_pt[1] - path_pt[1]

    cross_product = vx * uz - vz * ux

    return "right" if cross_product > 0 else "left"


def determine_average_point(points):
    if not points:
        return None
    if len(points) == 1:
        return points[0]

    x_sum = sum(pt[0] for pt in points)
    y_sum = sum(pt[1] for pt in points)

    return (round(x_sum / len(points)), round(y_sum / len(points)))


def determine_bound(path, radius):
    bound = set()
    for pt in path:
        bound.update(bresenham_filled_circle(int(pt[0]), int(pt[1]), radius))
    return bound


def line_leaves_bound(line, bound):
    for pt in line:
        if pt not in bound:
            return True
    return False