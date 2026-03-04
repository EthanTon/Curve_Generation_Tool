import math
from util.CoreUtil.blockUtil import rotate_block_state, mirror_block_state
from curveAssembly import _norm


def _stamp(xc, zc, section, silhouette, elev_lut, coord_map, pillar, min_y_lut=None):
    use_min_y = min_y_lut is not None
    center_y = elev_lut.get((xc, zc), 0)
    for block, offsets in section.items():
        for rx, ry, rz in offsets:
            wx, wz = _norm(xc + rx), _norm(zc + rz)
            if (wx, wz) not in silhouette:
                continue
            wy = elev_lut.get((wx, wz), center_y) + ry
            if use_min_y:
                if wy < min_y_lut((wx, wz)):
                    continue
            coord = (wx, wy, wz)
            if coord in coord_map and coord_map[coord] != block:
                pillar[coord_map[coord]].discard(coord)
            coord_map[coord] = block
            pillar.setdefault(block, set()).add(coord)


def _pick_pillar_points(path, pillar_distance):
    if len(path) < 2:
        return [0]
    indices = [0]
    accum = 0.0
    for i in range(1, len(path)):
        dx = path[i][0] - path[i - 1][0]
        dz = path[i][1] - path[i - 1][1]
        accum += math.hypot(dx, dz)
        if accum >= pillar_distance:
            indices.append(i)
            accum = 0.0
    if indices[-1] != len(path) - 1:
        indices.append(len(path) - 1)
    return indices


def _precompute_pillar_sections(pillar_sections):
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

        base_section = pillar_sections[base_angle]
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


def assemble(
    path: list,
    tangents: list,
    elev_lut: dict,
    pillar_sections: dict,
    pillar_distance: int,
    mask: set,
    min_y_lut=None,
):
    sections = _precompute_pillar_sections(pillar_sections)
    pillar_indices = _pick_pillar_points(path, pillar_distance)
    all_blocks = {}
    origins = []

    for idx in pillar_indices:
        angle = float(tangents[idx])
        point = (_norm(path[idx][0]), _norm(path[idx][1]))
        coord_map = {}
        stamp_blocks = {}

        key = round(math.degrees(angle) / 22.5) * 22.5 % 360

        _stamp(
            point[0],
            point[1],
            sections[key],
            mask,
            elev_lut,
            coord_map,
            stamp_blocks,
            min_y_lut,
        )

        for block, pts in stamp_blocks.items():
            all_blocks.setdefault(block, set()).update(pts)

        cp0_y = elev_lut.get(point, 0)
        origins.append((point[0], cp0_y, point[1]))

    result = {block: list(pts) for block, pts in all_blocks.items() if pts}
    path_origin = origins[0] if origins else (0, 0, 0)
    return result, path_origin
