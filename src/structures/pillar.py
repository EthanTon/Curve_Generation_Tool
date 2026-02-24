import math
from ..util.CoreUtil.curveUtil import draw_path_silhouette
from ..util.CoreUtil.shapeUtil import bresenham_line
from ..util.CoreUtil.crossSectionUtil import precompute_sections


def _norm(v):
    return int(round(float(v)))


def _extend_pillar_path(pillar_path, angle, dept):
    ext = 2 * dept
    pt1 = (
        _norm(pillar_path[0][0] + ext * math.cos(-angle)),
        _norm(pillar_path[0][1] + ext * math.sin(-angle)),
    )
    pt2 = (
        _norm(pillar_path[-1][0] + ext * math.cos(angle)),
        _norm(pillar_path[-1][1] + ext * math.sin(angle)),
    )
    ext1 = bresenham_line(pt1[0], pt1[1], pillar_path[0][0], pillar_path[0][1])
    ext2 = bresenham_line(pillar_path[-1][0], pillar_path[-1][1], pt2[0], pt2[1])
    pre = ext1[:-1]
    post = ext2[1:]
    return pre + pillar_path + post, len(pre), len(pre) + len(pillar_path)


def pillar_silhouette(pillar_path, angle, width):
    return draw_path_silhouette(
        pillar_path,
        (pillar_path[0][0], pillar_path[0][-1], angle),
        (pillar_path[-1][0], pillar_path[-1][-1], angle),
        width,
    )


def pillar_path(point, angle, thickness, centered=False):
    if thickness == 1:
        return [point]
    if centered:
        pt1 = (
            _norm(point[0] + thickness * math.cos(-angle)),
            _norm(point[1] + thickness * math.sin(-angle)),
        )
    else:
        pt1 = point
    pt2 = (
        _norm(point[0] + thickness * math.cos(angle)),
        _norm(point[1] + thickness * math.sin(angle)),
    )
    return bresenham_line(pt1[0], pt1[1], pt2[0], pt2[1])


def _stamp(xc, zc, section, silhouette, elev_lut, coord_map, pillar):
    center_y = elev_lut.get((xc, zc), 0)
    for block, offsets in section.items():
        for rx, ry, rz in offsets:
            wx, wz = _norm(xc + rx), _norm(zc + rz)
            if (wx, wz) not in silhouette:
                continue
            wy = elev_lut.get((wx, wz), center_y) + ry
            coord = (wx, wy, wz)
            if coord in coord_map and coord_map[coord] != block:
                pillar[coord_map[coord]].discard(coord)
            coord_map[coord] = block
            pillar.setdefault(block, set()).add(coord)


def _walk_pillar_path(
    pillar_path, angle, sections, silhouette, elev_lut, coord_map, pillar
):
    angle_90 = int(round(math.degrees(angle) / 90.0) * 90) % 360
    key = (angle_90, False)
    for xc, zc in pillar_path:
        _stamp(xc, zc, sections[key], silhouette, elev_lut, coord_map, pillar)


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


def assemble_pillars(
    path,
    tangents,
    elev_lut,
    cross_section,
    cross_section_width,
    dept,
    pillar_thickness,
    pillar_distance,
    centered=False,
):
    sections = precompute_sections(cross_section)
    pillar_indices = _pick_pillar_points(path, pillar_distance)
    all_blocks = {}
    origins = []

    for idx in pillar_indices:
        angle = float(tangents[idx])
        point = (_norm(path[idx][0]), _norm(path[idx][1]))

        p_path = pillar_path(point, angle, pillar_thickness, centered)
        full_path, raw_start, raw_end = _extend_pillar_path(p_path, angle, dept)
        silhouette = pillar_silhouette(p_path, angle, cross_section_width)

        coord_map = {}
        pillar = {}
        _walk_pillar_path(
            full_path, angle, sections, silhouette, elev_lut, coord_map, pillar
        )

        for block, pts in pillar.items():
            all_blocks.setdefault(block, set()).update(pts)

        cp0_y = elev_lut.get(point, 0)
        origins.append((point[0], cp0_y, point[1]))

    result = {block: list(pts) for block, pts in all_blocks.items() if pts}
    path_origin = origins[0] if origins else (0, 0, 0)
    return result, path_origin
