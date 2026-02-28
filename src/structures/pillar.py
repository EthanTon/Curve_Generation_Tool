import math

from util.CoreUtil.curveUtil import draw_path_silhouette
from util.CoreUtil.shapeUtil import bresenham_line
from util.CoreUtil.crossSectionUtil import precompute_sections
from curveAssembly import _norm


def _pillar_path(point, angle, thickness, centered=False):
    """Generate a short perpendicular line for a single pillar."""
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


def _extend_pillar_path(p_path, angle, dept):
    """Extend pillar path at both ends for cross-section coverage."""
    ext = 2 * dept
    pt1 = (
        _norm(p_path[0][0] + ext * math.cos(-angle)),
        _norm(p_path[0][1] + ext * math.sin(-angle)),
    )
    pt2 = (
        _norm(p_path[-1][0] + ext * math.cos(angle)),
        _norm(p_path[-1][1] + ext * math.sin(angle)),
    )
    pre = bresenham_line(pt1[0], pt1[1], p_path[0][0], p_path[0][1])[:-1]
    post = bresenham_line(p_path[-1][0], p_path[-1][1], pt2[0], pt2[1])[1:]
    return pre + p_path + post, len(pre), len(pre) + len(p_path)


def _pillar_silhouette(p_path, angle, width):
    return draw_path_silhouette(
        p_path,
        (p_path[0][0], p_path[0][-1], angle),
        (p_path[-1][0], p_path[-1][-1], angle),
        width,
    )


def _stamp(xc, zc, section, silhouette, elev_lut, coord_map, pillar):
    """Place cross-section at (xc, zc) for a pillar."""
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


def _walk_pillar_path(p_path, angle, sections, silhouette, elev_lut, coord_map, pillar):
    angle_90 = int(round(math.degrees(angle) / 90.0) * 90) % 360
    key = (angle_90, False)
    for xc, zc in p_path:
        _stamp(xc, zc, sections[key], silhouette, elev_lut, coord_map, pillar)


def _pick_pillar_points(path, pillar_distance):
    """Select path indices at regular arc-length intervals."""
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


def assemble(
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
    """Build pillars along a path. Returns (blocks_dict, path_origin).

    Designed to be called from advanceCurveAssembly with the shared
    path, tangents, and elevation data already computed by the curve pass.
    """
    sections = precompute_sections(cross_section)
    pillar_indices = _pick_pillar_points(path, pillar_distance)
    all_blocks = {}
    origins = []

    for idx in pillar_indices:
        angle = float(tangents[idx])
        point = (_norm(path[idx][0]), _norm(path[idx][1]))

        p_path = _pillar_path(point, angle, pillar_thickness, centered)
        full_path, _, _ = _extend_pillar_path(p_path, angle, dept)
        silhouette = _pillar_silhouette(p_path, angle, cross_section_width)

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
