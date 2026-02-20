import math
import numpy as np

from util.CoreUtil.blockUtil import rotate_block_state
from util.CoreUtil.curveUtil import draw_path, draw_path_silhouette
from util.CoreUtil.elevationUtil import generate_elevation_lookup
from util.CoreUtil.pathUtil import optimal_tangent_window, center_path_tangents
from util.CoreUtil.shapeUtil import bresenham_line


def _norm_coord(v):
    return int(round(float(v)))


def _snap_angle_90(angle_rad):
    """Snap an angle to the nearest 90-degree increment (0, 90, 180, 270).
    Transition happens at 45-degree boundaries."""
    angle_rad = float(angle_rad)
    steps = round(math.degrees(angle_rad) % 360 / 90) % 4
    return math.radians(steps * 90)


def _angle_to_steps(angle_rad):
    return round(math.degrees(float(angle_rad)) % 360 / 90) % 4


def _rotate_position(x, y, z, angle_rad):
    angle_rad = float(angle_rad)
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    return (cos_a * x - sin_a * z, y, sin_a * x + cos_a * z)


def _snap(x, y, z):
    return (_norm_coord(x), _norm_coord(y), _norm_coord(z))


def _extend_path(path, start_angle, end_angle, width, step_size):
    extension_dist = math.ceil(width / 2) * step_size

    sx, sz = path[0]
    ext_sx = _norm_coord(sx - extension_dist * math.cos(start_angle))
    ext_sz = _norm_coord(sz - extension_dist * math.sin(start_angle))
    start_line = bresenham_line(ext_sx, sx, ext_sz, sz)

    ex, ez = path[-1]
    ext_ex = _norm_coord(ex + extension_dist * math.cos(end_angle))
    ext_ez = _norm_coord(ez + extension_dist * math.sin(end_angle))
    end_line = bresenham_line(ex, ext_ex, ez, ext_ez)

    seen = set()
    extended = []

    for pt in start_line:
        if pt not in seen:
            seen.add(pt)
            extended.append(pt)

    for pt in path:
        if pt not in seen:
            seen.add(pt)
            extended.append(pt)

    for pt in end_line:
        if pt not in seen:
            seen.add(pt)
            extended.append(pt)

    return extended


def rotate_cross_section_point(block_string, offset, tangent_angle):
    snapped_angle = _snap_angle_90(tangent_angle)
    rotated_pos = _rotate_position(*offset, snapped_angle)
    snapped = _snap(*rotated_pos)
    rotation_steps = _angle_to_steps(snapped_angle)
    rotated_block = rotate_block_state(block_string, rotation_steps)
    return rotated_block, snapped


def _place_cross_section(
    path,
    idx,
    tangent_angle,
    cross_section,
    path_silhouette,
    elevation_lut,
    coord_map,
    curve,
    symmetrical,
):
    xc, zc = path[idx]
    snapped_angle = _snap_angle_90(tangent_angle)

    # Safely get the center elevation as a fallback in case edge pixels are missing from the LUT
    center_y = elevation_lut.get((xc, zc), 0)

    for block, offsets in cross_section.items():
        for pt in offsets:
            ox, oy, oz = pt[0], pt[1], pt[2]
            rotated_block, (rx, ry, rz) = rotate_cross_section_point(
                block, (ox, oy, oz), snapped_angle
            )
            wx = _norm_coord(xc + rx)
            wz = _norm_coord(zc + rz)
            if (wx, wz) not in path_silhouette:
                continue

            # Use .get() with the center path elevation as the fallback
            base_y = elevation_lut.get((wx, wz), center_y)
            wy = base_y + ry
            coord = (wx, wy, wz)

            if coord in coord_map and coord_map[coord] != rotated_block:
                curve[coord_map[coord]].discard(coord)
            coord_map[coord] = rotated_block
            curve.setdefault(rotated_block, set()).add(coord)

            if symmetrical and ox != 0:
                mirrored_block, (mrx, mry, mrz) = rotate_cross_section_point(
                    block, (-ox, oy, oz), snapped_angle
                )
                mwx = _norm_coord(xc + mrx)
                mwz = _norm_coord(zc + mrz)
                if (mwx, mwz) not in path_silhouette:
                    continue

                # Use .get() with the center path elevation as the fallback here as well
                mbase_y = elevation_lut.get((mwx, mwz), center_y)
                mwy = mbase_y + mry
                mcoord = (mwx, mwy, mwz)

                if mcoord in coord_map and coord_map[mcoord] != mirrored_block:
                    curve[coord_map[mcoord]].discard(mcoord)
                coord_map[mcoord] = mirrored_block
                curve.setdefault(mirrored_block, set()).add(mcoord)


def assemble_curve_path(
    control_points, radius, elevation_control_points=[], step_size=1
):
    path, _, _ = draw_path(control_points, radius)
    norm_path = [(_norm_coord(pt[0]), _norm_coord(pt[1])) for pt in path]
    path_set = set(norm_path)
    elevation_lut = generate_elevation_lookup(
        norm_path, 1, path_set, step_size, elevation_control_points
    )
    block_dict = {}
    for x, z in norm_path:
        y = elevation_lut.get((x, z), 0)
        block_dict.setdefault("minecraft:stone", []).append((x, y, z))
    return block_dict


def assemble_curve(
    control_points,
    radius,
    cross_section,
    cross_section_width,
    elevation_control_points=[],
    step_size=1,
    symmetrical=False,
):
    start_angle = control_points[0][-1]
    end_angle = control_points[-1][-1]

    raw_path, _, _ = draw_path(control_points, radius)
    raw_path = [(_norm_coord(pt[0]), _norm_coord(pt[1])) for pt in raw_path]
    path = _extend_path(
        raw_path, start_angle, end_angle, cross_section_width, step_size
    )

    path_silhouette = draw_path_silhouette(
        path, control_points[0], control_points[-1], cross_section_width
    )
    optimal_window = optimal_tangent_window(radius)
    raw_tangents = np.asarray(center_path_tangents(path, optimal_window), dtype=float)

    raw_tangents = raw_tangents.reshape(-1, 2)
    path_tangents = (
        np.arctan2(raw_tangents[:, 1], raw_tangents[:, 0]).flatten().tolist()
    )

    elevation_lut = generate_elevation_lookup(
        path, cross_section_width, path_silhouette, step_size, elevation_control_points
    )

    curve = {}
    coord_map = {}

    for idx in range(len(path)):
        _place_cross_section(
            path,
            idx,
            path_tangents[idx],
            cross_section,
            path_silhouette,
            elevation_lut,
            coord_map,
            curve,
            symmetrical,
        )

    return {block: list(pts) for block, pts in curve.items() if pts}
