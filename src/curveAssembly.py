from util.CoreUtil.curveUtil import draw_path, draw_path_silhouette
from util.CoreUtil.elevationUtil import generate_elevation_lookup


def assemble_curve_path(
    control_points, radius, elevation_control_points=[], step_size=1
):
    path = draw_path(control_points, radius)
    path_set = {(_norm_coord(pt[0]), _norm_coord(pt[1])) for pt in path}

    elevation_lut = generate_elevation_lookup(
        path, 1, path_set, step_size, elevation_control_points
    )

    block_dict = {}
    for x, z in path:
        if len(block_dict) == 0:
            block_dict["minecraft:stone"] = []
        else:
            block_dict["minecraft:stone"].append(x, elevation_lut[(x, z)], z)

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
    path = draw_path(control_points, radius)
    path_silhouette = draw_path_silhouette(
        path, control_points[0], control_points[-1], cross_section_width
    )
    elevation_lut = generate_elevation_lookup(
        path, cross_section_width, path_silhouette, step_size, elevation_control_points
    )

    curve = {}

    for xc, zc in path:
        for block in cross_section.keys():
            points = []
            for pt in cross_section[block]:
                if (pt[0], pt[2]) in path_silhouette:
                    offset_elevation = elevation_lut(pt[0], pt[2])
                    points.append(_offset_point(((xc, offset_elevation, zc), pt)))

            curve.setdefault(block, points)

    return curve


def _norm_coord(v):
    """Round any numeric value to the nearest Python int."""
    return int(round(float(v)))


def _offset_point(reference_coord, coord):
    return (
        coord[0] + reference_coord[0],
        coord[1] + reference_coord[1],
        coord[2] + reference_coord[2],
    )
