from util.CoreUtil.shapeUtil import step_line


def _elevation_at(x, z, elevation_lut, default=0):
    """Look up the ground elevation for a 2-D point from the flat LUT."""
    return elevation_lut.get((int(round(x)), int(round(z))), default)


def _nearest_elevation(x, z, elevation_lut, default=0):
    """Spiral outward from (x, z) until an elevation LUT entry is found."""
    ix, iz = int(round(x)), int(round(z))
    if (ix, iz) in elevation_lut:
        return elevation_lut[(ix, iz)]
    for r in range(1, 30):
        for dx in range(-r, r + 1):
            for dz in range(-r, r + 1):
                if abs(dx) != r and abs(dz) != r:
                    continue
                neighbour = (ix + dx, iz + dz)
                if neighbour in elevation_lut:
                    return elevation_lut[neighbour]
    return default


def generate_wire(intersections, elevation_lut):
    if not intersections or len(intersections) < 2:
        return []

    wire = []

    for i in range(len(intersections) - 1):
        x0, z0 = intersections[i]
        x1, z1 = intersections[i + 1]

        if x0 is None or z0 is None or x1 is None or z1 is None:
            continue

        # Resolve the y-level at each intersection (pole) from the
        # ground elevation LUT.
        y_start = _nearest_elevation(x0, z0, elevation_lut)
        y_end = _nearest_elevation(x1, z1, elevation_lut)

        segment_2d = step_line(int(x0), int(z0), int(x1), int(z1))
        seg_len = len(segment_2d)

        for j, (x, z) in enumerate(segment_2d):
            # Skip the first pixel of subsequent segments to avoid
            # duplicating the shared junction pixel.
            if i > 0 and j == 0:
                continue

            # Linearly interpolate y between the two intersection
            # elevations based on position within the segment.
            if seg_len > 1:
                t = j / (seg_len - 1)
            else:
                t = 0.0
            y = y_start + t * (y_end - y_start)

            wire.append((x, z, round(y)))

    return wire


def apply_elevation(wire, elevation_lut):
    return [
        (x, z, _elevation_at(x, z, elevation_lut))
        for x, z in wire
    ]