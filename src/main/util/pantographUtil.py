import numpy as np
from util.shapeUtil import (
    bresenham_filled_circle,
    bresenham_line,
    step_line,
)


# ── Public API ────────────────────────────────────────────────────────────────

WIRE_INSET = 3  # wire anchors sit 3 pixels in from each pole on the cross-line

# Threshold for switching between step_line and bresenham_line.
# tan(22.5°) ≈ 0.414 — lines more diagonal than this use bresenham.
_DIAGONAL_THRESHOLD = 0.26794919243
# Near-45° threshold: lines within this ratio of a true diagonal revert to
# step_line to avoid aliasing gaps.
_NEAR_45_THRESHOLD = 0.8  # ~38.7°–51.3°


def apply_pantograph(
    center_track,
    base,
    base_width,
    elevation_map,
    track_outer1=None,
    track_outer2=None,
    track_center1=None,
    track_center2=None,
    pantograph_interval=45,
    start_offset=0,
):
    base_set = set(base)
    base_edge = determine_base_edge(center_track, base_width, base_set)

    track1 = _to_xy_set(track_center1)
    track2 = _to_xy_set(track_center2)
    outer1 = _to_xy_set(track_outer1)
    outer2 = _to_xy_set(track_outer2)

    # ── 1. Generate all poles (including intermediates) ────────────────────
    poles, pole_indices = generate_pantograph(
        center_track,
        base_set,
        base_width,
        base_edge,
        track1,
        track2,
        outer1,
        outer2,
        pantograph_interval=pantograph_interval,
        start_offset=start_offset,
    )

    # ── 2. Build cross-lines (Universal: Inner=Step, Outer=Smart-Bresenham) ─
    cross_lines = []
    for pole_pair, pidx in zip(poles, pole_indices):
        left, right = pole_pair
        cc = _estimate_curvature_center(center_track, pidx)
        
        cross_line = _three_segment_cross_line(
            _ri(left[0]), _ri(left[1]),
            _ri(right[0]), _ri(right[1]),
            track1, track2, cc
        )
        cross_lines.append(cross_line)

    # ── 3. Generate wires from cross-line anchors ───────────────────────
    wire1, wire2, wire_anchors_1, wire_anchors_2 = generate_wires(
        cross_lines, poles=poles, tc1=track1, tc2=track2,
        center_track=center_track, base_width=base_width,
    )
    overhead_wire_pixels = list(set(wire1 + wire2))

    tc1_intersections = [a for a in wire_anchors_1 if a]
    tc2_intersections = [a for a in wire_anchors_2 if a]

    # ── 4. 3-D interpolated elevation ─────────────────────────────────────
    pole_lines = {}

    # Cross-line elevation
    for line in cross_lines:
        if len(line) < 2:
            continue
        y_start = _get_elevation(line[0], elevation_map)
        y_end = _get_elevation(line[-1], elevation_map)
        n = len(line)
        for idx, pt in enumerate(line):
            t = idx / max(n - 1, 1)
            pole_lines[pt] = y_start + (y_end - y_start) * t

    # Wire elevation
    _interpolate_wire_segments(wire_anchors_1, wire1, elevation_map, pole_lines,
                               center_track=center_track, base_width=base_width)
    _interpolate_wire_segments(wire_anchors_2, wire2, elevation_map, pole_lines,
                               center_track=center_track, base_width=base_width)

    return (
        poles,
        pole_lines,
        cross_lines,
        overhead_wire_pixels,
        tc1_intersections,
        tc2_intersections,
        wire1,
        wire2,
    )


# ── Universal Line Logic ──────────────────────────────────────────────────────


def _universal_line(x0, y0, x1, y1, force_step=False):
    """
    Universal line drawer combining 'Curve Aware' and 'Angle Aware' logic.
    
    Args:
        force_step: If True (Inner Curve context), forces step_line for stability.
                    If False (Outer Curve context), uses angle-aware logic 
                    (Bresenham optimized for aesthetics).
    """
    if force_step:
        return step_line(x0, y0, x1, y1)
    
    # Angle Aware Logic for the Outer side
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    max_d = max(dx, dy, 1)
    min_d = min(dx, dy)
    diagonal_ratio = min_d / max_d

    # Use step_line for very flat/steep lines OR lines extremely close to 45°
    if diagonal_ratio <= _DIAGONAL_THRESHOLD:
        return step_line(x0, y0, x1, y1)
    if diagonal_ratio >= _NEAR_45_THRESHOLD:
        return step_line(x0, y0, x1, y1)
    
    # Otherwise, use standard Bresenham
    return bresenham_line(x0, y0, x1, y1)


def _three_segment_cross_line(x0, y0, x1, y1, tc1, tc2, curvature_center):
    """
    Builds a cross-line with 3 distinct segments:
    1. Pole -> Track Center 1
    2. Track Center 1 -> Track Center 2
    3. Track Center 2 -> Pole
    
    Applies Universal logic:
    - Inner Side segments (including the middle connector) use Step (force_step=True).
    - Outer Side segments use Angle Aware (force_step=False).
    """
    # Fallback if track data missing
    if not tc1 or not tc2:
        return _universal_line(x0, y0, x1, y1, force_step=False)

    # 1. Find intersection points (Anchors) via simple scan
    scan = bresenham_line(x0, y0, x1, y1)
    hit1 = None
    hit2 = None
    
    for p in scan:
        if not hit1 and p in tc1: hit1 = p
        if not hit2 and p in tc2: hit2 = p
    
    # Retry reverse scan for grazing hits
    if not hit1 or not hit2:
        for p in reversed(scan):
            if not hit1 and p in tc1: hit1 = p
            if not hit2 and p in tc2: hit2 = p

    if not hit1 or not hit2:
         return _universal_line(x0, y0, x1, y1, force_step=False)

    # 2. Sort hits by distance from Start (x0, y0)
    d1 = (hit1[0]-x0)**2 + (hit1[1]-y0)**2
    d2 = (hit2[0]-x0)**2 + (hit2[1]-y0)**2
    
    if d1 <= d2:
        p_mid1, p_mid2 = hit1, hit2
    else:
        p_mid1, p_mid2 = hit2, hit1

    # 3. Determine Geometry (Inner vs Outer)
    start_is_inner = False
    
    if curvature_center:
        ccx, ccy = curvature_center
        dist_start = (x0 - ccx)**2 + (y0 - ccy)**2
        dist_end = (x1 - ccx)**2 + (y1 - ccy)**2
        if dist_start < dist_end:
            start_is_inner = True
    else:
        # Default for straight track: treat symmetrically or default to outer logic
        # But to prevent mix-and-match jitter, we just treat everything as "Outer" (Angle Aware)
        # unless user specifically wanted step.
        pass

    # 4. Generate Segments using Universal Logic
    if curvature_center:
        if start_is_inner:
            # Layout: [Start/Inner] --(Step)--> [TC_Inner] --(Step)--> [TC_Outer] --(Smart)--> [End/Outer]
            seg1 = _universal_line(x0, y0, p_mid1[0], p_mid1[1], force_step=True)
            seg2 = _universal_line(p_mid1[0], p_mid1[1], p_mid2[0], p_mid2[1], force_step=True)
            seg3 = _universal_line(p_mid2[0], p_mid2[1], x1, y1, force_step=False)
        else:
            # Layout: [Start/Outer] --(Smart)--> [TC_Outer] --(Step)--> [TC_Inner] --(Step)--> [End/Inner]
            seg1 = _universal_line(x0, y0, p_mid1[0], p_mid1[1], force_step=False)
            seg2 = _universal_line(p_mid1[0], p_mid1[1], p_mid2[0], p_mid2[1], force_step=True)
            seg3 = _universal_line(p_mid2[0], p_mid2[1], x1, y1, force_step=True)
    else:
        # Straight track: Use Angle Aware for all to look best
        seg1 = _universal_line(x0, y0, p_mid1[0], p_mid1[1], force_step=False)
        seg2 = _universal_line(p_mid1[0], p_mid1[1], p_mid2[0], p_mid2[1], force_step=False)
        seg3 = _universal_line(p_mid2[0], p_mid2[1], x1, y1, force_step=False)

    # 5. Combine segments
    def join_segs(a, b):
        if not a: return b
        if not b: return a
        if a[-1] == b[0]: return a + b[1:]
        return a + b

    result = join_segs(list(seg1), list(seg2))
    result = join_segs(result, list(seg3))
        
    return result


# ── Curvature Helper ──────────────────────────────────────────────────────────


def _estimate_curvature_center(path, index):
    """Estimate the centre of the osculating circle at *index* on *path*."""
    n = len(path)
    if n < 3:
        return None

    i0 = max(index - 1, 0)
    i2 = min(index + 1, n - 1)
    if i0 == index:
        i2 = min(index + 2, n - 1)
    if i2 == index:
        i0 = max(index - 2, 0)
    if i0 == i2:
        return None

    ax, ay = float(path[i0][0]), float(path[i0][1])
    bx, by = float(path[index][0]), float(path[index][1])
    cx, cy = float(path[i2][0]), float(path[i2][1])

    D = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(D) < 1e-9:
        return None  # collinear

    ux = ((ax * ax + ay * ay) * (by - cy) +
          (bx * bx + by * by) * (cy - ay) +
          (cx * cx + cy * cy) * (ay - by)) / D
    uy = ((ax * ax + ay * ay) * (cx - bx) +
          (bx * bx + by * by) * (ax - cx) +
          (cx * cx + cy * cy) * (bx - ax)) / D
    return (ux, uy)


# ── Pole Generation ──────────────────────────────────────────────────────────


def generate_pantograph(
    path,
    base,
    base_width,
    base_edge,
    track1,
    track2,
    outer1=None,
    outer2=None,
    pantograph_interval=35,
    start_offset=0,
):
    path_arr = np.asarray(path, dtype=float)
    if len(path_arr) < 2:
        return [], []

    tc1 = set(track1)
    tc2 = set(track2)
    to1 = set(outer1) if outer1 else set()
    to2 = set(outer2) if outer2 else set()

    # ── Pass 1: place poles at regular intervals ──────────────────────────
    poles = []
    pole_indices = []

    first_idx = max(0, min(start_offset, len(path_arr) - 1))
    pole = _create_pole_at_index(path, first_idx, base, base_width, base_edge)
    if pole is not None:
        poles.append(pole)
        pole_indices.append(first_idx)

    accumulated = 0.0
    begin = max(first_idx + 1, 1)

    for i in range(begin, len(path_arr)):
        accumulated += np.linalg.norm(path_arr[i] - path_arr[i - 1])
        if accumulated >= pantograph_interval:
            accumulated = 0.0
            pole = _create_pole_at_index(path, i, base, base_width, base_edge)
            if pole is not None:
                poles.append(pole)
                pole_indices.append(i)

    # ── Pass 2: insert intermediate poles where the wire would leave track ─
    if len(poles) >= 2:
        poles, pole_indices = _insert_intermediate_poles(
            path,
            poles,
            pole_indices,
            base,
            base_width,
            base_edge,
            tc1,
            tc2,
            to1,
            to2,
        )

    return poles, pole_indices


def _insert_intermediate_poles(
    path,
    poles,
    pole_indices,
    base,
    base_width,
    base_edge,
    tc1,
    tc2,
    outer1,
    outer2,
    max_iterations=1,
    min_gap=3,
):
    allowed_side1 = tc1 | outer1
    allowed_side2 = tc2 | outer2

    cur_poles = list(poles)
    cur_indices = list(pole_indices)

    for _iteration in range(max_iterations):
        inserted_any = False
        next_poles = [cur_poles[0]]
        next_indices = [cur_indices[0]]

        i = 1
        while i < len(cur_poles):
            prev_pole = next_poles[-1]
            curr_pole = cur_poles[i]
            prev_idx = next_indices[-1]
            curr_idx = cur_indices[i]

            prev_cl = _build_cross_line(prev_pole)
            curr_cl = _build_cross_line(curr_pole)

            prev_a1, prev_a2 = _anchor_by_intersection(prev_cl, tc1, tc2)
            curr_a1, curr_a2 = _anchor_by_intersection(curr_cl, tc1, tc2)

            needs_split = _wire_leaves_track(
                prev_a1, curr_a1, allowed_side1
            ) or _wire_leaves_track(prev_a2, curr_a2, allowed_side2)

            if needs_split and (curr_idx - prev_idx) > min_gap:
                mid_idx = (prev_idx + curr_idx) // 2
                mid_pole = _create_pole_at_index(
                    path,
                    mid_idx,
                    base,
                    base_width,
                    base_edge,
                )
                if mid_pole is not None:
                    next_poles.append(mid_pole)
                    next_indices.append(mid_idx)
                    inserted_any = True

            next_poles.append(curr_pole)
            next_indices.append(curr_idx)
            i += 1

        cur_poles = next_poles
        cur_indices = next_indices

        if not inserted_any:
            break

    return cur_poles, cur_indices


# ── Cross-line helpers ────────────────────────────────────────────────────────


def _build_cross_line(pole, curvature_center=None):
    """Build a basic cross-line (used only for intermediate checks)."""
    # Uses simple universal line without 3-segment logic for speed/simplicity
    # in the pole insertion phase.
    x0, y0 = _ri(pole[0][0]), _ri(pole[0][1])
    x1, y1 = _ri(pole[1][0]), _ri(pole[1][1])
    return _universal_line(x0, y0, x1, y1, force_step=False)


def _cross_line_anchors(cross_line):
    """Return (anchor_left, anchor_right) from a cross-line at WIRE_INSET."""
    n = len(cross_line)
    if n >= (WIRE_INSET + 1) * 2:
        return cross_line[WIRE_INSET], cross_line[-(WIRE_INSET + 1)]
    if n > 0:
        mid = n // 2
        return (
            cross_line[min(WIRE_INSET, mid)],
            cross_line[max(n - WIRE_INSET - 1, mid)],
        )
    return None, None


def _anchor_by_intersection(cross_line, tc1, tc2):
    a_start, a_end = _cross_line_anchors(cross_line)
    if a_start is None:
        return None, None

    # Find the first pixel on the cross-line that hits each track set
    hit_tc1 = None
    hit_tc2 = None
    for pt in cross_line:
        if hit_tc1 is None and pt in tc1:
            hit_tc1 = pt
        if hit_tc2 is None and pt in tc2:
            hit_tc2 = pt
        if hit_tc1 and hit_tc2:
            break

    if hit_tc1 is None or hit_tc2 is None:
        for pt in reversed(cross_line):
            if hit_tc1 is None and pt in tc1:
                hit_tc1 = pt
            if hit_tc2 is None and pt in tc2:
                hit_tc2 = pt
            if hit_tc1 and hit_tc2:
                break

    if hit_tc1 is None and hit_tc2 is None:
        return a_start, a_end

    def _dist2(a, b):
        if a is None or b is None:
            return float("inf")
        return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2

    # Determine which anchor is closer to the tc1 intersection
    d_start_tc1 = _dist2(a_start, hit_tc1)
    d_end_tc1 = _dist2(a_end, hit_tc1)

    if d_start_tc1 <= d_end_tc1:
        return a_start, a_end  # a_start → tc1, a_end → tc2
    else:
        return a_end, a_start  # a_end → tc1, a_start → tc2


def _wire_leaves_track(anchor_prev, anchor_curr, allowed_set):
    if not anchor_prev or not anchor_curr:
        return False
    if not allowed_set:
        return False
    wire = bresenham_line(
        _ri(anchor_prev[0]),
        _ri(anchor_prev[1]),
        _ri(anchor_curr[0]),
        _ri(anchor_curr[1]),
    )
    for pt in wire:
        if pt not in allowed_set:
            return True
    return False


# ── Wire Generation ──────────────────────────────────────────────────────────


def generate_wires(cross_lines, poles=None, tc1=None, tc2=None,
                   center_track=None, base_width=None):
    wire_anchors_1 = []
    wire_anchors_2 = []

    use_intersection = (
        poles is not None
        and tc1 is not None
        and tc2 is not None
        and len(tc1) > 0
        and len(tc2) > 0
    )

    for idx, cl in enumerate(cross_lines):
        if use_intersection:
            a1, a2 = _anchor_by_intersection(cl, tc1, tc2)
        else:
            a1, a2 = _cross_line_anchors(cl)
        if a1 is not None:
            wire_anchors_1.append(a1)
        if a2 is not None:
            wire_anchors_2.append(a2)

    wire1 = _draw_wire_segments(wire_anchors_1, center_track=center_track,
                                base_width=base_width)
    wire2 = _draw_wire_segments(wire_anchors_2, center_track=center_track,
                                base_width=base_width)

    return wire1, wire2, wire_anchors_1, wire_anchors_2


# ── Orthogonal normal via weighted PCA ────────────────────────────────────────


def weighted_pca_orthogonal(path, index, sigma=2):
    pts = np.array(path, dtype=float)
    indices = np.arange(len(path))
    weights = np.exp(-0.5 * ((indices - index) / sigma) ** 2)
    weighted_center = np.average(pts, axis=0, weights=weights)
    centered = pts - weighted_center
    W = np.diag(weights)
    cov = (centered.T @ W @ centered) / weights.sum()
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    tangent = eigenvectors[:, np.argmax(eigenvalues)]
    normal = (-tangent[1], tangent[0])
    return normal


def determine_base_edge(path, width, base):
    filter_set = set()
    for x, y in path:
        filter_set.update(bresenham_filled_circle(x, y, width // 2 - 1))
    return set(base - filter_set)


# ── Internal helpers ──────────────────────────────────────────────────────────


def _ri(v):
    return int(round(float(v)))


def _to_xy_set(collection):
    if collection is None:
        return set()
    return {(_ri(p[0]), _ri(p[1])) for p in collection}


def _create_pole_at_index(path, index, base_set, base_width, base_edge):
    cx, cz = path[index]

    scalar = base_width // 2
    normal_vector = weighted_pca_orthogonal(path, index, 2.5)

    left = (_ri(normal_vector[0] * scalar + cx), _ri(normal_vector[1] * scalar + cz))
    right = (_ri(-normal_vector[0] * scalar + cx), _ri(-normal_vector[1] * scalar + cz))

    if left not in base_edge:
        left = (
            _ri(normal_vector[0] * (scalar + 1) + cx),
            _ri(normal_vector[1] * (scalar + 1) + cz),
        )
    if right not in base_edge:
        right = (
            _ri(-normal_vector[0] * (scalar + 1) + cx),
            _ri(-normal_vector[1] * (scalar + 1) + cz),
        )

    return (left, right)


# ── Wire drawing (distance-aware) ────────────────────────────────────────────


def _nearest_center_dist_sq(px, py, center_track):
    best = float("inf")
    for cp in center_track:
        d = (px - cp[0]) ** 2 + (py - cp[1]) ** 2
        if d < best:
            best = d
    return best


def _draw_wire_segments(anchors, center_track=None, base_width=None):
    use_distance = (
        center_track is not None
        and base_width is not None
        and len(center_track) > 0
    )
    dist_thresh_sq = (base_width / 2.0) ** 2 if use_distance else 0

    wires = []
    for i in range(len(anchors) - 1):
        p1 = anchors[i]
        p2 = anchors[i + 1]
        if not (p1 and p2):
            continue

        line_fn = step_line  # default

        if use_distance:
            mx = (p1[0] + p2[0]) / 2.0
            my = (p1[1] + p2[1]) / 2.0
            d_sq = _nearest_center_dist_sq(mx, my, center_track)
            if d_sq > dist_thresh_sq:
                line_fn = bresenham_line

        wires.extend(line_fn(p1[0], p1[1], p2[0], p2[1]))
    return wires


# ── 3-D elevation interpolation ──────────────────────────────────────────────


def _interpolate_wire_segments(anchors, wire_pixels, elevation_map, pole_lines,
                               center_track=None, base_width=None):
    if len(anchors) < 2 or not wire_pixels:
        return

    use_distance = (
        center_track is not None
        and base_width is not None
        and len(center_track) > 0
    )
    dist_thresh_sq = (base_width / 2.0) ** 2 if use_distance else 0

    seg_lengths = []
    for j in range(len(anchors) - 1):
        p1 = anchors[j]
        p2 = anchors[j + 1]
        if p1 and p2:
            line_fn = step_line
            if use_distance:
                mx = (p1[0] + p2[0]) / 2.0
                my = (p1[1] + p2[1]) / 2.0
                if _nearest_center_dist_sq(mx, my, center_track) > dist_thresh_sq:
                    line_fn = bresenham_line
            seg_lengths.append(len(line_fn(p1[0], p1[1], p2[0], p2[1])))
        else:
            seg_lengths.append(0)

    offset = 0
    for j, seg_len in enumerate(seg_lengths):
        if seg_len == 0:
            continue
        anchor_start = anchors[j]
        anchor_end = anchors[j + 1]
        y_start = pole_lines.get(
            anchor_start, _get_elevation(anchor_start, elevation_map)
        )
        y_end = pole_lines.get(anchor_end, _get_elevation(anchor_end, elevation_map))
        for k in range(seg_len):
            if offset + k >= len(wire_pixels):
                break
            pt = wire_pixels[offset + k]
            t = k / max(seg_len - 1, 1)
            pole_lines[pt] = y_start + (y_end - y_start) * t
        offset += seg_len


def _get_elevation(pt, elevation_map, default=0):
    if not pt:
        return default
    if pt in elevation_map:
        return elevation_map[pt]
    for r in range(1, 20):
        for dx in range(-r, r + 1):
            for dz in range(-r, r + 1):
                if abs(dx) != r and abs(dz) != r:
                    continue
                neighbour = (pt[0] + dx, pt[1] + dz)
                if neighbour in elevation_map:
                    return elevation_map[neighbour]
    return default