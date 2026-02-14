import numpy as np

from util.dubinsUtil import int_dubins_path
from util.trackUtil import track, generate_base
from util.elevationUtil import generate_elevation_lookup, generate_elevation_mask
from util.catenaryUtil import generate_catenary
from util.wireUtil import generate_wire, apply_elevation
from util.blockUtil import (
    _norm_coord,
    _vector_to_facing,
    _opposite_facing,
    _get_connection_cardinal,
    _resolve_track_shapes,
    _resolve_anvil_facings,
    _resolve_iron_bar_states,
    _nearest_elevation_lut,
    _emit_pole_detail_blocks,
)


DEFAULT_CONFIG = {
    "base_width": 15,
    "track_width": 5,
    "turn_radius": 20,
    "catenary_interval": 30,
    "catenary_offset": 0,
    "base_block": "minecraft:gray_wool",
    "brim_block": "minecraft:gray_wool",
    "elevation_interval": 1,
}
# ---------------------------------------------------------------------------
# Assembly — the single public entry point called by main.py
# ---------------------------------------------------------------------------


def assemble_path(control_points, radius):
    if len(control_points) < 2:
        raise ValueError("At least two control points are required to generate a path.")
    path = []
    for i in range(len(control_points) - 1):
        start = control_points[i]
        end = control_points[i + 1]
        segment = int_dubins_path(start, end, radius)
        if i > 0 and path and len(segment) > 0:
            if np.array_equal(np.asarray(path[-1])[:2], np.asarray(segment[0])[:2]):
                segment = segment[1:]
        path.extend(segment)

    # Ensure the final control point is represented as the last path point
    if path and not np.array_equal(
        np.asarray(path[-1])[:2], np.asarray(control_points[-1])[:2]
    ):
        # Match the dimensionality of existing path points
        pt = np.asarray(control_points[-1])
        target_dim = len(path[-1])
        path.append(tuple(pt[:target_dim]))

    return path, control_points[0], control_points[-1]


def assemble_track(control_points, elevation_control_points=None, config=None):
    cfg = dict(DEFAULT_CONFIG)
    if config:
        cfg.update(config)

    base_width = cfg["base_width"]
    track_width = cfg["track_width"]
    turn_radius = cfg["turn_radius"]
    catenary_interval = cfg["catenary_interval"]
    catenary_offset = cfg["catenary_offset"]
    base_block = cfg["base_block"]
    brim_block = cfg["brim_block"]
    elevation_interval = cfg["elevation_interval"]

    # ------------------------------------------------------------------ #
    # 1. Build the Dubins path (merged, no overlap)                       #
    # ------------------------------------------------------------------ #
    path, start, end = assemble_path(control_points, turn_radius)

    if len(path) < 2:
        raise ValueError("Path too short to build a track.")

    # ------------------------------------------------------------------ #
    # 2. Generate track geometry                                          #
    # ------------------------------------------------------------------ #
    (base, brim), (track_center, rail_0, rail_1) = track(
        path, start, end, base_width, track_width
    )

    base_set = set(map(tuple, base))

    # ------------------------------------------------------------------ #
    # 3. Build combined elevation mask                                    #
    # ------------------------------------------------------------------ #
    elevation_lut = generate_elevation_lookup(
        path,
        base_width,
        base_set,
        step_size=elevation_interval,
        elevation_control_points=elevation_control_points,
    )

    # ------------------------------------------------------------------ #
    # 4. Generate catenary poles / cantilevers along the entire path      #
    # ------------------------------------------------------------------ #
    # track_center holds (x, z, dx, dz) tuples; catenary needs plain
    # (x, z) lists wrapped in an outer list for set-based intersection.
    tc_left_xz = [(_norm_coord(pt[0]), _norm_coord(pt[1])) for pt in track_center[0]]
    tc_right_xz = [(_norm_coord(pt[0]), _norm_coord(pt[1])) for pt in track_center[1]]

    catenary_data = generate_catenary(
        path,
        [tc_left_xz],
        [tc_right_xz],
        base_set,
        base_width,
        track_width,
        catenary_interval,
        offset=catenary_offset,
    )

    t1_poles, t1_cantilevers, t1_intersections = catenary_data[0]
    t2_poles, t2_cantilevers, t2_intersections = catenary_data[1]

    # Pair up poles with their cantilevers
    poles = []
    cant_left_list = []
    cant_right_list = []
    for i in range(len(t1_poles)):
        poles.append((t1_poles[i], t2_poles[i]))
        cant_left_list.append(t1_cantilevers[i][1:])
        cant_right_list.append(t2_cantilevers[i][1:])

    # ------------------------------------------------------------------ #
    # 5. Generate overhead wires                                          #
    # ------------------------------------------------------------------ #
    wire1_3d = generate_wire(t1_intersections, elevation_lut)
    wire2_3d = generate_wire(t2_intersections, elevation_lut)

    wire1_pixels = [(p[0], p[1]) for p in wire1_3d] if wire1_3d else []
    wire2_pixels = [(p[0], p[1]) for p in wire2_3d] if wire2_3d else []

    # Build a catenary-specific elevation LUT from the wire z values
    catenary_elevation_lut = {}
    for p in wire1_3d:
        catenary_elevation_lut[(_norm_coord(p[0]), _norm_coord(p[1]))] = int(
            round(float(p[2]))
        )
    for p in wire2_3d:
        catenary_elevation_lut[(_norm_coord(p[0]), _norm_coord(p[1]))] = int(
            round(float(p[2]))
        )

    # ------------------------------------------------------------------ #
    # 6. Resolve directional block-states                                 #
    # ------------------------------------------------------------------ #
    all_track_center = list(track_center[0]) + list(track_center[1])
    track_dir = _resolve_track_shapes(all_track_center)

    all_rail_points = []
    if rail_0:
        all_rail_points.extend(rail_0[0])
        all_rail_points.extend(rail_0[1])
    if rail_1:
        all_rail_points.extend(rail_1[0])
        all_rail_points.extend(rail_1[1])
    rail_dir = _resolve_anvil_facings(all_rail_points)

    track_coords = set(track_dir.keys())
    rail_coords = set(rail_dir.keys())

    pole_set = set()
    for left, right in poles:
        pole_set.add((_norm_coord(left[0]), _norm_coord(left[1])))
        pole_set.add((_norm_coord(right[0]), _norm_coord(right[1])))

    wire_dir = {}
    if wire1_pixels:
        norm_w1 = [(_norm_coord(p[0]), _norm_coord(p[1])) for p in wire1_pixels]
        wire_dir.update(_resolve_iron_bar_states(norm_w1))
    if wire2_pixels:
        norm_w2 = [(_norm_coord(p[0]), _norm_coord(p[1])) for p in wire2_pixels]
        wire_dir.update(_resolve_iron_bar_states(norm_w2))
    wire_set = set(wire_dir.keys())

    # ------------------------------------------------------------------ #
    # 7. Assemble block dict: block_string -> [(x, y, z)]                 #
    # ------------------------------------------------------------------ #
    blocks_dict = {}

    norm_base = {(_norm_coord(x), _norm_coord(z)) for x, z in base}
    norm_brim = {(_norm_coord(x), _norm_coord(z)) for x, z in brim}

    all_xz = norm_base | track_coords | rail_coords | pole_set | wire_set

    for xz in all_xz:
        x, z = xz
        y = elevation_lut.get(xz, 0)

        # Base slab (skip under poles — pole column replaces it)
        if xz in norm_base and xz not in pole_set:
            blocks_dict.setdefault(base_block, []).append((x, y, z))

        # Rails (anvils) one block above; brim fills the rest
        if xz in rail_dir:
            facing = rail_dir[xz]
            block = f"minecraft:anvil[facing={facing}]"
            blocks_dict.setdefault(block, []).append((x, y + 1, z))
        elif xz in norm_brim and xz not in pole_set:
            blocks_dict.setdefault(brim_block, []).append((x, y + 1, z))

        # Track centre rails two blocks above
        if xz in track_dir:
            shape = track_dir[xz]
            block = f"minecraft:rail[shape={shape},waterlogged=false]"
            blocks_dict.setdefault(block, []).append((x, y + 2, z))

        # Overhead wires eight blocks above ground elevation
        if xz in wire_set:
            if xz in catenary_elevation_lut:
                y_wire = catenary_elevation_lut[xz]
            elif xz in elevation_lut:
                y_wire = elevation_lut[xz]
            else:
                y_wire = _nearest_elevation_lut(xz, elevation_lut)
            blocks_dict.setdefault(wire_dir[xz], []).append((x, y_wire + 8, z))

    # ------------------------------------------------------------------ #
    # 8. Emit pole and cantilever detail blocks                           #
    # ------------------------------------------------------------------ #
    for i, pole_pair in enumerate(poles):
        left, right = pole_pair
        _emit_pole_detail_blocks(
            left,
            right,
            cant_left_list[i],
            cant_right_list[i],
            elevation_lut,
            catenary_elevation_lut,
            blocks_dict,
        )

    return blocks_dict
