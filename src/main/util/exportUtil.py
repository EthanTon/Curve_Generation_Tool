from util.schematicUtil.schematicutil import create_schematic


def _sign(v):
    """Return -1, 0, or 1 based on sign of v (with a small tolerance)."""
    if v > 0.01:
        return 1
    elif v < -0.01:
        return -1
    return 0


def _norm_coord(v):
    """Normalise a coordinate value to a plain Python int."""
    return int(round(float(v)))


def _vector_to_facing(dx, dz):
    """
    Convert a vector (dx, dz) to a cardinal direction string.
    Prioritises the dominant axis.
    """
    if abs(dx) >= abs(dz):
        return "east" if dx > 0 else "west"
    else:
        return "south" if dz > 0 else "north"


def _get_connection_cardinal(curr_x, curr_z, neighbor_x, neighbor_z):
    """
    Determine the cardinal direction of a connection FROM current TO neighbor.
    """
    dx = neighbor_x - curr_x
    dz = neighbor_z - curr_z
    return _vector_to_facing(dx, dz)


def _rail_shape_from_connections(connections):
    """
    Determine minecraft:rail shape from a set of connection directions.
    """
    corner_map = {
        frozenset(["north", "east"]): "north_east",
        frozenset(["north", "west"]): "north_west",
        frozenset(["south", "east"]): "south_east",
        frozenset(["south", "west"]): "south_west",
        frozenset(["north", "south"]): "north_south",
        frozenset(["east", "west"]): "east_west",
    }

    if len(connections) == 2:
        return corner_map.get(frozenset(connections), "north_south")
    elif len(connections) == 1:
        card = list(connections)[0]
        return "east_west" if card in ("east", "west") else "north_south"
    return "north_south"


def _resolve_track_shapes(points):
    """
    Determine minecraft:rail shapes from actual neighbor adjacency.
    """
    if not points:
        return {}

    coords = [(_norm_coord(pt[0]), _norm_coord(pt[1])) for pt in points]
    n = len(coords)
    result = {}

    for i in range(n):
        x, z = coords[i]
        connections = set()

        if i > 0:
            px, pz = coords[i - 1]
            if px != x or pz != z:
                connections.add(_get_connection_cardinal(x, z, px, pz))

        if i < n - 1:
            nx, nz = coords[i + 1]
            if nx != x or nz != z:
                connections.add(_get_connection_cardinal(x, z, nx, nz))

        result[(x, z)] = _rail_shape_from_connections(connections)

    return result


def _resolve_anvil_facings(points):
    """
    Determine minecraft:anvil facings using the explicit direction vectors
    (dx, dz) provided in the input points.
    """
    if not points:
        return {}

    result = {}
    for pt in points:
        if len(pt) >= 4:
            x_raw, z_raw, dx, dz = pt[0], pt[1], pt[2], pt[3]
            if abs(dx) < 0.001 and abs(dz) < 0.001:
                facing = "north"
            else:
                facing = _vector_to_facing(dx, dz)
        else:
            facing = "north"

        grid_pos = (_norm_coord(pt[0]), _norm_coord(pt[1]))
        result[grid_pos] = facing

    return result


# ── Iron-bar connectivity ─────────────────────────────────────────────────────

def _resolve_iron_bar_states(wire_pixels):
    """
    Given a collection of (x, z) positions that will become iron bars,
    compute the block-state string for each position by checking which of
    its four cardinal neighbours are also in the set.

    Returns  dict { (x, z): block_state_string }
    """
    wire_set = set(wire_pixels)
    result = {}

    for (x, z) in wire_set:
        east  = "true" if (x + 1, z) in wire_set else "false"
        west  = "true" if (x - 1, z) in wire_set else "false"
        south = "true" if (x, z + 1) in wire_set else "false"
        north = "true" if (x, z - 1) in wire_set else "false"
        result[(x, z)] = (
            f"minecraft:iron_bars"
            f"[east={east},north={north},south={south},"
            f"waterlogged=false,west={west}]"
        )

    return result


# ── Iron-trapdoor facing for cross-lines ──────────────────────────────────────

def _resolve_trapdoor_facings(cross_line_pixels):
    """
    For each cross-line pixel, derive a facing from its immediate neighbours
    in the pixel list (which is ordered along the line).  The trapdoor is
    placed flat (half=bottom, open=false).

    Returns  dict { (x, z): block_state_string }
    """
    if not cross_line_pixels:
        return {}

    coords = [(_norm_coord(pt[0]), _norm_coord(pt[1])) for pt in cross_line_pixels]
    n = len(coords)
    result = {}

    for i in range(n):
        x, z = coords[i]
        if i > 0 and i < n - 1:
            dx = coords[i + 1][0] - coords[i - 1][0]
            dz = coords[i + 1][1] - coords[i - 1][1]
        elif i < n - 1:
            dx = coords[i + 1][0] - x
            dz = coords[i + 1][1] - z
        elif i > 0:
            dx = x - coords[i - 1][0]
            dz = z - coords[i - 1][1]
        else:
            dx, dz = 1, 0

        if abs(dx) >= abs(dz):
            facing = "east" if dx > 0 else "west"
        else:
            facing = "south" if dz > 0 else "north"

        result[(x, z)] = (
            f"minecraft:iron_trapdoor"
            f"[facing={facing},half=bottom,open=false,"
            f"powered=false,waterlogged=false]"
        )

    return result


# ── Export Function ───────────────────────────────────────────────────────────

def export_full_track(
    base,
    brim,
    track_center,
    rail0_points,
    rail1_points,
    elevation_mask,
    filename="output.schem",
    base_block="minecraft:gray_wool",
    brim_block="minecraft:gray_wool",
    poles=None,
    pole_block="minecraft:polished_andesite",
    cross_line_pixels=None,
    overhead_wire_pixels=None,
    pantograph_elevation=None,
):
    """
    Export all layers to a Sponge Schematic v3 (.schem) file.

    Pantograph structures
    ---------------------
    * **Poles** – 1 block of *pole_block* at base level (Y+0), then
      6 blocks of andesite wall (all sides ``none``, ``up=true``)
      stacked at Y+1 … Y+6.
    * **Cross-lines** – iron trapdoor (half=bottom) at Y+9 connecting
      each pole pair.  Elevation is 3-D interpolated between the two
      pole endpoints via *pantograph_elevation*.
    * **Overhead wires** – iron bars at Y+7 with correct NSEW
      connectivity along each track centre.  Elevation is 3-D
      interpolated between consecutive intersection points via
      *pantograph_elevation*, independent of the base beneath.

    Parameters
    ----------
    pantograph_elevation : dict[(x,z) -> float] | None
        Per-pixel interpolated base elevation for cross-line and wire
        pixels.  When provided, wire blocks use this instead of the
        main *elevation_mask* so the overhead structures follow a
        smooth 3-D line between supports.
    """
    # ── Constants ─────────────────────────────────────────────────────────
    WALL_BLOCK = (
        "minecraft:andesite_wall"
        "[east=none,north=none,south=none,up=true,waterlogged=false,west=none]"
    )

    # 1. Build the base elevation look-up table
    elevation_lut = {}
    for y_level in sorted(elevation_mask.keys()):
        for pt, y in elevation_mask[y_level].items():
            elevation_lut[
                (_norm_coord(pt[0]), _norm_coord(pt[1]))
            ] = int(round(float(y)))

    # Pantograph-specific interpolated elevation LUT (may be None)
    panto_lut = {}
    if pantograph_elevation:
        for pt, y in pantograph_elevation.items():
            panto_lut[
                (_norm_coord(pt[0]), _norm_coord(pt[1]))
            ] = int(round(float(y)))

    # 2. Normalise base and brim to int-tuple sets
    base_set = {(_norm_coord(x), _norm_coord(z)) for x, z in base}
    brim_set = {(_norm_coord(x), _norm_coord(z)) for x, z in brim}

    # 3. Resolve directional blocks
    track_dir = _resolve_track_shapes(track_center)
    rail0_dir = _resolve_anvil_facings(rail0_points)
    rail1_dir = _resolve_anvil_facings(rail1_points)

    rail_dir = {}
    rail_dir.update(rail0_dir)
    rail_dir.update(rail1_dir)

    track_coords = set(track_dir.keys())
    rail_coords = set(rail_dir.keys())

    # 3b. Pole positions (left + right of every pair)
    pole_set = set()
    if poles:
        for left, right in poles:
            pole_set.add((_norm_coord(left[0]), _norm_coord(left[1])))
            pole_set.add((_norm_coord(right[0]), _norm_coord(right[1])))

    # 3c. Resolve cross-line trapdoors
    cross_line_dir = {}
    cross_line_set = set()
    if cross_line_pixels:
        cross_line_dir = _resolve_trapdoor_facings(cross_line_pixels)
        cross_line_set = set(cross_line_dir.keys())

    # 3d. Resolve overhead-wire iron-bar states
    wire_dir = {}
    wire_set = set()
    if overhead_wire_pixels:
        norm_wire = [
            (_norm_coord(p[0]), _norm_coord(p[1]))
            for p in overhead_wire_pixels
        ]
        wire_dir = _resolve_iron_bar_states(norm_wire)
        wire_set = set(wire_dir.keys())

    # 4. Emit blocks layer by layer
    all_xz = (
        base_set | track_coords | rail_coords
        | pole_set | cross_line_set | wire_set
    )
    blocks = []

    for xz in all_xz:
        x, z = xz
        # Base elevation for normal blocks
        y = elevation_lut.get(xz, 0)
        # Interpolated elevation for pantograph overhead structures
        y_panto = panto_lut.get(xz, y)

        # ── Y+0 : base layer ─────────────────────────────────────────
        if xz in pole_set:
            blocks.append((x, y, z, pole_block))
        elif xz in base_set:
            blocks.append((x, y, z, base_block))

        # ── Y+1 : rail (anvil) or brim ───────────────────────────────
        if xz in rail_dir:
            facing = rail_dir[xz]
            blocks.append(
                (x, y + 1, z, f"minecraft:anvil[facing={facing}]")
            )
        elif xz in brim_set and xz not in pole_set:
            blocks.append((x, y + 1, z, brim_block))

        # ── Y+2 : track (rail) ───────────────────────────────────────
        if xz in track_dir:
            shape = track_dir[xz]
            blocks.append(
                (x, y + 2, z,
                 f"minecraft:rail[shape={shape},waterlogged=false]")
            )

        # ── Pole column : Y+1 … Y+6  andesite wall ──────────────────
        if xz in pole_set:
            for dy in range(1, 7):
                blocks.append((x, y + dy, z, WALL_BLOCK))

        # ── Y+7 : overhead wire (iron bars) – uses interpolated elev ─
        if xz in wire_set:
            blocks.append((x, y_panto + 8, z, wire_dir[xz]))

        # ── Y+9 : cross-line (iron trapdoor) – uses interpolated elev
        if xz in cross_line_set:
            blocks.append((x, y_panto + 9, z, cross_line_dir[xz]))

    if not blocks:
        raise ValueError("No blocks to export – base is empty.")

    schematic = create_schematic(blocks)
    schematic.save(filename)
    return filename