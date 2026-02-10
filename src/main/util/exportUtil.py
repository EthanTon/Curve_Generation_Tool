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
    # Map sets of cardinal connections to rail shapes
    # Note: Straight rails (e.g. North-South) are handled by default/fallback
    corner_map = {
        frozenset(["north", "east"]): "north_east",
        frozenset(["north", "west"]): "north_west",
        frozenset(["south", "east"]): "south_east",
        frozenset(["south", "west"]): "south_west",
        frozenset(["north", "south"]): "north_south",
        frozenset(["east", "west"]): "east_west",
    }
    
    # If we have exactly 2 connections, check if it's a valid shape
    if len(connections) == 2:
        return corner_map.get(frozenset(connections), "north_south")
    
    # If only 1 connection (end of line), align with it
    elif len(connections) == 1:
        card = list(connections)[0]
        return "east_west" if card in ("east", "west") else "north_south"
        
    return "north_south"


def _resolve_track_shapes(points):
    """
    Determine minecraft:rail shapes from actual neighbor adjacency.
    
    Args:
        points: List of (x, z, ...) tuples.
        
    Returns:
        dict { (x, z): shape_string }
    """
    if not points:
        return {}

    # Pre-calculate normalised coords to ensure we check grid adjacency
    coords = [(_norm_coord(pt[0]), _norm_coord(pt[1])) for pt in points]
    n = len(coords)
    result = {}

    for i in range(n):
        x, z = coords[i]
        connections = set()

        # Check Previous Neighbor
        if i > 0:
            px, pz = coords[i - 1]
            # Verify it is actually adjacent or same grid cell (ignore duplicates)
            if px != x or pz != z:
                connections.add(_get_connection_cardinal(x, z, px, pz))

        # Check Next Neighbor
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
    
    Args:
        points: List of (x, z, dx, dz) tuples.
    
    Returns:
        dict { (x, z): facing_string }
    """
    if not points:
        return {}

    result = {}

    for pt in points:
        # Unpack: x, z, dx, dz
        # We use the explicit vector (index 2 and 3) for accurate rotation
        if len(pt) >= 4:
            x_raw, z_raw, dx, dz = pt[0], pt[1], pt[2], pt[3]
            
            # If the vector is zero length (shouldn't happen), skip or default
            if abs(dx) < 0.001 and abs(dz) < 0.001:
                facing = "north"
            else:
                facing = _vector_to_facing(dx, dz)
        else:
            # Fallback if no vector provided
            facing = "north"

        # Normalise grid position for the key
        grid_pos = (_norm_coord(pt[0]), _norm_coord(pt[1]))
        
        # We overwrite if multiple points map to same block, 
        # taking the last one (usually fine for ordered tracks)
        result[grid_pos] = facing

    return result


# ── Export Function ───────────────────────────────────────────────────────
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
):
    """
    Export all layers to a Sponge Schematic v3 (.schem) file.
    
    Args:
        base:           set of (x, z) - base surface
        brim:           set of (x, z) - brim surface
        track_center:   ordered list of (x, z, dx, dz) - centre rail
        rail0_points:   ordered list of (x, z, dx, dz) - first guardrail
        rail1_points:   ordered list of (x, z, dx, dz) - second guardrail
        elevation_mask: nested dict {y_level: {(x,z): y, ...}, ...}
        filename:       output path
        base_block:     block for the base layer
        brim_block:     block for the brim layer
    """

    # 1. Build the elevation look-up table
    elevation_lut = {}
    for y_level in sorted(elevation_mask.keys()):
        for pt, y in elevation_mask[y_level].items():
            elevation_lut[(_norm_coord(pt[0]), _norm_coord(pt[1]))] = int(round(float(y)))

    # 2. Normalise base and brim to int-tuple sets
    base_set = {(_norm_coord(x), _norm_coord(z)) for x, z in base}
    brim_set = {(_norm_coord(x), _norm_coord(z)) for x, z in brim}

    # 3. Resolve directional blocks
    # Track shapes are derived from neighbor connections (grid logic)
    track_dir = _resolve_track_shapes(track_center)
    
    # Anvil facings are derived from explicit tangent vectors (dx, dz)
    rail0_dir = _resolve_anvil_facings(rail0_points)
    rail1_dir = _resolve_anvil_facings(rail1_points)

    # Merge rail0 + rail1
    rail_dir = {}
    rail_dir.update(rail0_dir)
    rail_dir.update(rail1_dir)

    track_coords = set(track_dir.keys())
    rail_coords = set(rail_dir.keys())

    # 4. Emit blocks layer by layer
    all_xz = base_set | track_coords | rail_coords
    blocks = []

    for xz in all_xz:
        x, z = xz
        y = elevation_lut.get(xz, 0)

        # Y+0: base layer
        if xz in base_set:
            blocks.append((x, y, z, base_block))

        # Y+1: rail (anvil) takes precedence over brim
        if xz in rail_dir:
            facing = rail_dir[xz]
            blocks.append((x, y + 1, z, f"minecraft:anvil[facing={facing}]"))
        elif xz in brim_set:
            blocks.append((x, y + 1, z, brim_block))

        # Y+2: track (rail)
        if xz in track_dir:
            shape = track_dir[xz]
            blocks.append(
                (x, y + 2, z, f"minecraft:rail[shape={shape},waterlogged=false]")
            )

    if not blocks:
        raise ValueError("No blocks to export – base is empty.")

    schematic = create_schematic(blocks)
    schematic.save(filename)
    return filename