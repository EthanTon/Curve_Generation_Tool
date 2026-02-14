def _norm_coord(v):
    """Round any numeric value to the nearest Python int."""
    return int(round(float(v)))


def _vector_to_facing(dx, dz):
    """Convert a 2-D direction vector into a Minecraft cardinal facing."""
    if abs(dx) >= abs(dz):
        return "east" if dx > 0 else "west"
    return "south" if dz > 0 else "north"


def _opposite_facing(facing):
    return {"north": "south", "south": "north", "east": "west", "west": "east"}[facing]


def _get_connection_cardinal(curr_x, curr_z, neighbor_x, neighbor_z):
    return _vector_to_facing(neighbor_x - curr_x, neighbor_z - curr_z)


# ---------------------------------------------------------------------------
# Block-state resolution helpers
# ---------------------------------------------------------------------------


def _rail_shape_from_connections(connections):
    """Return the Minecraft rail ``shape`` value for a set of cardinal connections."""
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
    if len(connections) == 1:
        card = list(connections)[0]
        return "east_west" if card in ("east", "west") else "north_south"
    return "north_south"


def _resolve_track_shapes(points):
    """
    Walk an ordered list of track-center points and determine the rail
    ``shape`` block-state at every unique (x, z) based on its predecessor
    and successor in the sequence.
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
    For every rail point (which may carry an embedded direction as elements
    [2] and [3]), resolve the anvil ``facing`` block-state.
    """
    if not points:
        return {}
    result = {}
    for pt in points:
        if len(pt) >= 4:
            dx, dz = pt[2], pt[3]
            facing = (
                _vector_to_facing(dx, dz)
                if abs(dx) > 0.001 or abs(dz) > 0.001
                else "north"
            )
        else:
            facing = "north"
        result[(_norm_coord(pt[0]), _norm_coord(pt[1]))] = facing
    return result


def _resolve_iron_bar_states(wire_pixels):
    """
    Build the full iron-bars block string for each wire pixel, with
    connectivity flags for its four cardinal neighbours.
    """
    wire_set = set(wire_pixels)
    result = {}
    for x, z in wire_set:
        east = "true" if (x + 1, z) in wire_set else "false"
        west = "true" if (x - 1, z) in wire_set else "false"
        south = "true" if (x, z + 1) in wire_set else "false"
        north = "true" if (x, z - 1) in wire_set else "false"
        result[(x, z)] = (
            f"minecraft:iron_bars"
            f"[east={east},north={north},south={south},"
            f"waterlogged=false,west={west}]"
        )
    return result


# ---------------------------------------------------------------------------
# Wall / pole block helpers
# ---------------------------------------------------------------------------


def _wall_block(north="none", east="none", south="none", west="none", up="true"):
    return (
        f"minecraft:andesite_wall"
        f"[east={east},north={north},south={south},"
        f"up={up},waterlogged=false,west={west}]"
    )


def _wall_tall_toward(facing):
    sides = {"north": "none", "east": "none", "south": "none", "west": "none"}
    sides[facing] = "tall"
    return _wall_block(**sides, up="true")


def _wall_low_toward(facing):
    sides = {"north": "none", "east": "none", "south": "none", "west": "none"}
    sides[facing] = "low"
    return _wall_block(**sides, up="true")


def _nearest_elevation_lut(pt, elevation_lut, default=0):
    """Spiral outward from *pt* until an elevation LUT entry is found."""
    for r in range(1, 30):
        for dx in range(-r, r + 1):
            for dz in range(-r, r + 1):
                if abs(dx) != r and abs(dz) != r:
                    continue
                neighbour = (pt[0] + dx, pt[1] + dz)
                if neighbour in elevation_lut:
                    return elevation_lut[neighbour]
    return default


def _append_lever(x, y, z, toward_center_facing, blocks_dict):
    block = (
        f"minecraft:lever[face=wall,facing={toward_center_facing}," f"powered=false]"
    )
    blocks_dict.setdefault(block, []).append((x, y, z))


# ---------------------------------------------------------------------------
# Pole detail emission
# ---------------------------------------------------------------------------


def _emit_pole_detail_blocks(
    pole_left,
    pole_right,
    cantilever_left,
    cantilever_right,
    elevation_lut,
    catenary_elevation_lut,
    blocks_dict,
):
    """
    Place the full catenary-pole structure for one pole pair.

    Each pole receives a vertical column of andesite-wall blocks topped
    with a button, plus cantilever detail (fence gates, levers, trapdoors,
    snow layers, torchflower crops) using the precomputed cantilever pixel
    lists rather than a cross-line between poles.
    """
    lx = _norm_coord(pole_left[0])
    lz = _norm_coord(pole_left[1])
    rx = _norm_coord(pole_right[0])
    rz = _norm_coord(pole_right[1])

    left_to_center_dx = rx - lx
    left_to_center_dz = rz - lz
    right_to_center_dx = lx - rx
    right_to_center_dz = lz - rz

    left_center_facing = _vector_to_facing(left_to_center_dx, left_to_center_dz)
    right_center_facing = _vector_to_facing(right_to_center_dx, right_to_center_dz)
    left_outward_facing = _opposite_facing(left_center_facing)
    right_outward_facing = _opposite_facing(right_center_facing)

    WALL_NONE = _wall_block()

    # -- Vertical pole columns (both sides) --
    for px, pz, center_facing in [
        (lx, lz, left_center_facing),
        (rx, rz, right_center_facing),
    ]:
        y = elevation_lut.get((px, pz), 0)

        blocks_dict.setdefault("minecraft:polished_andesite", []).append((px, y, pz))

        for dy in range(1, 7):
            blocks_dict.setdefault(WALL_NONE, []).append((px, y + dy, pz))

        tall_block = _wall_tall_toward(center_facing)
        for dy in range(7, 10):
            blocks_dict.setdefault(tall_block, []).append((px, y + dy, pz))

        blocks_dict.setdefault(WALL_NONE, []).append((px, y + 10, pz))

        low_block = _wall_low_toward(center_facing)
        blocks_dict.setdefault(low_block, []).append((px, y + 11, pz))

        button_block = (
            f"minecraft:stone_button[face=floor,facing={center_facing},"
            f"powered=false]"
        )
        blocks_dict.setdefault(button_block, []).append((px, y + 12, pz))

    y_left = elevation_lut.get((lx, lz), 0)
    y_right = elevation_lut.get((rx, rz), 0)

    # Use the cantilever lists to place detail blocks.
    # Each cantilever is an ordered list of pixels from the pole toward
    # the track.  Index [0] is nearest to the pole, last is nearest to
    # the track intersection.
    # We need at least 3 pixels in each cantilever for the detail blocks.

    if len(cantilever_left) >= 3:
        # -- Left cantilever pixel [0] (nearest pole) --
        p1 = cantilever_left[0]
        p1x, p1z = _norm_coord(p1[0]), _norm_coord(p1[1])
        y_p1 = y_left

        fg_block = (
            f"minecraft:pale_oak_fence_gate[facing={left_outward_facing},"
            f"in_wall=false,open=true,powered=false]"
        )
        blocks_dict.setdefault(fg_block, []).append((p1x, y_p1 + 7, p1z))
        _append_lever(p1x, y_p1 + 8, p1z, right_center_facing, blocks_dict)
        blocks_dict.setdefault("minecraft:snow[layers=2]", []).append(
            (p1x, y_p1 + 9, p1z)
        )

        # -- Left cantilever pixel [1] --
        p2 = cantilever_left[1]
        p2x, p2z = _norm_coord(p2[0]), _norm_coord(p2[1])
        y_p2 = y_left

        fg_block2 = (
            f"minecraft:pale_oak_fence_gate[facing={left_center_facing},"
            f"in_wall=false,open=true,powered=false]"
        )
        blocks_dict.setdefault(fg_block2, []).append((p2x, y_p2 + 8, p2z))
        td_block = (
            f"minecraft:iron_trapdoor[facing={left_center_facing},"
            f"half=bottom,open=false,powered=false,waterlogged=false]"
        )
        blocks_dict.setdefault(td_block, []).append((p2x, y_p2 + 9, p2z))

        # -- Left cantilever pixel [2] --
        p3 = cantilever_left[2]
        p3x, p3z = _norm_coord(p3[0]), _norm_coord(p3[1])
        y_p3 = y_left

        lever_block = (
            f"minecraft:lever[face=floor,facing={left_center_facing}," f"powered=false]"
        )
        blocks_dict.setdefault(lever_block, []).append((p3x, y_p3 + 8, p3z))
        blocks_dict.setdefault("minecraft:torchflower_crop[age=0]", []).append(
            (p3x, y_p3 + 9, p3z)
        )

    if len(cantilever_right) >= 3:
        # -- Right cantilever pixel [0] (nearest pole) --
        pr1 = cantilever_right[0]
        pr1x, pr1z = _norm_coord(pr1[0]), _norm_coord(pr1[1])
        y_pr1 = y_right

        fg_block_r1 = (
            f"minecraft:pale_oak_fence_gate[facing={right_outward_facing},"
            f"in_wall=false,open=true,powered=false]"
        )
        blocks_dict.setdefault(fg_block_r1, []).append((pr1x, y_pr1 + 7, pr1z))
        _append_lever(pr1x, y_pr1 + 8, pr1z, left_center_facing, blocks_dict)
        blocks_dict.setdefault("minecraft:snow[layers=2]", []).append(
            (pr1x, y_pr1 + 9, pr1z)
        )

        # -- Right cantilever pixel [1] --
        pr2 = cantilever_right[1]
        pr2x, pr2z = _norm_coord(pr2[0]), _norm_coord(pr2[1])
        y_pr2 = y_right

        fg_block_r2 = (
            f"minecraft:pale_oak_fence_gate[facing={right_center_facing},"
            f"in_wall=false,open=true,powered=false]"
        )
        blocks_dict.setdefault(fg_block_r2, []).append((pr2x, y_pr2 + 8, pr2z))
        td_block_r2 = (
            f"minecraft:iron_trapdoor[facing={right_center_facing},"
            f"half=bottom,open=false,powered=false,waterlogged=false]"
        )
        blocks_dict.setdefault(td_block_r2, []).append((pr2x, y_pr2 + 9, pr2z))

        # -- Right cantilever pixel [2] --
        pr3 = cantilever_right[2]
        pr3x, pr3z = _norm_coord(pr3[0]), _norm_coord(pr3[1])
        y_pr3 = y_right

        lever_block_r3 = (
            f"minecraft:lever[face=floor,facing={right_center_facing},"
            f"powered=false]"
        )
        blocks_dict.setdefault(lever_block_r3, []).append((pr3x, y_pr3 + 8, pr3z))
        blocks_dict.setdefault("minecraft:torchflower_crop[age=0]", []).append(
            (pr3x, y_pr3 + 9, pr3z)
        )

    # -- Top iron trapdoors at Y+11 adjacent to each pole --
    if len(cantilever_left) >= 1:
        pt_l = cantilever_left[0]
        plx, plz = _norm_coord(pt_l[0]), _norm_coord(pt_l[1])
        y_l = y_left
        td_top_l = (
            f"minecraft:iron_trapdoor[facing={left_center_facing},"
            f"half=top,open=false,powered=false,waterlogged=false]"
        )
        blocks_dict.setdefault(td_top_l, []).append((plx, y_l + 11, plz))

    if len(cantilever_right) >= 1:
        pt_r = cantilever_right[0]
        prx, prz = _norm_coord(pt_r[0]), _norm_coord(pt_r[1])
        y_r = y_right
        td_top_r = (
            f"minecraft:iron_trapdoor[facing={right_center_facing},"
            f"half=top,open=false,powered=false,waterlogged=false]"
        )
        blocks_dict.setdefault(td_top_r, []).append((prx, y_r + 11, prz))
