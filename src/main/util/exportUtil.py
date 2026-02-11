from util.schematicUtil.schematicutil import create_schematic
from util.shapeUtil import step_line


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


def _opposite_facing(facing):
    return {"north": "south", "south": "north", "east": "west", "west": "east"}[facing]


def _get_connection_cardinal(curr_x, curr_z, neighbor_x, neighbor_z):
    dx = neighbor_x - curr_x
    dz = neighbor_z - curr_z
    return _vector_to_facing(dx, dz)


def _rail_shape_from_connections(connections):
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
    if not points:
        return {}
    result = {}
    for pt in points:
        if len(pt) >= 4:
            dx, dz = pt[2], pt[3]
            facing = _vector_to_facing(dx, dz) if abs(dx) > 0.001 or abs(dz) > 0.001 else "north"
        else:
            facing = "north"
        result[(_norm_coord(pt[0]), _norm_coord(pt[1]))] = facing
    return result


def _resolve_iron_bar_states(wire_pixels):
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


# ── Andesite wall helpers ────────────────────────────────────────────────────

def _wall_block(north="none", east="none", south="none", west="none", up="true"):
    """Build an andesite_wall block state string."""
    return (
        f"minecraft:andesite_wall"
        f"[east={east},north={north},south={south},"
        f"up={up},waterlogged=false,west={west}]"
    )


def _wall_tall_toward(facing):
    """Andesite wall with the side facing *facing* set to tall, rest none."""
    sides = {"north": "none", "east": "none", "south": "none", "west": "none"}
    sides[facing] = "tall"
    return _wall_block(**sides, up="true")


def _wall_low_toward(facing):
    """Andesite wall with the side facing *facing* set to low, rest none."""
    sides = {"north": "none", "east": "none", "south": "none", "west": "none"}
    sides[facing] = "low"
    return _wall_block(**sides, up="true")


# ── Per-pole cross-line block emission ───────────────────────────────────────

def _emit_pole_detail_blocks(
    pole_left, pole_right, cross_line, elevation_lut, panto_lut, blocks
):
    """
    Emit all blocks for one pole pair's column and cross-line detail.

    Pole layout (symmetric — each side mirrors about centre):

    Pole column (at pole_left and pole_right positions):
      Y+0  : polished_andesite
      Y+1…Y+6  : andesite_wall [all=none, up=true]           (6 blocks)
      Y+7…Y+9  : andesite_wall [tall toward centre]          (3 blocks)
      Y+10 : andesite_wall [all=none, up=true]
      Y+11 : andesite_wall [low toward centre]
      Y+12 : stone_button [face=floor, facing=toward centre]

    Cross-line (first pixel in from each pole toward centre):
      Y+7  : pale_oak_fence_gate [open=true, facing=toward pole]
      Y+8  : lever on wall [facing=toward centre]
      Y+9  : snow [layers=2]

    Cross-line (second pixel in from each pole toward centre):
      Y+8  : pale_oak_fence_gate [open=true, facing=toward centre]
      Y+9  : iron_trapdoor [half=bottom, facing=toward centre]

    Cross-line (third pixel in from each pole toward centre):
      Y+8  : lever [face=floor, facing=toward centre]
      Y+9  : torchflower_crop [age=0]

    Cross-line middle (all remaining pixels between the two inner markers):
      Y+11 : iron_trapdoor [half=top, facing=toward centre]
    """
    lx = _norm_coord(pole_left[0])
    lz = _norm_coord(pole_left[1])
    rx = _norm_coord(pole_right[0])
    rz = _norm_coord(pole_right[1])

    # Direction vectors: from each pole toward centre of track
    # left pole → right pole direction = toward centre
    # right pole → left pole direction = toward centre
    left_to_center_dx = rx - lx
    left_to_center_dz = rz - lz
    right_to_center_dx = lx - rx
    right_to_center_dz = lz - rz

    left_center_facing = _vector_to_facing(left_to_center_dx, left_to_center_dz)
    right_center_facing = _vector_to_facing(right_to_center_dx, right_to_center_dz)

    left_outward_facing = _opposite_facing(left_center_facing)
    right_outward_facing = _opposite_facing(right_center_facing)

    # ── Pole columns ─────────────────────────────────────────────────────
    WALL_NONE = _wall_block()  # all=none, up=true

    for (px, pz, center_facing) in [
        (lx, lz, left_center_facing),
        (rx, rz, right_center_facing),
    ]:
        y = elevation_lut.get((px, pz), 0)

        # Y+0: polished_andesite
        blocks.append((px, y, pz, "minecraft:polished_andesite"))

        # Y+1…Y+6: plain andesite wall
        for dy in range(1, 7):
            blocks.append((px, y + dy, pz, WALL_NONE))

        # Y+7…Y+9: andesite wall tall toward centre (3 blocks)
        for dy in range(7, 10):
            blocks.append((px, y + dy, pz, _wall_tall_toward(center_facing)))

        # Y+10: andesite wall all=none
        blocks.append((px, y + 10, pz, WALL_NONE))

        # Y+11: andesite wall low toward centre
        blocks.append((px, y + 11, pz, _wall_low_toward(center_facing)))

        # Y+12: stone button on floor facing toward centre
        blocks.append((
            px, y + 12, pz,
            f"minecraft:stone_button[face=floor,facing={center_facing},"
            f"powered=false]"
        ))

    # ── Cross-line detail pixels ─────────────────────────────────────────
    # The cross_line is an ordered pixel list from left pole to right pole.
    # We need pixels [1], [2], [3] from the left end, and [-2], [-3], [-4]
    # from the right end.
    #
    # Detail pixels sit on the cross-line close to each pole.  To ensure
    # vertical stacks (lever→torchflower, fence_gate→lever→snow, etc.)
    # always connect properly, each side's detail pixels use the SAME
    # elevation as their adjacent pole column.  This prevents rounding
    # differences in interpolated elevation from splitting the stacks.
    y_left  = elevation_lut.get((lx, lz), 0)
    y_right = elevation_lut.get((rx, rz), 0)

    if len(cross_line) < 7:
        # Too short for detail — skip detail pixels
        return

    # Left side detail: pixel [1] (first in from left pole)
    p1 = cross_line[1]
    p1x, p1z = _norm_coord(p1[0]), _norm_coord(p1[1])
    y_p1 = y_left

    # fence gate open toward the pole (away from centre = outward)
    blocks.append((
        p1x, y_p1 + 7, p1z,
        f"minecraft:pale_oak_fence_gate[facing={left_outward_facing},"
        f"in_wall=false,open=true,powered=false]"
    ))
    # lever on wall facing toward centre
    _append_lever(p1x, y_p1 + 8, p1z, right_center_facing, blocks)
    # snow layers=2
    blocks.append((
        p1x, y_p1 + 9, p1z,
        "minecraft:snow[layers=2]"
    ))

    # Left side detail: pixel [2] (second in from left pole)
    p2 = cross_line[2]
    p2x, p2z = _norm_coord(p2[0]), _norm_coord(p2[1])
    y_p2 = y_left

    # fence gate open toward centre
    blocks.append((
        p2x, y_p2 + 8, p2z,
        f"minecraft:pale_oak_fence_gate[facing={left_center_facing},"
        f"in_wall=false,open=true,powered=false]"
    ))
    # iron trapdoor on top of fence gate, facing centre
    blocks.append((
        p2x, y_p2 + 9, p2z,
        f"minecraft:iron_trapdoor[facing={left_center_facing},"
        f"half=bottom,open=false,powered=false,waterlogged=false]"
    ))

    # Left side detail: pixel [3] (third in from left pole — one toward centre)
    p3 = cross_line[3]
    p3x, p3z = _norm_coord(p3[0]), _norm_coord(p3[1])
    y_p3 = y_left

    # lever on floor facing toward centre (same Y level as fence gate)
    blocks.append((
        p3x, y_p3 + 8, p3z,
        f"minecraft:lever[face=floor,facing={left_center_facing},"
        f"powered=false]"
    ))
    # torchflower_crop on top of the lever
    blocks.append((
        p3x, y_p3 + 9, p3z,
        "minecraft:torchflower_crop[age=0]"
    ))

    # Right side detail: pixel [-2] (first in from right pole)
    pr1 = cross_line[-2]
    pr1x, pr1z = _norm_coord(pr1[0]), _norm_coord(pr1[1])
    y_pr1 = y_right

    # fence gate open toward the pole (away from centre = outward)
    blocks.append((
        pr1x, y_pr1 + 7, pr1z,
        f"minecraft:pale_oak_fence_gate[facing={right_outward_facing},"
        f"in_wall=false,open=true,powered=false]"
    ))
    # lever on wall facing toward centre
    _append_lever(pr1x, y_pr1 + 8, pr1z, left_center_facing, blocks)
    # snow layers=2
    blocks.append((
        pr1x, y_pr1 + 9, pr1z,
        "minecraft:snow[layers=2]"
    ))

    # Right side detail: pixel [-3] (second in from right pole)
    pr2 = cross_line[-3]
    pr2x, pr2z = _norm_coord(pr2[0]), _norm_coord(pr2[1])
    y_pr2 = y_right

    # fence gate open toward centre
    blocks.append((
        pr2x, y_pr2 + 8, pr2z,
        f"minecraft:pale_oak_fence_gate[facing={right_center_facing},"
        f"in_wall=false,open=true,powered=false]"
    ))
    # iron trapdoor on top of fence gate, facing centre
    blocks.append((
        pr2x, y_pr2 + 9, pr2z,
        f"minecraft:iron_trapdoor[facing={right_center_facing},"
        f"half=bottom,open=false,powered=false,waterlogged=false]"
    ))

    # Right side detail: pixel [-4] (third in from right pole — one toward centre)
    pr3 = cross_line[-4]
    pr3x, pr3z = _norm_coord(pr3[0]), _norm_coord(pr3[1])
    y_pr3 = y_right

    # lever on floor facing toward centre (same Y level as fence gate)
    blocks.append((
        pr3x, y_pr3 + 8, pr3z,
        f"minecraft:lever[face=floor,facing={right_center_facing},"
        f"powered=false]"
    ))
    # torchflower_crop on top of the lever
    blocks.append((
        pr3x, y_pr3 + 9, pr3z,
        "minecraft:torchflower_crop[age=0]"
    ))

    # ── Top: one iron_trapdoor upper adjacent to each pole at Y+11 ─────
    # One pixel toward centre from each pole, at the same level as the
    # andesite wall with low side.  Not the whole cross-line.
    if len(cross_line) >= 3:
        # Left pole: pixel [1] is first toward centre
        pt_l = cross_line[1]
        plx, plz = _norm_coord(pt_l[0]), _norm_coord(pt_l[1])
        y_l = y_left
        blocks.append((
            plx, y_l + 11, plz,
            f"minecraft:iron_trapdoor[facing={left_center_facing},"
            f"half=top,open=false,powered=false,waterlogged=false]"
        ))

        # Right pole: pixel [-2] is first toward centre
        pt_r = cross_line[-2]
        prx, prz = _norm_coord(pt_r[0]), _norm_coord(pt_r[1])
        y_r = y_right
        blocks.append((
            prx, y_r + 11, prz,
            f"minecraft:iron_trapdoor[facing={right_center_facing},"
            f"half=top,open=false,powered=false,waterlogged=false]"
        ))


def _append_lever(x, y, z, toward_center_facing, blocks):
    """
    Append a lever attached to a wall, facing toward the centre of the track.
    The lever's 'facing' in Minecraft indicates which wall it's attached to.
    A lever facing north means it's on the south face of a block to the north.
    We want it on the wall that's toward the pole (opposite of toward_center),
    so the lever faces toward centre.
    """
    blocks.append((
        x, y, z,
        f"minecraft:lever[face=wall,facing={toward_center_facing},"
        f"powered=false]"
    ))


def _nearest_elevation_lut(pt, elevation_lut, default=0):
    """Spiral search for the nearest known elevation in the LUT."""
    for r in range(1, 30):
        for dx in range(-r, r + 1):
            for dz in range(-r, r + 1):
                if abs(dx) != r and abs(dz) != r:
                    continue
                neighbour = (pt[0] + dx, pt[1] + dz)
                if neighbour in elevation_lut:
                    return elevation_lut[neighbour]
    return default


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
    cross_lines=None,
    overhead_wire_pixels=None,
    wire1_pixels=None,
    wire2_pixels=None,
    pantograph_elevation=None,
):
    """
    Export all layers to a Sponge Schematic v3 (.schem) file.

    Parameters
    ----------
    cross_lines : list[list[(x,z)]] | None
        Each element is one cross-line (ordered pixel list per pole pair).
        Used for per-pole detail block emission.
    overhead_wire_pixels : list[(x,z)] | None
        Combined (deduplicated) wire pixels from both tracks.  Used as
        a fallback when wire1_pixels / wire2_pixels are not provided.
    wire1_pixels : list[(x,z)] | None
        Ordered wire pixels for track 1.  When provided, iron-bar
        connectivity is resolved per-track for correct neighbours.
    wire2_pixels : list[(x,z)] | None
        Ordered wire pixels for track 2.
    pantograph_elevation : dict[(x,z) -> float] | None
        Per-pixel interpolated base elevation for overhead structures.
    """
    # 1. Build the base elevation look-up table
    elevation_lut = {}
    for y_level in sorted(elevation_mask.keys()):
        for pt, y in elevation_mask[y_level].items():
            elevation_lut[
                (_norm_coord(pt[0]), _norm_coord(pt[1]))
            ] = int(round(float(y)))

    # Pantograph-specific interpolated elevation LUT
    panto_lut = {}
    if pantograph_elevation:
        for pt, y in pantograph_elevation.items():
            panto_lut[
                (_norm_coord(pt[0]), _norm_coord(pt[1]))
            ] = int(round(float(y)))

    # 2. Normalise base and brim
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

    # Pole positions (for exclusion from base/brim rendering)
    pole_set = set()
    if poles:
        for left, right in poles:
            pole_set.add((_norm_coord(left[0]), _norm_coord(left[1])))
            pole_set.add((_norm_coord(right[0]), _norm_coord(right[1])))

    # Resolve overhead-wire iron-bar states
    # When separate per-track wire lists are available, resolve connectivity
    # independently so each wire's iron bars only connect to neighbours on the
    # same wire.  Then merge into a single dict for block emission.
    wire_dir = {}
    wire_set = set()

    if wire1_pixels or wire2_pixels:
        # Per-track resolution
        if wire1_pixels:
            norm_w1 = [(_norm_coord(p[0]), _norm_coord(p[1])) for p in wire1_pixels]
            wire_dir.update(_resolve_iron_bar_states(norm_w1))
        if wire2_pixels:
            norm_w2 = [(_norm_coord(p[0]), _norm_coord(p[1])) for p in wire2_pixels]
            wire_dir.update(_resolve_iron_bar_states(norm_w2))
        wire_set = set(wire_dir.keys())
    elif overhead_wire_pixels:
        # Fallback: combined list (legacy behaviour)
        norm_wire = [
            (_norm_coord(p[0]), _norm_coord(p[1]))
            for p in overhead_wire_pixels
        ]
        wire_dir = _resolve_iron_bar_states(norm_wire)
        wire_set = set(wire_dir.keys())

    # 4. Emit base, brim, rail, track blocks
    all_xz = base_set | track_coords | rail_coords | pole_set | wire_set
    blocks = []

    for xz in all_xz:
        x, z = xz
        y = elevation_lut.get(xz, 0)

        # Y+0: base
        if xz in base_set and xz not in pole_set:
            blocks.append((x, y, z, base_block))

        # Y+1: rail (anvil) or brim
        if xz in rail_dir:
            facing = rail_dir[xz]
            blocks.append(
                (x, y + 1, z, f"minecraft:anvil[facing={facing}]")
            )
        elif xz in brim_set and xz not in pole_set:
            blocks.append((x, y + 1, z, brim_block))

        # Y+2: track (rail)
        if xz in track_dir:
            shape = track_dir[xz]
            blocks.append(
                (x, y + 2, z,
                 f"minecraft:rail[shape={shape},waterlogged=false]")
            )

        # Overhead wire (iron bars) at Y+8 using interpolated elevation.
        # Look up from panto_lut first, then elevation_lut, then spiral
        # search so wire pixels outside the base footprint still get a
        # sensible height.
        if xz in wire_set:
            if xz in panto_lut:
                y_wire = panto_lut[xz]
            elif xz in elevation_lut:
                y_wire = elevation_lut[xz]
            else:
                y_wire = _nearest_elevation_lut(xz, elevation_lut)
            blocks.append((x, y_wire + 8, z, wire_dir[xz]))

    # 6. Emit all pole + cross-line detail blocks
    if poles and cross_lines:
        for pole_pair, cross_line in zip(poles, cross_lines):
            left, right = pole_pair
            _emit_pole_detail_blocks(
                left, right, cross_line,
                elevation_lut, panto_lut, blocks
            )

    if not blocks:
        raise ValueError("No blocks to export – base is empty.")

    schematic = create_schematic(blocks)
    schematic.save(filename)
    return filename