import re

_CARDINAL_CW = ["north", "east", "south", "west"]
_AXIS_ROTATION = {
    "x": ["x", "z", "x", "z"],
    "z": ["z", "x", "z", "x"],
    "y": ["y", "y", "y", "y"],
}
_RAIL_SHAPE_CW = {
    "north_south": ["north_south", "east_west", "north_south", "east_west"],
    "east_west": ["east_west", "north_south", "east_west", "north_south"],
    "ascending_north": [
        "ascending_north",
        "ascending_east",
        "ascending_south",
        "ascending_west",
    ],
    "ascending_east": [
        "ascending_east",
        "ascending_south",
        "ascending_west",
        "ascending_north",
    ],
    "ascending_south": [
        "ascending_south",
        "ascending_west",
        "ascending_north",
        "ascending_east",
    ],
    "ascending_west": [
        "ascending_west",
        "ascending_north",
        "ascending_east",
        "ascending_south",
    ],
    "north_east": ["north_east", "south_east", "south_west", "north_west"],
    "south_east": ["south_east", "south_west", "north_west", "north_east"],
    "south_west": ["south_west", "north_west", "north_east", "south_east"],
    "north_west": ["north_west", "north_east", "south_east", "south_west"],
}
_ORIENT_DIR_CW = {
    "north": "east",
    "east": "south",
    "south": "west",
    "west": "north",
    "up": "up",
    "down": "down",
}

_RAIL_BLOCKS = frozenset(
    {
        "minecraft:rail",
        "minecraft:powered_rail",
        "minecraft:detector_rail",
        "minecraft:activator_rail",
    }
)
_CURVE_RAIL_BLOCKS = frozenset({"minecraft:rail"})

_BLOCK_RE = re.compile(r"^(?P<id>[a-z_:]+?)(?:\[(?P<states>.+)])?$")


def parse_block(block_string):
    m = _BLOCK_RE.match(block_string)
    if m is None:
        return block_string, {}
    block_id = m.group("id")
    raw = m.group("states")
    if raw is None:
        return block_id, {}
    states = {}
    for pair in raw.split(","):
        k, v = pair.split("=", 1)
        states[k.strip()] = v.strip()
    return block_id, states


def serialize_block(block_id, states):
    if not states:
        return block_id
    state_str = ",".join(f"{k}={v}" for k, v in sorted(states.items()))
    return f"{block_id}[{state_str}]"


def _rotate_facing(states, steps):
    val = states.get("facing")
    if val in _CARDINAL_CW:
        states["facing"] = _CARDINAL_CW[(_CARDINAL_CW.index(val) + steps) % 4]


def _rotate_axis(states, steps):
    val = states.get("axis")
    if val in _AXIS_ROTATION:
        states["axis"] = _AXIS_ROTATION[val][steps % 4]


def _rotate_rotation(states, steps):
    val = states.get("rotation")
    if val is not None:
        states["rotation"] = str((int(val) + 4 * steps) % 16)


def _rotate_rail_shape(states, steps):
    val = states.get("shape")
    if val in _RAIL_SHAPE_CW:
        states["shape"] = _RAIL_SHAPE_CW[val][steps % 4]


def _rotate_directional_booleans(states, steps):
    dir_keys = {"north", "east", "south", "west"}
    if len(dir_keys & states.keys()) != 4:
        return
    original = {d: states[d] for d in dir_keys}
    for d in dir_keys:
        src_dir = _CARDINAL_CW[(_CARDINAL_CW.index(d) - steps) % 4]
        states[d] = original[src_dir]


def _rotate_orientation(states, steps):
    val = states.get("orientation")
    if val is None:
        return
    parts = val.split("_")
    rotated = []
    for p in parts:
        r = p
        for _ in range(steps % 4):
            r = _ORIENT_DIR_CW.get(r, r)
        rotated.append(r)
    states["orientation"] = "_".join(rotated)


def rotate_block_state(block_string, steps):
    steps = steps % 4
    if steps == 0:
        return block_string
    block_id, states = parse_block(block_string)
    if not states:
        return block_string
    _rotate_facing(states, steps)
    _rotate_axis(states, steps)
    _rotate_rotation(states, steps)
    _rotate_rail_shape(states, steps)
    _rotate_directional_booleans(states, steps)
    _rotate_orientation(states, steps)
    return serialize_block(block_id, states)


_MIRROR_X_FACING = {"east": "west", "west": "east", "north": "north", "south": "south"}
_MIRROR_Z_FACING = {"north": "south", "south": "north", "east": "east", "west": "west"}
_MIRROR_X_RAIL_SHAPE = {
    "north_east": "north_west",
    "north_west": "north_east",
    "south_east": "south_west",
    "south_west": "south_east",
}
_MIRROR_Z_RAIL_SHAPE = {
    "north_east": "south_east",
    "south_east": "north_east",
    "north_west": "south_west",
    "south_west": "north_west",
}


def mirror_block_state(block_string, axis="x"):
    """Mirror block state across an axis. axis='x' swaps east↔west, 'z' swaps north↔south."""
    block_id, states = parse_block(block_string)
    if not states:
        return block_string
    facing_lut = _MIRROR_X_FACING if axis == "x" else _MIRROR_Z_FACING
    shape_lut = _MIRROR_X_RAIL_SHAPE if axis == "x" else _MIRROR_Z_RAIL_SHAPE
    swap_a, swap_b = ("east", "west") if axis == "x" else ("north", "south")

    val = states.get("facing")
    if val in facing_lut:
        states["facing"] = facing_lut[val]
    val = states.get("shape")
    if val in shape_lut:
        states["shape"] = shape_lut[val]
    if {"north", "east", "south", "west"} <= states.keys():
        states[swap_a], states[swap_b] = states[swap_b], states[swap_a]
    val = states.get("rotation")
    if val is not None:
        states["rotation"] = str((16 - int(val)) % 16)
    val = states.get("orientation")
    if val is not None:
        parts = val.split("_")
        states["orientation"] = "_".join(
            swap_b if p == swap_a else swap_a if p == swap_b else p for p in parts
        )
    return serialize_block(block_id, states)


def is_rail_block(block_string):
    block_id, _ = parse_block(block_string)
    return block_id in _RAIL_BLOCKS


_CARDINALS = [("north", 0, -1), ("east", 1, 0), ("south", 0, 1), ("west", -1, 0)]
_STRAIGHT_SHAPES = {
    frozenset(("north", "south")): "north_south",
    frozenset(("east", "west")): "east_west",
}
_CURVE_SHAPES = {
    frozenset(("north", "east")): "north_east",
    frozenset(("east", "south")): "south_east",
    frozenset(("south", "west")): "south_west",
    frozenset(("west", "north")): "north_west",
}


def _infer_rail_shape(block_id, connections, ascending):
    """Determine the rail shape from connected directions."""
    if ascending:
        return f"ascending_{ascending[0]}"
    if len(connections) >= 2:
        pair = frozenset(connections[:2])
        if pair in _STRAIGHT_SHAPES:
            return _STRAIGHT_SHAPES[pair]
        if pair in _CURVE_SHAPES and block_id in _CURVE_RAIL_BLOCKS:
            return _CURVE_SHAPES[pair]
        return "north_south" if connections[0] in ("north", "south") else "east_west"
    if len(connections) == 1:
        return "north_south" if connections[0] in ("north", "south") else "east_west"
    return None


def resolve_rail_shapes(blocks_dict, rail_groups=None):
    """Resolve rail shape properties based on neighbor connectivity.
    rail_groups: list of position-sets resolved independently (None = one group).
    """
    pos_to_block = {}
    for block, coords in blocks_dict.items():
        if not is_rail_block(block):
            continue
        for pos in coords:
            pos_to_block[tuple(pos)] = block

    if not pos_to_block:
        return blocks_dict

    if rail_groups is None:
        rail_groups = [set(pos_to_block.keys())]

    resolved = {}
    for group in rail_groups:
        for pos in group:
            if pos not in pos_to_block:
                continue
            x, y, z = pos
            block_id, states = parse_block(pos_to_block[pos])

            flat, ascending = [], []
            for dname, dx, dz in _CARDINALS:
                if (x + dx, y, z + dz) in group:
                    flat.append(dname)
                elif (x + dx, y + 1, z + dz) in group:
                    ascending.append(dname)
                elif (x + dx, y - 1, z + dz) in group:
                    flat.append(dname)

            shape = _infer_rail_shape(block_id, flat + ascending, ascending)
            if shape:
                states["shape"] = shape
            resolved[pos] = serialize_block(block_id, states)

    new_dict = {}
    for block, coords in blocks_dict.items():
        if not is_rail_block(block):
            new_dict.setdefault(block, []).extend(coords)
            continue
        for pos in coords:
            new_block = resolved.get(tuple(pos), block)
            new_dict.setdefault(new_block, []).append(pos)
    return new_dict
