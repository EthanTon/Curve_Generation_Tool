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
