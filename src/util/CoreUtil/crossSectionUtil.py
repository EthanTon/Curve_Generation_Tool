AIR_BLOCKS = frozenset(
    {
        "minecraft:air",
        "minecraft:cave_air",
        "minecraft:void_air",
    }
)


def _is_air(block_str):
    """Return True if *block_str* represents any variant of air."""
    name = block_str.split("[")[0].strip().lower()
    return name in AIR_BLOCKS


def parse_cross_section(blocks_dict):
    return {
        block_str: positions
        for block_str, positions in blocks_dict.items()
        if not _is_air(block_str)
    }


def cross_section_at_y(blocks_dict, y_level):
    result = {}
    for block_str, positions in blocks_dict.items():
        if _is_air(block_str):
            continue
        layer = [(x, z) for x, y, z in positions if y == y_level]
        if layer:
            result[block_str] = layer
    return result


def cross_section_at_x(blocks_dict, x_level):
    result = {}
    for block_str, positions in blocks_dict.items():
        if _is_air(block_str):
            continue
        layer = [(y, z) for x, y, z in positions if x == x_level]
        if layer:
            result[block_str] = layer
    return result


def cross_section_at_z(blocks_dict, z_level):
    result = {}
    for block_str, positions in blocks_dict.items():
        if _is_air(block_str):
            continue
        layer = [(x, y) for x, y, z in positions if z == z_level]
        if layer:
            result[block_str] = layer
    return result
