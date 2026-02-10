from util.schematicUtil.schematicutil import create_schematic


def export_base_brim(
    base,
    brim,
    elevation_mask,
    filename="output.schem",
    base_block="minecraft:gray_wool",
    brim_block="minecraft:red_wool",
):
    """
    Export base and brim point sets to a Sponge Schematic v3 (.schem) file,
    using the elevation mask to determine each block's Y coordinate.

    Args:
        base: set of (x, z) tuples forming the base surface
        brim: set of (x, z) tuples forming the brim surface
        elevation_mask: dict returned by generate_elevation_mask —
                        {segment_index: {(x, z): y_value, ...}}
        filename: output .schem file path
        base_block: block id for the base area (default: gray_wool)
        brim_block: block id for the brim area (default: red_wool)
    """
    # Flatten the elevation mask into a single {(x, z): y} lookup.
    # Later segments overwrite earlier ones, so the last write wins.
    elevation_lut = {}
    for seg_index in sorted(elevation_mask.keys()):
        for pt, y in elevation_mask[seg_index].items():
            elevation_lut[pt] = y

    blocks = []
    for x, z in base:
        y = int(elevation_lut.get((x, z), 0))
        if (x, z) in brim:
            blocks.append((int(x), y, int(z), base_block))
            blocks.append((int(x), y + 1, int(z), brim_block))
        else:
            blocks.append((int(x), y, int(z), base_block))

    if not blocks:
        raise ValueError("No blocks to export — base is empty.")

    schematic = create_schematic(blocks)
    schematic.save(filename)
    return filename