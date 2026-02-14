from util.schematicUtil.schematicutil import create_schematic


def export_schematic(blocks_dict, filename="output.schem", offset=(0, 0, 0), data_version=3700):
    blocks = []
    for block_str, positions in blocks_dict.items():
        for x, y, z in positions:
            blocks.append((
                int(round(float(x))),
                int(round(float(y))),
                int(round(float(z))),
                block_str,
            ))

    if not blocks:
        raise ValueError("No blocks to export – blocks_dict is empty.")

    schematic = create_schematic(blocks, offset=offset, data_version=data_version)
    schematic.save(filename)
    return filename