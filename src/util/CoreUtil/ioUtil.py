from ..SchematicUtil.schematicUtil import create_schematic, read_schematic
from ..WorldUtil.worldUtil import (
    _read_region,
    _get_nbt,
    _dump_nbt,
    _write_region,
    _patch_chunk,
)

import os
from collections import defaultdict


def import_schematic(filename, origin=(0, 0, 0)):
    blocks, dims, offset = read_schematic(filename)
    ox, oy, oz = offset

    blocks_dict = defaultdict(list)
    for lx, ly, lz, block_str in blocks:
        blocks_dict[block_str].append(
            (
                lx + ox + origin[0],
                ly + oy + origin[1],
                lz + oz + origin[2],
            )
        )

    return dict(blocks_dict)


def export_schematic(
    blocks_dict, filename="output.schem", offset=(0, 0, 0), data_version=3700
):
    blocks = []
    for block_str, positions in blocks_dict.items():
        for x, y, z in positions:
            blocks.append(
                (
                    int(round(float(x))),
                    int(round(float(y))),
                    int(round(float(z))),
                    block_str,
                )
            )

    if not blocks:
        raise ValueError("No blocks to export – blocks_dict is empty.")

    schematic = create_schematic(blocks, offset=offset, data_version=data_version)
    schematic.save_to(filename, compressed=True)
    return filename


def export_world(blocks_dict, world_dir, dimension="overworld"):
    dim_paths = {
        "overworld": os.path.join(world_dir, "region"),
        "nether": os.path.join(world_dir, "DIM-1", "region"),
        "end": os.path.join(world_dir, "DIM1", "region"),
    }
    region_dir = dim_paths[dimension]

    if not os.path.isdir(region_dir):
        raise FileNotFoundError(f"Region directory not found: {region_dir}")

    hierarchy = defaultdict(
        lambda: defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
    )

    block_count = 0
    for block_str, positions in blocks_dict.items():
        for x, y, z in positions:
            bx = int(round(float(x)))
            by = int(round(float(y)))
            bz = int(round(float(z)))
            cx, cz = bx >> 4, bz >> 4
            rx, rz = cx >> 5, cz >> 5
            lx, ly, lz = bx & 0xF, by & 0xF, bz & 0xF
            hierarchy[(rx, rz)][(cx, cz)][by >> 4][block_str].append((lx, ly, lz))
            block_count += 1

    if not block_count:
        raise ValueError("No blocks to export – blocks_dict is empty.")

    for (rx, rz), chunk_groups in hierarchy.items():
        path = os.path.join(region_dir, f"r.{rx}.{rz}.mca")
        if not os.path.isfile(path):
            raise FileNotFoundError(f"Region file not found: {path}")

        chunks = _read_region(path)

        for (cx, cz), sections_data in chunk_groups.items():
            idx = (cx & 31) + (cz & 31) * 32
            if idx not in chunks:
                raise ValueError(
                    f"Chunk ({cx}, {cz}) not generated in {os.path.basename(path)}"
                )

            chunk = _get_nbt(chunks[idx])
            _patch_chunk(chunk, sections_data)
            chunks[idx] = _dump_nbt(chunk)

        _write_region(path, chunks)

    return block_count
