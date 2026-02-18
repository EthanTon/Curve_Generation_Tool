import numpy as np
import amulet_nbt as nbt
from . import varintWriter as varintWriter


def load_schematic(filename):
    if filename.split(".")[-1] != "schem":
        return None

    loaded = nbt.load(filename, compressed=True)
    root = loaded.compound

    if "Schematic" not in root:
        raise ValueError("The provided file does not contain a 'Schematic' root tag.")

    return root


def get_block_data(file):
    return file["Schematic"]["Blocks"]


def get_biome_data(file):
    if "Biomes" in file["Schematic"]:
        return file["Schematic"]["Biomes"]
    return None


def get_entities(file):
    if "Entities" in file["Schematic"]:
        return file["Schematic"]["Entities"]
    return nbt.ListTag([])


def get_dimension(file):
    s = file["Schematic"]
    return (s["Width"], s["Height"], s["Length"])


def get_offset(file):
    o = file["Schematic"]["Offset"]
    return (o[0], o[1], o[2])


def get_index(x, y, z, width, length):
    return int(x + z * width + y * width * length)


def get_local_coordinate(index, width, length):
    localY = int(index / (width * length))
    localZ = int((index % (width * length)) / width)
    localX = int((index % (width * length)) % width)
    return (localX, localY, localZ)


def get_relative_coordinates(index, width, length, offsetX=0, offsetY=0, offsetZ=0):
    lx, ly, lz = get_local_coordinate(index, width, length)
    return (lx + offsetX, ly + offsetY, lz + offsetZ)


def get_global_coordinates(
    index,
    width,
    length,
    offsetX=0,
    offsetY=0,
    offsetZ=0,
    originX=0,
    originY=0,
    originZ=0,
):
    rx, ry, rz = get_relative_coordinates(
        index, width, length, offsetX, offsetY, offsetZ
    )
    return (rx + originX, ry + originY, rz + originZ)


def create_schematic(
    blocks, offset=(0, 0, 0), data_version=3700, fill_block="minecraft:air"
):
    if not blocks:
        raise ValueError("blocks list must not be empty")

    xs = [b[0] for b in blocks]
    ys = [b[1] for b in blocks]
    zs = [b[2] for b in blocks]

    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    min_z, max_z = min(zs), max(zs)

    width = max_x - min_x + 1
    height = max_y - min_y + 1
    length = max_z - min_z + 1

    palette = {fill_block: nbt.IntTag(0)}
    volume = [0] * (width * height * length)

    for x, y, z, block_str in blocks:
        nx, ny, nz = x - min_x, y - min_y, z - min_z
        if block_str not in palette:
            palette[block_str] = nbt.IntTag(len(palette))
        idx = get_index(nx, ny, nz, width, length)
        volume[idx] = int(palette[block_str])

    encoded_data = varintWriter.write(volume, width, height, length)

    schematic = nbt.NamedTag(
        nbt.CompoundTag(
            {
                "Schematic": nbt.CompoundTag(
                    {
                        "Version": nbt.IntTag(3),
                        "DataVersion": nbt.IntTag(data_version),
                        "Width": nbt.ShortTag(width),
                        "Height": nbt.ShortTag(height),
                        "Length": nbt.ShortTag(length),
                        "Offset": nbt.IntArrayTag(
                            np.array([offset[0], offset[1], offset[2]], dtype=np.int32)
                        ),
                        "Blocks": nbt.CompoundTag(
                            {
                                "Palette": nbt.CompoundTag(palette),
                                "Data": nbt.ByteArrayTag(encoded_data),
                                "BlockEntities": nbt.ListTag([]),
                            }
                        ),
                        "Entities": nbt.ListTag([]),
                    }
                )
            }
        )
    )

    return schematic


def swap_palette(source_palette):
    palette = {}
    for value, block in source_palette.items():
        try:
            palette[block] = int(value)
        except (ValueError, TypeError):
            palette[block] = value
    return palette
