import nbtlib
import util.schematicUtil.varintWriter as varintWriter
from typing import Tuple, Optional


def load_schematic(filename: str) -> Optional[nbtlib.File]:
    """
    Load a schematic file and return its NBT data structure.

    Args:
        filename (str): Path to the .schem file

    Returns:
        nbtlib.File: Loaded NBT file object, or None if file extension is incorrect
    """
    if filename.split(".")[-1] != "schem":
        return None

    loaded_file = nbtlib.load(filename, gzipped=True)

    if "Schematic" not in loaded_file:
        raise ValueError("The provided file does not contain a 'Schematic' root tag.")

    return loaded_file


def get_block_data(file: nbtlib.tag.Compound) -> nbtlib.tag.Compound:
    return file["Schematic"]["Blocks"]


def get_biome_data(file: nbtlib.tag.Compound) -> nbtlib.tag.Compound:
    if "Biomes" in file["Schematic"]:
        return file["Schematic"]["Biomes"]
    else:
        return None


def get_entities(file: nbtlib.tag.List) -> nbtlib.tag.List:
    if "Entities" in file["Schematic"]:
        return file["Schematic"]["Entities"]
    else:
        return nbtlib.List([])


def get_dimension(
    file: nbtlib.tag.Compound,
) -> Tuple[nbtlib.tag.Short, nbtlib.tag.Short, nbtlib.tag.Short]:
    width = file["Schematic"]["Width"]
    height = file["Schematic"]["Height"]
    length = file["Schematic"]["Length"]

    return (width, height, length)


def get_offset(
    file: nbtlib.tag.Compound,
) -> Tuple[nbtlib.tag.Int, nbtlib.tag.Int, nbtlib.tag.Int]:
    offsetX = file["Schematic"]["Offset"][0]
    offsetY = file["Schematic"]["Offset"][1]
    offsetZ = file["Schematic"]["Offset"][2]

    return (offsetX, offsetY, offsetZ)


def get_index(x, y, z, width, length):
    """Calculate the correct index in the block array using full dimensions"""
    return int(x + z * width + y * width * length)


def get_local_coordinate(index: int, width: int, length: int) -> Tuple[int, int, int]:
    """
    Convert linear index to local coordinates.

    Args:
        index (int): Linear index in the schematic data
        width (int): Width of the schematic
        length (int): Length of the schematic

    Returns:
        Tuple[int, int, int]: (x, y, z) local coordinates
    """
    localY = int(index / (width * length))
    localZ = int((index % (width * length)) / width)
    localX = int((index % (width * length)) % width)
    return (localX, localY, localZ)


def get_relative_coordinates(
    index: int,
    width: int,
    length: int,
    offsetX: int = 0,
    offsetY: int = 0,
    offsetZ: int = 0,
) -> Tuple[int, int, int]:
    """
    Get coordinates relative to offset.

    Args:
        index (int): Linear index in the schematic data
        width (int): Width of the schematic
        length (int): Length of the schematic
        offsetX (int): X-axis offset
        offsetY (int): Y-axis offset
        offsetZ (int): Z-axis offset

    Returns:
        Tuple[int, int, int]: (x, y, z) relative coordinates
    """
    localCoordinates = get_local_coordinate(index, width, length)
    return (
        localCoordinates[0] + offsetX,
        localCoordinates[1] + offsetY,
        localCoordinates[2] + offsetZ,
    )


def get_global_coordinates(
    index: int,
    width: int,
    length: int,
    offsetX: int = 0,
    offsetY: int = 0,
    offsetZ: int = 0,
    originX: int = 0,
    originY: int = 0,
    originZ: int = 0,
) -> Tuple[int, int, int]:
    """
    Get global coordinates including offset and origin.

    Args:
        index (int): Linear index in the schematic data
        width (int): Width of the schematic
        length (int): Length of the schematic
        offsetX (int): X-axis offset
        offsetY (int): Y-axis offset
        offsetZ (int): Z-axis offset
        originX (int): X-axis origin
        originY (int): Y-axis origin
        originZ (int): Z-axis origin

    Returns:
        Tuple[int, int, int]: (x, y, z) global coordinates
    """
    relativeCoordinates = get_relative_coordinates(
        index, width, length, offsetX, offsetY, offsetZ
    )
    return (
        relativeCoordinates[0] + originX,
        relativeCoordinates[1] + originY,
        relativeCoordinates[2] + originZ,
    )


def create_schematic(
    blocks: list,
    offset: Tuple[int, int, int] = (0, 0, 0),
    data_version: int = 3700,
    fill_block: str = "minecraft:air",
) -> nbtlib.File:

    if not blocks:
        raise ValueError("blocks list must not be empty")

    # ------------------------------------------------------------------
    # 1. Determine bounding box and normalise coordinates
    # ------------------------------------------------------------------
    xs = [b[0] for b in blocks]
    ys = [b[1] for b in blocks]
    zs = [b[2] for b in blocks]

    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    min_z, max_z = min(zs), max(zs)

    width = max_x - min_x + 1
    height = max_y - min_y + 1
    length = max_z - min_z + 1

    # ------------------------------------------------------------------
    # 2. Build palette and block array
    # ------------------------------------------------------------------
    palette = {}
    # Ensure the fill block is always palette entry 0
    palette[fill_block] = nbtlib.Int(0)

    # Pre-fill the volume with the fill block (palette id 0)
    volume = [0] * (width * height * length)

    for x, y, z, block_str in blocks:
        nx, ny, nz = x - min_x, y - min_y, z - min_z

        if block_str not in palette:
            palette[block_str] = nbtlib.Int(len(palette))

        idx = get_index(nx, ny, nz, width, length)
        volume[idx] = int(palette[block_str])

    # ------------------------------------------------------------------
    # 3. Encode block data as varint bytes
    # ------------------------------------------------------------------
    encoded_data = varintWriter.write(volume, width, height, length)

    # ------------------------------------------------------------------
    # 4. Assemble the NBT structure (Sponge Schematic v3)
    # ------------------------------------------------------------------
    schematic = nbtlib.File(
        {
            "Schematic": nbtlib.Compound(
                {
                    "Version": nbtlib.Int(3),
                    "DataVersion": nbtlib.Int(data_version),
                    "Width": nbtlib.Short(width),
                    "Height": nbtlib.Short(height),
                    "Length": nbtlib.Short(length),
                    "Offset": nbtlib.IntArray([offset[0], offset[1], offset[2]]),
                    "Blocks": nbtlib.Compound(
                        {
                            "Palette": nbtlib.Compound(palette),
                            "Data": nbtlib.ByteArray(encoded_data),
                            "BlockEntities": nbtlib.List[nbtlib.Compound]([]),
                        }
                    ),
                    "Entities": nbtlib.List[nbtlib.Compound]([]),
                }
            )
        },
        gzipped=True,
    )

    return schematic


def swap_palette(source_palette):
    palette = {}
    for value, block in source_palette.items():
        try:
            palette[block] = int(value)
        except (ValueError, TypeError):
            # Keep original value if conversion fails
            palette[block] = value
    return palette
