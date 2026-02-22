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


def cross_section_at_y(blocks_dict, y_level, offset=(0, 0, 0)):
    """Return a horizontal cross-section at *y_level* (world coordinates).

    Parameters
    ----------
    blocks_dict : dict[str, list[tuple]]
        Mapping of block strings to lists of (x, y, z) positions in **local**
        schematic coordinates.
    y_level : int
        The Y level to slice at, in **world** coordinates (i.e. accounting
        for the schematic offset).
    offset : tuple[int, int, int], optional
        The (ox, oy, oz) offset stored in the schematic.  Local positions are
        shifted by this offset before the slice comparison, so the returned
        (x, z) pairs are in world coordinates.
    """
    ox, oy, oz = offset
    result = {}
    for block_str, positions in blocks_dict.items():
        if _is_air(block_str):
            continue
        layer = [(x + ox, z + oz) for x, y, z in positions if y + oy == y_level]
        if layer:
            result[block_str] = layer
    return result


def cross_section_at_x(blocks_dict, x_level, offset=(0, 0, 0)):
    ox, oy, oz = offset
    result = {}
    for block_str, positions in blocks_dict.items():
        if _is_air(block_str):
            continue
        layer = [(y + oy, z + oz) for x, y, z in positions if x + ox == x_level]
        if layer:
            result[block_str] = layer
    return result


def cross_section_at_z(blocks_dict, z_level, offset=(0, 0, 0)):
    ox, oy, oz = offset
    result = {}
    for block_str, positions in blocks_dict.items():
        if _is_air(block_str):
            continue
        layer = [(x + ox, y + oy) for x, y, z in positions if z + oz == z_level]
        if layer:
            result[block_str] = layer
    return result


def to_curve_offsets(cross_section_2d, copy_point, axis="z"):
    """Convert a 2D cross-section slice to 3D offsets relative to a copy/paste point.

    The returned dict maps block strings to lists of (ox, oy, oz) offsets
    suitable for use with ``assemble_curve``.  ``ox`` is perpendicular to the
    default path direction, ``oy`` is vertical, and ``oz`` is along the path.

    Parameters
    ----------
    cross_section_2d : dict[str, list[tuple]]
        Output from one of the ``cross_section_at_*`` functions.
    copy_point : tuple
        The paste/copy reference point **in the same coordinate space** as the
        values in *cross_section_2d*.

        * axis ``'z'`` (from ``cross_section_at_z``) → ``(x, y)``
        * axis ``'x'`` (from ``cross_section_at_x``) → ``(y, z)``
        * axis ``'y'`` (from ``cross_section_at_y``) → ``(x, z)``
    axis : str
        Which axis the slice was taken along (``'x'``, ``'y'``, or ``'z'``).
    """
    offsets = {}
    cp0, cp1 = copy_point

    for block_str, positions in cross_section_2d.items():
        converted = []
        for a, b in positions:
            da, db = a - cp0, b - cp1
            if axis == "z":
                # cross_section_at_z returns (x_world, y_world)
                converted.append((da, db, 0))  # ox=dx, oy=dy, oz=0
            elif axis == "x":
                # cross_section_at_x returns (y_world, z_world)
                converted.append((db, da, 0))  # ox=dz, oy=dy, oz=0
            elif axis == "y":
                # cross_section_at_y returns (x_world, z_world)
                converted.append((da, 0, db))  # ox=dx, oy=0, oz=dz
        if converted:
            offsets[block_str] = converted

    return offsets
