import json
import sys
import argparse

import numpy as np

from trackAssembler import assemble_track, assemble_path, DEFAULT_CONFIG
from util.exportUtil import export_schematic
from util.elevationUtil import generate_elevation_lookup
from util.blockUtil import _norm_coord


def parse_input(filepath):
    """
    Parse a JSON input file containing control points, elevation, and config.

    Expected JSON format:
    {
        "control_points": [
            [x, z, angle],
            [x, z, angle],
            ...
        ],
        "elevation": [
            [x, z, elevation],
            [x, z, elevation],
            ...
        ],
        "config": {
            "base_width": 15,
            "track_width": 5,
            "turn_radius": 20,
            "catenary_interval": 30,
            "catenary_offset": 0,
            "base_block": "minecraft:gray_wool",
            "brim_block": "minecraft:gray_wool",
            "elevation_interval": 1
        },
        "output": "my_track.schem"
    }

    control_points are required. Each is (x, z, angle) where angle is
    the heading in degrees (converted to radians internally).

    elevation is optional. Each entry is (x, z, elevation) where the
    2D point should lie on or near the center path. Consecutive pairs
    define linear elevation ramps.

    config and output are optional.
    """
    with open(filepath, "r") as f:
        data = json.load(f)

    if "control_points" not in data:
        raise ValueError("Input JSON must contain a 'control_points' array.")

    control_points = [
        (pt[0], pt[1], pt[2] * np.pi / 180) for pt in data["control_points"]
    ]

    elevation = None
    if "elevation" in data and data["elevation"]:
        elevation = [(ep[0], ep[1], ep[2]) for ep in data["elevation"]]

    config = data.get("config", None)
    output = data.get("output", "output.schem")

    return control_points, elevation, config, output


def build_path_blocks(path, elevation_lut=None, block="minecraft:stone"):
    """
    Convert a Dubins path into a blocks_dict with a single block type.

    Each unique (x, z) in the path becomes one block.  When
    *elevation_lut* is provided the Y coordinate is looked up from
    the table; otherwise Y defaults to 0.
    """
    blocks_dict = {}
    seen = set()
    for pt in path:
        xz = (_norm_coord(pt[0]), _norm_coord(pt[1]))
        if xz in seen:
            continue
        seen.add(xz)
        x, z = xz
        y = elevation_lut.get(xz, 0) if elevation_lut else 0
        blocks_dict.setdefault(block, []).append((x, y, z))
    return blocks_dict


def first_path_offset(path, blocks_dict, elevation_lut=None):
    """
    Compute a schematic offset so that the paste point aligns with the
    first path point at 1 Y level above its elevation.

    create_schematic normalises coordinates by subtracting the bounding-
    box minimum, so a block at world (wx, wy, wz) ends up at local
    (wx - min_x, wy - min_y, wz - min_z).  The Sponge Schematic
    ``Offset`` is added to the player position to find where local
    (0, 0, 0) goes, meaning the block appears at
    ``paste_pos + offset + local``.

    To land the first path point at ``(paste_x, paste_y + 1, paste_z)``
    we need::

        offset = ( -(first_x - min_x),
                   -(first_y - min_y) + 1,
                   -(first_z - min_z) )
    """
    # First path point
    pt = path[0]
    fx = _norm_coord(pt[0])
    fz = _norm_coord(pt[1])
    fy = elevation_lut.get((fx, fz), 0) if elevation_lut else 0

    # Bounding-box minimums across every block in the dict
    all_positions = [pos for positions in blocks_dict.values() for pos in positions]
    min_x = min(p[0] for p in all_positions)
    min_y = min(p[1] for p in all_positions)
    min_z = min(p[2] for p in all_positions)

    return (-(fx - min_x), -(fy - min_y) - 1, -(fz - min_z))


def main():
    parser = argparse.ArgumentParser(
        description="Generate a Minecraft track schematic from control points."
    )
    parser.add_argument(
        "input", help="JSON file with control_points (and optional elevation / config)"
    )
    parser.add_argument(
        "--path-only",
        action="store_true",
        help="Export only the Dubins path as blocks instead of the full track.",
    )
    parser.add_argument(
        "--no-elevation",
        action="store_true",
        help="Ignore elevation data (only meaningful with --path-only).",
    )
    parser.add_argument(
        "-o",
        "--output",
        default=None,
        help="Override the export filename (takes priority over the JSON 'output' field).",
    )

    args = parser.parse_args()

    control_points, elevation, config, output = parse_input(args.input)

    if args.output:
        output = args.output

    cfg = dict(DEFAULT_CONFIG)
    if config:
        cfg.update(config)

    print(f"Parsed {len(control_points)} control points")
    if elevation:
        print(f"Parsed {len(elevation)} elevation control points")
    if config:
        print(f"Config overrides: {list(config.keys())}")

    # ------------------------------------------------------------------ #
    # Path-only mode: export just the Dubins path                         #
    # ------------------------------------------------------------------ #
    if args.path_only:
        path, start, end = assemble_path(control_points, cfg["turn_radius"])
        print(f"Dubins path: {len(path)} points")

        elevation_lut = None
        if elevation and not args.no_elevation:
            path_set = {(_norm_coord(pt[0]), _norm_coord(pt[1])) for pt in path}
            elevation_lut = generate_elevation_lookup(
                path,
                1,
                path_set,
                step_size=cfg["elevation_interval"],
                elevation_control_points=elevation,
            )
            print("Applied elevation to path")

        blocks_dict = build_path_blocks(path, elevation_lut)
        offset = first_path_offset(path, blocks_dict, elevation_lut)

        total_blocks = sum(len(pos) for pos in blocks_dict.values())
        print(f"Assembled {total_blocks} path blocks")

        export_schematic(blocks_dict, filename=output, offset=offset)
        print(f"Exported path schematic to {output}")
        return

    # ------------------------------------------------------------------ #
    # Full track mode (default)                                           #
    # ------------------------------------------------------------------ #
    blocks_dict = assemble_track(control_points, elevation, config)

    # Derive the path again so we can compute the offset
    path, _, _ = assemble_path(control_points, cfg["turn_radius"])
    # Build an elevation LUT for the offset lookup
    if elevation:
        path_set = {(_norm_coord(pt[0]), _norm_coord(pt[1])) for pt in path}
        elev_lut = generate_elevation_lookup(
            path,
            1,
            path_set,
            step_size=cfg["elevation_interval"],
            elevation_control_points=elevation,
        )
    else:
        elev_lut = None
    offset = first_path_offset(path, blocks_dict, elev_lut)

    total_blocks = sum(len(positions) for positions in blocks_dict.values())
    print(f"Assembled {total_blocks} blocks across {len(blocks_dict)} block types")

    export_schematic(blocks_dict, filename=output, offset=offset)
    print(f"Exported schematic to {output}")


if __name__ == "__main__":
    main()
