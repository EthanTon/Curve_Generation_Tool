#!/usr/bin/env python3
"""
main.py – CLI for assembling Minecraft curves.

Usage examples:
  # Curve with cross-section
  python main.py -i road.schem -o output.schem --radius 20 --width 5 points.json

  # Curve with cross-section, export to world
  python main.py -i road.schem -o /path/to/world --radius 20 --width 5 points.json

  # Path-only (no cross-section), default block
  python main.py -o output.schem --radius 20 points.json

  # Path-only with custom block
  python main.py -o output.schem --radius 20 --block minecraft:oak_planks points.json

Block positions are normalised to the minimum corner during export,
so pasting at your player position places the structure at your feet.

Expected JSON structure (just two flat lists):
{
    "control_points": [[x, z, angle_degrees], ...],
    "elevation_points": [[x, z, y], ...]
}

Angles in control_points are specified in degrees and are
converted to radians internally via np.radians().
Either key may be omitted; both default to [].
"""

import argparse
import json
import os
import sys

import numpy as np

from curveAssembly import assemble_curve, assemble_curve_path
from util.CoreUtil.ioUtil import import_schematic, export_schematic, export_world
from util.CoreUtil.crossSectionUtil import (
    parse_cross_section,
    cross_section_at_x,
    cross_section_at_y,
    cross_section_at_z,
)


# ── helpers ─────────────────────────────────────────────────────────────


def _load_points(path):
    """Load control_points and elevation_points from a JSON file.

    Returns two lists of tuples.  Missing keys default to [].
    """
    with open(path, "r") as f:
        data = json.load(f)

    raw_control = data.get("control_points", [])
    control = [(pt[0], pt[1], np.radians(pt[2])) for pt in raw_control]
    elevation = [tuple(pt) for pt in data.get("elevation_points", [])]
    return control, elevation


def _load_cross_section(schematic_path, axis=None, level=None):
    """Import a schematic and optionally slice it along an axis."""
    blocks = import_schematic(schematic_path)

    if axis is None:
        return parse_cross_section(blocks)

    slice_funcs = {
        "x": cross_section_at_x,
        "y": cross_section_at_y,
        "z": cross_section_at_z,
    }
    if axis not in slice_funcs:
        raise ValueError(f"Invalid axis '{axis}'. Use 'x', 'y', or 'z'.")
    if level is None:
        raise ValueError("--slice-level is required when --slice-axis is set.")

    return slice_funcs[axis](blocks, level)


def _is_world_dir(path):
    """Treat the output as a world directory if it is an existing dir or has
    no file extension."""
    if os.path.isdir(path):
        return True
    _, ext = os.path.splitext(path)
    return ext == ""


def _export(
    blocks_dict, output_path, origin=(0, 0, 0), dimension="overworld", data_version=3700
):
    if _is_world_dir(output_path):
        count = export_world(blocks_dict, output_path, dimension=dimension)
        print(f"Placed {count} blocks in world: {output_path} ({dimension})")
    else:
        export_schematic(
            blocks_dict, filename=output_path, offset=origin, data_version=data_version
        )
        print(f"Saved schematic: {output_path} (origin: {origin})")


def _patch_block_type(blocks_dict, block_type):
    """Replace every block string in the dict with *block_type*."""
    merged = []
    for positions in blocks_dict.values():
        merged.extend(positions)
    return {block_type: merged}


# ── CLI ─────────────────────────────────────────────────────────────────


def build_parser():
    p = argparse.ArgumentParser(description="Assemble a Minecraft curve and export it.")

    # positional – optional JSON with points
    p.add_argument(
        "points",
        nargs="?",
        default=None,
        metavar="POINTS_JSON",
        help="JSON file with control_points and/or elevation_points. "
        "Omit to use empty lists for both.",
    )

    # I/O
    p.add_argument(
        "-i",
        "--input",
        default=None,
        metavar="SCHEM",
        help="Cross-section schematic (.schem). Omit for path-only mode.",
    )
    p.add_argument(
        "-o",
        "--output",
        required=True,
        metavar="PATH",
        help="Output .schem file or Minecraft world directory.",
    )

    # curve parameters
    p.add_argument("--radius", type=float, required=True, help="Dubins turning radius.")
    p.add_argument(
        "--width",
        type=int,
        default=None,
        help="Cross-section width (required with -i).",
    )
    p.add_argument(
        "--step-size",
        type=int,
        default=1,
        help="Step size along the path (default: 1).",
    )
    p.add_argument(
        "--symmetrical",
        action="store_true",
        help="Mirror the cross-section across the path center.",
    )

    # path-only options
    p.add_argument(
        "--block",
        default="minecraft:stone",
        metavar="BLOCK",
        help="Block type for path-only mode (default: minecraft:stone). "
        "Ignored when -i is provided.",
    )

    # cross-section slicing
    p.add_argument(
        "--slice-axis",
        choices=["x", "y", "z"],
        default=None,
        help="Axis to slice the imported cross-section schematic on.",
    )
    p.add_argument(
        "--slice-level",
        type=int,
        default=None,
        help="Coordinate level for the slice (requires --slice-axis).",
    )

    # export options
    p.add_argument(
        "--dimension",
        default="overworld",
        choices=["overworld", "nether", "end"],
        help="Target dimension for world export (default: overworld).",
    )
    p.add_argument(
        "--data-version",
        type=int,
        default=3700,
        help="Minecraft data version for .schem export (default: 3700).",
    )

    return p


# ── main ────────────────────────────────────────────────────────────────


def main():
    parser = build_parser()
    args = parser.parse_args()

    # Load points (or use empty lists)
    if args.points:
        control_points, elevation_points = _load_points(args.points)
    else:
        control_points, elevation_points = [], []

    if args.input:
        # ── Cross-section mode ──────────────────────────────────────────
        if len(control_points) < 2:
            parser.error(
                "Cross-section mode requires at least 2 control points "
                "in the JSON file."
            )
        if args.width is None:
            parser.error("--width is required when using a cross-section (-i).")

        cross_section = _load_cross_section(
            args.input,
            axis=args.slice_axis,
            level=args.slice_level,
        )
        if not cross_section:
            print(
                "Warning: cross-section is empty after filtering air blocks.",
                file=sys.stderr,
            )

        blocks_dict = assemble_curve(
            control_points=control_points,
            radius=args.radius,
            cross_section=cross_section,
            cross_section_width=args.width,
            elevation_control_points=elevation_points,
            step_size=args.step_size,
            symmetrical=args.symmetrical,
        )
    else:
        # ── Path-only mode ──────────────────────────────────────────────
        if len(control_points) < 2:
            parser.error("At least 2 control points are required in the JSON file.")

        blocks_dict = assemble_curve_path(
            control_points=control_points,
            radius=args.radius,
            elevation_control_points=elevation_points,
            step_size=args.step_size,
        )

        if args.block != "minecraft:stone":
            blocks_dict = _patch_block_type(blocks_dict, args.block)

    if not blocks_dict:
        print("Error: curve assembly produced no blocks.", file=sys.stderr)
        sys.exit(1)

    total = sum(len(v) for v in blocks_dict.values())
    print(f"Assembled {total} blocks ({len(blocks_dict)} palette entries)")

    _export(
        blocks_dict,
        args.output,
        origin=(0, 0, 0),
        dimension=args.dimension,
        data_version=args.data_version,
    )


if __name__ == "__main__":
    main()
