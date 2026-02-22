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

Cross-section workflow
----------------------
The schematic's copy/paste origin (the player position when //copy was run)
acts as the *paste point* for each step along the curve.  Block offsets in the
cross-section are relative to that origin, exactly as WorldEdit would place
them with //paste.

During assembly every path point becomes the paste point.  Only at export
time are all coordinates shifted so that the **first control point** sits at
the schematic origin – meaning a WorldEdit //paste at your feet puts the
start of the curve right where you stand.

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
from collections import defaultdict

from util.CoreUtil.ioUtil import import_schematic, export_schematic, export_world
from util.SchematicUtil.schematicUtil import read_schematic
from util.CoreUtil.crossSectionUtil import (
    parse_cross_section,
    cross_section_at_x,
    cross_section_at_y,
    cross_section_at_z,
    to_curve_offsets,
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


def _apply_offset(blocks_dict, offset):
    """Shift every position in *blocks_dict* by *offset*.

    This converts local schematic coordinates into paste-origin-relative
    offsets – i.e. the displacement each block would have from the player
    position during a WorldEdit //paste.
    """
    ox, oy, oz = offset
    result = {}
    for block_str, positions in blocks_dict.items():
        result[block_str] = [(x + ox, y + oy, z + oz) for x, y, z in positions]
    return result


def _load_cross_section(schematic_path, axis=None, level=None):
    """Import a schematic and return a cross-section with paste-origin-relative
    offsets.

    Every position in the returned dict is expressed as an offset from the
    schematic's copy/paste origin, so stamping at a path point reproduces
    the same spatial relationship WorldEdit //paste would give.

    When *axis*/*level* are provided the schematic is sliced first (the
    level is specified in **world** coordinates, i.e. accounting for the
    schematic offset) and the resulting 2-D slice is converted into 3-D
    curve offsets via ``to_curve_offsets`` with the paste origin as the
    reference point.
    """
    raw_blocks, dims, offset = read_schematic(schematic_path)

    # Build a blocks_dict in local schematic coordinates.
    blocks_dict = defaultdict(list)
    for lx, ly, lz, block_str in raw_blocks:
        blocks_dict[block_str].append((lx, ly, lz))
    blocks_dict = dict(blocks_dict)

    if axis is None:
        # Full 3-D cross-section – convert local coords → paste-origin offsets.
        cross_section = parse_cross_section(blocks_dict)
        return _apply_offset(cross_section, offset)

    # ── Sliced cross-section ────────────────────────────────────────────
    slice_funcs = {
        "x": cross_section_at_x,
        "y": cross_section_at_y,
        "z": cross_section_at_z,
    }
    if axis not in slice_funcs:
        raise ValueError(f"Invalid axis '{axis}'. Use 'x', 'y', or 'z'.")
    if level is None:
        raise ValueError("--slice-level is required when --slice-axis is set.")

    # Slice functions accept local coords + offset and return 2-D positions
    # in world coordinates.  The paste origin in world coords is (0, 0, 0)
    # when expressed relative to itself, so copy_point = (0, 0).
    slice_2d = slice_funcs[axis](blocks_dict, level, offset=offset)
    return to_curve_offsets(slice_2d, copy_point=(0, 0), axis=axis)


def _is_world_dir(path):
    """Treat the output as a world directory if it is an existing dir or has
    no file extension."""
    if os.path.isdir(path):
        return True
    _, ext = os.path.splitext(path)
    return ext == ""


def _compute_export_offset(blocks_dict, path_origin):
    """Compute a schematic offset so that the first control point aligns with
    the player's paste position in WorldEdit.

    ``create_schematic`` normalises blocks to their minimum corner.  The
    offset stored in the .schem file compensates for that normalisation so
    that //paste places the *path_origin* at the player's feet.
    """
    all_pos = [pos for positions in blocks_dict.values() for pos in positions]
    min_x = min(p[0] for p in all_pos)
    min_y = min(p[1] for p in all_pos)
    min_z = min(p[2] for p in all_pos)
    ox, oy, oz = path_origin
    return (min_x - ox, min_y - oy, min_z - oz)


def _export(
    blocks_dict, output_path, path_origin=(0, 0, 0), dimension="overworld", data_version=3700
):
    if _is_world_dir(output_path):
        count = export_world(blocks_dict, output_path, dimension=dimension)
        print(f"Placed {count} blocks in world: {output_path} ({dimension})")
    else:
        offset = _compute_export_offset(blocks_dict, path_origin)
        export_schematic(
            blocks_dict, filename=output_path, offset=offset, data_version=data_version
        )
        print(f"Saved schematic: {output_path} (path origin: {path_origin}, offset: {offset})")


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
        "--dept",
        type=int,
        required=True,
        help="Depth value used in backtrack distance calculation (width - dept + 2).",
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
    p.add_argument(
        "--resolve-rails",
        action="store_true",
        help="Resolve rail shape properties based on neighbor connectivity.",
    )
    p.add_argument(
        "--side",
        choices=["left", "right"],
        default="left",
        help="Which side of the path the cross-section represents (default: left).",
    )
    p.add_argument(
        "--separate-halves",
        action="store_true",
        help="Export each mirror half as a separate schematic (symmetrical mode only). "
        "Produces <output>_half0.schem, <output>_half1.schem alongside the merged file.",
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

        # Cross-section offsets are now paste-origin-relative.
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

        curve_result = assemble_curve(
            control_points=control_points,
            radius=args.radius,
            cross_section=cross_section,
            cross_section_width=args.width,
            dept=args.dept,
            elevation_control_points=elevation_points,
            step_size=args.step_size,
            symmetrical=args.symmetrical,
            resolve_rails=args.resolve_rails,
            side=args.side,
        )

        if args.separate_halves:
            blocks_dict, path_origin, halves_list = curve_result
        else:
            blocks_dict, path_origin = curve_result
            halves_list = None
    else:
        # ── Path-only mode ──────────────────────────────────────────────
        if len(control_points) < 2:
            parser.error("At least 2 control points are required in the JSON file.")

        blocks_dict, path_origin = assemble_curve_path(
            control_points=control_points,
            radius=args.radius,
            elevation_control_points=elevation_points,
            step_size=args.step_size,
        )
        halves_list = None

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
        path_origin=path_origin,
        dimension=args.dimension,
        data_version=args.data_version,
    )

    # Export individual halves when requested
    if halves_list:
        base, ext = os.path.splitext(args.output)
        for i, half_dict in enumerate(halves_list):
            if not half_dict:
                continue
            half_path = f"{base}_half{i}{ext}"
            _export(
                half_dict,
                half_path,
                path_origin=path_origin,
                dimension=args.dimension,
                data_version=args.data_version,
            )
            half_total = sum(len(v) for v in half_dict.values())
            print(f"  Half {i}: {half_total} blocks -> {half_path}")


if __name__ == "__main__":
    main()