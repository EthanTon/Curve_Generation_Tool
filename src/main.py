#!/usr/bin/env python3
"""
main.py – CLI for assembling Minecraft curves.

Usage examples:
  # Single cross-section (original mode)
  python main.py -i road.schem -o output.schem --radius 20 --width 5 --dept 3 points.json

  # Advance mode – multiple cross-sections + structures defined in JSON
  python main.py -o output.schem --radius 20 advance.json

  # Path-only with custom block
  python main.py -o output.schem --radius 20 --dept 3 --block minecraft:oak_planks points.json

Mode selection
--------------
  -i SCHEM + JSON         → original assemble_curve (single cross-section)
  JSON with cross_sections key → advance assemble_advance_curve
  neither                      → path-only

Expected JSON — simple (points only):
{
    "control_points": [[x, z, angle_degrees], ...],
    "elevation_points": [[x, z, y], ...]
}

Expected JSON — advance (multi cross-section + structures):
{
    "control_points": [[x, z, angle_degrees], ...],
    "elevation_points": [[x, z, y], ...],
    "cross_sections": {
        "road": {
            "schematic": "road.schem",
            "width": 5,
            "dept": 3,
            "points": [[[x1, z1], [x2, z2]], ...]
        },
        "bridge": {
            "schematic": "bridge.schem",
            "width": 8,
            "dept": 4,
            "slice_axis": "y",
            "slice_level": 64,
            "points": [[[x1, z1], [x2, z2]], ...]
        }
    },
    "structures": [
        {
            "type": "pillar",
            "schematic": "pillar.schem",
            "cross_section_width": 5,
            "dept": 3,
            "pillar_thickness": 2,
            "pillar_distance": 20,
            "centered": false
        }
    ]
}

Each cross_sections entry names a schematic file (resolved relative to the
JSON file), per-section "width" and "dept" values, and a list of point-pairs
marking where that cross-section is active on the path.  Optional slice_axis /
slice_level work identically to the --slice-axis / --slice-level CLI flags.

Structures are passed through to advanceCurveAssembly.  Any entry with a
"schematic" key gets its cross_section loaded automatically before dispatch.
"""

import argparse
import json
import os
import sys

import numpy as np

from curveAssembly import assemble_curve, assemble_curve_path
from advanceCurveAssembly import assemble_advance_curve
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


def _load_json(path):
    """Load and return the full JSON dict."""
    with open(path, "r") as f:
        return json.load(f)


def _parse_points(data):
    """Extract control_points and elevation_points from a JSON dict."""
    raw_control = data.get("control_points", [])
    control = [(pt[0], pt[1], np.radians(pt[2])) for pt in raw_control]
    elevation = [tuple(pt) for pt in data.get("elevation_points", [])]
    return control, elevation


def _resolve_path(json_dir, filename):
    """Resolve a filename relative to the JSON file's directory."""
    if os.path.isabs(filename):
        return filename
    return os.path.join(json_dir, filename)


def _apply_offset(blocks_dict, offset):
    """Shift every position in *blocks_dict* by *offset*."""
    ox, oy, oz = offset
    return {
        block: [(x + ox, y + oy, z + oz) for x, y, z in positions]
        for block, positions in blocks_dict.items()
    }


def _load_cross_section(schematic_path, axis=None, level=None):
    """Import a schematic and return paste-origin-relative cross-section offsets.

    When *axis*/*level* are provided the schematic is sliced first.
    """
    raw_blocks, dims, offset = read_schematic(schematic_path)

    blocks_dict = defaultdict(list)
    for lx, ly, lz, block_str in raw_blocks:
        blocks_dict[block_str].append((lx, ly, lz))
    blocks_dict = dict(blocks_dict)

    if axis is None:
        return _apply_offset(parse_cross_section(blocks_dict), offset)

    slice_funcs = {
        "x": cross_section_at_x,
        "y": cross_section_at_y,
        "z": cross_section_at_z,
    }
    if axis not in slice_funcs:
        raise ValueError(f"Invalid axis '{axis}'. Use 'x', 'y', or 'z'.")
    if level is None:
        raise ValueError("slice_level is required when slice_axis is set.")

    slice_2d = slice_funcs[axis](blocks_dict, level, offset=offset)
    return to_curve_offsets(slice_2d, copy_point=(0, 0), axis=axis)


def _load_advance_cross_sections(cs_config, json_dir):
    """Load all cross-sections and point-pairs from the JSON cross_sections block.
    Returns cross_sections dict keyed by name, each containing
    'cross_section', 'width', 'dept', and 'point_pairs'.
    """
    cross_sections = {}
    for name, entry in cs_config.items():
        schem_path = _resolve_path(json_dir, entry["schematic"])
        cs_data = _load_cross_section(
            schem_path,
            axis=entry.get("slice_axis"),
            level=entry.get("slice_level"),
        )
        if not cs_data:
            print(
                f"Warning: cross-section '{name}' is empty after filtering air.",
                file=sys.stderr,
            )
        if "width" not in entry:
            raise ValueError(
                f"Cross-section '{name}' is missing required 'width' field."
            )
        if "dept" not in entry:
            raise ValueError(
                f"Cross-section '{name}' is missing required 'dept' field."
            )
        cross_sections[name] = {
            "cross_section": cs_data,
            "width": entry["width"],
            "dept": entry["dept"],
            "point_pairs": [
                (tuple(pair[0]), tuple(pair[1])) for pair in entry["points"]
            ],
        }
    return cross_sections


def _load_advance_structures(struct_configs, json_dir):
    """Resolve schematic paths in structure configs, loading cross-sections.

    Returns a list of structure dicts ready for assemble_advance_curve.
    """
    resolved = []
    for cfg in struct_configs:
        cfg = dict(cfg)  # shallow copy
        if "schematic" in cfg:
            schem_path = _resolve_path(json_dir, cfg.pop("schematic"))
            cfg["cross_section"] = _load_cross_section(
                schem_path,
                axis=cfg.pop("slice_axis", None),
                level=cfg.pop("slice_level", None),
            )
        resolved.append(cfg)
    return resolved


def _is_world_dir(path):
    if os.path.isdir(path):
        return True
    _, ext = os.path.splitext(path)
    return ext == ""


def _compute_export_offset(blocks_dict, path_origin):
    all_pos = [pos for positions in blocks_dict.values() for pos in positions]
    min_x = min(p[0] for p in all_pos)
    min_y = min(p[1] for p in all_pos)
    min_z = min(p[2] for p in all_pos)
    ox, oy, oz = path_origin
    return (min_x - ox, min_y - oy, min_z - oz)


def _export(
    blocks_dict,
    output_path,
    path_origin=(0, 0, 0),
    dimension="overworld",
    data_version=3700,
):
    if _is_world_dir(output_path):
        count = export_world(blocks_dict, output_path, dimension=dimension)
        print(f"Placed {count} blocks in world: {output_path} ({dimension})")
    else:
        offset = _compute_export_offset(blocks_dict, path_origin)
        export_schematic(
            blocks_dict, filename=output_path, offset=offset, data_version=data_version
        )
        print(
            f"Saved schematic: {output_path} (path origin: {path_origin}, offset: {offset})"
        )


def _patch_block_type(blocks_dict, block_type):
    merged = []
    for positions in blocks_dict.values():
        merged.extend(positions)
    return {block_type: merged}


# ── CLI ─────────────────────────────────────────────────────────────────


def build_parser():
    p = argparse.ArgumentParser(
        description="Assemble a Minecraft curve and export it.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Modes:\n"
            "  -i SCHEM + JSON              single cross-section (original)\n"
            "  JSON with 'cross_sections'   advance multi-cross-section\n"
            "  neither                       path-only\n"
        ),
    )

    p.add_argument(
        "points",
        nargs="?",
        default=None,
        metavar="JSON",
        help="JSON file with control/elevation points (and optionally cross_sections + structures).",
    )

    # I/O
    p.add_argument(
        "-i",
        "--input",
        default=None,
        metavar="SCHEM",
        help="Cross-section schematic for original single-section mode.",
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
        help="Cross-section width (required with -i, ignored in advance mode).",
    )
    p.add_argument(
        "--dept",
        type=int,
        default=None,
        help="Depth value for backtrack calculation (required with -i, ignored in advance mode).",
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
        help="Mirror cross-section(s) across the path center.",
    )
    p.add_argument(
        "--resolve-rails", action="store_true", help="Resolve rail shape properties."
    )
    p.add_argument(
        "--separate-halves",
        action="store_true",
        help="Export each mirror half separately (symmetrical mode only).",
    )

    # path-only
    p.add_argument(
        "--block",
        default="minecraft:stone",
        metavar="BLOCK",
        help="Block type for path-only mode.",
    )

    # slicing (original mode only)
    p.add_argument(
        "--slice-axis",
        choices=["x", "y", "z"],
        default=None,
        help="Axis to slice the cross-section schematic on (original mode).",
    )
    p.add_argument(
        "--slice-level",
        type=int,
        default=None,
        help="Slice coordinate (requires --slice-axis).",
    )

    # export
    p.add_argument(
        "--dimension",
        default="overworld",
        choices=["overworld", "nether", "end"],
        help="Target dimension for world export.",
    )
    p.add_argument(
        "--data-version",
        type=int,
        default=3700,
        help="Minecraft data version for .schem export.",
    )

    return p


# ── main ────────────────────────────────────────────────────────────────


def main():
    parser = build_parser()
    args = parser.parse_args()

    # Load JSON
    data = {}
    json_dir = "."
    if args.points:
        data = _load_json(args.points)
        json_dir = os.path.dirname(os.path.abspath(args.points))

    control_points, elevation_points = _parse_points(data)
    has_advance = "cross_sections" in data

    # ── Advance mode ────────────────────────────────────────────────
    if has_advance:
        if args.input:
            parser.error(
                "Cannot combine -i with a JSON that has 'cross_sections'. Use one or the other."
            )
        if len(control_points) < 2:
            parser.error("Advance mode requires at least 2 control points.")

        cross_sections = _load_advance_cross_sections(
            data["cross_sections"],
            json_dir,
        )
        structures = _load_advance_structures(data.get("structures", []), json_dir)

        blocks_dict, path_origin = assemble_advance_curve(
            control_points=control_points,
            radius=args.radius,
            cross_sections=cross_sections,
            elevation_control_points=elevation_points,
            step_size=args.step_size,
            symmetrical=args.symmetrical,
            resolve_rails=args.resolve_rails,
        )
        halves_list = None

    # ── Original single cross-section mode ──────────────────────────
    elif args.input:
        if len(control_points) < 2:
            parser.error("Cross-section mode requires at least 2 control points.")
        if args.width is None:
            parser.error("--width is required when using a cross-section (-i).")
        if args.dept is None:
            parser.error("--dept is required when using a cross-section (-i).")

        cross_section = _load_cross_section(
            args.input, axis=args.slice_axis, level=args.slice_level
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
        )

        if args.separate_halves:
            blocks_dict, path_origin, halves_list = curve_result
        else:
            blocks_dict, path_origin = curve_result
            halves_list = None

    # ── Path-only mode ──────────────────────────────────────────────
    else:
        if len(control_points) < 2:
            parser.error("At least 2 control points are required.")
        if args.dept is None:
            parser.error("--dept is required in path-only mode.")

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
            print(
                f"  Half {i}: {sum(len(v) for v in half_dict.values())} blocks -> {half_path}"
            )


if __name__ == "__main__":
    main()
