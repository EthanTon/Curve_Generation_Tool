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
        "main": {
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
            "cross_section": "bridge",
            "0": "pillar_0.schem",
            "22.5": "pillar_22.schem",
            "45": "pillar_45.schem",
            "distance": 20
        },
        {
            "type": "catenary",
            "cross_section": "main",
            "schematics": ["slice_pole.schem", "slice_1.schem", "...", "slice_track.schem"],
            "catenary_interval": 20,
            "base_width": 13,
            "track_width": 5,
            "offset": 0
        },
        {
            "type": "wire",
            "cross_sections": ["main", "bridge"],
            "schematic": "wire.schem",
            "points": [[[0, 0], [750, 500]]],
            "use_step_line": true,
            "resolve_blocks": true,
            "override_catenary": false
        }
    ]
}

Every cross-section — including "main" — must declare explicit "points"
(point-pairs) to define which path segments it covers.  There is no implicit
default; uncovered segments are simply skipped.

Structures are declared in a top-level "structures" list.  Each entry must
contain "type" and either "cross_section" (a single name) or "cross_sections"
(a name or list of names) referencing cross_sections entries.  A structure
with multiple cross-sections is generated once for each referenced
cross-section.

Pillar structures supply three schematics keyed by base angle ("0", "22.5",
"45").  All 16 orientations (0° through 337.5° in 22.5° steps) are derived
from these three via 90° rotation and mirroring.

Catenary structures supply an ordered list of schematics ("schematics") where
each file is a vertical cross-section slice.  Index 0 is at the pole (road
edge) and the last index is at the track intersection (near centre).  The
number of slices must equal the cantilever length
(base_width // 2 - track_width // 2 + 1).  "base_width" sets the distance
between poles across the path and "track_width" is the width of each track
half.  Each slice is defined as left at 0 degrees and facing centre; track-2
(right side) slices are automatically mirrored.

Wire structures supply a single schematic ("schematic") whose cross-section
is stamped along the wire path.  Wires use catenary intersection points as
their generation base — lines are drawn between consecutive intersection
points on each track.  The optional "points" field is a list of point-pairs
(same format as cross-section points) that act as a mask: only catenary
intersections whose pole position falls within the masked path segments are
used.  When "points" is omitted the wire generates across the full
cross-section coverage.  When no catenary data exists for the referenced
cross-section, the wire falls back to stamping directly on rail block
positions.  Additional wire options:
    "use_step_line": true/false (default true)  — true for axis-aligned
        stepping (step_line), false for diagonal Bresenham (bresenham_line).
    "resolve_blocks": true/false (default true) — auto-resolve block
        connectivity (e.g. rail shapes) so connected blocks match.
    "override_catenary": true/false (default false) — when false, wire
        blocks that overlap catenary positions are removed.
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


_PILLAR_ANGLE_KEYS = (0, 22.5, 45)


def _load_structure(cfg, json_dir):
    """Resolve a single structure config, loading its schematic(s).

    Every structure must include ``"type"`` and ``"cross_section"`` (a name
    or list of names) referencing cross_sections entries.

    Returns a structure dict ready for advanceCurveAssembly.
    """
    cfg = dict(cfg)  # shallow copy

    # Normalise: accept "cross_section" (str) or "cross_sections" (str/list)
    raw = cfg.get("cross_sections", cfg.get("cross_section"))
    if raw is None:
        raise ValueError(
            "Every structure must have a 'cross_section' or 'cross_sections' "
            "field referencing cross_sections entries."
        )
    cs_names = [raw] if isinstance(raw, str) else list(raw)

    if cfg.get("type") == "pillar":
        pillar_sections = {}
        for angle in _PILLAR_ANGLE_KEYS:
            str_key = str(angle)
            if str_key not in cfg:
                str_key = str(int(angle)) if angle == int(angle) else str(angle)
            if str_key not in cfg:
                raise ValueError(
                    f"Pillar structure missing required schematic for angle {angle}."
                )
            schem_path = _resolve_path(json_dir, cfg.pop(str_key))
            pillar_sections[angle] = _load_cross_section(schem_path)

        return {
            "type": "pillar",
            "cross_sections": cs_names,
            "pillar_sections": pillar_sections,
            "distance": cfg["distance"],
        }

    if cfg.get("type") == "catenary":
        schematics = cfg.pop("schematics", [])
        if not schematics:
            raise ValueError(
                "Catenary structure requires a 'schematics' list of schematic files "
                "(ordered from pole to track intersection)."
            )
        if "base_width" not in cfg:
            raise ValueError(
                "Catenary structure requires 'base_width' (distance between poles "
                "across the path)."
            )
        if "track_width" not in cfg:
            raise ValueError(
                "Catenary structure requires 'track_width' (width of each track)."
            )
        slice_axis = cfg.pop("slice_axis", None)
        slice_level = cfg.pop("slice_level", None)
        catenary_cross_section = []
        for schem_file in schematics:
            schem_path = _resolve_path(json_dir, schem_file)
            catenary_cross_section.append(
                _load_cross_section(schem_path, axis=slice_axis, level=slice_level)
            )
        return {
            "type": "catenary",
            "cross_sections": cs_names,
            "catenary_cross_section": catenary_cross_section,
            "catenary_interval": cfg["catenary_interval"],
            "base_width": cfg["base_width"],
            "track_width": cfg["track_width"],
            "offset": cfg.get("offset", 0),
        }

    if cfg.get("type") == "wire":
        if "schematic" not in cfg:
            raise ValueError(
                "Wire structure requires a 'schematic' for the wire cross-section."
            )
        schem_path = _resolve_path(json_dir, cfg["schematic"])
        wire_cs = _load_cross_section(
            schem_path,
            axis=cfg.get("slice_axis"),
            level=cfg.get("slice_level"),
        )
        # "points" are mask point-pairs (same format as cross-section points)
        mask_point_pairs = None
        raw_points = cfg.get("points")
        if raw_points:
            mask_point_pairs = [(tuple(pair[0]), tuple(pair[1])) for pair in raw_points]
        return {
            "type": "wire",
            "cross_sections": cs_names,
            "wire_cross_section": wire_cs,
            "mask_point_pairs": mask_point_pairs,
            "use_step_line": cfg.get("use_step_line", True),
            "resolve_blocks": cfg.get("resolve_blocks", True),
            "override_catenary": cfg.get("override_catenary", False),
        }

    # Non-pillar/catenary/wire structures: fall back to single-schematic loading
    if "schematic" in cfg:
        schem_path = _resolve_path(json_dir, cfg.pop("schematic"))
        cfg["cross_sections"] = cs_names
        cfg["cross_section_data"] = _load_cross_section(
            schem_path,
            axis=cfg.pop("slice_axis", None),
            level=cfg.pop("slice_level", None),
        )
    return cfg


def _load_advance_cross_sections(cs_config, json_dir):
    """Load all cross-sections and point-pairs from the JSON cross_sections
    block.

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


def _load_structures(raw_structures, json_dir):
    """Load the top-level structures list from JSON.

    Returns a list of resolved structure dicts ready for
    advanceCurveAssembly.
    """
    if not raw_structures:
        return []
    return [_load_structure(s, json_dir) for s in raw_structures]


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

        structures = _load_structures(
            data.get("structures", []),
            json_dir,
        )

        blocks_dict, path_origin = assemble_advance_curve(
            control_points=control_points,
            radius=args.radius,
            cross_sections=cross_sections,
            structures=structures,
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
