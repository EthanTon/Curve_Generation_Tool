import argparse
import math
import os
import sys

from auto.parser import parse_yaml, load_points
from auto.logic import (
    ConditionEvaluator,
    classify_path,
    generate_path_standalone,
    interpolate_elevation,
    resolve_structure_assignments,
    generate_path,
    generate_surface_region,
    write_surface_file,
)
from auto.output import create_json, write_json


def _read_yaml(file_path):
    return parse_yaml(file_path)


def _try_generate_path_full(control_points_rad, elevation_points, radius, max_width):
    try:

        result = generate_path(control_points_rad, elevation_points, radius, max_width)
        return result["path"], result["elev_lut"], result.get("silhouette")
    except ImportError:
        return None


def _try_get_surface(world_path, path_points):
    try:
        from auto.logic import get_surface_map

        return get_surface_map(world_path, path_points)
    except ImportError:
        return None


def auto(yml_path, output_path, world_path=None, use_curve_util=False):
    config = _read_yaml(yml_path)
    curve_cfg = config["curve"]
    surface_cfg = config["surface"]

    ctrl_raw, elev_raw = load_points(curve_cfg["points_path"])
    radius = curve_cfg["radius"]

    if len(ctrl_raw) < 2:
        print("Error: at least 2 control points required.", file=sys.stderr)
        sys.exit(1)

    ctrl_rad = [(pt[0], pt[1], math.radians(pt[2])) for pt in ctrl_raw]
    elev_tuples = [tuple(pt) for pt in elev_raw]

    max_width = 1
    for sec in config["sections"]:
        for cs in sec["cross_sections"]:
            w = cs["vars"].get("width", 0)
            if w > max_width:
                max_width = w

    path = None
    elev_lut = None
    silhouette = None

    if use_curve_util:
        result = _try_generate_path_full(ctrl_rad, elev_tuples, radius, max_width)
        if result:
            path, elev_lut, silhouette = result
            print(f"Generated Dubins path: {len(path)} points")

    if path is None:
        path = generate_path_standalone(ctrl_raw)
        elev_lut = interpolate_elevation(path, elev_raw)
        print(f"Generated linear path (standalone): {len(path)} points")

    surface_map = None
    effective_world = world_path or (surface_cfg["path"] if surface_cfg else None)
    if effective_world:
        surface_map = _try_get_surface(effective_world, path)
        if surface_map:
            print(f"Loaded surface data: {len(surface_map)} points")
        else:
            print(
                "Warning: could not load surface data " "(surfaceUtil not available).",
                file=sys.stderr,
            )

    lenient = surface_map is None
    segments = classify_path(path, config, elev_lut, surface_map, lenient=lenient)
    if lenient:
        print(
            "Note: no surface data — using lenient mode "
            "(first matching cross_section per group wins)."
        )
    print(f"Classified {len(segments)} segment(s):")
    for start, end, sec_name, cs_name in segments:
        print(f"  {start} -> {end}  [{sec_name} / {cs_name}]")

    struct_assignments = resolve_structure_assignments(segments, config)
    print(f"Resolved {len(struct_assignments)} structure assignment(s)")

    surface_file_path = None
    if effective_world:
        region = generate_surface_region(
            path, segments, config, max_width, silhouette
        )
        # Fall back to full path silhouette if no cross sections declare
        # use_y_min / use_y_max (so the surface file is always produced).
        if not region:
            if silhouette:
                region = silhouette
            else:
                from auto.logic import _approx_silhouette
                region = _approx_silhouette(path, max_width)
        if region:
            surf_data = _try_get_surface(effective_world, list(region))
            if surf_data:
                surface_file_path = os.path.splitext(output_path)[0] + "_surface.json"
                write_surface_file(surf_data, surface_file_path)
                print(f"Wrote surface file: {len(surf_data)} points -> {surface_file_path}")

    file = create_json(config, ctrl_raw, elev_raw, segments, struct_assignments, surface_file_path)
    write_json(file, output_path)
    print(f"\nWrote {output_path}")

    return file

def build_parser():
    p = argparse.ArgumentParser(
        description=(
            "Generate an egg.json-compatible config from a YAML definition. "
            "The output JSON can be passed directly to main.py's advance mode."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  python auto.py sample.yml -o sample_out.json\n"
            "  python auto.py sample.yml -o sample_out.json --world ./my_world/\n"
            "  python auto.py sample.yml -o sample_out.json --use-curve-util\n"
        ),
    )
    p.add_argument(
        "config",
        metavar="YAML",
        help="Path to the YAML configuration file.",
    )
    p.add_argument(
        "-o",
        "--output",
        required=True,
        metavar="JSON",
        help="Output JSON file path.",
    )
    p.add_argument(
        "--world",
        default=None,
        metavar="DIR",
        help="Minecraft world directory for surface queries.",
    )
    p.add_argument(
        "--use-curve-util",
        action="store_true",
        help=(
            "Use the full curveUtil library for Dubins path generation. "
            "Requires the util package on sys.path."
        ),
    )
    return p


def main():
    parser = build_parser()
    args = parser.parse_args()

    if not os.path.isfile(args.config):
        print(f"Error: config file not found: {args.config}", file=sys.stderr)
        sys.exit(1)

    auto(
        args.config,
        args.output,
        world_path=args.world,
        use_curve_util=args.use_curve_util,
    )

main()