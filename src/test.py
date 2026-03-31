import argparse
import json
import numpy as np

from util.WorldUtil.surfaceUtil import get_surface
from util.CoreUtil.curveUtil import draw_path, draw_path_silhouette
from util.CoreUtil.elevationUtil import generate_elevation_lookup


def _norm(v):
    return int(round(float(v)))


def assign_cross_sections(egg_path, world_path, threshold=10, radius=300):
    """
    Reads egg.json, generates the path, queries the Minecraft world surface,
    compares surface Y to the planned elevation Y along the path centerline,
    and reassigns the cross_section "points" so that:
      - segments where abs(elevation_y - surface_y) < threshold  → "standard"
      - segments where abs(elevation_y - surface_y) >= threshold → "bridge"

    Every coordinate written into "points" is a real path coordinate.
    Segments are contiguous: the end coord of segment N is the start coord
    of segment N+1, which is what advanceCurveAssembly expects.

    Only cross_sections.*.points are touched — structure entries are left
    unchanged because their "points" fields serve different purposes (e.g.
    wire mask ranges).
    """

    with open(egg_path, "r") as f:
        egg = json.load(f)

    control_points = [(pt[0], pt[1], np.radians(pt[2])) for pt in egg["control_points"]]
    elevation_points = [tuple(pt) for pt in egg["elevation_points"]]

    # --- 1. Generate the full path (same as advanceCurveAssembly) ---
    raw_path, _, _ = draw_path(control_points, radius)
    norm_path = [(_norm(pt[0]), _norm(pt[1])) for pt in raw_path]

    # --- 2. Build the silhouette with the widest cross section ---
    max_width = max(cs["width"] for cs in egg["cross_sections"].values())
    silhouette = draw_path_silhouette(
        norm_path, control_points[0], control_points[-1], max_width
    )

    # --- 3. Build the elevation lookup (planned Y along path) ---
    elev_lut = generate_elevation_lookup(
        norm_path,
        max_width,
        silhouette,
        step_size=1,
        elevation_control_points=elevation_points,
    )

    # --- 4. Get the actual Minecraft surface Y for path centerline ---
    unique_path_points = list(dict.fromkeys(norm_path))
    surface_y_map = get_surface(world_path, unique_path_points)

    # --- 5. Classify each path point ---
    classifications = []
    for px, pz in norm_path:
        planned_y = elev_lut.get((px, pz))
        surface_y = surface_y_map.get((px, pz))

        if planned_y is None or surface_y is None:
            # Default to standard when data is unavailable
            classifications.append("standard")
            continue

        diff = float(planned_y) - float(surface_y)
        classifications.append("bridge" if diff >= threshold else "standard")

    # --- 6. Build contiguous segments using path coordinates ---
    if not classifications:
        return []

    segments = []  # (start_coord, end_coord, section_name)
    current_section = classifications[0]
    segment_start_coord = list(norm_path[0])

    for i in range(1, len(classifications)):
        if classifications[i] != current_section:
            transition_coord = list(norm_path[i])
            segments.append((segment_start_coord, transition_coord, current_section))
            current_section = classifications[i]
            segment_start_coord = transition_coord

    # Close the final segment with the last path coordinate
    segments.append((segment_start_coord, list(norm_path[-1]), current_section))

    # --- 7. Build point pairs per cross section ---
    standard_points = []
    bridge_points = []

    for start_coord, end_coord, name in segments:
        if start_coord == end_coord:
            continue
        point_pair = [start_coord, end_coord]
        if name == "standard":
            standard_points.append(point_pair)
        else:
            bridge_points.append(point_pair)

    # Only update the cross_sections "points" — main.py reads these and
    # converts them to point_pairs for advanceCurveAssembly.
    # Structure entries (catenary, wire, pillar) are NOT touched.
    egg["cross_sections"]["standard"]["points"] = standard_points
    egg["cross_sections"]["bridge"]["points"] = bridge_points

    # --- 8. Write the updated egg.json ---
    with open(egg_path, "w") as f:
        json.dump(egg, f, indent="\t")

    return segments


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Analyze surface elevation along a curve path and assign "
            "cross sections (standard / bridge) based on the height "
            "difference between the planned elevation and the Minecraft "
            "world surface."
        )
    )
    parser.add_argument(
        "world",
        type=str,
        help="Path to the Minecraft world directory",
    )
    parser.add_argument(
        "egg",
        type=str,
        help="Path to the egg.json configuration file",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=10,
        help=(
            "Y-difference threshold: abs(elevation - surface) < threshold "
            "-> standard, >= threshold -> bridge (default: 10)"
        ),
    )
    parser.add_argument(
        "--radius",
        type=float,
        default=300,
        help="Turning radius for Dubins path generation (default: 300)",
    )

    args = parser.parse_args()

    try:
        segments = assign_cross_sections(
            args.egg, args.world, args.threshold, args.radius
        )

        print(f"Path analyzed. {len(segments)} segment(s) classified:")
        for start_coord, end_coord, name in segments:
            print(f"  {start_coord} -> {end_coord}  [{name}]")

        std_count = sum(1 for _, _, n in segments if n == "standard")
        brg_count = sum(1 for _, _, n in segments if n == "bridge")
        print(f"\n  {std_count} standard, {brg_count} bridge segment(s)")
        print(f"Updated cross-section points written back to {args.egg}")

    except Exception as e:
        import traceback

        traceback.print_exc()
        print(f"Error: {e}")


if __name__ == "__main__":
    main()
