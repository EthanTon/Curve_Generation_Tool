import util.dubinsUtil as dubinUtil
import util.trackUtil as trackUtil
import util.elevationUtil as elevUtil
import util.exportUtil as exportUtil
import numpy as np


def parse_segments(segments, radius=30.0, base_width=13, track_width=7, slope_min=35):
    results = []

    for segment in segments:
        start = segment["start"]
        end = segment["end"]

        center_track = dubinUtil.int_dubins_path(start[0], end[0], radius)
        base, track = trackUtil.track(
            center_track, start[0], end[0], base_width, track_width
        )
        elevation_mask = elevUtil.generate_elevation_mask(
            center_track, base_width, list(base[0]), slope_min, 0, -1, segment["z_start"]
        )

        results.append(
            {
                "start": start,
                "end": end,
                "center_track": center_track,
                "base": base,
                "track": track,
                "elevation_mask": elevation_mask,
            }
        )

    return results


def main():
    segments = [
        # --- Uphill ---
        {
            "start": [(0, 0, 0), 0],
            "end": [(0, 70, np.pi / 3), 11],
            "z_start": 0,
            "label": "uphill_gentle",
        },
        {
            "start": [(0, 70, np.pi / 3), 11],
            "end": [(60, 120, np.pi / 2), 12],
            "z_start": 11,
            "label": "uphill_steep",
        },
    ]

    results = parse_segments(
        segments, radius=30.0, base_width=13, track_width=7, slope_min=35
    )

    # Merge all segments into combined sets
    combined_base = set()
    combined_brim = set()
    combined_elevation = {}

    for i, res in enumerate(results):
        base_set, brim_set = res["base"]
        combined_base.update(map(tuple, base_set))
        combined_brim.update(map(tuple, brim_set))

        # Merge elevation mask — later segments overwrite overlapping points
        for seg_idx, point_map in res["elevation_mask"].items():
            for pt, y in point_map.items():
                combined_elevation[pt] = y

        print(f"Segment {i}: center_track shape={np.array(res['center_track']).shape}")

    # Wrap combined elevation into the format export_base_brim expects
    merged_elevation_mask = {0: combined_elevation}

    exportUtil.export_base_brim(
        combined_base,
        combined_brim,
        merged_elevation_mask,
        filename="track.schem",
    )

    print(f"Exported {len(combined_base)} blocks to track.schem")


main()