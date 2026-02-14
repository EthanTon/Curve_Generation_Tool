# Curve Generation Tool

Generates Minecraft Sponge Schematic (`.schem`) files from a set of control points using Dubins paths. Define a route with headings, optionally add elevation ramps, and export a paste-ready schematic with a configurable track surface, base, brim, and catenary supports.

## Setup (Python 3.11.5)

```bash
python3.11 -m venv venv
source venv/bin/activate   # Windows: venv\Scripts\activate
pip install numpy
```

> The project also depends on the local packages `trackAssembler` and `util` — make sure they are present in the project root.

## Usage

```bash
# Full track (default)
python main.py sample.json

# Export with a custom filename
python main.py sample.json -o my_track.schem

# Dubins path only (no track surface/base)
python main.py sample.json --path-only

# Path only, ignoring elevation data
python main.py sample.json --path-only --no-elevation
```

### CLI Arguments

| Argument | Description |
|---|---|
| `input` | Path to the JSON file containing control points. |
| `-o`, `--output` | Override the export filename (takes priority over the JSON `output` field). |
| `--path-only` | Export only the Dubins path as blocks instead of the full track. |
| `--no-elevation` | Ignore elevation data (only meaningful with `--path-only`). |

## Input JSON Format

See `sample.json` for a working example.

```jsonc
{
  "control_points": [         // Required – [x, z, angle_degrees]
    [0, 0, 0],
    [300, 150, 45],
    [400, 300, 45]
  ],
  "elevation": [              // Optional – [x, z, y_elevation]
    [0, 0, 0],
    [300, 150, 6]
  ],
  "config": {                 // Optional – overrides defaults
    "base_width": 13,
    "track_width": 7,
    "turn_radius": 150,
    "catenary_interval": 35,
    "catenary_offset": 0,
    "base_block": "minecraft:gray_wool",
    "brim_block": "minecraft:gray_wool",
    "elevation_step_size": 35
  },
  "output": "testtrack112.schem"  // Optional – default is output.schem
}
```

- **control_points** define the route. Each entry is `[x, z, heading°]` where the heading is converted to radians internally.
- **elevation** entries define linear ramps between consecutive pairs along the centre path.
- **config** lets you tweak track dimensions, block types, and catenary spacing. Any omitted key falls back to the built-in default.
- **output** sets the filename (overridden by the `-o` CLI flag).
