# Curve Generator

Generate Minecraft Sponge Schematic (`.schem`) files or write directly to world saves using [Dubins paths](https://en.wikipedia.org/wiki/Dubins_path). Define a route with headings and elevations, then export a paste-ready schematic complete with configurable track surfaces, pillars, and catenary supports.

## Features

- **Dubins-path routing** — smooth curves between waypoints with a configurable turning radius.
- **Elevation ramps** — linear elevation interpolation between control points.
- **Multiple cross-sections** — assign different road profiles (e.g., standard, bridge) to specific path segments.
- **Dynamic Structures** — automatically place pillars, catenary wires, and custom wire paths.
- **Auto-Resolution** — optionally resolve Minecraft rail shapes and block connectivity along the curve.
- **Flexible export** — output to `.schem` files or directly into a Minecraft world directory.

## Requirements

- Python 3.14+
- [NumPy](https://numpy.org/)
- [amulet-nbt](https://github.com/Amulet-Team/Amulet-NBT)

The project also depends on the local packages **`curveAssembly`**, **`advanceCurveAssembly`**, and **`util`** — ensure they are present in the project root.

## Installation

```bash
python3.14 -m venv venv
source venv/bin/activate   # Windows: venv\Scripts\activate
pip install numpy amulet-nbt

```

## Quick Start

```bash
# Advance mode — multiple cross-sections + structures
python main.py track.json -o output.schem --radius 20

# Single cross-section mode
python main.py -i road.schem -o output.schem --radius 20 --width 5 --dept 3 points.json

# Path-only with a custom block
python main.py -o output.schem --radius 20 --dept 3 --block minecraft:oak_planks points.json

```

## Operating Modes

The tool selects a mode automatically based on the arguments you provide:

| Mode                     | Trigger                              | Description                                                                 |
| ------------------------ | ------------------------------------ | --------------------------------------------------------------------------- |
| **Advance**              | JSON contains a `cross_sections` key | Multiple profiles with per-section structures (Pillars, Catenaries, Wires). |
| **Single cross-section** | `-i SCHEM` flag provided             | One schematic applied uniformly along the path.                             |
| **Path-only**            | Neither of the above                 | Renders the Dubins path itself as a simple block line.                      |

---

## CLI Reference

```bash
python main.py [JSON] [options]

```

### Options

| Flag                | Default           | Description                                                               |
| ------------------- | ----------------- | ------------------------------------------------------------------------- |
| `-i`, `--input`     | —                 | Cross-section `.schem` file (triggers single cross-section mode).         |
| `-o`, `--output`    | _(required)_      | Output `.schem` file or Minecraft world directory path.                   |
| `--radius`          | _(required)_      | Dubins turning radius in blocks.                                          |
| `--width`           | —                 | Cross-section width (required with `-i`).                                 |
| `--dept`            | —                 | Depth for backtrack calculation (required with `-i` or path-only).        |
| `--step-size`       | `1`               | Distance between points before stepping up/down ($y \pm 1$).              |
| `--block`           | `minecraft:stone` | Block type for path-only mode.                                            |
| `--symmetrical`     | off               | Mirror cross-section(s) across the path center.                           |
| `--resolve-rails`   | off               | Automatically resolve Minecraft rail shape properties (e.g., north_east). |
| `--separate-halves` | off               | Export mirror halves as separate files (symmetrical mode only).           |
| `--slice-axis`      | —                 | Axis (`x`, `y`, or `z`) to slice the input schematic on.                  |
| `--slice-level`     | —                 | Coordinate for the slice (required if `--slice-axis` is set).             |
| `--dimension`       | `overworld`       | Target dimension for world export (`overworld`, `nether`, `end`).         |
| `--data-version`    | `3700`            | Minecraft data version (default 1.20.4+).                                 |

---

## Input JSON Format

### Basic Pathing

```json
{
	"control_points": [
		[0, 0, 0],
		[750, 500, 60]
	],
	"elevation_points": [
		[0, 0, 64],
		[750, 500, 72]
	]
}
```

- **`control_points`**: `[x, z, heading_degrees]`.
- **`elevation_points`**: `[x, z, y]`. Elevation is linearly interpolated between pairs.

### Structure Types (Advance Mode)

Structures are defined in a top-level `"structures"` list. Every structure must reference one or more `cross_sections`.

#### 1. Pillar

Supporting columns placed at regular intervals. Provide three base-angle schematics (0°, 22.5°, 45°); the tool derives all 16 compass orientations automatically.

```json
{
	"type": "pillar",
	"cross_section": "bridge",
	"0": "pillar_0.schem",
	"22.5": "pillar_22.schem",
	"45": "pillar_45.schem",
	"distance": 20
}
```

#### 2. Catenary

Overhead wire supports. Requires an ordered list of vertical slices from the pole (road edge) to the track intersection.

```json
{
	"type": "catenary",
	"cross_section": "main",
	"schematics": ["pole.schem", "slice1.schem", "slice_track.schem"],
	"catenary_interval": 25,
	"base_width": 13,
	"track_width": 7
}
```

#### 3. Wire

Stamps a cross-section along the wire path. By default, it uses catenary intersection points as the generation base.

```json
{
	"type": "wire",
	"cross_sections": ["main", "bridge"],
	"schematic": "wire_core.schem",
	"use_step_line": true,
	"resolve_blocks": true,
	"override_catenary": false
}
```

- **`use_step_line`**: True for axis-aligned stepping; False for diagonal Bresenham.
- **`resolve_blocks`**: Auto-resolves connectivity (like rail shapes) along the wire.

---

## Exporting

The tool automatically detects the export format:

- **`.schem` extension**: Generates a Sponge Schematic. The output is shifted so the path origin is at the relative coordinate $(0, 0, 0)$ of the schematic.
- **No extension (Directory)**: Writes blocks directly into a Minecraft world save folder. Requires `--dimension` to be set correctly.
