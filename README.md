# CurveGen: Minecraft Curve & Structure Assembler

Generate Minecraft Sponge Schematic (`.schem`) files or write directly to world saves using [Dubins paths](https://en.wikipedia.org/wiki/Dubins_path). Define a route with headings and elevations, then export a paste-ready schematic complete with configurable track surfaces, pillars, and overhead catenary supports.

## Features

- **Dubins-Path Routing:** Smooth curves between waypoints with a configurable turning radius.
- **Elevation Ramps:** Linear elevation interpolation between control points.
- **Multiple Cross-Sections:** Assign different road/track profiles (e.g., standard, bridge) to specific, discrete path segments.
- **Dynamic Structures:** Automatically place angled pillars, segmented catenary wires, and continuous custom wire paths.
- **Auto-Resolution:** Optionally resolve Minecraft rail shapes and block connectivity along the generated curves.
- **Flexible Export:** Output to `.schem` files or write blocks directly into a specified Minecraft world directory.

## Requirements

- Python 3.14+
- [NumPy](https://numpy.org/)
- [amulet-nbt](https://github.com/Amulet-Team/Amulet-NBT)

The project also depends on local packages (`curveAssembly`, `advanceCurveAssembly`, and `util`). Ensure they are present in your project root.

## Installation

```bash
python3.14 -m venv venv
source venv/bin/activate   # Windows: venv\Scripts\activate
pip install numpy amulet-nbt

```

## Quick Start

```bash
# Advance mode (Multiple cross-sections + structures defined in JSON)
python main.py advance.json -o output.schem --radius 20

# Single cross-section mode (Uniform profile)
python main.py points.json -i road.schem -o output.schem --radius 20 --width 5 --dept 3

# Path-only (No schematic, just plots the block path)
python main.py points.json -o output.schem --radius 20 --dept 3 --block minecraft:oak_planks

```

---

## Operating Modes

The CLI automatically infers your intended mode based on the arguments and the contents of your JSON file:

| Mode                     | Trigger                              | Description                                                                                                              |
| ------------------------ | ------------------------------------ | ------------------------------------------------------------------------------------------------------------------------ |
| **Advance**              | JSON contains a `cross_sections` key | Applies multiple profiles with per-section structures (Pillars, Catenaries, Wires) mapped to specific coordinate bounds. |
| **Single Cross-Section** | `-i SCHEM` flag is provided          | Applies a single `.schem` profile uniformly along the entire path length.                                                |
| **Path-Only**            | Neither of the above                 | Renders the Dubins path itself as a simple 1-block-wide line using a specified block.                                    |

---

## CLI Reference

```bash
python main.py [JSON] [options]

```

### Options

| Flag                | Default           | Description                                                                                 |
| ------------------- | ----------------- | ------------------------------------------------------------------------------------------- |
| `-i`, `--input`     | `None`            | Cross-section `.schem` file (triggers single cross-section mode).                           |
| `-o`, `--output`    | _(required)_      | Output `.schem` file or Minecraft world directory path.                                     |
| `--radius`          | _(required)_      | Dubins turning radius in blocks.                                                            |
| `--width`           | `None`            | Cross-section width (required with `-i`, ignored in Advance mode).                          |
| `--dept`            | `None`            | Depth for backtrack calculation (required with `-i` or path-only, ignored in Advance mode). |
| `--step-size`       | `1`               | Distance between points before stepping up/down ($y \pm 1$).                                |
| `--block`           | `minecraft:stone` | Block type used for Path-Only mode.                                                         |
| `--symmetrical`     | `False`           | Mirror cross-section(s) across the path center.                                             |
| `--resolve-rails`   | `False`           | Automatically resolve Minecraft rail shape properties (e.g., `north_east`).                 |
| `--separate-halves` | `False`           | Export mirror halves as separate files (symmetrical mode only).                             |
| `--slice-axis`      | `None`            | Axis (`x`, `y`, or `z`) to slice the input schematic on (single mode only).                 |
| `--slice-level`     | `None`            | Coordinate for the slice (required if `--slice-axis` is set).                               |
| `--dimension`       | `overworld`       | Target dimension for world export (`overworld`, `nether`, `end`).                           |
| `--data-version`    | `3700`            | Minecraft data version (default 1.20.4+).                                                   |

---

## JSON Configuration Guide

Your JSON file dictates the path routing and, in Advance mode, the structural makeup of the curve.

### 1. Simple Pathing (Path-Only & Single Cross-Section)

For basic usage, your JSON only needs pathing coordinates.

- `control_points`: Defines the 2D path. Format is `[x, z, heading_degrees]`.
- `elevation_points`: Defines the vertical slope. Format is `[x, z, y]`. Elevations are linearly interpolated between these points.

**Example (`points.json`):**

```json
{
	"control_points": [
		[0, 0, 0],
		[750, 500, 60],
		[1000, 1500, 0]
	],
	"elevation_points": [
		[0, 0, 64],
		[750, 500, 72],
		[1000, 1500, 64]
	]
}
```

### 2. Advance Mode (Cross-Sections & Structures)

Advance mode requires defining `cross_sections` and an optional `structures` array.

**Important Note on Coverage:** Every cross-section must declare explicit `points` (point-pairs) to define exactly which segments of the path it covers. There is no implicit default; uncovered path segments are simply skipped.

#### Cross-Sections Definition

Each key under `cross_sections` acts as an identifier for structures to reference later.

- `schematic`: Path to the `.schem` file.
- `width` & `dept`: Profile dimensions (replaces the CLI arguments).
- `points`: An array of coordinate pairs `[[[startX, startZ], [endX, endZ]], ...]` defining where this cross-section generates.

#### Structure Definitions

Structures are placed in a top-level `"structures"` list. Every structure must include a `"type"` and reference one or more cross-sections via `"cross_section"` (string) or `"cross_sections"` (array of strings).

**A. Pillar**
Places structural supports at regular intervals. Requires three schematics keyed by base angle; all 16 compass orientations (in 22.5° steps) are derived from these three via 90° rotation and mirroring.

- `0`: Schematic for 0°.
- `22.5`: Schematic for 22.5°.
- `45`: Schematic for 45°.
- `distance`: Interval between pillars.

**B. Catenary**
Overhead wire supports built from an ordered list of vertical slices.

- `schematics`: Array of schematic files. Index `0` is at the pole (road edge), and the last index is at the track intersection (near center). The number of slices must equal the cantilever length (`base_width // 2 - track_width // 2 + 1`). Track-2 (right side) slices are automatically mirrored.
- `catenary_interval`: Distance between catenary poles.
- `base_width`: Distance between poles across the path.
- `track_width`: Width of each track half.

**C. Wire**
Stamps a cross-section along a wire path. Wires use catenary intersection points as their generation base.

- `schematic`: The wire profile `.schem`.
- `points` (optional): A mask of point-pairs. Only catenary intersections falling within these segments are used. If omitted, it generates across full coverage.
- `use_step_line`: `true` for axis-aligned stepping, `false` for diagonal Bresenham rendering.
- `resolve_blocks`: Auto-resolves connectivity (like rail blocks) so they join seamlessly.
- `override_catenary`: If `false`, wire blocks overlapping catenary blocks are removed.

---

### Complete Advance Mode Example

**Example (`sample.json`):**

```json
{
	"control_points": [
		[0, 0, 0],
		[750, 500, 60],
		[1000, 1500, 0]
	],
	"elevation_points": [
		[0, 0, 64],
		[750, 500, 72],
		[1000, 1500, 64]
	],
	"cross_sections": {
		"standard": {
			"schematic": "stdCrossSection.schem",
			"width": 19,
			"dept": 3,
			"points": [
				[
					[0, 0],
					[750, 500]
				]
			]
		},
		"bridge": {
			"schematic": "bridgeCrossSection.schem",
			"width": 13,
			"dept": 3,
			"points": [
				[
					[750, 500],
					[1000, 1500]
				]
			]
		}
	},
	"structures": [
		{
			"type": "catenary",
			"cross_section": "standard",
			"schematics": [
				"catPole.schem",
				"catCant1.schem",
				"catCant2.schem",
				"catCant3.schem"
			],
			"catenary_interval": 36,
			"base_width": 13,
			"track_width": 7,
			"offset": 0
		},
		{
			"type": "wire",
			"cross_sections": ["standard", "bridge"],
			"schematic": "wire.schem",
			"points": [
				[
					[0, 0],
					[1000, 1500]
				]
			],
			"use_step_line": true,
			"resolve_blocks": true,
			"override_catenary": false
		},
		{
			"type": "pillar",
			"cross_section": "bridge",
			"0": "thinPillar0.schem",
			"22.5": "thinPillar22.schem",
			"45": "thinPillar45.schem",
			"distance": 6
		},
		{
			"type": "catenary",
			"cross_section": "bridge",
			"schematics": [
				"bridgeCatPole.schem",
				"catCant1.schem",
				"catCant2.schem",
				"catCant3.schem"
			],
			"catenary_interval": 36,
			"base_width": 13,
			"track_width": 7,
			"offset": 0
		}
	]
}
```

### Coordinate System & Headings

CurveGen uses a specific heading reference point tied directly to Minecraft's coordinate system:

* **0 Degrees:** Points **East** (Positive +X axis).
* **90 Degrees:** Points **South** (Positive +Z axis).

### Cross-Section Orientation & Symmetry

When designing and saving your cross-section `.schem` files, they must follow a strict orientation to generate correctly—especially when using symmetrical modes.

* **Axis Alignment:** The major axis of your cross-section must be aligned to **North/South** (Z axis).
* **Dimension Definitions (Symmetrical Mode):**
* **`dept`**: Refers to the number of block changes along the **X-axis**.
* **`width`**: Refers to the number of block changes along the **Z-axis**.



### Structure Rules & Origins

Structures follow strict rules regarding symmetry and how their schematic files must be copied/saved relative to your clipboard origin.

#### 1. Pillars

Pillars are **not symmetrical**. They are placed exactly as defined by their three base-angle schematics (0°, 22.5°, 45°), which the tool rotates to cover all 16 orientations.

#### 2. Catenaries (Optional for Wires)

Catenaries are an optional support structure for wires. When defining the vertical slice schematics for a catenary:

* **Order:** Slices are defined starting from the **end (outer edge)** moving inward toward the **center block**.
* **Copy Origin:** **Crucial:** Every single slice schematic *must* be copied using the exact same **center block** as your clipboard origin.

#### 3. Wires

The vertical and horizontal placement of a wire is dictated entirely by where you stand when you copy the schematic.

* **Copy Origin Offset:** The wire generates exactly relative to its copy origin.
* *Example:* If you want a wire to float exactly 5 blocks above the path with no X or Z shift, you must position yourself directly below the wire, exactly 5 blocks down, before copying the wire block to your schematic.

## Exporting

The tool automatically determines how to export the final blocks based on the `--output` argument:

- **`.schem` Output:** If the output path has a file extension, it generates a Sponge Schematic. The schematic's offset will automatically adjust so the curve's origin sits at $(0, 0, 0)$ relative to your paste position.
- **World Directory Output:** If the output path is a directory (has no extension), the script attempts to write the blocks directly into the region files of a Minecraft world save. Make sure `--dimension` is set correctly (`overworld`, `nether`, or `end`).

