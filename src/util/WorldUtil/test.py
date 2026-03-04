import os
import math
import time
import argparse
import worldUtil
from collections import defaultdict


def is_valid_surface(block_name: str) -> bool:
    """
    Determines if a block counts as a valid solid surface.
    Explicitly keeps logs, wood, and leaves. Ignores air, liquids, and non-solid plants.
    """
    name = block_name.replace("minecraft:", "")

    # 1. EXPLICITLY KEEP: Logs, wood, nether stems/hyphae, and leaves
    # This catches oak_log, stripped_birch_wood, crimson_stem, warped_hyphae, cherry_leaves, etc.
    if (
        "log" in name
        or "wood" in name
        or "stem" in name
        or "hyphae" in name
        or "leaves" in name
        or "leaf" in name
    ):
        return False

    # 2. EXCLUDE: Air and liquids
    if "air" in name or "void" in name:
        return False
    if name in ["water", "lava", "bubble_column", "water_cauldron", "lava_cauldron"]:
        return False

    # 3. EXCLUDE: Non-solid plants and grasses
    # Note: 'grass' is the pre-1.20.3 plant ID, 'short_grass' is the modern plant ID.
    # We DO NOT put 'grass_block' here, so it safely defaults to True at the end.
    plants = {
        "short_grass",
        "tall_grass",
        "fern",
        "large_fern",
        "dandelion",
        "poppy",
        "blue_orchid",
        "allium",
        "azure_bluet",
        "red_tulip",
        "orange_tulip",
        "white_tulip",
        "pink_tulip",
        "oxeye_daisy",
        "cornflower",
        "lily_of_the_valley",
        "wither_rose",
        "sunflower",
        "lilac",
        "rose_bush",
        "peony",
        "sugar_cane",
        "kelp",
        "kelp_plant",
        "seagrass",
        "tall_seagrass",
        "brown_mushroom",
        "red_mushroom",
        "vine",
        "weeping_vines",
        "twisting_vines",
        "glow_lichen",
        "moss_carpet",
    }
    if name in plants:
        return False

    # If it isn't explicitly excluded above, it is considered a solid surface.
    return True


def get_surface_bulk(world_dir: str, coordinates: list) -> dict:
    """
    Efficiently calculates surface Y for a massive list of (X, Z) tuples.
    Returns a dictionary mapping (X, Z) -> Y.
    """
    # 1. Group coordinates by Region -> Chunk -> local X/Z
    tasks = defaultdict(lambda: defaultdict(list))
    for x, z in coordinates:
        chunk_x, chunk_z = x >> 4, z >> 4
        region_x, region_z = chunk_x >> 5, chunk_z >> 5
        tasks[(region_x, region_z)][(chunk_x, chunk_z)].append((x, z))

    results = {}

    # 2. Process Region by Region (Minimizes file I/O)
    for (rx, rz), chunks_in_region in tasks.items():
        region_path = os.path.join(world_dir, "region", f"r.{rx}.{rz}.mca")
        if not os.path.exists(region_path):
            for cx, cz in chunks_in_region:
                for x, z in chunks_in_region[(cx, cz)]:
                    results[(x, z)] = None  # Unloaded region
            continue

        loaded_chunks = worldUtil._read_region(region_path)

        # 3. Process Chunk by Chunk
        for (cx, cz), coords in chunks_in_region.items():
            local_cx, local_cz = cx & 31, cz & 31
            chunk_idx = local_cx + (local_cz * 32)

            if chunk_idx not in loaded_chunks:
                for x, z in coords:
                    results[(x, z)] = None
                continue

            chunk_nbt = worldUtil._get_nbt(loaded_chunks[chunk_idx])

            # --- Extract WORLD_SURFACE Heightmap ---
            heightmaps = chunk_nbt.get(
                "Heightmaps", chunk_nbt.get("Level", {}).get("Heightmaps", {})
            )
            world_surface_array = heightmaps.get("WORLD_SURFACE")

            parsed_heightmap = {}
            if world_surface_array is not None:
                arr = world_surface_array.np_array
                bpe = 9  # Modern Minecraft height packing
                entries_per_long = 64 // bpe
                mask = (1 << bpe) - 1
                for i in range(256):
                    long_idx = i // entries_per_long
                    shift = (i % entries_per_long) * bpe
                    if long_idx < len(arr):
                        val = int(arr[long_idx])
                        if val < 0:
                            val += 1 << 64
                        parsed_heightmap[i] = (
                            ((val >> shift) & mask) - 64 - 1
                        )  # Convert to absolute Y

            # --- Prepare Chunk Sections ---
            sections = chunk_nbt.get(
                "sections", chunk_nbt.get("Level", {}).get("Sections", [])
            )
            section_cache = {}  # Cache unpacked sections: sec_y -> (palette, indices)

            def get_block(lx, y, lz):
                sec_y = y >> 4
                if sec_y not in section_cache:
                    # Find the right section NBT
                    sec_nbt = next((s for s in sections if int(s["Y"]) == sec_y), None)
                    if not sec_nbt:
                        section_cache[sec_y] = (["minecraft:air"], [0] * 4096)
                    else:
                        st = sec_nbt.get("block_states", sec_nbt)  # 1.18+ vs older
                        pal_tag = st.get("palette", st.get("Palette", []))
                        data_tag = st.get("data", st.get("BlockStates", None))

                        palette = (
                            [str(p["Name"]) for p in pal_tag]
                            if pal_tag
                            else ["minecraft:air"]
                        )
                        if data_tag is not None and len(palette) > 1:
                            indices = worldUtil._unpack_states(
                                list(data_tag.np_array), len(palette)
                            )
                            indices = (indices + [0] * 4096)[:4096]
                        else:
                            indices = [0] * 4096
                        section_cache[sec_y] = (palette, indices)

                palette, indices = section_cache[sec_y]
                local_y = y & 15
                idx = (local_y << 8) | (lz << 4) | lx
                return palette[indices[idx]]

            # 4. Resolve exact Y for each coordinate in this chunk
            for x, z in coords:
                lx, lz = x & 15, z & 15
                hm_index = lz * 16 + lx

                # Start at the highest non-air block, or Y=319 if heightmap is missing
                start_y = parsed_heightmap.get(hm_index, 319)

                surface_y = None
                for y in range(start_y, -64, -1):
                    block_name = get_block(lx, y, lz)
                    if is_valid_surface(block_name):
                        surface_y = y
                        break

                results[(x, z)] = surface_y

    return results


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Find the solid surface Y value for an X, Z coordinate."
    )
    parser.add_argument("world", type=str, help="Path to the Minecraft world directory")
    parser.add_argument("x", type=int, help="X coordinate")
    parser.add_argument("z", type=int, help="Z coordinate")

    args = parser.parse_args()

    try:
        y_val = get_surface_bulk(args.world, [(args.x, args.z)])
        print(f"The surface Y coordinate at X={args.x}, Z={args.z} is: {y_val}")
    except Exception as e:
        print(f"Error: {e}")
