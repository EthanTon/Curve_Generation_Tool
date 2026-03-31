from collections import defaultdict
import numpy as np
import util.WorldUtil.worldUtil as worldUtil
import os

INVALID_BLOCKS: set[str] = {
    # ── Leaves ────────────────────────────────────────────────────────────
    "minecraft:acacia_leaves",
    "minecraft:azalea_leaves",
    "minecraft:birch_leaves",
    "minecraft:cherry_leaves",
    "minecraft:dark_oak_leaves",
    "minecraft:flowering_azalea_leaves",
    "minecraft:jungle_leaves",
    "minecraft:mangrove_leaves",
    "minecraft:oak_leaves",
    "minecraft:pale_oak_leaves",
    "minecraft:spruce_leaves",
    # ── Logs & wood ────────────────────────────────────────────────────────
    "minecraft:acacia_log",
    "minecraft:acacia_wood",
    "minecraft:birch_log",
    "minecraft:birch_wood",
    "minecraft:cherry_log",
    "minecraft:cherry_wood",
    "minecraft:dark_oak_log",
    "minecraft:dark_oak_wood",
    "minecraft:jungle_log",
    "minecraft:jungle_wood",
    "minecraft:mangrove_log",
    "minecraft:mangrove_wood",
    "minecraft:oak_log",
    "minecraft:oak_wood",
    "minecraft:pale_oak_log",
    "minecraft:pale_oak_wood",
    "minecraft:spruce_log",
    "minecraft:spruce_wood",
    "minecraft:stripped_acacia_log",
    "minecraft:stripped_acacia_wood",
    "minecraft:stripped_birch_log",
    "minecraft:stripped_birch_wood",
    "minecraft:stripped_cherry_log",
    "minecraft:stripped_cherry_wood",
    "minecraft:stripped_dark_oak_log",
    "minecraft:stripped_dark_oak_wood",
    "minecraft:stripped_jungle_log",
    "minecraft:stripped_jungle_wood",
    "minecraft:stripped_mangrove_log",
    "minecraft:stripped_mangrove_wood",
    "minecraft:stripped_oak_log",
    "minecraft:stripped_oak_wood",
    "minecraft:stripped_pale_oak_log",
    "minecraft:stripped_pale_oak_wood",
    "minecraft:stripped_spruce_log",
    "minecraft:stripped_spruce_wood",
    "minecraft:crimson_stem",
    "minecraft:stripped_crimson_stem",
    "minecraft:crimson_hyphae",
    "minecraft:stripped_crimson_hyphae",
    "minecraft:warped_stem",
    "minecraft:stripped_warped_stem",
    "minecraft:warped_hyphae",
    "minecraft:stripped_warped_hyphae",
    "minecraft:bamboo_block",
    "minecraft:stripped_bamboo_block",
    # ── Saplings ──────────────────────────────────────────────────────────
    "minecraft:acacia_sapling",
    "minecraft:bamboo_sapling",
    "minecraft:birch_sapling",
    "minecraft:cherry_sapling",
    "minecraft:dark_oak_sapling",
    "minecraft:jungle_sapling",
    "minecraft:mangrove_propagule",
    "minecraft:oak_sapling",
    "minecraft:pale_oak_sapling",
    "minecraft:spruce_sapling",
    # ── Grass & ferns ─────────────────────────────────────────────────────
    "minecraft:short_grass",
    "minecraft:tall_grass",
    "minecraft:fern",
    "minecraft:large_fern",
    "minecraft:short_dry_grass",
    "minecraft:tall_dry_grass",
    # ── Flowers ───────────────────────────────────────────────────────────
    "minecraft:allium",
    "minecraft:azure_bluet",
    "minecraft:blue_orchid",
    "minecraft:cornflower",
    "minecraft:dandelion",
    "minecraft:lilac",
    "minecraft:lily_of_the_valley",
    "minecraft:orange_tulip",
    "minecraft:oxeye_daisy",
    "minecraft:peony",
    "minecraft:pink_petals",
    "minecraft:pink_tulip",
    "minecraft:poppy",
    "minecraft:red_tulip",
    "minecraft:rose_bush",
    "minecraft:sunflower",
    "minecraft:torchflower",
    "minecraft:white_tulip",
    "minecraft:wildflowers",
    "minecraft:wither_rose",
    "minecraft:cactus_flower",
    "minecraft:spore_blossom",
    "minecraft:chorus_flower",
    # ── Crops ─────────────────────────────────────────────────────────────
    "minecraft:wheat",
    "minecraft:carrots",
    "minecraft:potatoes",
    "minecraft:beetroots",
    "minecraft:melon_stem",
    "minecraft:attached_melon_stem",
    "minecraft:pumpkin_stem",
    "minecraft:attached_pumpkin_stem",
    "minecraft:cocoa",
    "minecraft:sweet_berry_bush",
    "minecraft:torchflower_crop",
    "minecraft:pitcher_crop",
    "minecraft:pitcher_plant",
    # ── Vines & climbing ──────────────────────────────────────────────────
    "minecraft:vine",
    "minecraft:cave_vines",
    "minecraft:cave_vines_plant",
    "minecraft:twisting_vines",
    "minecraft:twisting_vines_plant",
    "minecraft:weeping_vines",
    "minecraft:weeping_vines_plant",
    "minecraft:glow_lichen",
    # ── Aquatic vegetation ────────────────────────────────────────────────
    "minecraft:kelp",
    "minecraft:kelp_plant",
    "minecraft:seagrass",
    "minecraft:tall_seagrass",
    "minecraft:lily_pad",
    # ── Bamboo & sugarcane & cactus ───────────────────────────────────────
    "minecraft:bamboo",
    "minecraft:sugar_cane",
    "minecraft:cactus",
    # ── Azalea bushes ─────────────────────────────────────────────────────
    "minecraft:azalea",
    "minecraft:flowering_azalea",
    # ── Moss & pale moss ──────────────────────────────────────────────────
    "minecraft:moss_carpet",
    "minecraft:pale_moss_carpet",
    "minecraft:leaf_litter",
    # ── Dripleaf ──────────────────────────────────────────────────────────
    "minecraft:big_dripleaf",
    "minecraft:big_dripleaf_stem",
    "minecraft:small_dripleaf",
    # ── Nether vegetation ─────────────────────────────────────────────────
    "minecraft:crimson_fungus",
    "minecraft:warped_fungus",
    "minecraft:crimson_roots",
    "minecraft:warped_roots",
    "minecraft:nether_sprouts",
    "minecraft:nether_wart",
    # ── Chorus plant (End) ────────────────────────────────────────────────
    "minecraft:chorus_plant",
    # ── Mushrooms ─────────────────────────────────────────────────────────
    "minecraft:brown_mushroom",
    "minecraft:red_mushroom",
    "minecraft:brown_mushroom_block",
    "minecraft:red_mushroom_block",
    "minecraft:mushroom_stem",
    # ── Hanging roots / misc organic ──────────────────────────────────────
    "minecraft:hanging_roots",
    "minecraft:mangrove_roots",
    "minecraft:muddy_mangrove_roots",
    "minecraft:dead_bush",
    "minecraft:bush",
    # ── Potted plants (flower pots with contents) ─────────────────────────
    "minecraft:flower_pot",
    "minecraft:potted_acacia_sapling",
    "minecraft:potted_allium",
    "minecraft:potted_azalea_bush",
    "minecraft:potted_bamboo",
    "minecraft:potted_birch_sapling",
    "minecraft:potted_blue_orchid",
    "minecraft:potted_brown_mushroom",
    "minecraft:potted_cactus",
    "minecraft:potted_cherry_sapling",
    "minecraft:potted_cornflower",
    "minecraft:potted_crimson_fungus",
    "minecraft:potted_crimson_roots",
    "minecraft:potted_dandelion",
    "minecraft:potted_dark_oak_sapling",
    "minecraft:potted_dead_bush",
    "minecraft:potted_fern",
    "minecraft:potted_flowering_azalea_bush",
    "minecraft:potted_jungle_sapling",
    "minecraft:potted_lily_of_the_valley",
    "minecraft:potted_mangrove_propagule",
    "minecraft:potted_oak_sapling",
    "minecraft:potted_orange_tulip",
    "minecraft:potted_oxeye_daisy",
    "minecraft:potted_pale_oak_sapling",
    "minecraft:potted_pink_tulip",
    "minecraft:potted_poppy",
    "minecraft:potted_red_mushroom",
    "minecraft:potted_red_tulip",
    "minecraft:potted_spruce_sapling",
    "minecraft:potted_torchflower",
    "minecraft:potted_warped_fungus",
    "minecraft:potted_warped_roots",
    "minecraft:potted_white_tulip",
    "minecraft:potted_wither_rose",
}


_NON_SURFACE = INVALID_BLOCKS | {
    "minecraft:air",
    "minecraft:cave_air",
    "minecraft:void_air",
    "minecraft:water",
}


def _parse_heightmap_numpy(raw_array) -> np.ndarray:
    arr = raw_array.np_array.astype(np.uint64)
    bpe = np.uint64(9)
    entries_per_long = 7  # 64 // 9

    indices = np.arange(256, dtype=np.uint64)
    long_idx = (indices // np.uint64(entries_per_long)).astype(np.intp)
    shift = (indices % np.uint64(entries_per_long)) * bpe

    long_idx = np.clip(long_idx, 0, len(arr) - 1)

    values = arr[long_idx]
    mask = np.uint64((1 << 9) - 1)
    heights = ((values >> shift) & mask).astype(np.int32) - 64 - 1
    return heights


def _cache_section(sec_y: int, section_lookup: dict, section_cache: dict):
    sec_nbt = section_lookup.get(sec_y)
    if sec_nbt is None:
        section_cache[sec_y] = None
        return

    st = sec_nbt.get("block_states", sec_nbt)
    pal_tag = st.get("palette", st.get("Palette", []))
    data_tag = st.get("data", st.get("BlockStates", None))

    palette = [str(p["Name"]) for p in pal_tag] if pal_tag else ["minecraft:air"]

    if data_tag is not None and len(palette) > 1:
        indices = worldUtil._unpack_states(list(data_tag.np_array), len(palette))
        if len(indices) < 4096:
            indices.extend([0] * (4096 - len(indices)))
        elif len(indices) > 4096:
            indices = indices[:4096]
    else:
        indices = None

    section_cache[sec_y] = (palette, indices)


def get_surface(world_dir: str, coordinates: list) -> dict:
    tasks = defaultdict(lambda: defaultdict(list))
    for x, z in coordinates:
        chunk_x, chunk_z = x >> 4, z >> 4
        region_x, region_z = chunk_x >> 5, chunk_z >> 5
        tasks[(region_x, region_z)][(chunk_x, chunk_z)].append((x, z))

    results = {}

    for (rx, rz), chunks_in_region in tasks.items():
        region_path = os.path.join(world_dir, "region", f"r.{rx}.{rz}.mca")
        if not os.path.exists(region_path):
            for coords in chunks_in_region.values():
                for x, z in coords:
                    results[(x, z)] = None
            continue

        loaded_chunks = worldUtil._read_region(region_path)

        for (cx, cz), coords in chunks_in_region.items():
            chunk_idx = (cx & 31) + ((cz & 31) * 32)

            if chunk_idx not in loaded_chunks:
                for x, z in coords:
                    results[(x, z)] = None
                continue

            chunk_nbt = worldUtil._get_nbt(loaded_chunks[chunk_idx])

            heightmaps = chunk_nbt.get(
                "Heightmaps", chunk_nbt.get("Level", {}).get("Heightmaps", {})
            )
            
            world_surface_array = heightmaps.get("OCEAN_FLOOR")

            if world_surface_array is not None:
                parsed_heights = _parse_heightmap_numpy(world_surface_array)
            else:
                parsed_heights = None

            sections = chunk_nbt.get(
                "sections", chunk_nbt.get("Level", {}).get("Sections", [])
            )
            section_lookup = {int(s["Y"]): s for s in sections}
            section_cache = {}

            for x, z in coords:
                lx, lz = x & 15, z & 15
                hm_index = lz * 16 + lx

                start_y = (
                    int(parsed_heights[hm_index]) if parsed_heights is not None else 319
                )

                surface_y = None
                y = start_y
                while y >= -64:
                    sec_y = y >> 4

                    if sec_y not in section_cache:
                        _cache_section(sec_y, section_lookup, section_cache)

                    cached = section_cache[sec_y]

                    if cached is None:
                        y = (sec_y << 4) - 1
                        continue

                    palette, indices = cached

                    if len(palette) == 1:
                        block_base = palette[0].split("[", 1)[0]
                        if block_base in _NON_SURFACE:
                            y = (sec_y << 4) - 1
                            continue
                        else:
                            surface_y = min(y, (sec_y << 4) + 15)
                            break

                    sec_floor = max(sec_y << 4, -64)
                    while y >= sec_floor:
                        local_y = y & 15
                        idx = (local_y << 8) | (lz << 4) | lx
                        if indices is not None:
                            block_name = palette[indices[idx]]
                        else:
                            block_name = palette[0]

                        base = block_name.split("[", 1)[0]
                        if base not in _NON_SURFACE:
                            surface_y = y
                            break
                        y -= 1
                    else:
                        y = sec_floor - 1
                        continue
                    break

                results[(x, z)] = surface_y

    return results
