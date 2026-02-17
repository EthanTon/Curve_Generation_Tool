import math
import struct
import time
import zlib

import numpy as np
import amulet_nbt as nbt

SECTOR = 4096

def _bits_per_entry(palette_size):
    return max(4, math.ceil(math.log2(max(palette_size, 2))))


def _pack_states(indices, palette_size):
    bpe = _bits_per_entry(palette_size)
    per_long = 64 // bpe
    mask = (1 << bpe) - 1
    longs = []
    idx = 0
    for _ in range(math.ceil(4096 / per_long)):
        val = 0
        for slot in range(per_long):
            if idx < len(indices):
                val |= (indices[idx] & mask) << (slot * bpe)
                idx += 1
        if val >= (1 << 63):
            val -= 1 << 64
        longs.append(val)
    return longs


def _unpack_states(long_array, palette_size):
    bpe = _bits_per_entry(palette_size)
    per_long = 64 // bpe
    mask = (1 << bpe) - 1
    indices = []
    for v in long_array:
        if v < 0:
            v += 1 << 64
        for slot in range(per_long):
            if len(indices) >= 4096:
                break
            indices.append((v >> (slot * bpe)) & mask)
    return indices[:4096]


def _read_region(path):
    chunks = {}
    with open(path, "rb") as f:
        data = f.read()
    if len(data) < SECTOR * 2:
        return chunks
    for i in range(1024):
        off = i * 4
        sector_offset = (data[off] << 16) | (data[off + 1] << 8) | data[off + 2]
        if sector_offset == 0:
            continue
        start = sector_offset * SECTOR
        if start + 5 > len(data):
            continue
        length = struct.unpack_from(">I", data, start)[0]
        comp = data[start + 4]
        raw = data[start + 5 : start + 4 + length]
        if comp == 2:
            try:
                chunks[i] = zlib.decompress(raw)
            except zlib.error:
                pass
    return chunks


def _write_region(path, chunks):
    compressed = {i: zlib.compress(raw) for i, raw in chunks.items()}

    next_sector = 2
    layout = {}
    for i in sorted(compressed):
        total = 5 + len(compressed[i])
        sectors = math.ceil(total / SECTOR)
        layout[i] = (next_sector, sectors)
        next_sector += sectors

    out = bytearray(next_sector * SECTOR)

    for i, (sec_off, sec_cnt) in layout.items():
        off = i * 4
        out[off] = (sec_off >> 16) & 0xFF
        out[off + 1] = (sec_off >> 8) & 0xFF
        out[off + 2] = sec_off & 0xFF
        out[off + 3] = sec_cnt
        struct.pack_into(">I", out, SECTOR + i * 4, int(time.time()))

        payload = compressed[i]
        start = sec_off * SECTOR
        struct.pack_into(">I", out, start, 1 + len(payload))
        out[start + 4] = 2
        out[start + 5 : start + 5 + len(payload)] = payload

    with open(path, "wb") as f:
        f.write(out)


def _get_nbt(raw):
    return nbt.load(raw, compressed=False).compound


def _dump_nbt(compound):
    return nbt.NamedTag(compound).save_to(compressed=False)


def _patch_chunk(chunk, sections_data):
    if "sections" not in chunk:
        chunk["sections"] = nbt.ListTag([])

    sections_tag = chunk["sections"]
    by_y = {}
    for sec in sections_tag:
        by_y[int(sec["Y"])] = sec

    for sec_y, block_map in sections_data.items():
        if sec_y in by_y:
            _patch_section(by_y[sec_y], block_map)
        else:
            sections_tag.append(_new_section(sec_y, block_map))


def _patch_section(section_tag, block_map):
    bs = section_tag["block_states"]
    palette_tag = bs["palette"]

    palette = [str(e["Name"]) for e in palette_tag]
    palette_idx = {n: i for i, n in enumerate(palette)}

    for name in block_map:
        if name not in palette_idx:
            palette_idx[name] = len(palette)
            palette.append(name)
            palette_tag.append(nbt.CompoundTag({"Name": nbt.StringTag(name)}))

    if "data" in bs:
        indices = _unpack_states(list(bs["data"].np_array), len(palette))
        indices = (indices + [0] * 4096)[:4096]
    else:
        indices = [0] * 4096

    for name, positions in block_map.items():
        pid = palette_idx[name]
        for lx, ly, lz in positions:
            indices[(ly << 8) | (lz << 4) | lx] = pid

    if len(palette) > 1:
        bs["data"] = nbt.LongArrayTag(
            np.array(_pack_states(indices, len(palette)), dtype=np.int64)
        )


def _new_section(sec_y, block_map):
    palette = ["minecraft:air"]
    palette_idx = {"minecraft:air": 0}
    for name in block_map:
        if name not in palette_idx:
            palette_idx[name] = len(palette)
            palette.append(name)

    indices = [0] * 4096
    for name, positions in block_map.items():
        pid = palette_idx[name]
        for lx, ly, lz in positions:
            indices[(ly << 8) | (lz << 4) | lx] = pid

    palette_tag = nbt.ListTag(
        [nbt.CompoundTag({"Name": nbt.StringTag(name)}) for name in palette]
    )

    block_states = nbt.CompoundTag({"palette": palette_tag})
    if len(palette) > 1:
        block_states["data"] = nbt.LongArrayTag(
            np.array(_pack_states(indices, len(palette)), dtype=np.int64)
        )

    return nbt.CompoundTag(
        {
            "Y": nbt.ByteTag(sec_y),
            "block_states": block_states,
        }
    )
