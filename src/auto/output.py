import json


def create_json(
    config,
    control_points,
    elevation_points,
    segments,
    structure_assignments,
    surface_file=None,
):
    file_content = {
        "control_points": control_points,
        "elevation_points": elevation_points,
        "cross_sections": _build_cross_sections(config, segments),
        "structures": _build_structures(structure_assignments),
    }
    if surface_file:
        file_content["surface_file"] = surface_file
    return file_content


# Cross Section (cs)
def _build_cross_sections(config, segments):
    cs_defs = {}
    for sec in config["sections"]:
        for cs in sec["cross_sections"]:
            cs_defs[cs["name"]] = cs

    cs_points = {}
    for start, end, _sec_name, cs_name in segments:
        if cs_name is None:
            continue
        if start == end:
            continue
        cs_points.setdefault(cs_name, []).append([start, end])

    result = {}
    for name, point_pairs in cs_points.items():
        cs_def = cs_defs.get(name, {})
        entry = {
            "schematic": cs_def.get("schematic", ""),
            "width": cs_def.get("vars", {}).get("width", 0),
            "dept": cs_def.get("vars", {}).get("dept", 0),
            "points": point_pairs,
        }

        _INTERNAL_VARS = {"width", "dept", "min_length"}
        for k, v in cs_def.get("vars", {}).items():
            if k not in _INTERNAL_VARS:
                entry[k] = v
        result[name] = entry

    return result


# Structures
def _build_structures(structure_assignments):
    result = []

    for assignment in structure_assignments:
        struct_def = assignment["struct"]
        applies_to = assignment["applies_to"]

        stype = struct_def.get("type", "")
        schematic = struct_def.get("schematic", "")
        svars = struct_def.get("vars", {})

        if stype == "pillar":
            for cs_name in applies_to:
                entry = {"type": "pillar"}

                entry["cross_section"] = cs_name

                if isinstance(schematic, dict):
                    for angle_key, schem_path in schematic.items():
                        entry[str(angle_key)] = schem_path

                for k, v in svars.items():
                    entry[k] = v

                result.append(entry)

        elif stype == "catenary":
            entry = {"type": "catenary"}

            if len(applies_to) == 1:
                entry["cross_section"] = applies_to[0]
            else:
                entry["cross_sections"] = applies_to

            if isinstance(schematic, dict):
                for angle_key, schem_path in schematic.items():
                    entry[str(angle_key)] = schem_path

            for k, v in svars.items():
                entry[k] = v

            result.append(entry)

        elif stype == "wire":
            entry = {"type": "wire"}
            if len(applies_to) == 1:
                entry["cross_section"] = applies_to[0]
            else:
                entry["cross_sections"] = applies_to

            # Per-wire keys go into the wires sub-list
            _WIRE_ENTRY_KEYS = {"use_step_line", "use_lines"}

            # Structure-level vars (track_width etc.) go at entry root
            for k, v in svars.items():
                if k not in _WIRE_ENTRY_KEYS:
                    entry[k] = v

            # Build the wires sub-list
            if isinstance(schematic, str) and schematic:
                wire_entry = {"schematic": schematic}
                for k in _WIRE_ENTRY_KEYS:
                    if k in svars:
                        wire_entry[k] = svars[k]
                    else:
                        wire_entry[k] = True  # defaults
                entry["wires"] = [wire_entry]

            result.append(entry)

        else:
            # Generic / unknown structure type
            entry = {"type": stype}
            if len(applies_to) == 1:
                entry["cross_section"] = applies_to[0]
            else:
                entry["cross_sections"] = applies_to
            if isinstance(schematic, dict):
                for angle_key, schem_path in schematic.items():
                    entry[str(angle_key)] = schem_path
            elif schematic:
                entry["schematic"] = schematic
            for k, v in svars.items():
                entry[k] = v
            result.append(entry)

    return result


def write_json(file_content, output_path):
    """Write the json dict to a JSON file."""
    with open(output_path, "w") as f:
        json.dump(file_content, f, indent="\t")
