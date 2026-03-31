# Parses content yaml file that is inputted
import json
import os
import yaml


def parse_yaml(file_path):
    """Parse a YAML config file and return a normalised configuration dict.

    Returns
    -------
    dict with keys:
        curve    – points_path (resolved), radius, step_size
        surface  – path, dim  (or None)
        sections – list of section group dicts
    """
    with open(file_path, "r") as f:
        raw = yaml.safe_load(f)

    yml_dir = os.path.dirname(os.path.abspath(file_path))

    return {
        "curve": _parse_curve(raw.get("curve", {}), yml_dir),
        "surface": _parse_surface(raw.get("surface")),
        "sections": _parse_sections(raw.get("sections", [])),
    }


# Curve


def _parse_curve(raw, yml_dir):
    if "points" not in raw:
        raise ValueError("curve.points is required.")
    if "radius" not in raw:
        raise ValueError("curve.radius is required.")

    points_path = raw["points"]
    if not os.path.isabs(points_path):
        points_path = os.path.normpath(os.path.join(yml_dir, points_path))

    return {
        "points_path": points_path,
        "radius": float(raw["radius"]),
        "step_size": int(raw.get("step_size", 1)),
    }


# Surface


def _parse_surface(raw):
    if raw is None:
        return None
    return {
        "path": raw.get("path", ""),
        "dim": raw.get("dim", "overworld"),
        "use_as_y_min": bool(raw.get("use_as_y_min", False)),
        "use_as_y_max": bool(raw.get("use_as_y_max", False)),
    }


# Sections


def _parse_sections(raw_sections):
    sections = []
    for raw in raw_sections or []:
        section = {
            "name": raw.get("name", "unnamed"),
            "when": _parse_when(raw.get("when")),
            "cross_sections": _parse_cross_sections(raw.get("cross_sections", [])),
            "structures": _parse_structures(raw.get("structures", [])),
        }
        sections.append(section)
    return sections


def _parse_cross_sections(raw_list):
    result = []
    for raw in raw_list or []:
        cs = {
            "name": raw.get("name", "unnamed"),
            "schematic": raw.get("schematic", ""),
            "vars": _extract_vars(raw),
            "when": _parse_when(raw.get("when")),
        }
        result.append(cs)
    return result


def _parse_structures(raw_list):
    result = []
    for raw in raw_list or []:
        raw_type = raw.get("type", "")
        raw_name = raw.get("name", "unnamed")
        # Infer type from name if not explicitly set
        if not raw_type and raw_name:
            for known in ("pillar", "catenary", "wire"):
                if known in raw_name.lower():
                    raw_type = known
                    break

        struct = {
            "name": raw_name,
            "type": raw_type,
            "schematic": _parse_schematic_field(raw.get("schematic", "")),
            "vars": _extract_vars(raw),
            "when": _parse_when(raw.get("when")),
        }
        result.append(struct)
    return result

_RESERVED_KEYS = {"name", "type", "schematic", "when", "cross_sections", "structures"}


def _extract_vars(raw):
    """Pull vars sub-dict, plus top-level non-reserved keys."""
    explicit = dict(raw.get("vars", {}))
    for k, v in raw.items():
        if k not in _RESERVED_KEYS and k != "vars":
            explicit.setdefault(k, v)
    return explicit


def _parse_schematic_field(raw):
    """A schematic can be a plain string or an angle->path dict."""
    if isinstance(raw, dict):
        return {str(k): str(v) for k, v in raw.items()}
    return str(raw) if raw else ""


def _parse_when(raw):
    """Normalise 'when' into a list of condition strings (implicit AND).

    YAML allows:
        when: surface.dy < 10                    -> single string
        when:                                     -> list (implicit AND)
          - surface.dy > 10
          - surface.dy <= 15
        when: (a <= 10 and b > 0) or c > 15      -> compound string

    Returns None when absent, or a list of raw condition strings.
    """
    if raw is None:
        return None
    if isinstance(raw, list):
        return [str(c).strip() for c in raw]
    return [str(raw).strip()]


# Points


def load_points(points_path):
    """Read a points JSON and return (control_points, elevation_points)."""
    with open(points_path, "r") as f:
        data = json.load(f)
    return data.get("control_points", []), data.get("elevation_points", [])