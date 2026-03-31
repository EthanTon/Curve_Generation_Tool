# Validates content of the yaml
import math
import os
import re
import sys

from parser import parse_yaml, load_points

COMPARISON_OPS = {"==", "!=", ">", "<", ">=", "<="}
LOGICAL_OPS = {"and", "or", "not"}
KNOWN_VARIABLES = {
    "curve.xz",
    "curve.y",
    "curve.all",
    "surface.dy",
    "surface.y",
    "cross_section",
    "cross_sections",
    "cross_sections.name",
}
VALID_STRUCTURE_TYPES = {"pillar", "catenary", "wire"}


def validate(file_path):
    """Validate a YAML config file.  Returns a list of error strings.
    An empty list means the file is valid."""
    errors = []

    try:
        config = parse_yaml(file_path)
    except Exception as exc:
        return [f"Parse error: {exc}"]

    curve = config["curve"]
    pts_path = curve["points_path"]
    if not os.path.isfile(pts_path):
        errors.append(f"Points file not found: {pts_path}")
    else:
        try:
            ctrl, elev = load_points(pts_path)
            errors.extend(_validate_points(ctrl, elev, curve["radius"]))
        except Exception as exc:
            errors.append(f"Error reading points file: {exc}")

    if curve["radius"] <= 0:
        errors.append("curve.radius must be positive.")
    if curve["step_size"] < 1:
        errors.append("curve.step_size must be >= 1.")

    surface = config["surface"]
    if surface is not None:
        if not surface["path"]:
            errors.append("surface.path is required when surface is declared.")
        if surface["dim"] not in ("overworld", "nether", "end"):
            errors.append(
                f"Invalid surface.dim '{surface['dim']}'. "
                "Expected overworld, nether, or end."
            )
        if surface.get("use_as_y_min") and not surface["path"]:
            errors.append("surface.path required when use_as_y_min is set.")
        if surface.get("use_as_y_max") and not surface["path"]:
            errors.append("surface.path required when use_as_y_max is set.")

    for sec in config["sections"]:
        cs_names_in_group = {cs["name"] for cs in sec["cross_sections"]}

        # Validate cross sections
        for cs in sec["cross_sections"]:
            if not cs["schematic"]:
                errors.append(
                    f"Section '{sec['name']}' cross_section '{cs['name']}': "
                    "schematic is required."
                )
            v = cs["vars"]
            if "width" not in v:
                errors.append(
                    f"Cross section '{cs['name']}' in '{sec['name']}': "
                    "width is required."
                )
            if "dept" not in v:
                errors.append(
                    f"Cross section '{cs['name']}' in '{sec['name']}': "
                    "dept is required."
                )
            if cs["when"]:
                errors.extend(
                    _validate_conditions(cs["when"], f"cross_section '{cs['name']}'")
                )
            if v.get("use_y_min") and (
                surface is None or not surface.get("use_as_y_min")
            ):
                errors.append(
                    f"Cross section '{cs['name']}' in '{sec['name']}': "
                    "use_y_min requires surface with use_as_y_min."
                )
            if v.get("use_y_max") and (
                surface is None or not surface.get("use_as_y_max")
            ):
                errors.append(
                    f"Cross section '{cs['name']}' in '{sec['name']}': "
                    "use_y_max requires surface with use_as_y_max."
                )

        # Validate structures
        for st in sec["structures"]:
            if st["type"] and st["type"] not in VALID_STRUCTURE_TYPES:
                errors.append(
                    f"Structure '{st['name']}' in '{sec['name']}': "
                    f"unknown type '{st['type']}'."
                )
            if st["when"]:
                errors.extend(
                    _validate_conditions(st["when"], f"structure '{st['name']}'")
                )
                # Check cross_section references
                errors.extend(
                    _validate_cross_section_refs(
                        st["when"], cs_names_in_group, st["name"], sec["name"]
                    )
                )

            # Schematic required for pillar / catenary
            if st["type"] in ("pillar", "catenary"):
                if not isinstance(st["schematic"], dict) or not st["schematic"]:
                    errors.append(
                        f"Structure '{st['name']}' in '{sec['name']}': "
                        f"{st['type']} requires angle->schematic mapping."
                    )

        # Section-level when
        if sec["when"]:
            errors.extend(_validate_conditions(sec["when"], f"section '{sec['name']}'"))

    return errors


def _invalid_points(pt1, pt2, radius):
    """Return True if two consecutive control points are infeasible
    (no Dubins path exists with the given radius)."""
    x1, y1, theta1_deg = pt1
    x2, y2, theta2_deg = pt2

    dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    if dist == 0 and theta1_deg == theta2_deg:
        return False

    theta1 = math.radians(theta1_deg)
    theta2 = math.radians(theta2_deg)

    lc1 = (x1 - radius * math.sin(theta1), y1 + radius * math.cos(theta1))
    rc1 = (x1 + radius * math.sin(theta1), y1 - radius * math.cos(theta1))
    lc2 = (x2 - radius * math.sin(theta2), y2 + radius * math.cos(theta2))
    rc2 = (x2 + radius * math.sin(theta2), y2 - radius * math.cos(theta2))

    dist_l1_r2 = math.sqrt((lc1[0] - rc2[0]) ** 2 + (lc1[1] - rc2[1]) ** 2)
    dist_r1_l2 = math.sqrt((rc1[0] - lc2[0]) ** 2 + (rc1[1] - lc2[1]) ** 2)

    if dist_l1_r2 < 2 * radius and dist_r1_l2 < 2 * radius:
        return True
    return False


def _validate_points(control_points, elevation_points, radius):
    errors = []
    if len(control_points) < 2:
        errors.append("At least 2 control points are required.")
    if len(elevation_points) < 2:
        errors.append("At least 2 elevation points are required.")

    for i in range(len(control_points) - 1):
        if _invalid_points(control_points[i], control_points[i + 1], radius):
            errors.append(
                f"Control points {i} -> {i+1} are infeasible with radius {radius}. "
                f"Points are too close or angles are incompatible."
            )

    # Elevation points should reference existing control point x,z pairs
    ctrl_xz = {(pt[0], pt[1]) for pt in control_points}
    for i, ept in enumerate(elevation_points):
        if (ept[0], ept[1]) not in ctrl_xz:
            errors.append(
                f"Elevation point {i} at ({ept[0]}, {ept[1]}) does not "
                "match any control point x,z."
            )
    return errors


# ── Condition validation ────────────────────────────────────────────────

_TOKEN_RE = re.compile(
    r"""
    \(|\)|,|                       # parens, comma
    >=|<=|!=|==|>|<|               # comparison ops
    \b(?:and|or|not)\b|            # logical ops
    [a-zA-Z_][a-zA-Z0-9_.]*|      # identifiers
    -?\d+(?:\.\d+)?                # numbers
    """,
    re.VERBOSE,
)


def _validate_conditions(cond_list, context):
    """Check that condition strings contain only known operators and
    well-formed variable references."""
    errors = []
    for cond_str in cond_list:
        tokens = _TOKEN_RE.findall(cond_str)
        for tok in tokens:
            if tok in ("(", ")", ","):
                continue
            if tok in COMPARISON_OPS or tok in LOGICAL_OPS:
                continue
            # Number
            try:
                float(tok)
                continue
            except ValueError:
                pass
            # Variable / identifier – allow known vars and bare names
            # (bare names like 'foo' are cross_section name references)
            if tok in KNOWN_VARIABLES:
                continue
            # Allow any simple identifier (cross_section name etc.)
            if re.fullmatch(r"[a-zA-Z_][a-zA-Z0-9_]*", tok):
                continue
            errors.append(f"{context}: unknown token '{tok}' in condition '{cond_str}'")
    return errors


def _validate_cross_section_refs(cond_list, valid_names, struct_name, sec_name):
    """When a structure condition references cross_section == <name>,
    verify that <name> exists in the same section group."""
    errors = []
    for cond_str in cond_list:
        # Find patterns like: cross_section == <name>  or cross_sections == <name>
        refs = re.findall(
            r"cross_sections?(?:\.name)?\s*==\s*([a-zA-Z_][a-zA-Z0-9_]*)",
            cond_str,
        )
        for ref in refs:
            if ref not in valid_names:
                errors.append(
                    f"Structure '{struct_name}' in section '{sec_name}' "
                    f"references cross_section '{ref}' which does not exist "
                    f"in the same section group. Available: {valid_names}"
                )
    return errors


# ── CLI ─────────────────────────────────────────────────────────────────


def main():
    if len(sys.argv) < 2:
        print("Usage: python validate.py <config.yml>")
        sys.exit(1)

    file_path = sys.argv[1]
    if not os.path.isfile(file_path):
        print(f"Error: file not found: {file_path}")
        sys.exit(1)

    errors = validate(file_path)
    if errors:
        print(f"Validation FAILED — {len(errors)} error(s):\n")
        for i, e in enumerate(errors, 1):
            print(f"  {i}. {e}")
        sys.exit(1)
    else:
        print("Validation passed.")


if __name__ == "__main__":
    main()
