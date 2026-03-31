# Handles logic such as operators and surface reading
import math
import re

from util.CoreUtil.curveUtil import draw_path, draw_path_silhouette
from util.CoreUtil.elevationUtil import generate_elevation_lookup


def _norm(v):
    return int(round(float(v)))


def generate_path(control_points, elevation_points, radius, max_width):

    raw_path, _, _ = draw_path(control_points, radius)
    norm_path = [(_norm(pt[0]), _norm(pt[1])) for pt in raw_path]

    silhouette = draw_path_silhouette(
        norm_path, control_points[0], control_points[-1], max_width
    )

    elev_lut = generate_elevation_lookup(
        norm_path,
        max_width,
        silhouette,
        step_size=1,
        elevation_control_points=elevation_points,
    )

    return {"path": norm_path, "elev_lut": elev_lut, "silhouette": silhouette}


def get_surface_map(world_path, path_points):
    from util.WorldUtil.surfaceUtil import get_surface

    unique_pts = list(dict.fromkeys(path_points))
    return get_surface(world_path, unique_pts)


def generate_path_standalone(control_points):
    path = []
    for i in range(len(control_points) - 1):
        x1, z1 = control_points[i][0], control_points[i][1]
        x2, z2 = control_points[i + 1][0], control_points[i + 1][1]
        dist = math.sqrt((x2 - x1) ** 2 + (z2 - z1) ** 2)
        steps = max(int(dist), 1)
        for s in range(steps):
            t = s / steps
            px = int(round(x1 + t * (x2 - x1)))
            pz = int(round(z1 + t * (z2 - z1)))
            if not path or (px, pz) != path[-1]:
                path.append((px, pz))
    # Last point
    lx, lz = control_points[-1][0], control_points[-1][1]
    last = (int(round(lx)), int(round(lz)))
    if not path or path[-1] != last:
        path.append(last)
    return path


def interpolate_elevation(path, elevation_points):
    if not elevation_points or not path:
        return {}

    # Build cumulative distances along the path
    cum_dist = [0.0]
    for i in range(1, len(path)):
        dx = path[i][0] - path[i - 1][0]
        dz = path[i][1] - path[i - 1][1]
        cum_dist.append(cum_dist[-1] + math.sqrt(dx * dx + dz * dz))

    # Map each elevation control point to a path parameter (closest point)
    elev_params = []
    for ep in elevation_points:
        ex, ez, ey = ep[0], ep[1], ep[2]
        best_idx = 0
        best_d = float("inf")
        for j, (px, pz) in enumerate(path):
            d = (px - ex) ** 2 + (pz - ez) ** 2
            if d < best_d:
                best_d = d
                best_idx = j
        elev_params.append((cum_dist[best_idx], ey))
    elev_params.sort(key=lambda x: x[0])

    # Interpolate for every path point
    lut = {}
    for i, (px, pz) in enumerate(path):
        d = cum_dist[i]
        # Clamp
        if d <= elev_params[0][0]:
            lut[(px, pz)] = elev_params[0][1]
        elif d >= elev_params[-1][0]:
            lut[(px, pz)] = elev_params[-1][1]
        else:
            for j in range(len(elev_params) - 1):
                d0, y0 = elev_params[j]
                d1, y1 = elev_params[j + 1]
                if d0 <= d <= d1:
                    t = (d - d0) / (d1 - d0) if d1 != d0 else 0
                    lut[(px, pz)] = y0 + t * (y1 - y0)
                    break
    return lut


# Condition evaluation

_TOKEN_RE = re.compile(
    r"""
    \(|\)|,|                         # parens, comma
    >=|<=|!=|==|>|<|                 # comparison ops
    \b(?:and|or|not)\b|              # logical ops
    [a-zA-Z_][a-zA-Z0-9_.]*|        # identifiers
    -?\d+(?:\.\d+)?                  # numbers
    """,
    re.VERBOSE,
)


def _tokenize(expr):
    return _TOKEN_RE.findall(expr)


class ConditionEvaluator:

    def __init__(self):
        self._path_index = {}  # (x,z) -> int

    def set_path_index_map(self, path):
        self._path_index = {}
        for i, pt in enumerate(path):
            pt_key = (int(round(pt[0])), int(round(pt[1])))
            if pt_key not in self._path_index:
                self._path_index[pt_key] = i

    def evaluate(self, cond_list, context):
        if cond_list is None:
            return True
        return all(self._eval_expr(c, context) for c in cond_list)

    # recursive-descent mini-parser

    def _eval_expr(self, expr_str, ctx):
        tokens = _tokenize(expr_str)
        pos, result = self._parse_or(tokens, 0, ctx)
        return bool(result)

    def _parse_or(self, tokens, pos, ctx):
        pos, left = self._parse_and(tokens, pos, ctx)
        while pos < len(tokens) and tokens[pos] == "or":
            pos += 1
            pos, right = self._parse_and(tokens, pos, ctx)
            left = left or right
        return pos, left

    def _parse_and(self, tokens, pos, ctx):
        pos, left = self._parse_not(tokens, pos, ctx)
        while pos < len(tokens) and tokens[pos] == "and":
            pos += 1
            pos, right = self._parse_not(tokens, pos, ctx)
            left = left and right
        return pos, left

    def _parse_not(self, tokens, pos, ctx):
        if pos < len(tokens) and tokens[pos] == "not":
            pos += 1
            pos, val = self._parse_not(tokens, pos, ctx)
            return pos, not val
        return self._parse_comparison(tokens, pos, ctx)

    def _parse_comparison(self, tokens, pos, ctx):
        pos, left = self._parse_atom(tokens, pos, ctx)
        if pos < len(tokens) and tokens[pos] in ("==", "!=", "<", ">", "<=", ">="):
            op = tokens[pos]
            pos += 1
            pos, right = self._parse_atom(tokens, pos, ctx)
            return pos, self._compare(left, op, right)
        return pos, left

    def _parse_atom(self, tokens, pos, ctx):
        if pos >= len(tokens):
            return pos, None

        tok = tokens[pos]

        # Parenthesised group or tuple
        if tok == "(":
            pos += 1
            # Collect elements separated by commas
            elements = []
            pos, first = self._parse_or(tokens, pos, ctx)
            elements.append(first)
            while pos < len(tokens) and tokens[pos] == ",":
                pos += 1  # skip comma
                pos, nxt = self._parse_or(tokens, pos, ctx)
                elements.append(nxt)
            if pos < len(tokens) and tokens[pos] == ")":
                pos += 1
            if len(elements) == 1:
                return pos, elements[0]
            return pos, tuple(elements)

        # Number
        try:
            val = float(tok)
            if val == int(val):
                val = int(val)
            return pos + 1, val
        except ValueError:
            pass

        # Variable / identifier
        val = ctx.get(tok, tok)  # resolve variable or keep as string literal
        return pos + 1, val

    def _compare(self, left, op, right):
        if left is None or right is None:
            if op == "==":
                return left == right
            if op == "!=":
                return left != right
            return False  # can't order against None

        # If both are tuples, compare by path index
        if isinstance(left, tuple) and isinstance(right, tuple):
            li = self._path_index.get((int(round(left[0])), int(round(left[1]))), 0)
            ri = self._path_index.get((int(round(right[0])), int(round(right[1]))), 0)
            # If the right-hand tuple isn't exactly on the path, find nearest
            if (int(round(right[0])), int(round(right[1]))) not in self._path_index:
                ri = self._nearest_index(right)
            left, right = li, ri

        ops = {
            "==": lambda a, b: a == b,
            "!=": lambda a, b: a != b,
            "<": lambda a, b: a < b,
            ">": lambda a, b: a > b,
            "<=": lambda a, b: a <= b,
            ">=": lambda a, b: a >= b,
        }
        try:
            return ops[op](left, right)
        except TypeError:
            # String equality for cross_section comparisons
            return ops[op](str(left), str(right))

    def _nearest_index(self, point):
        """Find the path index closest to a given (x, z) point."""
        tx, tz = float(point[0]), float(point[1])
        best_idx = 0
        best_d = float("inf")
        for (px, pz), idx in self._path_index.items():
            d = (px - tx) ** 2 + (pz - tz) ** 2
            if d < best_d:
                best_d = d
                best_idx = idx
        return best_idx


# Segment Classifier

def classify_path(path, config, elev_lut=None, surface_map=None, lenient=False):

    evaluator = ConditionEvaluator()
    evaluator.set_path_index_map(path)
    sections = config["sections"]

    min_length_lut = {}
    for sec in sections:
        for cs in sec["cross_sections"]:
            ml = cs["vars"].get("min_length", 0)
            min_length_lut[cs["name"]] = int(ml)

    #raw per-point classification
    raw_classifications = []

    for px, pz in path:
        planned_y = elev_lut.get((px, pz)) if elev_lut else None
        surface_y = surface_map.get((px, pz)) if surface_map else None
        
        dy = None
        if planned_y is not None and surface_y is not None:
            dy = float(planned_y) - float(surface_y)

        ctx = {
            "curve.xz": (px, pz),
            "curve.y": planned_y,
            "surface.dy": dy,
            "surface.y": surface_y,
        }

        matched_section = None
        matched_cs = None

        for sec in sections:
            if evaluator.evaluate(sec["when"], ctx):
                matched_section = sec["name"]
                for cs in sec["cross_sections"]:
                    if evaluator.evaluate(cs["when"], ctx):
                        matched_cs = cs["name"]
                        break
                # Lenient fallback: pick first CS if none matched
                if matched_cs is None and lenient and sec["cross_sections"]:
                    matched_cs = sec["cross_sections"][0]["name"]
                break

        raw_classifications.append((matched_section, matched_cs))

    # enforce min_length
    # Walk forward.  Track the current (section, cs) and how many steps
    # it has been running.  A switch is only allowed when either:
    #   - the section changes (section boundaries always break), or
    #   - the current CS has been running for >= its min_length.

    if not raw_classifications:
        return []

    final = []
    cur_sec, cur_cs = raw_classifications[0]
    run_length = 1

    for i in range(1, len(raw_classifications)):
        desired_sec, desired_cs = raw_classifications[i]

        # Section change always takes effect immediately
        if desired_sec != cur_sec:
            final.append((cur_sec, cur_cs))
            cur_sec, cur_cs = desired_sec, desired_cs
            run_length = 1
            continue

        # Same section — check if CS wants to change
        if desired_cs != cur_cs:
            ml = min_length_lut.get(cur_cs, 0)
            if ml > 0 and run_length < ml:
                # Not enough steps yet — suppress the switch
                final.append((cur_sec, cur_cs))
                run_length += 1
            else:
                # Switch is allowed
                final.append((cur_sec, cur_cs))
                cur_sec, cur_cs = desired_sec, desired_cs
                run_length = 1
        else:
            final.append((cur_sec, cur_cs))
            run_length += 1

    final.append((cur_sec, cur_cs))

    # build contiguous segments

    segments = []
    cur_sec, cur_cs = final[0]
    seg_start = list(path[0])

    for i in range(1, len(final)):
        s, c = final[i]
        if s != cur_sec or c != cur_cs:
            segments.append((seg_start, list(path[i]), cur_sec, cur_cs))
            cur_sec, cur_cs = s, c
            seg_start = list(path[i])

    segments.append((seg_start, list(path[-1]), cur_sec, cur_cs))
    return segments


def _extract_cs_names_from_conditions(cond_list):
    if cond_list is None:
        return []

    names = []
    for cond_str in cond_list:
        refs = re.findall(
            r"cross_sections?(?:\.name)?\s*==\s*([a-zA-Z_][a-zA-Z0-9_]*)",
            cond_str,
        )
        names.extend(refs)
    return names


def resolve_structure_assignments(segments, config):
    active_cs = set()
    for _, _, sec_name, cs_name in segments:
        if cs_name:
            active_cs.add(cs_name)

    resolved = []
    for sec in config["sections"]:
        cs_in_group = {cs["name"] for cs in sec["cross_sections"]}
        for struct in sec["structures"]:
            referenced = _extract_cs_names_from_conditions(struct["when"])

            if referenced:
                applies_to = [
                    n for n in referenced if n in cs_in_group and n in active_cs
                ]
            else:
                applies_to = [n for n in cs_in_group if n in active_cs]

            if applies_to:
                resolved.append(
                    {
                        "struct": struct,
                        "section_name": sec["name"],
                        "applies_to": applies_to,
                    }
                )
    return resolved


# Surface file

def _surface_cs_names(config):
    names = set()
    widths = {}
    for sec in config["sections"]:
        for cs in sec["cross_sections"]:
            v = cs["vars"]
            if v.get("use_y_min") or v.get("use_y_max"):
                names.add(cs["name"])
                widths[cs["name"]] = v.get("width", 1)
    return names, widths


def _find_path_index(path, point):
    pt = (int(round(point[0])), int(round(point[1])))
    for i, (x, z) in enumerate(path):
        if (x, z) == pt:
            return i
    return 0


def _approx_silhouette(path, width):
    sil = set()
    r = width // 2 + 1
    r_sq = r * r
    for x, z in path:
        for dx in range(-r, r + 1):
            for dz in range(-r, r + 1):
                if dx * dx + dz * dz <= r_sq:
                    sil.add((x + dx, z + dz))
    return sil


def generate_surface_region(path, segments, config, max_width, silhouette=None):
    try:
        from util.CoreUtil.maskingUtil import mask
    except ImportError:
        mask = None

    cs_names, cs_widths = _surface_cs_names(config)
    if not cs_names:
        return set()

    surf_width = max(cs_widths.values()) if cs_widths else max_width

    if silhouette is None:
        silhouette = _approx_silhouette(path, surf_width)

    if mask is None:
        return silhouette

    region = set()
    for start, end, _sec, cs_name in segments:
        if cs_name not in cs_names:
            continue
        s_idx = _find_path_index(path, start)
        e_idx = _find_path_index(path, end)
        region.update(mask(path, s_idx, e_idx, surf_width, silhouette))
    return region


def write_surface_file(surface_data, output_path):
    import json as _json
    pts = [[x, z, y] for (x, z), y in surface_data.items()]
    with open(output_path, "w") as f:
        _json.dump(pts, f)


def load_surface_file(file_path):
    import json as _json
    with open(file_path, "r") as f:
        pts = _json.load(f)
    return {(int(pt[0]), int(pt[1])): pt[2] for pt in pts}