# Scaffold
from util.CoreUtil.maskingUtil import mask, mask_all
from util.CoreUtil.shapeUtil import (
    bresenham_filled_circle,
    bresenham_filled_arc,
    bresenham_circle,
)
from util.CoreUtil.pathUtil import weighted_pca_orthogonal, add_directions_to_points
from util.TrackUtil.elevationUtil import generate_elevation_lookup

import numpy as np

DEFAULT_CONFIG = {
    "base_width": 15,
    "track_width": 5,
    "turn_radius": 20,
    "catenary_interval": 30,
    "catenary_offset": 0,
    "base_block": "minecraft:gray_wool",
    "brim_block": "minecraft:gray_wool",
    "elevation_interval": 1,
}


def assemble_foundation(
    path, elevation_control_points, foundation_control_points=None, config=None
):
    """
    Assembles the foundation blocks for the given path and elevation lookup.

    Args:
        path (list of tuples): The list of (x, y) coordinates representing the path.
        elevation_lut (dict): A lookup table mapping (x, y) coordinates to elevation values.
        base_width (int): The width of the foundation blocks.
        layer_number (int): The number layers that will be generated
    Returns:
        dict: A dictionary mapping (x, y) coordinates to block types for the foundation.
    """

    cfg = dict(DEFAULT_CONFIG)
    if config:
        cfg.update(config)
    elevation_interval = cfg["elevation_interval"]
    base_width = cfg["base_width"]

    foundation_width = base_width + 6

    foundation_mask = _foundation_mask_(path, foundation_width)

    elevation_lut = generate_elevation_lookup(
        path,
        foundation_width,
        foundation_mask,
        step_size=elevation_interval,
        elevation_control_points=elevation_control_points,
    )

    foundation_blocks = {}

    return foundation_blocks


# assume we generate the either foundation
def generate_foundation(path, foundation_width, foundation_mask):
    foundation_radius = foundation_width // 2
    layer1 = set()
    
    # create layer 1
    for x, z in path:
        layer1.update(bresenham_filled_circle(x, z, foundation_radius - 4))

    layer1 = layer1 & foundation_mask

    layer2_edge = generate_layer_edge(path, foundation_width, foundation_mask)
    layer2_body = foundation_mask - layer2_edge


def _foundation_mask_(path, foundation_width):
    foundation_mask = set()
    foundation_radius = foundation_width // 2

    for x, z in path:
        foundation_mask.update(bresenham_filled_circle(x, z, foundation_radius))

    start_vec = tuple(
        int(n * foundation_radius) for n in weighted_pca_orthogonal(path, 0)
    )
    end_vec = tuple(
        int(n * foundation_radius) for n in weighted_pca_orthogonal(path, -1)
    )

    start_excess = bresenham_filled_arc(
        path[0][0], path[0][1], foundation_radius, start_vec[0], start_vec[1], True
    )
    end_excess = bresenham_filled_arc(
        path[-1][0], path[-1][1], foundation_radius, end_vec[0], end_vec[1], False
    )

    return foundation_mask - start_excess - end_excess


def generate_layer_edge(path, layer_width, foundation):
    if len(path) < 2:
        return [], []
    path_xy = [(pt[0], pt[1]) for pt in path]
    path_set = set(path_xy)
    layer_edge_points = set()
    for pt in path_xy:
        for n in bresenham_circle(pt[0], pt[1], layer_width // 2):
            if n not in path_set and n in foundation:
                layer_edge_points.add(n)
    if not layer_edge_points:
        return [], []
    path_arr = np.asarray(path_xy, dtype=float)
    directions = np.empty_like(path_arr)
    directions[0] = path_arr[1] - path_arr[0]
    directions[-1] = path_arr[-1] - path_arr[-2]
    if len(path_arr) > 2:
        directions[1:-1] = path_arr[2:] - path_arr[:-2]
    lengths = np.linalg.norm(directions, axis=1, keepdims=True)
    lengths[lengths == 0] = 1
    directions /= lengths
    left_normals = np.column_stack([-directions[:, 1], directions[:, 0]])

    # le is layer edge
    le1_points = []
    le2_points = []
    for pt in layer_edge_points:
        diffs = path_arr - np.array([pt[0], pt[1]])
        dists_sq = np.sum(diffs**2, axis=1)
        closest_idx = np.argmin(dists_sq)
        px, py = path_arr[closest_idx]
        nx, ny = left_normals[closest_idx]
        dx, dy = pt[0] - px, pt[1] - py
        if dx * nx + dy * ny > 0:
            le2_points.append((closest_idx, pt))
        else:
            le1_points.append((closest_idx, pt))
    le1_points.sort(key=lambda x: x[0])
    le2_points.sort(key=lambda x: x[0])
    le1_xy = [pt for _, pt in le1_points]
    le2_xy = [pt for _, pt in le2_points]

    # layer_edge_1 = add_directions_to_points(le1_xy, center_path=path_xy)
    # layer_edge_2 = add_directions_to_points(le2_xy, center_path=path_xy)
    return le1_xy, le2_xy
