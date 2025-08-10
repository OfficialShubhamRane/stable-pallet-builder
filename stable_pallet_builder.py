import random
import itertools
# import matplotlib
# matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import signal
import sys
import json
import plotly.graph_objects as go
import numpy as np
import math
from copy import deepcopy
from itertools import permutations

# Configurable parameters
PALLET_WIDTH = 40      # inches
PALLET_DEPTH = 48      # inches
MAX_HEIGHT = 60        # inches
NUM_BOXES = 25
GRID_STEP = 2          # inch resolution
SUPPORT_THRESHOLD = 0.75  # minimum support ratio for stability

# ----- DATA STRUCTURE -----
class Box:
    def __init__(self, w, d, h, weight, name):
        self.w = w
        self.d = d
        self.h = h
        self.weight = weight
        self.name = name
        self.x = None  # placement coordinates on pallet
        self.y = None
        self.z = None

    def volume(self):
        return self.w * self.d * self.h

    def density(self):
        return self.weight / self.volume()

    def __repr__(self):
        return f"{self.name}(W:{self.w},D:{self.d},H:{self.h},Wt:{self.weight})"


# ----- STABILITY SCORING -----
def stability_score(arrangement):
    """
    arrangement: list of (box, x, y, z)
    Returns a score between 0 and 1
    """
    total_weight = sum(b.weight for b, *_ in arrangement)
    if total_weight == 0:
        return 0

    # 1. Weight at bottom ratio
    half_height = MAX_HEIGHT / 2
    bottom_weight = sum(b.weight for b, _, _, z in arrangement if z + b.h <= half_height)
    w_bottom_ratio = bottom_weight / total_weight

    # 2. Balance factor
    left_weight = sum(b.weight for b, x, _, _ in arrangement if x + b.w/2 < PALLET_WIDTH/2)
    right_weight = total_weight - left_weight
    front_weight = sum(b.weight for b, _, y, _ in arrangement if y + b.d/2 < PALLET_DEPTH/2)
    back_weight = total_weight - front_weight

    balance_factor = 1 - (abs(left_weight - right_weight) / total_weight +
                          abs(front_weight - back_weight) / total_weight) / 2

    # 3. No overhang (penalize if any box exceeds pallet dimensions)
    no_overhang = 1.0
    for b, x, y, _ in arrangement:
        if x + b.w > PALLET_WIDTH or y + b.d > PALLET_DEPTH:
            no_overhang -= 0.2

    # Final weighted score
    score = w_bottom_ratio * 0.4 + balance_factor * 0.4 + no_overhang * 0.2
    return round(score, 4)


#prioritize center of pallet, but not too much, this one works
# def build_pallet(boxes):
    grid_w = int(PALLET_WIDTH / GRID_STEP)
    grid_d = int(PALLET_DEPTH / GRID_STEP)
    height_map = [[0] * grid_d for _ in range(grid_w)]

    arrangement = []
    boxes = sorted(boxes, key=lambda b: b.density(), reverse=True)  # heavier first

    # Center coordinates in grid units
    center_x = grid_w // 2
    center_y = grid_d // 2

    def get_max_height(x_idx, y_idx, w_idx, d_idx):
        max_h = 0
        for xi in range(x_idx, x_idx + w_idx):
            for yi in range(y_idx, y_idx + d_idx):
                if xi >= grid_w or yi >= grid_d:
                    return None  # out of bounds
                max_h = max(max_h, height_map[xi][yi])
        return max_h

    def update_height_map(x_idx, y_idx, w_idx, d_idx, new_height):
        for xi in range(x_idx, x_idx + w_idx):
            for yi in range(y_idx, y_idx + d_idx):
                height_map[xi][yi] = new_height

    # Generate grid positions sorted by distance from center
    def positions_from_center():
        positions = [(x, y) for x in range(grid_w) for y in range(grid_d)]
        positions.sort(key=lambda p: ((p[0] - center_x) ** 2 + (p[1] - center_y) ** 2))
        return positions

    candidate_positions = positions_from_center()

    for b in boxes:
        placed = False
        best_pos = None
        best_height = None
        best_orientation = None

        for w, d in [(b.w, b.d), (b.d, b.w)]:  # try both orientations
            w_idx = int(w / GRID_STEP)
            d_idx = int(d / GRID_STEP)

            for x_idx, y_idx in candidate_positions:
                if x_idx + w_idx > grid_w or y_idx + d_idx > grid_d:
                    continue

                z = get_max_height(x_idx, y_idx, w_idx, d_idx)
                if z is None:
                    continue
                if z + b.h <= MAX_HEIGHT:
                    if best_height is None or z < best_height:
                        best_height = z
                        best_pos = (x_idx, y_idx)
                        best_orientation = (w, d)

        if best_pos:
            x_real = best_pos[0] * GRID_STEP
            y_real = best_pos[1] * GRID_STEP
            z_real = best_height
            b.w, b.d = best_orientation
            arrangement.append((b, x_real, y_real, z_real))

            w_idx = int(b.w / GRID_STEP)
            d_idx = int(b.d / GRID_STEP)
            update_height_map(best_pos[0], best_pos[1], w_idx, d_idx, best_height + b.h)
            placed = True

        if not placed:
            print(f"Box {b} could not be placed (no space).")

    return arrangement


# def build_pallet(boxes):
    grid_w = int(PALLET_WIDTH / GRID_STEP)
    grid_d = int(PALLET_DEPTH / GRID_STEP)
    height_map = [[0] * grid_d for _ in range(grid_w)]

    arrangement = []
    boxes = sorted(boxes, key=lambda b: b.density(), reverse=True)

    center_x = grid_w // 2
    center_y = grid_d // 2

    def get_max_height(x_idx, y_idx, w_idx, d_idx):
        max_h = 0
        for xi in range(x_idx, x_idx + w_idx):
            for yi in range(y_idx, y_idx + d_idx):
                if xi >= grid_w or yi >= grid_d:
                    return None
                max_h = max(max_h, height_map[xi][yi])
        return max_h

    def update_height_map(x_idx, y_idx, w_idx, d_idx, new_height):
        for xi in range(x_idx, x_idx + w_idx):
            for yi in range(y_idx, y_idx + d_idx):
                height_map[xi][yi] = new_height

    def positions_from_center():
        positions = [(x, y) for x in range(grid_w) for y in range(grid_d)]
        positions.sort(key=lambda p: ((p[0] - center_x) ** 2 + (p[1] - center_y) ** 2))
        return positions

    candidate_positions = positions_from_center()

    def generate_orientations(b):
        dims = [b.w, b.d, b.h]
        # All unique permutations of dimensions as (width, depth, height)
        return [
            (dims[0], dims[1], dims[2]),
            (dims[1], dims[0], dims[2]),
            (dims[2], dims[0], dims[1]),
            (dims[2], dims[1], dims[0]),
            (dims[0], dims[2], dims[1]),
            (dims[1], dims[2], dims[0]),
        ]

    for b in boxes:
        placed = False
        best_pos = None
        best_height = None
        best_orientation = None

        for w, d, h in generate_orientations(b):
            w_idx = int(w / GRID_STEP)
            d_idx = int(d / GRID_STEP)

            if w_idx == 0 or d_idx == 0:
                continue  # Ignore zero dimension footprints

            for x_idx, y_idx in candidate_positions:
                if x_idx + w_idx > grid_w or y_idx + d_idx > grid_d:
                    continue

                z = get_max_height(x_idx, y_idx, w_idx, d_idx)
                if z is None:
                    continue
                if z + h <= MAX_HEIGHT:
                    if best_height is None or z < best_height:
                        best_height = z
                        best_pos = (x_idx, y_idx)
                        best_orientation = (w, d, h)

        if best_pos:
            x_real = best_pos[0] * GRID_STEP
            y_real = best_pos[1] * GRID_STEP
            z_real = best_height
            b.w, b.d, b.h = best_orientation  # update box dimensions to chosen orientation
            arrangement.append((b, x_real, y_real, z_real))

            w_idx = int(b.w / GRID_STEP)
            d_idx = int(b.d / GRID_STEP)
            update_height_map(best_pos[0], best_pos[1], w_idx, d_idx, best_height + b.h)
            placed = True

        if not placed:
            print(f"Box {b} could not be placed (no space).")

    return arrangement

# this works
# def build_pallet(boxes):
    grid_w = int(PALLET_WIDTH / GRID_STEP)
    grid_d = int(PALLET_DEPTH / GRID_STEP)
    height_map = [[0] * grid_d for _ in range(grid_w)]
    weight_map = [[0] * grid_d for _ in range(grid_w)]  # stores max box weight at each cell

    arrangement = []
    boxes = sorted(boxes, key=lambda b: b.density(), reverse=True)  # heavier first

    center_x = grid_w // 2
    center_y = grid_d // 2

    def get_max_height(x_idx, y_idx, w_idx, d_idx):
        max_h = 0
        for xi in range(x_idx, x_idx + w_idx):
            for yi in range(y_idx, y_idx + d_idx):
                if xi >= grid_w or yi >= grid_d:
                    return None
                max_h = max(max_h, height_map[xi][yi])
        return max_h

    def get_max_supporting_weight(x_idx, y_idx, w_idx, d_idx):
        max_wt = 0
        for xi in range(x_idx, x_idx + w_idx):
            for yi in range(y_idx, y_idx + d_idx):
                if xi >= grid_w or yi >= grid_d:
                    return 0
                max_wt = max(max_wt, weight_map[xi][yi])
        return max_wt

    def update_maps(x_idx, y_idx, w_idx, d_idx, new_height, box_weight):
        for xi in range(x_idx, x_idx + w_idx):
            for yi in range(y_idx, y_idx + d_idx):
                height_map[xi][yi] = new_height
                weight_map[xi][yi] = box_weight

    def positions_from_center():
        positions = [(x, y) for x in range(grid_w) for y in range(grid_d)]
        positions.sort(key=lambda p: ((p[0] - center_x) ** 2 + (p[1] - center_y) ** 2))
        return positions

    candidate_positions = positions_from_center()

    for b in boxes:
        placed = False
        best_pos = None
        best_height = None
        best_orientation = None

        for w, d in [(b.w, b.d), (b.d, b.w), (b.h, b.d), (b.d, b.h), (b.w, b.h), (b.h, b.w)]:  # try rotations including height
            w_idx = int(w / GRID_STEP)
            d_idx = int(d / GRID_STEP)

            if w_idx == 0 or d_idx == 0:
                continue

            for x_idx, y_idx in candidate_positions:
                if x_idx + w_idx > grid_w or y_idx + d_idx > grid_d:
                    continue

                z = get_max_height(x_idx, y_idx, w_idx, d_idx)
                if z is None:
                    continue
                if z + b.h > MAX_HEIGHT:
                    continue

                max_support_wt = get_max_supporting_weight(x_idx, y_idx, w_idx, d_idx)

                # Allow placement if:
                # 1. At ground level (z == 0)
                # 2. Or supporting weight >= box weight
                if z == 0 or max_support_wt >= b.weight:
                    if best_height is None or z < best_height:
                        best_height = z
                        best_pos = (x_idx, y_idx)
                        best_orientation = (w, d)

        if best_pos:
            x_real = best_pos[0] * GRID_STEP
            y_real = best_pos[1] * GRID_STEP
            z_real = best_height
            b.w, b.d = best_orientation
            arrangement.append((b, x_real, y_real, z_real))

            w_idx = int(b.w / GRID_STEP)
            d_idx = int(b.d / GRID_STEP)
            update_maps(best_pos[0], best_pos[1], w_idx, d_idx, best_height + b.h, b.weight)
            placed = True

        if not placed:
            print(f"Box {b} could not be placed (no space or weight constraints).")

    return arrangement


# with basic backtracking
def build_pallet(boxes, max_depth=1):
    grid_w = int(PALLET_WIDTH / GRID_STEP)
    grid_d = int(PALLET_DEPTH / GRID_STEP)

    def init_height_map():
        return [[0] * grid_d for _ in range(grid_w)]

    # Helper: check max height under footprint or None if OOB
    def get_max_height(height_map, x_idx, y_idx, w_idx, d_idx):
        max_h = 0
        for xi in range(x_idx, x_idx + w_idx):
            for yi in range(y_idx, y_idx + d_idx):
                if xi >= grid_w or yi >= grid_d:
                    return None
                max_h = max(max_h, height_map[xi][yi])
        return max_h

    # Helper: update height map footprint with new height
    def update_height_map(height_map, x_idx, y_idx, w_idx, d_idx, new_height):
        for xi in range(x_idx, x_idx + w_idx):
            for yi in range(y_idx, y_idx + d_idx):
                height_map[xi][yi] = new_height

    # Sort positions from center outward for placement attempts
    center_x = grid_w // 2
    center_y = grid_d // 2
    def positions_from_center():
        positions = [(x, y) for x in range(grid_w) for y in range(grid_d)]
        positions.sort(key=lambda p: ((p[0] - center_x) ** 2 + (p[1] - center_y) ** 2))
        return positions

    # Try placing one box on a given height_map; returns placement or None
    def try_place_box(height_map, b):
        candidate_positions = positions_from_center()
        best_pos = None
        best_height = None
        best_orientation = None

        # Try all rotations of box (w,d,h permutations)
        # For simplicity, rotate only dimensions, height stays last dimension
        dims = [b.w, b.d, b.h]
        # Use permutations of (w,d,h), but height is always vertical, so rotate only base
        base_orientations = [(dims[0], dims[1], dims[2]), (dims[1], dims[0], dims[2])]
        for (w, d, h) in base_orientations:
            w_idx = int(w / GRID_STEP)
            d_idx = int(d / GRID_STEP)
            for x_idx, y_idx in candidate_positions:
                if x_idx + w_idx > grid_w or y_idx + d_idx > grid_d:
                    continue
                max_h = get_max_height(height_map, x_idx, y_idx, w_idx, d_idx)
                if max_h is None:
                    continue
                if max_h + h <= MAX_HEIGHT:
                    # Greedy choose lowest height placement
                    if best_height is None or max_h < best_height:
                        best_height = max_h
                        best_pos = (x_idx, y_idx)
                        best_orientation = (w, d, h)

        if best_pos:
            # Return box placement data and updated height map
            x_real = best_pos[0] * GRID_STEP
            y_real = best_pos[1] * GRID_STEP
            z_real = best_height

            # Create a copy of height map to update
            new_height_map = deepcopy(height_map)
            update_height_map(new_height_map, best_pos[0], best_pos[1],
                              int(best_orientation[0]/GRID_STEP),
                              int(best_orientation[1]/GRID_STEP),
                              best_height + best_orientation[2])
            # Return placement and new height map
            placed_box = deepcopy(b)
            placed_box.w, placed_box.d, placed_box.h = best_orientation
            return (placed_box, x_real, y_real, z_real), new_height_map

        return None, None

    # Recursive backtracking packing
    def pack_recursive(box_list, height_map, placed_list, depth):
        if not box_list:
            return placed_list  # All boxes placed

        box = box_list[0]
        rest_boxes = box_list[1:]

        # Try placing current box
        placement, new_height_map = try_place_box(height_map, box)
        if placement:
            # Successful placement, recurse with rest
            result = pack_recursive(rest_boxes, new_height_map, placed_list + [placement], depth)
            if result is not None:
                return result

        # If cannot place and depth > 0, try backtracking by removing one placed box and retry
        if depth > 0 and placed_list:
            for i, placed in enumerate(placed_list):
                # Remove placed[i] and try packing with current + removed box + rest
                removed_box = placed[0]
                new_placed = placed_list[:i] + placed_list[i+1:]
                retry_boxes = [box, removed_box] + rest_boxes

                # Rebuild height map from scratch with new_placed
                fresh_height_map = init_height_map()
                valid = True
                for p_box, px, py, pz in new_placed:
                    w_idx = int(p_box.w / GRID_STEP)
                    d_idx = int(p_box.d / GRID_STEP)
                    max_h = get_max_height(fresh_height_map, int(px / GRID_STEP), int(py / GRID_STEP), w_idx, d_idx)
                    if max_h is None or max_h > pz:
                        valid = False
                        break
                    update_height_map(fresh_height_map, int(px / GRID_STEP), int(py / GRID_STEP), w_idx, d_idx, pz + p_box.h)
                if not valid:
                    continue  # skip invalid partial arrangement

                # Try packing recursively with backtracking depth-1
                result = pack_recursive(retry_boxes, fresh_height_map, new_placed, depth-1)
                if result is not None:
                    return result

        # No valid arrangement found
        return None

    # Sort boxes by density descending (heaviest first)
    sorted_boxes = sorted(boxes, key=lambda b: b.density(), reverse=True)

    initial_height_map = init_height_map()
    arrangement = pack_recursive(sorted_boxes, initial_height_map, [], max_depth)

    if arrangement is None:
        print("Failed to place all boxes with backtracking.")
        # Optionally, you could return partial arrangement or empty
        return []

    return arrangement


# without hover feature for weight and dims
def plot_pallet_three_views(arrangement):
    fig = plt.figure(figsize=(18, 6))

    angles = [
        (20, 30),   # elevation, azimuth
        (20, 120),
        (60, 30)
    ]

    colors = plt.cm.tab20.colors

    for i, (elev, azim) in enumerate(angles, start=1):
        ax = fig.add_subplot(1, 3, i, projection='3d')

        # Draw pallet base
        pallet = Poly3DCollection(
            [[[0, 0, 0], [PALLET_WIDTH, 0, 0], [PALLET_WIDTH, PALLET_DEPTH, 0], [0, PALLET_DEPTH, 0]]],
            color='saddlebrown', alpha=0.5
        )
        ax.add_collection3d(pallet)

        # Draw boxes
        for j, (b, x, y, z) in enumerate(arrangement):
            X = [x, x + b.w]
            Y = [y, y + b.d]
            Z = [z, z + b.h]

            vertices = [
                # bottom
                [[X[0], Y[0], Z[0]], [X[1], Y[0], Z[0]], [X[1], Y[1], Z[0]], [X[0], Y[1], Z[0]]],
                # top
                [[X[0], Y[0], Z[1]], [X[1], Y[0], Z[1]], [X[1], Y[1], Z[1]], [X[0], Y[1], Z[1]]],
                # sides
                [[X[0], Y[0], Z[0]], [X[1], Y[0], Z[0]], [X[1], Y[0], Z[1]], [X[0], Y[0], Z[1]]],
                [[X[1], Y[0], Z[0]], [X[1], Y[1], Z[0]], [X[1], Y[1], Z[1]], [X[1], Y[0], Z[1]]],
                [[X[1], Y[1], Z[0]], [X[0], Y[1], Z[0]], [X[0], Y[1], Z[1]], [X[1], Y[1], Z[1]]],
                [[X[0], Y[1], Z[0]], [X[0], Y[0], Z[0]], [X[0], Y[0], Z[1]], [X[0], Y[1], Z[1]]],
            ]
            ax.add_collection3d(Poly3DCollection(vertices, facecolors=colors[j % len(colors)], alpha=0.7, linewidths=0.5, edgecolors='black'))

        ax.set_xlabel('Width')
        ax.set_ylabel('Depth')
        ax.set_zlabel('Height')
        ax.set_xlim(0, PALLET_WIDTH)
        ax.set_ylim(0, PALLET_DEPTH)
        ax.set_zlim(0, MAX_HEIGHT)
        ax.view_init(elev=elev, azim=azim)
        ax.set_box_aspect([PALLET_WIDTH, PALLET_DEPTH, MAX_HEIGHT])

    plt.tight_layout()
    plt.show()

    plt.savefig("pallet.png", dpi=300)
    print("Saved pallet visualization to pallet.png")


def plot_pallet_plotly(boxes, pallet_width=PALLET_WIDTH, pallet_depth=PALLET_DEPTH, max_height=MAX_HEIGHT):
    # Determine min/max weights for coloring scale
    min_wt = min(b.weight for b in boxes)
    max_wt = max(b.weight for b in boxes)

    def weight_to_color(weight):
        t = (weight - min_wt) / (max_wt - min_wt) if max_wt > min_wt else 0
        if t < 0.5:
            r = int(255 * (t * 2))
            g = 255
            b = 0
        else:
            r = 255
            g = int(255 * (1 - (t - 0.5) * 2))
            b = 0
        return f"rgb({r},{g},{b})"

    fig = go.Figure()

    # Optional: draw pallet base as a thin box at z=0
    fig.add_trace(go.Mesh3d(
        x=[0, pallet_width, pallet_width, 0, 0, pallet_width, pallet_width, 0],
        y=[0, 0, pallet_depth, pallet_depth, 0, 0, pallet_depth, pallet_depth],
        z=[0, 0, 0, 0, 0.1, 0.1, 0.1, 0.1],
        color='saddlebrown',
        opacity=0.5,
        name='Pallet Base',
        hoverinfo='skip'
    ))

    for b in boxes:
        x0, y0, z0 = b.x, b.y, b.z
        x1, y1, z1 = x0 + b.w, y0 + b.d, z0 + b.h

        corners = [
            [x0, y0, z0],
            [x1, y0, z0],
            [x1, y1, z0],
            [x0, y1, z0],
            [x0, y0, z1],
            [x1, y0, z1],
            [x1, y1, z1],
            [x0, y1, z1]
        ]

        faces = [
            [0, 1, 2], [0, 2, 3],
            [4, 5, 6], [4, 6, 7],
            [0, 1, 5], [0, 5, 4],
            [1, 2, 6], [1, 6, 5],
            [2, 3, 7], [2, 7, 6],
            [3, 0, 4], [3, 4, 7],
        ]

        xs, ys, zs = zip(*corners)

        fig.add_trace(go.Mesh3d(
            x=xs, y=ys, z=zs,
            i=[f[0] for f in faces],
            j=[f[1] for f in faces],
            k=[f[2] for f in faces],
            opacity=0.5,
            color=weight_to_color(b.weight),
            hovertext=f"id:{b.name} W:{b.w} D:{b.d} H:{b.h}<br>Wt:{b.weight}",
            hoverinfo="text"
        ))

        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7)
        ]
        for e in edges:
            fig.add_trace(go.Scatter3d(
                x=[corners[e[0]][0], corners[e[1]][0]],
                y=[corners[e[0]][1], corners[e[1]][1]],
                z=[corners[e[0]][2], corners[e[1]][2]],
                mode="lines",
                line=dict(color="black", width=4),
                showlegend=False
            ))

    fig.update_layout(
        scene=dict(
            xaxis=dict(title="Width", range=[0, pallet_width]),
            yaxis=dict(title="Depth", range=[0, pallet_depth]),
            zaxis=dict(title="Height", range=[0, max_height]),
            aspectmode='data'  # Keep aspect ratio proportional to data
        ),
        title="Pallet Arrangement Visualization (Weight-colored with Borders)"
    )

    fig.show()


def best_orientation(box):
    # possible orientations as tuples (w, d, h)
    orientations = [
        (box.w, box.d, box.h),
        (box.h, box.d, box.w),
        (box.w, box.h, box.d),
    ]
    # Find orientation with minimum height
    # but also prefers longest dimension in base (width or depth)
    best = None
    min_height = float('inf')

    for w, d, h in orientations:
        longest_side_on_base = max(w, d)
        # Condition: height should be minimum, and longest side on base should be max possible (to keep stable)
        if h < min_height or (h == min_height and longest_side_on_base > max(best[0], best[1]) if best else 0):
            best = (w, d, h)

    return best


def boxes_overlap(b1, x1, y1, z1, b2, x2, y2, z2):
    # Returns True if two boxes overlap in 3D space
    def overlap_1d(a_min, a_max, b_min, b_max):
        return not (a_max <= b_min or b_max <= a_min)

    x_overlap = overlap_1d(x1, x1 + b1.w, x2, x2 + b2.w)
    y_overlap = overlap_1d(y1, y1 + b1.d, y2, y2 + b2.d)
    z_overlap = overlap_1d(z1, z1 + b1.h, z2, z2 + b2.h)

    return x_overlap and y_overlap and z_overlap


def can_place(arrangement, box, x, y, z):
    # Check if box fits inside pallet bounds
    if x < 0 or y < 0 or z < 0:
        return False
    if x + box.w > PALLET_WIDTH or y + box.d > PALLET_DEPTH or z + box.h > MAX_HEIGHT:
        return False
    # Check collision with existing boxes
    for b, bx, by, bz in arrangement:
        if boxes_overlap(b, bx, by, bz, box, x, y, z):
            return False
    return True


def can_fit(x, y, w, d):
    return 0 <= x <= PALLET_WIDTH - w and 0 <= y <= PALLET_DEPTH - d


def get_orientations(box):
    # Return all valid box orientations (w, d, h)
    # Assuming box can be rotated on any axis but height always vertical
    return [
        (box.w, box.d, box.h),
        (box.d, box.w, box.h),
        (box.h, box.w, box.d),
        (box.h, box.d, box.w),
        (box.w, box.h, box.d),
        (box.d, box.h, box.w),
    ]


def is_supported(height_map, x, y, w, d, base_z, support_threshold=0.75):
    """
    Calculate the fraction of the box footprint area that is supported
    at the base_z level or higher.
    """
    footprint = height_map[x:x+w, y:y+d]
    supported_cells = np.sum(footprint >= base_z)
    total_cells = w * d
    support_ratio = supported_cells / total_cells
    return support_ratio >= support_threshold


def weight_support_check(weight_map, x, y, w, d, box_weight):
    """
    Check if weight_map cells underneath footprint can support this box's weight.
    Assume weight_map records max supported weight at each cell.
    """
    footprint_weights = weight_map[x:x+w, y:y+d]
    # Check if all footprint cells support this box's weight
    return np.all(footprint_weights >= box_weight)


def save_arrangement_json(arrangement, filename="pallet.json"):
    data = []
    for b, x, y, z in arrangement:
        data.append({
            "name": b.name,
            "width": b.w,
            "depth": b.d,
            "height": b.h,
            "weight": b.weight,
            "x": x,
            "y": y,
            "z": z,
        })
    with open(filename, "w") as f:
        json.dump(data, f, indent=2)
    print(f"Saved pallet arrangement JSON to {filename}")


# ----- MAIN -----
if __name__ == "__main__":
    
    def signal_handler(sig, frame):
        print("\nCtrl+C detected! Exiting gracefully...")
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    # Generate random boxes
    boxes = [
        Box(
            w=random.randint(5, 20),   # smaller boxes max 20" width
            d=random.randint(5, 20),   # smaller boxes max 20" depth
            h=random.randint(5, 20),   # smaller boxes max 20" height
            weight=random.randint(5, 50),  # weight in lbs
            name=f"B{i+1}"
        )
        for i in range(NUM_BOXES)
    ]

    # Try different random shuffles & pick best score
    best_score = 0
    best_arrangement = None

    for _ in range(500):  # number of random tries
        random.shuffle(boxes)
        arr = build_pallet(boxes)
        score = stability_score(arr)
        if score > best_score:
            best_score = score
            best_arrangement = arr

    print(f"Best stability score: {best_score}")
    print("Arrangement (Box, X, Y, Z):")
    for b, x, y, z in best_arrangement:
        print(f"{b} -> Pos({x},{y},{z})")

    # plot_pallet_three_views(best_arrangement)

    best_best_arrangement = []
    for b, x, y, z in best_arrangement:
        b.x, b.y, b.z = x, y, z
        best_best_arrangement.append(b)
    plot_pallet_plotly(best_best_arrangement)
    
    # save_arrangement_json(best_arrangement)

    
