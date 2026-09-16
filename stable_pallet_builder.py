"""Stable pallet builder.

Packs a set of boxes onto a pallet with a height-map greedy placer, scoring
each candidate position on support, height, centering and wasted volume, then
searches over box orderings for the most stable arrangement.
"""

import itertools
import json
import math
import random
import signal
import sys

import ep_packer
import matplotlib.pyplot as plt
import numpy as np
import plotly.graph_objects as go
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from numpy.lib.stride_tricks import sliding_window_view

# Configurable parameters
PALLET_WIDTH = 40      # inches
PALLET_DEPTH = 48      # inches
MAX_HEIGHT = 60        # inches
NUM_BOXES = 25
GRID_STEP = 2          # inch resolution
SUPPORT_THRESHOLD = 0.75  # minimum support ratio for stability

# Search parameters
SEARCH_ITERATIONS = 500

# Carton mix. Real pallets carry a handful of repeated carton sizes, not 25
# one-off shapes, so boxes are drawn from a small catalog instead of being
# randomised dimension by dimension. Fewer types = less variability = tighter,
# more stable packs.
CARTON_TYPES = 5          # distinct carton sizes in the catalog
CARTON_MIN = 6            # smallest carton side, inches
CARTON_MAX = 20           # largest carton side, inches
WEIGHT_JITTER = 0.15      # per-box weight variation within a carton type (0 = identical)
LOAD_BEARING_PSI = (0.3, 1.5)  # lbs/sq-in a carton top can bear before crushing

# Extreme-point packer (see ep_packer.py and REFERENCES.md)
EP_ITERATIONS = 200       # GRASP iterations
EP_WEIGHT_BIAS = 8.0      # how hard heavy cargo is pushed toward the deck
EP_STABILITY_MARGIN = 0.5  # inches the centre of gravity must clear the support hull
# Cartons usually carry a "this way up" constraint, so only the two base
# rotations are allowed by default. Tipping a carton onto its edge also packs
# measurably worse, so this is not a restriction that costs anything here.
EP_ALLOW_TIPPING = False

# Placement cost weights (lower cost wins). Tune these to change packing style.
W_HEIGHT = 1.0    # prefer low placements, scaled up for heavy boxes
W_SUPPORT = 0.6   # prefer fully supported footprints
W_CENTER = 0.25   # prefer positions near the pallet centre
W_GAP = 0.4       # penalise voids left under the box

GRID_W = int(PALLET_WIDTH / GRID_STEP)
GRID_D = int(PALLET_DEPTH / GRID_STEP)


# ----- DATA STRUCTURE -----
class Box:
    def __init__(self, w, d, h, weight, name, carton=None, max_load=None):
        self.w = w
        self.d = d
        self.h = h
        self.weight = weight
        self.name = name
        self.carton = carton  # label of the carton size this box came from
        self.max_load = max_load  # lbs this box can carry on top of it
        self.x = None  # placement coordinates on pallet
        self.y = None
        self.z = None

    def volume(self):
        return self.w * self.d * self.h

    def density(self):
        return self.weight / self.volume()

    def copy(self):
        return Box(self.w, self.d, self.h, self.weight, self.name, self.carton,
                   self.max_load)

    def __repr__(self):
        tag = f"/{self.carton}" if self.carton else ""
        return f"{self.name}{tag}(W:{self.w},D:{self.d},H:{self.h},Wt:{self.weight})"


# ----- STABILITY SCORING -----
def stability_score(arrangement):
    """
    arrangement: list of (box, x, y, z)
    Returns a score between 0 and 1
    """
    if not arrangement:
        return 0

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
    no_overhang = max(0.0, no_overhang)  # never let the term go negative

    # Final weighted score
    score = w_bottom_ratio * 0.4 + balance_factor * 0.4 + no_overhang * 0.2
    return round(score, 4)


# ----- PACKING -----
def _cells(dim):
    """Grid cells a real dimension occupies.

    Rounds UP: a 7" box on a 2" grid reserves 4 cells (8"), not 3 (6").
    Truncating here lets neighbouring boxes physically overlap.
    """
    return max(1, int(math.ceil(dim / GRID_STEP - 1e-9)))


def _base_orientations(box):
    """Unique footprint orientations, height stays vertical."""
    if box.w == box.d:
        return [(box.w, box.d, box.h)]
    return [(box.w, box.d, box.h), (box.d, box.w, box.h)]


# Centre-distance maps are reused across every placement, so build them once
# per footprint size instead of re-sorting the position list for every box.
_DIST_CACHE = {}


def _centre_distance(wi, di):
    key = (wi, di)
    cached = _DIST_CACHE.get(key)
    if cached is None:
        xs = np.arange(GRID_W - wi + 1, dtype=float)[:, None] + wi / 2.0
        ys = np.arange(GRID_D - di + 1, dtype=float)[None, :] + di / 2.0
        dist = np.sqrt((xs - GRID_W / 2.0) ** 2 + (ys - GRID_D / 2.0) ** 2)
        peak = dist.max()
        cached = dist / peak if peak > 0 else dist
        _DIST_CACHE[key] = cached
    return cached


def _best_placement(height_map, box, weight_norm):
    """Lowest-cost (x_idx, y_idx, z, orientation) for one box, or None.

    Every candidate position is evaluated at once with a sliding window over the
    height map, rather than looping over ~GRID_W*GRID_D positions in Python.
    """
    best = None
    best_cost = math.inf

    for w, d, h in _base_orientations(box):
        wi, di = _cells(w), _cells(d)
        if wi > GRID_W or di > GRID_D:
            continue

        win = sliding_window_view(height_map, (wi, di))  # (nx, ny, wi, di)
        area = wi * di
        z = win.max(axis=(2, 3))                          # resting height
        contact = (win == z[:, :, None, None]).sum(axis=(2, 3))
        support = contact / area
        void = (z * area - win.sum(axis=(2, 3))) / float(area * MAX_HEIGHT)

        valid = (z + h) <= MAX_HEIGHT
        # Floor placements are always fully supported; stacked ones must meet
        # SUPPORT_THRESHOLD so boxes cannot balance on a corner.
        valid &= (z == 0) | (support >= SUPPORT_THRESHOLD)
        if not valid.any():
            continue

        # Heavy boxes pay a steeper penalty for going high, which keeps mass low.
        cost = (W_HEIGHT * (1.0 + 2.0 * weight_norm) * (z / float(MAX_HEIGHT))
                + W_SUPPORT * (1.0 - support)
                + W_CENTER * _centre_distance(wi, di)
                + W_GAP * void)
        cost = np.where(valid, cost, np.inf)

        flat = int(np.argmin(cost))
        c = float(cost.flat[flat])
        if c < best_cost:
            xi, yi = np.unravel_index(flat, cost.shape)
            best_cost = c
            best = (int(xi), int(yi), int(z[xi, yi]), (w, d, h), wi, di)

    return best


def build_pallet(boxes, max_depth=1, presort=False):
    """Pack boxes onto the pallet.

    boxes    -- placement is attempted in the order given. Set presort=True to
                sort by density first (the old hard-coded behaviour, which made
                the caller's ordering irrelevant).
    max_depth -- extra retry passes over boxes that did not fit. Heights change
                as other boxes land, so a retry can succeed where the first
                attempt failed.

    Returns (arrangement, unplaced) where arrangement is a list of
    (box, x, y, z) and unplaced is the list of boxes that did not fit.
    """
    order = sorted(boxes, key=lambda b: b.density(), reverse=True) if presort else list(boxes)

    height_map = np.zeros((GRID_W, GRID_D), dtype=np.int32)
    arrangement = []
    pending = order
    max_weight = max((b.weight for b in order), default=1) or 1

    for _ in range(max_depth + 1):
        if not pending:
            break
        unplaced = []
        for b in pending:
            spot = _best_placement(height_map, b, b.weight / max_weight)
            if spot is None:
                unplaced.append(b)
                continue
            xi, yi, z, (w, d, h), wi, di = spot
            placed = b.copy()
            placed.w, placed.d, placed.h = w, d, h
            arrangement.append((placed, xi * GRID_STEP, yi * GRID_STEP, z))
            height_map[xi:xi + wi, yi:yi + di] = z + h
        pending = unplaced

    return arrangement, pending


def find_overlaps(arrangement):
    """Return every physically overlapping pair. Should always be empty."""
    bad = []
    for (b1, x1, y1, z1), (b2, x2, y2, z2) in itertools.combinations(arrangement, 2):
        if boxes_overlap(b1, x1, y1, z1, b2, x2, y2, z2):
            bad.append((b1.name, b2.name))
    return bad


def search_best_arrangement(boxes, iterations=SEARCH_ITERATIONS, seed=None, verbose=False):
    """Search box orderings for the most stable arrangement.

    The old loop shuffled the box list and then let build_pallet re-sort it by
    density, so all iterations produced the same arrangement. Here the ordering
    the search picks is the ordering that gets packed: seeded heuristics first,
    then local perturbations of the best order found so far.
    """
    rng = random.Random(seed)

    seeds = [
        sorted(boxes, key=lambda b: b.density(), reverse=True),
        sorted(boxes, key=lambda b: b.weight, reverse=True),
        sorted(boxes, key=lambda b: b.volume(), reverse=True),
        sorted(boxes, key=lambda b: b.w * b.d, reverse=True),
        sorted(boxes, key=lambda b: (b.h, b.weight), reverse=True),
    ]

    best_order = None
    best_result = None
    best_key = (-1, -1.0)

    for i in range(iterations):
        if i < len(seeds):
            order = seeds[i]
        elif best_order is None:
            order = rng.sample(list(boxes), len(boxes))
        else:
            # Local search: perturb the best known order with a few swaps.
            order = list(best_order)
            for _ in range(rng.randint(1, max(2, len(order) // 4))):
                a, c = rng.randrange(len(order)), rng.randrange(len(order))
                order[a], order[c] = order[c], order[a]

        arrangement, unplaced = build_pallet(order)
        # Placing more boxes always beats a prettier score on fewer boxes.
        key = (len(arrangement), stability_score(arrangement))
        if key > best_key:
            best_key = key
            best_order = order
            best_result = (arrangement, unplaced)
            if verbose:
                print(f"  iter {i:4d}: placed {key[0]}/{len(boxes)}  score {key[1]}")

    return best_result[0], best_result[1], best_key[1]
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
            hovertext=f"id:{b.name} type:{b.carton} W:{b.w} D:{b.d} H:{b.h}<br>Wt:{b.weight}",
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
            "carton": b.carton,
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
def build_carton_catalog(types=CARTON_TYPES, seed=None):
    """A small set of distinct carton sizes, like a real warehouse would stock.

    Sides are multiples of GRID_STEP so a carton occupies whole height-map
    cells; odd sizes get rounded up into the grid and waste the remainder.
    """
    rng = random.Random(seed)
    step = GRID_STEP
    lo = max(1, CARTON_MIN // step)
    hi = max(lo, CARTON_MAX // step)

    catalog = []
    seen = set()
    attempts = 0
    while len(catalog) < types and attempts < types * 200:
        attempts += 1
        w, d, h = (rng.randint(lo, hi) * step for _ in range(3))
        w, d = max(w, d), min(w, d)  # normalise footprint so W/D swaps aren't duplicates
        if (w, d, h) in seen:
            continue
        seen.add((w, d, h))
        catalog.append({
            "label": f"C{len(catalog)+1}",
            "w": w, "d": d, "h": h,
            # lbs per cubic inch, so weight tracks carton size sensibly
            "density": rng.uniform(0.004, 0.02),
            # lbs per square inch the carton top can bear (Bischoff 2006)
            "bearing": rng.uniform(*LOAD_BEARING_PSI),
        })
    return catalog


def check_load_bearing(arrangement, tol=1e-6):
    """Cartons carrying more than their rated top load. Should be empty.

    Weight is passed down through whatever a box rests on, split by contact
    area, following Bischoff (2006).
    """
    borne = {id(b): 0.0 for b, *_ in arrangement}
    index = {id(b): b for b, *_ in arrangement}
    order = sorted(arrangement, key=lambda t: -t[3])  # top down
    for b, x, y, z in order:
        load = b.weight + borne[id(b)]
        supports = []
        for o, ox, oy, oz in arrangement:
            if o is b or abs(oz + o.h - z) > 1e-6:
                continue
            ow = min(x + b.w, ox + o.w) - max(x, ox)
            od = min(y + b.d, oy + o.d) - max(y, oy)
            if ow > 1e-6 and od > 1e-6:
                supports.append((o, ow * od))
        total = sum(a for _, a in supports)
        for o, a in supports:
            borne[id(o)] += load * a / total
    over = []
    for key, load in borne.items():
        b = index[key]
        if b.max_load is not None and load > b.max_load + tol:
            over.append(f"{b.name} {load:.0f}/{b.max_load:.0f}lb")
    return over


def check_stability(arrangement):
    """Boxes whose centre of gravity falls outside their support hull."""
    bad = []
    for b, x, y, z in arrangement:
        if z <= 1e-6:
            continue
        pts = []
        for o, ox, oy, oz in arrangement:
            if o is b or abs(oz + o.h - z) > 1e-6:
                continue
            x0, y0 = max(x, ox), max(y, oy)
            x1, y1 = min(x + b.w, ox + o.w), min(y + b.d, oy + o.d)
            if x1 - x0 > 1e-6 and y1 - y0 > 1e-6:
                pts += [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        hull = ep_packer._convex_hull(pts)
        if not ep_packer._inside_hull(hull, x + b.w / 2, y + b.d / 2, 0.0):
            bad.append(b.name)
    return bad


def generate_boxes(count=NUM_BOXES, seed=None, types=CARTON_TYPES, jitter=WEIGHT_JITTER):
    """Draw `count` boxes from a catalog of `types` distinct carton sizes."""
    catalog = build_carton_catalog(types, seed=seed)
    rng = random.Random(seed)

    boxes = []
    for i in range(count):
        c = catalog[i % len(catalog)] if i < len(catalog) else rng.choice(catalog)
        base = c["w"] * c["d"] * c["h"] * c["density"]
        weight = int(round(base * rng.uniform(1 - jitter, 1 + jitter)))
        boxes.append(Box(
            w=c["w"], d=c["d"], h=c["h"],
            weight=max(1, weight),
            name=f"B{i+1}",
            carton=c["label"],
            max_load=round(c["bearing"] * c["w"] * c["d"], 1),
        ))
    rng.shuffle(boxes)
    return boxes


def describe_mix(boxes):
    """One line per carton type: size, count, weight range."""
    groups = {}
    for b in boxes:
        groups.setdefault(b.carton or "-", []).append(b)
    lines = []
    for label in sorted(groups, key=lambda k: -len(groups[k])):
        g = groups[label]
        b = g[0]
        weights = [x.weight for x in g]
        span = f"{min(weights)}" if min(weights) == max(weights) else f"{min(weights)}-{max(weights)}"
        lines.append(f"  {label}: {b.w}x{b.d}x{b.h}\"  x{len(g):<3} {span} lbs")
    return lines


if __name__ == "__main__":

    def signal_handler(sig, frame):
        print("\nCtrl+C detected! Exiting gracefully...")
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    args = sys.argv[1:]

    def arg(flag, default, cast=int):
        if flag in args:
            return cast(args[args.index(flag) + 1])
        return default

    seed = arg("--seed", None)
    iterations = arg("--iterations", SEARCH_ITERATIONS)
    show_plot = "--no-plot" not in args

    carton_types = arg("--carton-types", CARTON_TYPES)

    packer = "heightmap" if "--packer" in args and args[args.index("--packer") + 1] == "heightmap" else "ep"

    boxes = generate_boxes(NUM_BOXES, seed=seed, types=carton_types)
    print(f"Carton mix ({carton_types} type(s), {len(boxes)} boxes):")
    for line in describe_mix(boxes):
        print(line)

    if packer == "ep":
        arrangement, unplaced, score = ep_packer.search(
            boxes, PALLET_WIDTH, PALLET_DEPTH, MAX_HEIGHT, stability_score,
            iterations=min(iterations, EP_ITERATIONS), seed=seed,
            weight_bias=EP_WEIGHT_BIAS, stability_margin=EP_STABILITY_MARGIN,
            allow_tipping_rotations=EP_ALLOW_TIPPING or "--allow-tipping" in args,
            verbose="--verbose" in args,
        )
    else:
        arrangement, unplaced, score = search_best_arrangement(
            boxes, iterations=iterations, seed=seed, verbose="--verbose" in args
        )
    print(f"Packer: {packer}")

    print(f"Best stability score: {score}")
    print(f"Placed {len(arrangement)}/{len(boxes)} boxes"
          + (f" (unplaced: {', '.join(b.name for b in unplaced)})" if unplaced else ""))

    overlaps = find_overlaps(arrangement)
    if overlaps:
        print(f"WARNING: {len(overlaps)} overlapping pairs: {overlaps[:5]}")
    else:
        print("No overlapping boxes.")

    unstable = check_stability(arrangement)
    print("Equilibrium: "
          + ("every box's centre of gravity is over its support"
             if not unstable else f"UNSTABLE: {unstable}"))

    print(f"Load bearing: "
          + ("no carton over its rated top load"
             if not check_load_bearing(arrangement)
             else f"OVER LIMIT: {check_load_bearing(arrangement)}"))
    print(f"Stack height: {max((z + b.h for b, _, _, z in arrangement), default=0)}\" of {MAX_HEIGHT}\"")
    print("Arrangement (Box, X, Y, Z):")
    for b, x, y, z in arrangement:
        print(f"{b} -> Pos({x},{y},{z})")

    save_arrangement_json(arrangement)

    if show_plot:
        positioned = []
        for b, x, y, z in arrangement:
            b.x, b.y, b.z = x, y, z
            positioned.append(b)
        plot_pallet_plotly(positioned)
