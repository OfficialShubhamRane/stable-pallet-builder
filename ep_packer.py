"""Research-grounded pallet packer.

Implements, in place of the height-map greedy heuristic:

* Extreme Point placement and the Residual Space merit function
  -- Crainic, Perboli & Tadei (2008), "Extreme Point-Based Heuristics for
     Three-Dimensional Bin Packing", INFORMS J. on Computing 20(3), 368-384.
     (working paper: CIRRELT-2007-41)
* Item ordering rules (Volume-Height, Area-Height, Clustered Area-Height, ...)
  -- same paper, Section 4.1.
* Static mechanical equilibrium as the stability test: an item is stable when
  its centre of gravity projects inside the convex hull of its contact points
  ("support polygon"), which replaces the blunt full-support / percentage-area
  proxy
  -- Ramos, Oliveira, Goncalves & Lopes (2016), "A container loading algorithm
     with static mechanical equilibrium stability constraints",
     Transportation Research Part B 91, 565-581.
* Load-bearing limits propagated down the support chain
  -- Bischoff (2006), "Three-dimensional packing of items with limited load
     bearing strength", EJOR 168(3), 952-966.
* GRASP construction with a restricted candidate list, plus a
  destroy-and-rebuild improvement phase (the 50% removal ratio is the value
  tuned in the source)
  -- Parreno, Alvarez-Valdes, Oliveira & Tamarit (2008), "A Maximal-Space
     Algorithm for the Container Loading Problem", INFORMS J. on Computing
     20(3), 412-422, and the 2010 hybrid GRASP/VND follow-up.

See REFERENCES.md for what was taken from each and what was left out.
"""

import math
import random

EPS = 1e-6


# ----- geometry helpers -----
def _convex_hull(points):
    """Monotone chain hull. Returns CCW hull vertices, or [] if degenerate."""
    pts = sorted(set(points))
    if len(pts) < 3:
        return []

    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2:
                (ax, ay), (bx, by) = out[-2], out[-1]
                if (bx - ax) * (p[1] - ay) - (by - ay) * (p[0] - ax) <= 0:
                    out.pop()
                else:
                    break
            out.append(p)
        return out

    lower = half(pts)
    upper = half(reversed(pts))
    hull = lower[:-1] + upper[:-1]
    return hull if len(hull) >= 3 else []


def _inside_hull(hull, px, py, margin=0.0):
    """True if (px,py) is inside a CCW hull by at least `margin`."""
    n = len(hull)
    if n < 3:
        return False
    for i in range(n):
        ax, ay = hull[i]
        bx, by = hull[(i + 1) % n]
        ex, ey = bx - ax, by - ay
        length = math.hypot(ex, ey)
        if length < EPS:
            continue
        # signed distance to the edge; positive is the interior side for CCW
        if (ex * (py - ay) - ey * (px - ax)) / length < margin:
            return False
    return True


def _polygon_area(poly):
    """Absolute area of a simple polygon (shoelace)."""
    n = len(poly)
    if n < 3:
        return 0.0
    total = 0.0
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        total += ax * by - bx * ay
    return abs(total) / 2.0


def _clip_to_rect(poly, x0, y0, x1, y1):
    """Sutherland-Hodgman clip of a convex polygon to an axis-aligned rect."""
    def clip(pts, inside, intersect):
        out = []
        for i in range(len(pts)):
            cur, prv = pts[i], pts[i - 1]
            c_in, p_in = inside(cur), inside(prv)
            if c_in:
                if not p_in:
                    out.append(intersect(prv, cur))
                out.append(cur)
            elif p_in:
                out.append(intersect(prv, cur))
        return out

    def lerp(a, b, t):
        return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)

    pts = list(poly)
    for inside, intersect in (
        (lambda p: p[0] >= x0 - EPS, lambda a, b: lerp(a, b, (x0 - a[0]) / (b[0] - a[0]) if abs(b[0] - a[0]) > EPS else 0.0)),
        (lambda p: p[0] <= x1 + EPS, lambda a, b: lerp(a, b, (x1 - a[0]) / (b[0] - a[0]) if abs(b[0] - a[0]) > EPS else 0.0)),
        (lambda p: p[1] >= y0 - EPS, lambda a, b: lerp(a, b, (y0 - a[1]) / (b[1] - a[1]) if abs(b[1] - a[1]) > EPS else 0.0)),
        (lambda p: p[1] <= y1 + EPS, lambda a, b: lerp(a, b, (y1 - a[1]) / (b[1] - a[1]) if abs(b[1] - a[1]) > EPS else 0.0)),
    ):
        if not pts:
            return []
        pts = clip(pts, inside, intersect)
    return pts


class Placement:
    """One box fixed in space, with its support relationships."""

    __slots__ = ("box", "x", "y", "z", "w", "d", "h", "supporters", "borne", "lbcp")

    def __init__(self, box, x, y, z, w, d, h):
        self.box = box
        self.x, self.y, self.z = x, y, z
        self.w, self.d, self.h = w, d, h
        self.supporters = []   # (Placement, share of this box's weight)
        self.borne = 0.0       # weight currently resting on top of this box
        # Load Bearable Convex Polygon: the part of this box's top face that can
        # actually carry load, which is not the whole face once the box is
        # itself only partially supported.
        self.lbcp = []

    @property
    def top(self):
        return self.z + self.h

    def as_tuple(self):
        placed = self.box.copy()
        placed.w, placed.d, placed.h = self.w, self.d, self.h
        return (placed, self.x, self.y, self.z)


class PalletPacker:
    """Extreme-point packer with equilibrium and load-bearing constraints."""

    def __init__(self, width, depth, max_height,
                 stability_margin=0.5, allow_tipping_rotations=True,
                 check_load_bearing=True):
        self.W = float(width)
        self.D = float(depth)
        self.H = float(max_height)
        self.stability_margin = stability_margin
        self.allow_tipping = allow_tipping_rotations
        self.check_load_bearing = check_load_bearing
        self.reset()

    def reset(self):
        self.placed = []
        # Extreme Points. The empty pallet offers only the origin corner.
        self.eps = [(0.0, 0.0, 0.0)]

    # --- orientations ---
    def _orientations(self, box):
        dims = (box.w, box.d, box.h)
        if self.allow_tipping:
            cands = {
                (dims[0], dims[1], dims[2]), (dims[1], dims[0], dims[2]),
                (dims[2], dims[1], dims[0]), (dims[1], dims[2], dims[0]),
                (dims[0], dims[2], dims[1]), (dims[2], dims[0], dims[1]),
            }
        else:
            cands = {(dims[0], dims[1], dims[2]), (dims[1], dims[0], dims[2])}
        return sorted(cands)

    # --- extreme point maintenance (Crainic et al., Algorithm 1) ---
    def _project(self, px, py, pz, axis):
        """Slide a point toward 0 along `axis` until it meets cargo or the wall."""
        best = 0.0
        for p in self.placed:
            if axis == 0:
                if (p.y - EPS <= py < p.y + p.d + EPS and
                        p.z - EPS <= pz < p.z + p.h + EPS and
                        p.x + p.w <= px + EPS):
                    best = max(best, p.x + p.w)
            elif axis == 1:
                if (p.x - EPS <= px < p.x + p.w + EPS and
                        p.z - EPS <= pz < p.z + p.h + EPS and
                        p.y + p.d <= py + EPS):
                    best = max(best, p.y + p.d)
            else:
                if (p.x - EPS <= px < p.x + p.w + EPS and
                        p.y - EPS <= py < p.y + p.d + EPS and
                        p.z + p.h <= pz + EPS):
                    best = max(best, p.z + p.h)
        return best

    def _spawn_eps(self, x, y, z, w, d, h):
        """The six EPs a newly placed item generates (paper, Section 3)."""
        new = [
            (x + w, self._project(x + w, y, z, 1), z),
            (x + w, y, self._project(x + w, y, z, 2)),
            (self._project(x, y + d, z, 0), y + d, z),
            (x, y + d, self._project(x, y + d, z, 2)),
            (self._project(x, y, z + h, 0), y, z + h),
            (x, self._project(x, y, z + h, 1), z + h),
        ]
        for p in new:
            if p[0] < self.W - EPS and p[1] < self.D - EPS and p[2] < self.H - EPS:
                self.eps.append(p)
        # Dedupe and keep the list ordered low-to-high, which makes the
        # first-fit scan naturally prefer deep, low positions.
        self.eps = sorted(set(self.eps), key=lambda p: (p[2], p[1], p[0]))

    # --- residual space (paper, Section 4.3) ---
    def _residual_space(self, x, y, z):
        rs_x, rs_y, rs_z = self.W - x, self.D - y, self.H - z
        for p in self.placed:
            if (p.y - EPS <= y < p.y + p.d + EPS and p.z - EPS <= z < p.z + p.h + EPS
                    and p.x >= x - EPS):
                rs_x = min(rs_x, p.x - x)
            if (p.x - EPS <= x < p.x + p.w + EPS and p.z - EPS <= z < p.z + p.h + EPS
                    and p.y >= y - EPS):
                rs_y = min(rs_y, p.y - y)
            if (p.x - EPS <= x < p.x + p.w + EPS and p.y - EPS <= y < p.y + p.d + EPS
                    and p.z >= z - EPS):
                rs_z = min(rs_z, p.z - z)
        return max(0.0, rs_x), max(0.0, rs_y), max(0.0, rs_z)

    # --- feasibility ---
    def _fits(self, x, y, z, w, d, h):
        if x < -EPS or y < -EPS or z < -EPS:
            return False
        if x + w > self.W + EPS or y + d > self.D + EPS or z + h > self.H + EPS:
            return False
        for p in self.placed:
            if (x < p.x + p.w - EPS and p.x < x + w - EPS and
                    y < p.y + p.d - EPS and p.y < y + d - EPS and
                    z < p.z + p.h - EPS and p.z < z + h - EPS):
                return False
        return True

    def _contacts(self, x, y, z, w, d):
        """Supporting placements under the footprint, with contact areas."""
        out = []
        for p in self.placed:
            if abs(p.top - z) > EPS:
                continue
            ox = min(x + w, p.x + p.w) - max(x, p.x)
            oy = min(y + d, p.y + p.d) - max(y, p.y)
            if ox > EPS and oy > EPS:
                out.append((p, ox * oy,
                            (max(x, p.x), max(y, p.y),
                             min(x + w, p.x + p.w), min(y + d, p.y + p.d))))
        return out

    def _support_polygon(self, x, y, z, w, d):
        """Convex hull of the load-bearable contact area under a footprint.

        Ramos, Oliveira & Lopes (2016) define the support polygon as the convex
        hull of the contact points. Gao et al. (2025) point out that raw
        geometric contact overstates it once a stack is three or more layers
        deep: a box can rest on a region that is itself unsupported underneath.
        They fix it by intersecting against Load Bearable Convex Polygons
        instead of whole top faces, which is what this does.
        """
        if z <= EPS:
            return [(x, y), (x + w, y), (x + w, y + d), (x, y + d)]
        pts = []
        for p in self.placed:
            if abs(p.top - z) > EPS or not p.lbcp:
                continue
            # A box that only meets this footprint along an edge or at a corner
            # carries no load. Without this test the degenerate sliver that
            # clipping returns still contributes points to the hull, which can
            # inflate it enough to swallow a centre of gravity that is in fact
            # unsupported.
            if (min(x + w, p.x + p.w) - max(x, p.x) <= EPS or
                    min(y + d, p.y + p.d) - max(y, p.y) <= EPS):
                continue
            clipped = _clip_to_rect(p.lbcp, x, y, x + w, y + d)
            if _polygon_area(clipped) <= EPS:
                continue
            pts.extend(clipped)
        return _convex_hull(pts)

    def _stable(self, x, y, z, w, d, contacts):
        """Static mechanical equilibrium: centre of gravity over the support.

        For a single rigid body this is exactly equivalent to asking whether a
        set of non-negative contact forces balancing weight and moment exists,
        which is the condition Ramos et al. solve for numerically.
        """
        if z <= EPS:
            return True
        if not contacts:
            return False
        hull = self._support_polygon(x, y, z, w, d)
        if not hull:
            return False
        return _inside_hull(hull, x + w / 2.0, y + d / 2.0, self.stability_margin)

    def _load_delta(self, contacts, weight, out):
        """Extra load each box down the support chain would have to carry.

        Weight splits between supporters in proportion to contact area, and
        each supporter passes its share on to whatever is holding it up.
        """
        total = sum(a for _, a, _ in contacts)
        if total <= EPS:
            return
        for p, area, _ in contacts:
            share = weight * area / total
            out[id(p)] = out.get(id(p), 0.0) + share
            self._propagate(p, share, out)

    def _propagate(self, placement, weight, out):
        for sup, frac in placement.supporters:
            passed = weight * frac
            out[id(sup)] = out.get(id(sup), 0.0) + passed
            self._propagate(sup, passed, out)

    def _load_ok(self, contacts, weight):
        if not self.check_load_bearing:
            return True, {}
        delta = {}
        self._load_delta(contacts, weight, delta)
        index = {id(p): p for p in self.placed}
        for key, extra in delta.items():
            p = index[key]
            cap = getattr(p.box, "max_load", None)
            if cap is not None and p.borne + extra > cap + EPS:
                return False, {}
        return True, delta

    # --- placement ---
    def try_place(self, box, merit="rs", rcl_size=1, rng=None, weight_bias=0.0):
        """Find candidate positions for `box` and place it. Returns bool.

        merit="rs" scores every feasible (EP, orientation) with the Residual
        Space function; a restricted candidate list of size `rcl_size` makes
        the choice randomized for GRASP.
        """
        cands = []
        max_w = max((p.box.weight for p in self.placed), default=box.weight) or 1

        for (x, y, z) in self.eps:
            for (w, d, h) in self._orientations(box):
                if not self._fits(x, y, z, w, d, h):
                    continue
                contacts = self._contacts(x, y, z, w, d)
                if not self._stable(x, y, z, w, d, contacts):
                    continue
                ok, delta = self._load_ok(contacts, box.weight)
                if not ok:
                    continue

                rs = self._residual_space(x, y, z)
                # Residual Space merit: minimise slack left around the item.
                score = (rs[0] - w) + (rs[1] - d) + (rs[2] - h)
                # Pallet-specific term: heavy cargo pays to go high.
                score += weight_bias * (box.weight / max_w) * (z / self.H) * (self.W + self.D + self.H)
                cands.append((score, x, y, z, w, d, h, contacts, delta))

        if not cands:
            return False

        cands.sort(key=lambda c: c[0])
        pick = cands[0]
        if rcl_size > 1 and rng is not None:
            pick = rng.choice(cands[:min(rcl_size, len(cands))])

        _, x, y, z, w, d, h, contacts, delta = pick
        pl = Placement(box, x, y, z, w, d, h)
        # A box on the deck can bear load anywhere on its top face; one that is
        # only partly supported can bear it only over its own support polygon.
        pl.lbcp = self._support_polygon(x, y, z, w, d)
        total = sum(a for _, a, _ in contacts)
        if total > EPS:
            pl.supporters = [(p, a / total) for p, a, _ in contacts]
        index = {id(p): p for p in self.placed}
        for key, extra in delta.items():
            index[key].borne += extra

        self.placed.append(pl)
        self._spawn_eps(x, y, z, w, d, h)
        return True

    def arrangement(self):
        return [p.as_tuple() for p in self.placed]


# ----- item ordering rules (Crainic et al., Section 4.1) -----
def _clustered(boxes, key, span, delta):
    """Bucket a measure into delta-percent clusters, then sort within."""
    width = max(span * delta / 100.0, EPS)
    return sorted(boxes, key=lambda b: (-int(key(b) / width), -b.w * b.d, -b.h))


def ordering_rules(pallet_w, pallet_d, pallet_h, delta=10):
    area = pallet_w * pallet_d
    return {
        "volume-height": lambda bs: sorted(bs, key=lambda b: (-b.volume(), -b.h)),
        "height-volume": lambda bs: sorted(bs, key=lambda b: (-b.h, -b.volume())),
        "area-height": lambda bs: sorted(bs, key=lambda b: (-(b.w * b.d), -b.h)),
        "height-area": lambda bs: sorted(bs, key=lambda b: (-b.h, -(b.w * b.d))),
        "clustered-area-height":
            lambda bs: _clustered(bs, lambda b: b.w * b.d, area, delta),
        "clustered-height-area":
            lambda bs: _clustered(bs, lambda b: b.h, pallet_h, delta),
        # Pallet-specific: densest cargo first so mass ends up low.
        "density": lambda bs: sorted(bs, key=lambda b: (-b.density(), -b.volume())),
        "weight": lambda bs: sorted(bs, key=lambda b: (-b.weight, -b.volume())),
    }


def pack_order(boxes, width, depth, max_height, rng=None, rcl_size=1,
               weight_bias=0.0, **kw):
    """Pack boxes in the given order. Returns (packer, unplaced)."""
    packer = PalletPacker(width, depth, max_height, **kw)
    unplaced = []
    for b in boxes:
        if not packer.try_place(b, rcl_size=rcl_size, rng=rng,
                                weight_bias=weight_bias):
            unplaced.append(b)
    return packer, unplaced


def search(boxes, width, depth, max_height, score_fn,
           iterations=60, seed=None, rebuild_ratio=0.5, rcl_size=3,
           weight_bias=1.0, stability_margin=0.5,
           allow_tipping_rotations=True, check_load_bearing=True,
           verbose=False):
    """GRASP construction plus destroy-and-rebuild improvement.

    Ranked by (boxes placed, score_fn) so a prettier score on fewer boxes never
    wins. The 0.5 rebuild ratio is the value Parreno et al. tuned to.
    """
    rng = random.Random(seed)
    rules = ordering_rules(width, depth, max_height)
    opts = dict(stability_margin=stability_margin,
                allow_tipping_rotations=allow_tipping_rotations,
                check_load_bearing=check_load_bearing,
                weight_bias=weight_bias)

    def evaluate(packer, unplaced):
        arr = packer.arrangement()
        return (len(arr), score_fn(arr)), arr, unplaced

    best_key, best_arr, best_unplaced, best_order = (-1, -1.0), [], list(boxes), None
    rule_names = list(rules)

    for i in range(iterations):
        if i < len(rule_names):
            order = rules[rule_names[i]](boxes)
            greedy = True
        elif best_order is not None and rng.random() < 0.5:
            # Destroy-and-rebuild: keep a prefix of the best order, reshuffle
            # the rest. Equivalent to dropping ~rebuild_ratio of the cargo.
            order = list(best_order)
            cut = int(len(order) * (1 - rebuild_ratio))
            tail = order[cut:]
            rng.shuffle(tail)
            order = order[:cut] + tail
            greedy = False
        else:
            order = rules[rng.choice(rule_names)](boxes)
            greedy = False

        packer, unplaced = pack_order(
            order, width, depth, max_height, rng=rng,
            rcl_size=1 if greedy else rcl_size, **opts)
        key, arr, unpl = evaluate(packer, unplaced)
        if key > best_key:
            best_key, best_arr, best_unplaced, best_order = key, arr, unpl, order
            if verbose:
                label = rule_names[i] if i < len(rule_names) else "grasp"
                print(f"  iter {i:3d} [{label}]: placed {key[0]}/{len(boxes)}  score {key[1]}")

    return best_arr, best_unplaced, best_key[1]
