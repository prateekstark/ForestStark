"""Planting layouts: roadside row (Katar), surveyed grid with fleet split (Jaal), contour rows
with tool choice (Pahadi). Pure functions over site data so they are unit-testable."""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .geometry import Polyline, polygon_bounds


@dataclass
class PlanSpot:
    id: int
    x: float
    y: float
    row: int = 0
    station: float = 0.0
    tool: str = "auto"
    note: str = ""
    skip: bool = False          # planned skip (survey says no)
    hidden_hazard: str = ""     # simulation ground truth only, never read by the robot
    extra: dict = field(default_factory=dict)


# ----------------------------------------------------------------------------- roadside (Katar)
def planting_line(site) -> Polyline:
    rs = site["roadside"]
    return Polyline(rs["shoulder_centerline"]).offset(rs["planting_offset"])


def roadside_spots(site, spacing: float, surveyed: bool = True, tree_clearance: float = 2.0):
    """Spots every `spacing` m along the planting line between the configured stations.

    surveyed=True: the utility survey is available, so every no-dig zone (including buried
    cables) is skipped. surveyed=False: only what the cab camera can see is skipped (visible
    zones, ±1.5 m around cable marker posts); hidden hazards stay in the plan and are flagged
    in hidden_hazard so the simulator can score strikes.
    """
    rs = site["roadside"]
    cl = Polyline(rs["shoulder_centerline"])
    line = cl.offset(rs["planting_offset"])
    s_line0, _, _ = line.project(*cl.at(rs["station_start"])[:2])
    s_line1, _, _ = line.project(*cl.at(rs["station_end"])[:2])
    spots = []
    k = 0
    s = s_line0
    while s <= s_line1:
        x, y, _ = line.at(s)
        sc, lat, _ = cl.project(x, y)
        sp = PlanSpot(id=k, x=x, y=y, station=s, tool="auger")
        for z in rs["no_dig"]:
            inside = z["s0"] <= sc <= z["s1"] and z["lat0"] <= lat <= z["lat1"]
            if not inside:
                continue
            if surveyed or z.get("visible", True):
                sp.skip, sp.note = True, f"no-dig: {z['name']}"
            else:
                near_post = any(abs(sc - m) < 1.5 for m in rs.get("marker_posts", []))
                if near_post:
                    sp.skip, sp.note = True, "no-dig: cable marker post"
                else:
                    sp.hidden_hazard = z["name"]
        for t in rs.get("trees", []):
            if math.hypot(t["x"] - x, t["y"] - y) < t.get("clearance", tree_clearance):
                sp.skip, sp.note = True, "clearance: existing tree"
        spots.append(sp)
        k += 1
        s += spacing
    return spots


# ----------------------------------------------------------------------------- grid (Jaal)
def grid_rows(polygon, spacing: float = 3.0, pattern: str = "square", headland: float = 1.5):
    """Rows of spots (list of lists of (x, y)) filling an axis-aligned block. Rows run along x."""
    x0, y0, x1, y1 = polygon_bounds(polygon)
    x0, y0, x1, y1 = x0 + headland, y0 + headland, x1 - headland, y1 - headland
    row_dy = spacing if pattern == "square" else spacing * math.sqrt(3) / 2
    rows = []
    y = y0
    j = 0
    while y <= y1 + 1e-9:
        shift = spacing / 2 if (pattern == "triangular" and j % 2) else 0.0
        xs = np.arange(x0 + shift, x1 + 1e-9, spacing)
        rows.append([(float(x), float(y)) for x in xs])
        y += row_dy
        j += 1
    return rows, row_dy


def resolve_obstacles(rows, obstacles, clearance: float = 0.35, max_shift: float = 0.6, seen=lambda o: True):
    """Shift spots that fall on a known obstacle along the row, or mark them skipped.
    Returns list of rows of PlanSpot (ids assigned later)."""
    out = []
    for j, row in enumerate(rows):
        r = []
        for (x, y) in row:
            sp = PlanSpot(id=-1, x=x, y=y, row=j, tool="auger")
            blockers = [o for o in obstacles if seen(o) and math.hypot(o["x"] - x, o["y"] - y) < o["r"] + clearance]
            if blockers:
                moved = False
                for dx in (0.3, -0.3, 0.6, -0.6):
                    if abs(dx) > max_shift:
                        continue
                    if all(math.hypot(o["x"] - (x + dx), o["y"] - y) >= o["r"] + clearance for o in obstacles if seen(o)):
                        sp.x += dx
                        sp.note = f"shifted {dx:+.1f} m off a {blockers[0]['kind'].replace('_', ' ')}"
                        moved = True
                        break
                if not moved:
                    sp.skip = True
                    sp.note = f"skip: {blockers[0]['kind'].replace('_', ' ')}"
            r.append(sp)
        out.append(r)
    return out


def split_rows(rows, n_robots: int):
    """Contiguous blocks of rows with balanced spot counts, one per robot: each boundary is put
    where the running total is closest to k/n of the whole."""
    counts = np.array([sum(1 for s in r if not s.skip) for r in rows], dtype=float)
    cum = np.concatenate([[0.0], np.cumsum(counts)])
    total = cum[-1]
    cuts = [0]
    for k in range(1, n_robots):
        target = total * k / n_robots
        j = int(np.argmin(np.abs(cum - target)))
        j = min(max(j, cuts[-1] + 1), len(rows) - (n_robots - k))
        cuts.append(j)
    cuts.append(len(rows))
    return [list(range(cuts[k], cuts[k + 1])) for k in range(n_robots)]


def serpentine(rows, row_ids, start_dir: int = 1):
    """Spots of the given rows in boustrophedon order."""
    order = []
    d = start_dir
    for j in row_ids:
        row = rows[j] if d > 0 else list(reversed(rows[j]))
        order.extend(row)
        d = -d
    return order


# ----------------------------------------------------------------------------- contour (Pahadi)
def contour_line(soil_map, h0: float, x0: float, x1: float, y_lo: float, y_hi: float, dx: float = 0.5):
    """Iso-elevation polyline at height h0 between x0 and x1 (terrain rising with y)."""
    pts = []
    for x in np.arange(x0, x1 + 1e-9, dx):
        lo, hi = y_lo, y_hi
        if not (soil_map.elevation(x, lo) <= h0 <= soil_map.elevation(x, hi)):
            continue
        for _ in range(40):
            mid = (lo + hi) / 2
            if soil_map.elevation(x, mid) < h0:
                lo = mid
            else:
                hi = mid
        pts.append((float(x), float((lo + hi) / 2)))
    if len(pts) < 2:
        return None
    p = np.array(pts)
    k = 5   # smooth so the tracks follow the hill, not every bump
    if len(p) > k:
        sm = np.copy(p)
        for i in range(len(p)):
            a, b = max(0, i - k // 2), min(len(p), i + k // 2 + 1)
            sm[i, 1] = p[a:b, 1].mean()
        p = sm
    return Polyline(p)


def smooth(path: Polyline, window_m: float) -> Polyline:
    """Moving-average smoothing over about window_m of path, ends held fixed."""
    p = path.p
    seg = np.mean(path.seg_len) if len(path.seg_len) else 1.0
    k = max(1, int(round(window_m / seg / 2)))
    out = p.copy()
    for i in range(1, len(p) - 1):
        a, b = max(0, i - k), min(len(p), i + k + 1)
        out[i] = p[a:b].mean(axis=0)
    return Polyline(out)


def avoid_boulders(path: Polyline, boulders, margin: float = 1.4) -> Polyline:
    """Push path points radially out of boulder keep-out circles."""
    pts = path.p.copy()
    for b in boulders:
        c = np.array([b["x"], b["y"]])
        R = b["r"] + margin
        d = pts - c
        dist = np.linalg.norm(d, axis=1)
        m = dist < R
        if m.any():
            pts[m] = c + d[m] / np.maximum(dist[m, None], 1e-6) * R
    return Polyline(pts)


def choose_tool(rock_depth: float, hole_depth: float = 0.25) -> str:
    if rock_depth >= hole_depth:
        return "auger"
    if rock_depth >= 0.12:
        return "pick"
    return "seedball"


def contour_plan(site, n_rows: int = 3, row_spacing: float = 4.0, spacing: float = 2.5, reach: float = 1.25,
                 side: str = "downhill", probe_sigma: float = 0.03, seed: int = 1):
    """Drive paths along contours and the planting spots beside them.

    Returns (rows, spots): rows = list of drive Polylines in driving order (alternating direction),
    spots = PlanSpot list with tool chosen from the surveyed depth to rock (with probe noise).
    """
    sm = site.soil_map
    x0, y0, x1, y1 = site["hill"]["work_area"]
    ymid = (y0 + y1) / 2
    xm = (x0 + x1) / 2
    h_start = float(sm.elevation(xm, ymid + (n_rows - 1) * row_spacing / 2))
    slope = math.radians(site.get("slope_deg", 14.0))
    dh = row_spacing * math.tan(slope)
    rng = np.random.default_rng(seed)
    rows, spots = [], []
    k = 0
    for r in range(n_rows):
        line = contour_line(sm, h_start - r * dh, x0, x1, y0 - 15, y1 + 15)
        if line is None:
            continue
        for _ in range(4):          # push out of the boulders, then smooth the kinks that leaves
            line = smooth(avoid_boulders(line.resample(0.5), site.get("boulders", []), margin=1.7), 5.0)
        if r % 2:
            line = line.reversed()
        line = line.resample(0.25)
        rows.append(line)
        s = 2.0
        while s < line.length - 2.0:
            x, y, yaw = line.at(s)
            nl = np.array([-math.sin(yaw), math.cos(yaw)])          # left of travel
            n = sm.normal(x, y)
            down = n[:2] / (np.linalg.norm(n[:2]) + 1e-9)           # the normal leans downhill
            sgn = 1.0 if (nl @ down > 0) == (side == "downhill") else -1.0
            px, py = np.array([x, y]) + sgn * reach * nl
            if any(math.hypot(b["x"] - px, b["y"] - py) < b["r"] + 0.4 for b in site.get("boulders", [])):
                s += spacing
                continue
            probe = float(sm.rock_depth(px, py)) + rng.normal(0, probe_sigma)
            spots.append(PlanSpot(id=k, x=float(px), y=float(py), row=r, station=s, tool=choose_tool(probe),
                                  extra={"side": "left" if sgn > 0 else "right", "probe_depth": probe}))
            k += 1
            s += spacing
    return rows, spots


# ----------------------------------------------------------------------------- turns
def arc(cx, cy, r, a0, a1, step=0.2):
    n = max(2, int(abs(a1 - a0) * r / step) + 1)
    return [(cx + r * math.cos(a), cy + r * math.sin(a)) for a in np.linspace(a0, a1, n)]


def u_turn(end, heading, start_next, radius: float):
    """Path from the end of one row to the start of the next row, which runs the other way.
    A plain U if the rows are at least two turning radii apart, otherwise an omega ('light bulb')
    turn: swing away, loop round, swing back. Never tighter than `radius`."""
    ex, ey = end
    c, s_ = math.cos(heading), math.sin(heading)
    rx, ry = start_next[0] - ex, start_next[1] - ey
    lx, ly = c * rx + s_ * ry, -s_ * rx + c * ry          # next start in the end's frame
    side = 1.0 if ly >= 0 else -1.0
    d = abs(ly)
    R = radius
    x0 = max(lx, 0.0)        # next row starts further on: run straight first; starts short: run on after
    if d >= 2 * R:
        pts = arc(0.0, side * d / 2, d / 2, -side * math.pi / 2, side * math.pi / 2)
        pts = [(x, y) for x, y in pts]
    else:
        # local frame mirrored so the next row is at +y; centres C1 (0,-R), C2 (xc, d/2), C3 (0, d+R)
        xc = math.sqrt(max(4 * R * R - (d / 2 + R) ** 2, 0.0))
        t1 = math.atan2(d / 2 + R, xc)                      # direction C1 -> C2
        p1 = arc(0.0, -R, R, math.pi / 2, t1)               # clockwise away from the next row
        a_in = math.atan2(-(d / 2 + R), -xc)                # C2 -> tangent with C1
        a_out = math.atan2(d / 2 + R, -xc)                  # C2 -> tangent with C3
        if a_out < a_in:
            a_out += 2 * math.pi
        p2 = arc(xc, d / 2, R, a_in, a_out)                 # counter-clockwise loop
        t3 = math.atan2(-(d / 2 + R), xc)                   # C3 -> tangent with C2
        p3 = arc(0.0, d + R, R, t3, -math.pi / 2)           # clockwise back onto the row
        pts = [(x, y) for x, y in p1 + p2 + p3]
        pts = [(x, side * y) for x, y in pts]
    pts = [(x + x0, y) for x, y in pts]
    if x0 > 0:
        pts.insert(0, (0.0, 0.0))
    out = [(ex + c * x - s_ * y, ey + s_ * x + c * y) for x, y in pts]
    out.append(tuple(start_next))
    return out


def max_curvature(pts) -> float:
    p = np.asarray(pts, float)
    if len(p) < 3:
        return 0.0
    a, b, c = p[:-2], p[1:-1], p[2:]
    ab, bc, ca = np.linalg.norm(b - a, axis=1), np.linalg.norm(c - b, axis=1), np.linalg.norm(a - c, axis=1)
    cross = np.abs((b - a)[:, 0] * (c - a)[:, 1] - (b - a)[:, 1] * (c - a)[:, 0])
    k = 2 * cross / np.maximum(ab * bc * ca, 1e-9)
    return float(k[(ab > 0.05) & (bc > 0.05)].max(initial=0.0))
