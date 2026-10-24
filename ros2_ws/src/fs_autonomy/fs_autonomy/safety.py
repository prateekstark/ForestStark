"""Safety logic shared by all three robots: lidar clustering, object tracking, speed-and-separation
zones, a tool zone, and what a navigator needs to decide between yielding and going round.
Pure numpy, no ROS; used by nodes/safety_monitor.py.

Policy
  People and animals (any tracked object that has moved) get the wide margins: the robot slows
  from 6 m, stops at 2.5 m in its path, stops if one is on a course to cross within 3 s, and
  stops whatever it is doing if one comes within 1.2 m of its footprint. It never plans a path
  close to them; a navigator may only go round one that has stood still in the path for a long
  time, with extra clearance.
  Static objects (stumps, a crate, a parked vehicle) get the normal margins: slow from 5 m, stop
  at 1.5 m, and a navigator may go round after a short wait if a side is free.
  Tool zone: a person within 2 m of the auger or duckbill holds the tool (spin, feed and all
  mechanism joints) until they have been clear for 3 s.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


# ------------------------------------------------------------------ geometry
@dataclass
class Footprint:
    """Axis-aligned box in base_link, m."""
    xmin: float
    xmax: float
    ymin: float
    ymax: float

    def distance(self, x, y):
        dx = np.maximum(np.maximum(self.xmin - np.asarray(x, float), np.asarray(x, float) - self.xmax), 0.0)
        dy = np.maximum(np.maximum(self.ymin - np.asarray(y, float), np.asarray(y, float) - self.ymax), 0.0)
        return np.hypot(dx, dy)

    def contains(self, x, y, pad=0.0):
        x, y = np.asarray(x, float), np.asarray(y, float)
        return (x > self.xmin - pad) & (x < self.xmax + pad) & (y > self.ymin - pad) & (y < self.ymax + pad)

    def corners(self):
        return np.array([[self.xmin, self.ymin], [self.xmin, self.ymax], [self.xmax, self.ymin], [self.xmax, self.ymax]])


FOOTPRINTS = {   # base_link boxes; Katar's covers tractor, trailer and the boom's working strip
    "jaal": Footprint(-1.45, 1.45, -0.70, 0.70),
    "katar": Footprint(-6.0, 1.9, -1.05, 3.7),
    "pahadi": Footprint(-2.25, 0.75, -0.62, 0.62),
}


@dataclass
class SafetyConfig:
    footprint: Footprint
    stop_static: float = 1.5      # gap ahead of the footprint at full speed, m
    stop_min: float = 0.5         # ... at crawl speed: braking distance + 0.4 m, never less than this
    decel: float = 0.5            # m/s^2, for the speed-dependent static stop distance
    slow_static: float = 5.0
    margin_static: float = 0.45   # corridor half-width beyond the footprint
    stop_person: float = 2.5
    slow_person: float = 6.0
    margin_person: float = 0.9
    ring_person: float = 1.2      # any direction, even when stopped
    ttc_horizon: float = 3.0
    crossing_extra: float = 1.0   # predicted to come this much closer than the person margin: stop
    tool_r: float = 2.0
    min_scale: float = 0.2        # slowest crawl inside a slow zone
    detour_clear: float = 1.1     # corridor margin + clearance a detour keeps from the blocker
    chain: object = None          # the certified layer's stop chain (safety_fields.StopChain): this layer
    chain_margin: float = 0.25    # always stops this much before that layer's protective field would trip

    @staticmethod
    def for_robot(kind: str) -> "SafetyConfig":
        from .safety_fields import CHAINS, BANDS
        cfg = SafetyConfig(FOOTPRINTS[kind], chain=CHAINS[kind] if BANDS[kind] else None)
        if kind == "katar":       # a driven tractor: the driver brakes for traffic; the monitor guards the boom strip
            cfg.stop_static, cfg.slow_static = 0.0, 0.0
        if kind == "pahadi":      # slow machine on a slope: stops earlier, crawls slower
            cfg.stop_person, cfg.slow_person, cfg.min_scale = 3.0, 6.0, 0.15
        return cfg


# ------------------------------------------------------------------ lidar
def scan_points(ranges, angle_min, angle_increment, range_min, range_max, T=None):
    """LaserScan to (N, 2) points, optionally through a 3x3 homogeneous 2-D transform (base <- lidar)."""
    r = np.asarray(ranges, float)
    a = angle_min + np.arange(len(r)) * angle_increment
    ok = np.isfinite(r) & (r > range_min) & (r < range_max * 0.98)
    p = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], axis=1)
    if T is not None and len(p):
        p = p @ np.asarray(T)[:2, :2].T + np.asarray(T)[:2, 2]
    return p


def above_ground(P, roll, pitch, yaw, base_xy, base_h, elevation, min_h=0.15, per_m=0.04):
    """Mask of lidar points (P: (N, 3) in base_link) that are at least min_h above the surveyed
    ground. A level planar lidar on a 14 deg hillside hits the uphill slope ~3 m away; with the
    machine's roll and pitch and the site elevation map those hits are recognised and dropped.
    base_h: height of base_link above the ground beneath it. The margin grows by per_m with range,
    because a 1 deg error in roll or pitch is 17 cm at 10 m."""
    if len(P) == 0:
        return np.zeros(0, bool)
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    R = (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
         @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))
    W = P @ R.T
    wx, wy = W[:, 0] + base_xy[0], W[:, 1] + base_xy[1]
    z = float(elevation(base_xy[0], base_xy[1])) + base_h + W[:, 2]
    rng = np.hypot(P[:, 0], P[:, 1])
    return z - np.asarray(elevation(wx, wy), float) > min_h + per_m * rng


def segment(points, gap=0.25):
    """Split points in scan order where consecutive points are more than `gap` apart."""
    if len(points) == 0:
        return []
    d = np.hypot(*np.diff(points, axis=0).T)
    cuts = np.where(d > gap)[0] + 1
    return [s for s in np.split(points, cuts) if len(s)]


def clusters(point_sets, gap=0.25, merge=0.35):
    """Detections (cx, cy, r) from one or more scans' points (each in scan order)."""
    out = []
    for pts in point_sets:
        for s in segment(pts, gap):
            c = s.mean(axis=0)
            out.append([float(c[0]), float(c[1]), max(0.06, float(np.max(np.hypot(*(s - c).T)))), len(s)])
    merged = True
    while merged and len(out) > 1:                 # the same object seen by two lidars, or split by a gap
        merged = False
        for i in range(len(out)):
            for j in range(i + 1, len(out)):
                a, b = out[i], out[j]
                if math.hypot(a[0] - b[0], a[1] - b[1]) < a[2] + b[2] + merge * 0.5 and a[2] + b[2] < 0.9:
                    n = a[3] + b[3]
                    cx, cy = (a[0] * a[3] + b[0] * b[3]) / n, (a[1] * a[3] + b[1] * b[3]) / n
                    r = max(math.hypot(a[0] - cx, a[1] - cy) + a[2], math.hypot(b[0] - cx, b[1] - cy) + b[2])
                    out[i] = [cx, cy, r, n]
                    del out[j]
                    merged = True
                    break
            if merged:
                break
    return [tuple(o[:3]) for o in out]


# ------------------------------------------------------------------ tracking
@dataclass
class Track:
    id: int
    x: float
    y: float
    r: float
    t_first: float
    t_last: float
    x0: float
    y0: float
    vx: float = 0.0
    vy: float = 0.0
    moving: bool = False          # sticky: once it has moved it is treated as a person or an animal
    kind: str = "object"          # object | robot (a fleet peer) | surveyed (born at a surveyed stump, mound, stone)
    hist: list = field(default_factory=list)      # (t, x, y) over the last window_s

    @property
    def speed(self):
        return math.hypot(self.vx, self.vy)


class Tracker:
    """Nearest-neighbour tracker in the map frame, so the robot's own motion is not mistaken for
    the object's. An object counts as moving once it has covered `move_dist` within the last
    `window_s` (and been tracked for at least `min_span`): a person walking at 0.3 m/s or faster.
    Two things are not movement: a crate seen from a passing robot shows a different face each
    scan, so its centre wobbles by up to half its size; and a track that jumps faster than
    `jump_speed` has been matched to the wrong cluster, so its history restarts. Beyond
    `classify_range` a cluster has too few points to judge. While the robot
    itself is moving, an object within `near` m must cover twice the distance: driving past a
    crate sweeps its visible outline round it. Large objects
    (r > max_person_r) never count as people, and neither do tracks that first appear at a
    surveyed obstacle (`static_zones`): a stump with roots seen side-on from a passing robot
    shifts by a metre as different parts come into view."""

    def __init__(self, gate=0.8, drop_s=1.5, move_dist=0.6, window_s=2.0, min_span=0.8, jump_speed=3.0,
                 max_person_r=0.35, near=3.5, classify_range=10.0, static_zones=()):
        self.near, self.ego, self.classify_range = near, [], classify_range
        self.zones = [(float(x), float(y), float(r) + 1.0) for x, y, r in static_zones]
        self.gate, self.drop_s = gate, drop_s
        self.move_dist, self.window_s, self.min_span, self.jump = move_dist, window_s, min_span, jump_speed
        self.max_r = max_person_r
        self.tracks: list[Track] = []
        self._next = 1

    def update(self, t, dets, peers=(), robot=None):
        """dets: (x, y, r) in the tracking frame. peers: (x, y) of other robots (their clusters are
        labelled). robot: (x, y[, yaw]) of this robot, for the ego-motion rule; pivoting counts, since
        it swings lidars mounted off-centre through an arc."""
        if robot is not None:
            yaw = robot[2] if len(robot) > 2 else 0.0
            self.ego = [e for e in self.ego if t - e[0] <= self.window_s] + [(t, robot[0], robot[1], yaw)]
        translating = len(self.ego) > 1 and math.hypot(self.ego[-1][1] - self.ego[0][1], self.ego[-1][2] - self.ego[0][2]) > 0.5
        pivoting = len(self.ego) > 1 and abs(math.atan2(math.sin(self.ego[-1][3] - self.ego[0][3]),
                                                        math.cos(self.ego[-1][3] - self.ego[0][3]))) > 0.3
        ego_moving = translating or pivoting
        near = 6.0 if pivoting else self.near           # a pivot swings the lidars through a wider arc
        free = list(range(len(dets)))
        pairs = sorted(((math.hypot(tr.x - dets[j][0], tr.y - dets[j][1]), i, j)
                        for i, tr in enumerate(self.tracks) for j in free), key=lambda p: p[0])
        used_t, used_d = set(), set()
        for d, i, j in pairs:
            if d > self.gate or i in used_t or j in used_d:
                continue
            used_t.add(i)
            used_d.add(j)
            tr, (x, y, r) = self.tracks[i], dets[j]
            dt = t - tr.t_last
            if dt > 1e-3 and math.hypot(x - tr.x, y - tr.y) / dt > self.jump:
                tr.hist, tr.vx, tr.vy = [], 0.0, 0.0          # matched to the wrong cluster: start again
            elif dt > 1e-3:
                a = min(1.0, dt / 0.5)                        # ~0.5 s velocity smoothing
                tr.vx += a * ((x - tr.x) / dt - tr.vx)
                tr.vy += a * ((y - tr.y) / dt - tr.vy)
            tr.x, tr.y, tr.r, tr.t_last = x, y, 0.7 * tr.r + 0.3 * r, t
            tr.hist = [h for h in tr.hist if t - h[0] <= self.window_s] + [(t, x, y)]
            if tr.r <= self.max_r and not tr.moving and tr.kind != "surveyed":
                t0, x0, y0 = tr.hist[0]
                need = self.move_dist
                dist = math.hypot(x - self.ego[-1][1], y - self.ego[-1][2]) if self.ego else 0.0
                if ego_moving and dist < near:
                    need *= 2.0
                if dist > self.classify_range:          # a few points at long range: centres are unreliable
                    need = math.inf
                if t - t0 >= self.min_span and math.hypot(x - x0, y - y0) > need:
                    tr.moving = True
        for j in free:
            if j not in used_d:
                x, y, r = dets[j]
                tr = Track(self._next, x, y, r, t, t, x, y, hist=[(t, x, y)])
                if any(math.hypot(x - zx, y - zy) < zr for zx, zy, zr in self.zones):
                    tr.kind = "surveyed"
                self.tracks.append(tr)
                self._next += 1
        self.tracks = [tr for tr in self.tracks if t - tr.t_last <= self.drop_s]
        for tr in self.tracks:
            if tr.kind != "surveyed":
                tr.kind = "robot" if any(math.hypot(tr.x - px, tr.y - py) < 2.0 for px, py in peers) else "object"
        return self.tracks


# ------------------------------------------------------------------ assessment
@dataclass
class Assessment:
    scale: float = 1.0
    reason: str = ""
    tool_hold: bool = False
    nearest_person: float = -1.0
    blocker: Track | None = None
    blocker_base: tuple = (0.0, 0.0)
    free_left: bool = True
    free_right: bool = True
    notes: list = field(default_factory=list)

    @property
    def level(self):
        return 2 if self.scale <= 0.0 else (1 if self.scale < 1.0 else 0)


def to_base(tracks, pose):
    """Tracks (map frame) -> list of (track, bx, by, bvx, bvy) in base_link for robot pose (x, y, yaw)."""
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    out = []
    for tr in tracks:
        dx, dy = tr.x - x, tr.y - y
        out.append((tr, c * dx + s * dy, -s * dx + c * dy, c * tr.vx + s * tr.vy, -s * tr.vx + c * tr.vy))
    return out


def _corridor(fp: Footprint, ux, uy):
    """Front extent and half-width of the footprint seen along travel direction (ux, uy)."""
    k = fp.corners()
    along = k @ np.array([ux, uy])
    across = k @ np.array([-uy, ux])
    return float(along.max()), float(np.abs(across).max())


def assess(objs, cmd, cfg: SafetyConfig, tool_xy=None) -> Assessment:
    """objs: output of to_base(). cmd: commanded (vx, vy, wz) in base_link."""
    A = Assessment()
    fp = cfg.footprint
    vx, vy, wz = cmd
    sp = math.hypot(vx, vy)
    people = [(tr, bx, by, bvx, bvy) for tr, bx, by, bvx, bvy in objs if tr.moving and tr.kind == "object"]
    if people:
        d = [float(fp.distance(bx, by)) - tr.r for tr, bx, by, *_ in people]
        A.nearest_person = max(0.0, min(d))
        if min(d) < cfg.ring_person:
            A.scale, A.reason = 0.0, "person_near"
            i = int(np.argmin(d))
            A.blocker, A.blocker_base = people[i][0], (people[i][1], people[i][2])
    if tool_xy is not None:
        for tr, bx, by, *_ in people:
            if math.hypot(bx - tool_xy[0], by - tool_xy[1]) - tr.r < cfg.tool_r:
                A.tool_hold = True
                A.notes.append(f"track {tr.id} at ({bx:+.1f}, {by:+.1f}) m")
    if abs(wz) > 0.05 and sp < 0.05:                          # pivoting: the corners sweep a circle
        R = float(np.max(np.hypot(*fp.corners().T)))
        for tr, bx, by, *_ in people:
            if math.hypot(bx, by) - tr.r < R + 0.5:
                A.scale, A.reason = 0.0, A.reason or "person_near"
    if sp < 0.02 or A.scale == 0.0:
        return A

    ux, uy = vx / sp, vy / sp
    front, half = _corridor(fp, ux, uy)
    best = None                                               # (scale, reason, obj)
    for o in objs:
        tr, bx, by, bvx, bvy = o
        person = tr.moving and tr.kind == "object"
        if not person and cfg.slow_static <= 0:
            continue
        stop, slow, margin = ((cfg.stop_person, cfg.slow_person, cfg.margin_person) if person
                              else (cfg.stop_static, cfg.slow_static, cfg.margin_static))
        if not person:                       # a protective field that shrinks with speed (people keep the full one)
            stop = min(stop, max(cfg.stop_min, 0.4 + sp * sp / (2 * cfg.decel)))
        if cfg.chain is not None:            # never inside the certified layer's protective field at this speed
            from .safety_fields import protective_length
            stop = max(stop, protective_length(sp, cfg.chain) + cfg.chain_margin)
        along, across = bx * ux + by * uy, -bx * uy + by * ux
        gap = along - tr.r - front
        k = 1.0
        if abs(across) < half + margin + tr.r and gap > -(fp.xmax - fp.xmin) and along > 0:
            if gap < stop:
                k = 0.0
            elif gap < slow:
                k = cfg.min_scale + (1 - cfg.min_scale) * (gap - stop) / (slow - stop)
        reason = "person_in_path" if person else "obstacle"
        if person and k > 0.0:
            # crossing: relative motion brings it within the margin of the footprint inside the horizon
            for t in np.arange(0.25, cfg.ttc_horizon + 1e-6, 0.25):
                px, py = bx + (bvx - vx) * t, by + (bvy - vy) * t
                if float(fp.distance(px, py)) - tr.r < margin + cfg.crossing_extra and along > -0.5:
                    in_path = abs(across) < half + margin + tr.r
                    k, reason = 0.0, ("person_in_path" if in_path else "person_crossing")
                    break
        if best is None or k < best[0]:
            best = (k, reason, o)
    if best is not None and best[0] < 1.0:
        A.scale, A.reason = best[0], best[1]
        A.blocker, A.blocker_base = best[2][0], (best[2][1], best[2][2])
        # can a detour pass on either side? (corridor shifted sideways by the detour offset)
        tr, bx, by = best[2][0], best[2][1], best[2][2]
        along_b = bx * ux + by * uy
        across_b = -bx * uy + by * ux
        off = max(1.6, abs(across_b) + max(tr.r, 0.5) + half + cfg.detour_clear)
        for side in (1.0, -1.0):
            lane = across_b + side * off if abs(across_b) > 0.2 else side * off
            free = True
            for tr2, bx2, by2, *_ in objs:
                if tr2 is tr:
                    continue
                a2, c2 = bx2 * ux + by2 * uy, -bx2 * uy + by2 * ux
                if -0.5 < a2 < along_b + tr.r + 3.0 + front and abs(c2 - lane) < half + 0.3 + tr2.r:
                    free = False
                    break
            if side > 0:
                A.free_left = free
            else:
                A.free_right = free
    return A
