"""Planar geometry used across the stack: polylines with arc length, projection, polygons, hulls."""
from __future__ import annotations

import math
from typing import Iterable, Sequence

import numpy as np


def wrap(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def rpy_from_quat(x: float, y: float, z: float, w: float):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return roll, pitch, yaw_from_quat(x, y, z, w)


def quat_from_yaw(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Polyline:
    """A 2-D path with cumulative arc length. Left of the direction of travel is positive lateral."""

    def __init__(self, pts: Sequence[Sequence[float]]):
        p = np.asarray(pts, dtype=float).reshape(-1, 2)
        if len(p) < 2:
            raise ValueError("a polyline needs at least two points")
        keep = np.concatenate([[True], np.linalg.norm(np.diff(p, axis=0), axis=1) > 1e-9])
        self.p = p[keep]
        d = np.diff(self.p, axis=0)
        self.seg_len = np.linalg.norm(d, axis=1)
        self.dir = d / self.seg_len[:, None]
        self.s = np.concatenate([[0.0], np.cumsum(self.seg_len)])
        self.length = float(self.s[-1])

    def _seg(self, s: float) -> int:
        return int(np.clip(np.searchsorted(self.s, s, side="right") - 1, 0, len(self.seg_len) - 1))

    def at(self, s: float):
        """Point and heading at arc length s (clamped to the ends)."""
        s = min(max(s, 0.0), self.length)
        i = self._seg(s)
        u = s - self.s[i]
        x, y = self.p[i] + self.dir[i] * u
        return float(x), float(y), math.atan2(self.dir[i, 1], self.dir[i, 0])

    def project(self, x: float, y: float, hint: int | None = None, window: int = 60):
        """Closest point: returns (s, lateral, segment index). lateral > 0 left of the path."""
        n = len(self.seg_len)
        lo, hi = (0, n) if hint is None else (max(0, hint - window), min(n, hint + window))
        a = self.p[lo:hi]
        d = self.dir[lo:hi]
        rel = np.array([x, y]) - a
        t = np.clip(np.einsum("ij,ij->i", rel, d), 0, self.seg_len[lo:hi])
        foot = a + d * t[:, None]
        dist2 = np.sum((np.array([x, y]) - foot) ** 2, axis=1)
        k = int(np.argmin(dist2))
        i = lo + k
        lat = float(d[k, 0] * rel[k, 1] - d[k, 1] * rel[k, 0])
        return float(self.s[i] + t[k]), lat, i

    def offset(self, d: float) -> "Polyline":
        """Parallel curve d metres to the left (negative: right)."""
        seg_n = np.stack([-self.dir[:, 1], self.dir[:, 0]], axis=1)
        n_in = np.vstack([seg_n[:1], seg_n])        # normal of the segment arriving at each vertex
        n_out = np.vstack([seg_n, seg_n[-1:]])      # normal of the segment leaving it
        n = n_in + n_out
        n /= np.linalg.norm(n, axis=1, keepdims=True)
        miter = 1.0 / np.maximum(np.einsum("ij,ij->i", n, n_out), 0.3)   # mitre join, capped at sharp bends
        return Polyline(self.p + n * (d * miter)[:, None])

    def resample(self, ds: float) -> "Polyline":
        ss = np.arange(0.0, self.length, ds).tolist() + [self.length]
        return Polyline([self.at(s)[:2] for s in ss])

    def slice(self, s0: float, s1: float) -> "Polyline":
        s0, s1 = max(0.0, s0), min(self.length, s1)
        pts = [self.at(s0)[:2]] + [tuple(q) for q, s in zip(self.p, self.s) if s0 < s < s1] + [self.at(s1)[:2]]
        return Polyline(pts)

    def reversed(self) -> "Polyline":
        return Polyline(self.p[::-1])


def convex_hull(points: Iterable[Sequence[float]]) -> list:
    """Andrew's monotone chain; returns hull vertices counter-clockwise (2-D)."""
    pts = sorted({(float(p[0]), float(p[1])) for p in points})
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def point_in_polygon(x: float, y: float, poly: Sequence[Sequence[float]]) -> bool:
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1 + 1e-15) + x1:
            inside = not inside
    return inside


def polygon_area(poly: Sequence[Sequence[float]]) -> float:
    a = 0.0
    for i in range(len(poly)):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % len(poly)]
        a += x1 * y2 - x2 * y1
    return abs(a) / 2


def polygon_bounds(poly):
    xs, ys = zip(*poly)
    return min(xs), min(ys), max(xs), max(ys)


def matrix_to_quat(R) -> tuple:
    """Rotation matrix -> (x, y, z, w), numerically safe for every angle (Shepperd's method)."""
    R = np.asarray(R, float)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s)
    if R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s)


def transform_to_matrix(t) -> np.ndarray:
    """geometry_msgs/Transform -> 4x4 homogeneous matrix."""
    q = t.rotation
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix(q.x, q.y, q.z, q.w)
    T[:3, 3] = (t.translation.x, t.translation.y, t.translation.z)
    return T
