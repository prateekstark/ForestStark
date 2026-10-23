"""Bekker-Wong terramechanics, mirroring fs_gazebo/include/fs_gazebo/SoilModel.hh.

Used on the robot side to predict traction before committing to a slope, to estimate slip from
odometry vs GNSS, and to draw the drawbar-pull curve in the traction monitor.
"""
from __future__ import annotations

import math

G = 9.81


def modulus(soil: dict, b: float) -> float:
    return (soil["kc"] / max(b, 1e-3) + soil["kphi"]) * 1e3


def sinkage(soil: dict, W: float, b: float, l: float) -> float:
    if W <= 0:
        return 0.0
    p = W / (max(b, 1e-3) * max(l, 1e-3))
    return (p / modulus(soil, b)) ** (1.0 / soil["n"])


def compaction_resistance(soil: dict, z: float, b: float) -> float:
    if z <= 0:
        return 0.0
    n = soil["n"]
    return b * modulus(soil, b) * z ** (n + 1) / (n + 1)


def shear_capacity(soil: dict, W: float, b: float, l: float) -> float:
    return max(0.0, b * l * soil["c"] * 1e3 + max(W, 0.0) * math.tan(math.radians(soil["phi"])))


def shear_fraction(i: float, l: float, K: float) -> float:
    i = min(max(abs(i), 0.0), 1.0)
    a = i * l / max(K, 1e-4)
    if a < 1e-6:
        return 0.5 * a
    return 1.0 - (1.0 - math.exp(-a)) / a


def drawbar_pull(soil: dict, loads, b: float, l: float, slip: float) -> float:
    """Net pull (N) of a set of contacts at a common slip: thrust minus compaction resistance."""
    thrust = sum(shear_capacity(soil, W, b, l) * shear_fraction(slip, l, soil["K"]) for W in loads)
    res = sum(compaction_resistance(soil, sinkage(soil, W, b, l), b) for W in loads)
    return thrust - res


def slip_for_pull(soil: dict, loads, b: float, l: float, required: float) -> float:
    """Slip at which drawbar pull meets the required force; 1.0 means it cannot (immobilised)."""
    if drawbar_pull(soil, loads, b, l, 1.0) < required:
        return 1.0
    lo, hi = 0.0, 1.0
    for _ in range(50):
        mid = (lo + hi) / 2
        if drawbar_pull(soil, loads, b, l, mid) >= required:
            hi = mid
        else:
            lo = mid
    return hi


def max_gradeability_deg(soil: dict, mass: float, n_contacts: int, b: float, l: float, slip: float = 0.4) -> float:
    """Steepest straight-up slope (deg) the machine can climb at the given slip."""
    lo, hi = 0.0, 60.0
    for _ in range(40):
        mid = (lo + hi) / 2
        th = math.radians(mid)
        W = mass * G * math.cos(th) / n_contacts
        if drawbar_pull(soil, [W] * n_contacts, b, l, slip) >= mass * G * math.sin(th):
            lo = mid
        else:
            hi = mid
    return lo
