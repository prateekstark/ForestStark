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


# moisture the Bekker constants were measured at, exponential softening above it, capillary
# cohesion of damp sand (kPa), and the share of c that is real cohesion (SoilModel.hh DefaultSoils)
MOISTURE = {
    "dry_sand": (0.05, 0.0, 3.0, 0.0),
    "sandy_loam": (0.15, 4.0, 0.5, 0.6),
    "clayey_soil": (0.20, 6.0, 0.0, 1.0),
    "heavy_clay": (0.25, 8.0, 0.0, 1.0),
    "lateritic_gravel": (0.10, 2.0, 0.0, 0.5),
}


def _mp(soil: dict):
    if "ref_moisture" in soil:       # read from the soil map (possibly calibrated)
        return soil["ref_moisture"], soil["wet_softening"], soil["capillary_c"], soil["true_cohesion"]
    return MOISTURE.get(soil.get("name", ""), (0.15, 4.0, 0.0, 1.0))


def strength_factor(soil: dict, m: float) -> float:
    ref, soft, _, _ = _mp(soil)
    d = m - ref
    return max(0.3, math.exp(-soft * d)) if d > 0 else min(1.4, 1.0 - 0.5 * soft * d)


def capillary_cohesion(soil: dict, m: float) -> float:
    cap = _mp(soil)[2]
    return cap * min(max((m - 0.03) / 0.07, 0.0), 1.0) * min(max((0.35 - m) / 0.2, 0.0), 1.0)


def at_moisture(soil: dict, m: float) -> dict:
    """The soil's constants at volumetric moisture m (sinkage grows as f^-1.5, c and phi soften)."""
    f = strength_factor(soil, m)
    km = f ** (1.5 * soil["n"])
    return dict(soil, kc=soil["kc"] * km, kphi=soil["kphi"] * km, c=soil["c"] * f + capillary_cohesion(soil, m),
                phi=soil["phi"] * min(max(0.6 + 0.4 * f, 0.6), 1.1))


def critical_wall_height(soil: dict, m: float, gamma: float = 16000.0) -> float:
    """Height (m) an unsupported auger-hole wall stands: Rankine x2 arching / 1.5 disturbance."""
    w = at_moisture(soil, m)
    c = _mp(soil)[3] * soil["c"] * strength_factor(soil, m) + capillary_cohesion(soil, m)
    return (2.0 / 1.5) * 4.0 * c * 1e3 / gamma * math.tan(math.radians(45.0 + 0.5 * w["phi"]))


def hole_collapse(soil: dict, m: float, d: float) -> float:
    """Loose spoil (m) at the bottom of a hole of depth d once the auger is out."""
    hc = critical_wall_height(soil, m)
    return 0.5 * (d - hc) if d > hc else 0.0


def multipass_resistance(soil: dict, z: float, z_prev: float, b: float, rebound: float = 0.15) -> float:
    fresh = compaction_resistance(soil, z, b)
    if z_prev <= 0:
        return fresh
    return max(fresh - compaction_resistance(soil, min(z, z_prev), b), rebound * fresh)


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
