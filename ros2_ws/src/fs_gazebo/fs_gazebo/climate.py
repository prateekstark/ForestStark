"""Seasonal soil water: rain, infiltration, drainage and drying through the year, per region and soil.

A daily bucket model of the top 0.3 m (the root zone of a new seedling) driven by stochastic rain
with each region's monthly normals:

  rain       wet days from a two-state Markov chain whose wet-day frequency matches the month's rainy
             days; amounts exponential with the month's mean per wet day
  infiltration  Green-Ampt capacity over the storm (Ks t + psi dtheta ln(1 + F / (psi dtheta))),
             the rest runs off (clay sheds most of a monsoon storm; sand takes it all)
  drainage   water above field capacity drains with a time constant set by Ks
  drying     ET0 x a bare-soil / young-plant coefficient, falling linearly from field capacity to
             wilting point (and to a residual below that)

Monthly rainfall and reference evapotranspiration are rounded IMD-station normals (Jaipur, Jodhpur,
Sri Ganganagar, Kota, Udaipur, Tehri-Garhwal mid-hills); soil hydraulic constants are Rawls,
Brakensiek and Saxton (1982) class means. It is a planning model: it gets the season's shape and
the difference between soils right, not tomorrow's moisture.

Used by the site generator (`date:=MM-DD` sets the map's moisture to the water balance on that day),
the survival model (a year of root-zone water after planting) and the pit-watering estimate.
"""
from __future__ import annotations

import datetime as dt
import math
from dataclasses import dataclass

import numpy as np

# monthly rain (mm), rainy days, reference ET0 (mm/day), Jan..Dec
CLIMATE = {
    "eastern_plain": ((8, 7, 5, 5, 15, 60, 200, 190, 80, 15, 4, 3),
                      (0.8, 0.7, 0.5, 0.5, 1.5, 4, 11, 11, 5, 1, 0.4, 0.3),
                      (3.0, 4.0, 5.5, 7.5, 9.0, 8.5, 5.5, 4.5, 5.0, 4.5, 3.5, 3.0)),
    "thar_arid": ((4, 5, 3, 3, 10, 35, 120, 120, 50, 8, 2, 2),
                  (0.4, 0.5, 0.3, 0.3, 1, 2.5, 7, 7, 3, 0.6, 0.2, 0.2),
                  (3.5, 4.5, 6.0, 8.0, 10.0, 9.5, 7.0, 5.5, 6.0, 5.5, 4.0, 3.3)),
    "ignp_canal": ((8, 10, 8, 5, 10, 30, 85, 70, 30, 3, 2, 4),
                   (0.8, 1, 0.8, 0.5, 1, 2.5, 5.5, 5, 2, 0.3, 0.2, 0.4),
                   (2.5, 3.5, 5.0, 7.5, 9.5, 9.5, 7.0, 6.0, 5.5, 4.5, 3.0, 2.3)),
    "hadoti_vertisol": ((6, 5, 3, 3, 10, 90, 300, 300, 120, 25, 8, 5),
                        (0.6, 0.5, 0.3, 0.3, 1, 5, 14, 14, 7, 1.5, 0.6, 0.4),
                        (3.0, 4.0, 5.5, 7.5, 9.0, 8.0, 5.0, 4.0, 4.5, 4.5, 3.5, 3.0)),
    "aravalli_hills": ((5, 4, 3, 2, 8, 85, 220, 210, 90, 15, 6, 3),
                       (0.5, 0.4, 0.3, 0.2, 1, 5, 12, 12, 5, 1, 0.5, 0.3),
                       (2.8, 3.8, 5.2, 7.0, 8.5, 7.5, 5.0, 4.2, 4.6, 4.2, 3.3, 2.8)),
    "himalayan_midhills": ((60, 70, 55, 30, 50, 180, 420, 400, 210, 50, 10, 25),
                           (3.5, 4, 3.5, 2.5, 4, 10, 21, 21, 12, 3, 1, 2),
                           (1.8, 2.4, 3.3, 4.5, 5.2, 4.6, 3.6, 3.4, 3.3, 2.9, 2.2, 1.7)),
}
DAYS = (31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31)


@dataclass(frozen=True)
class Hydraulics:
    theta_s: float      # saturated volumetric water content
    fc: float           # field capacity
    wp: float           # wilting point
    ks: float           # saturated conductivity, mm/day
    psi: float          # Green-Ampt wetting-front suction, mm


SOIL_WATER = {
    "dry_sand": Hydraulics(0.437, 0.091, 0.033, 5040.0, 49.5),
    "sandy_loam": Hydraulics(0.453, 0.207, 0.095, 262.0, 110.0),
    "clayey_soil": Hydraulics(0.464, 0.318, 0.197, 24.0, 209.0),
    "heavy_clay": Hydraulics(0.475, 0.396, 0.272, 7.2, 316.0),
    "lateritic_gravel": Hydraulics(0.40, 0.20, 0.10, 1440.0, 80.0),
}
LAYER = 0.30           # m of soil the bucket represents


def doy(date: str | int | dt.date) -> int:
    """Day of year (1..365) from 'MM-DD', a date or an int."""
    if isinstance(date, int):
        return (date - 1) % 365 + 1
    if isinstance(date, str):
        m, d = (int(v) for v in date.split("-")[-2:])
        date = dt.date(2025, m, d)
    return date.timetuple().tm_yday


def month_of(day: int) -> int:
    acc = 0
    for m, n in enumerate(DAYS):
        acc += n
        if day <= acc:
            return m
    return 11


def rainfall(region: str, days: int = 365, seed: int = 0, start: int = 1) -> np.ndarray:
    """Daily rain (mm) for `days` days from day-of-year `start`."""
    rain, wet, _ = CLIMATE[region]
    rng = np.random.default_rng(seed)
    out = np.zeros(days)
    was_wet = False
    for k in range(days):
        m = month_of((start + k - 1) % 365 + 1)
        p = min(wet[m] / DAYS[m], 0.95)
        # persistence: a wet day is likelier after a wet day (monsoon spells), mean frequency kept at p
        p_ww = min(0.95, p + 0.45 * (1 - p)) if p > 0.05 else p
        p_dw = p * (1 - p_ww) / max(1 - p, 1e-6)
        was_wet = rng.random() < (p_ww if was_wet else p_dw)
        if was_wet and wet[m] > 0:
            out[k] = rng.exponential(rain[m] / wet[m])
    return out


def infiltrate(h: Hydraulics, theta: float, rain_mm: float, hours: float = 3.0) -> float:
    """Water (mm) a storm of rain_mm over `hours` puts into the soil (Green-Ampt, ponded)."""
    if rain_mm <= 0:
        return 0.0
    dtheta = max(h.theta_s - theta, 0.01)
    t = hours / 24.0
    F = rain_mm
    for _ in range(30):                      # cumulative infiltration capacity after t days
        F = h.ks * t + h.psi * dtheta * math.log(1 + F / (h.psi * dtheta))
    return min(rain_mm, F)


@dataclass
class Bucket:
    """One root-zone bucket. depth m; theta volumetric; kc bare-soil / young plant ET coefficient;
    mulch cuts evaporation; extra_mm: storage a hydrogel adds (water held above field capacity)."""
    h: Hydraulics
    theta: float
    depth: float = LAYER
    kc: float = 0.55
    mulch: float = 0.0
    extra_mm: float = 0.0
    gel: float = 0.0              # water in the gel, mm

    def step(self, rain_mm: float, et0: float, harvest: float = 1.0, irrigation_mm: float = 0.0) -> dict:
        h, D = self.h, self.depth * 1000.0
        inflow = infiltrate(h, self.theta, rain_mm * harvest) + irrigation_mm
        runoff = rain_mm * harvest - (inflow - irrigation_mm)
        w = self.theta * D + inflow
        # gel soaks up water above field capacity, gives it back as the soil dries
        if self.extra_mm:
            take = min(max(w - h.fc * D, 0.0), self.extra_mm - self.gel)
            self.gel += take
            w -= take
        drain = 0.0
        if w > h.fc * D:
            tau = max(0.5, 3.0 * math.sqrt(100.0 / max(h.ks, 1e-3)))     # days: sand drains in hours, loam in two days, clay in ~10
            drain = (w - h.fc * D) * (1 - math.exp(-1.0 / tau))
            w -= drain
        stress = min(max((w / D - h.wp) / max(h.fc - h.wp, 1e-6), 0.0), 1.0)
        et = et0 * self.kc * (1 - 0.5 * self.mulch) * max(stress, 0.15 * (w / D > 0.5 * h.wp))
        w = max(w - et, 0.3 * h.wp * D)
        if self.gel > 0 and w < h.fc * D:
            give = min(self.gel, h.fc * D - w)
            self.gel -= give
            w += give
        w = min(w, h.theta_s * D)
        self.theta = w / D
        return {"theta": self.theta, "runoff": runoff, "drain": drain, "et": et,
                "saturated": self.theta >= h.theta_s - 0.02}


def topsoil_series(region: str, soil: str, days: int = 365, seed: int = 0, start: int = 1,
                   spinup: int = 365) -> np.ndarray:
    """Daily topsoil moisture for `days` days from day-of-year `start`, after a year's spin-up."""
    h = SOIL_WATER[soil]
    b = Bucket(h, 0.5 * (h.fc + h.wp))
    _, _, et0 = CLIMATE[region]
    s0 = (start - spinup - 1) % 365 + 1
    rain = rainfall(region, spinup + days, seed, s0)
    out = np.zeros(days)
    for k in range(spinup + days):
        r = b.step(rain[k], et0[month_of((s0 + k - 1) % 365 + 1)])
        if k >= spinup:
            out[k - spinup] = r["theta"]
    return out


def moisture_on(region: str, soil: str, date, seed: int = 0) -> float:
    """Topsoil moisture on a date: the mean of five rain sequences, so one storm does not decide it."""
    d = doy(date)
    return float(np.mean([topsoil_series(region, soil, 1, seed + k, d)[0] for k in range(5)]))


def pit_wetting(soil: str, theta: float, litres: float, area_m2: float) -> float:
    """Depth (m) the wetting front of a dose poured into a pit or basin of `area_m2` reaches:
    the dose fills the soil from theta up to field capacity (Green-Ampt piston front)."""
    h = SOIL_WATER[soil]
    return litres / 1000.0 / max(area_m2, 1e-3) / max(h.fc - theta, 0.02)
