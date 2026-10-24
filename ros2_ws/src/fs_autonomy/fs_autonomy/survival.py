"""First-year survival of a planted seedling, from the soil, the season and what the robot did.

A year of daily root-zone water after planting (fs_gazebo.climate: stochastic rain on the region's
normals, Green-Ampt infiltration, drainage, drying), with roots that grow only while there is water
to grow on, and a daily death hazard from:

  drought     root-zone water below 25 % of the available range; a seedling whose roots are still in
              the pit suffers most, so a method that waters well and harvests rain in the first monsoon
              gets roots down before the dry season
  waterlog    the root zone saturated for more than two days running: canal seepage or perched water
              on a pan in a low patch through the monsoon (two weeks in three, Jul-Oct, on waterlogged
              ground), or a clay that cannot drain a storm; a mound keeps the collar and upper roots
              above it. The same shallow water feeds roots that reach 0.5 m, so drought spares them
  sodicity    exchangeable sodium above 15 % in the root zone, worse as the soil dries and salts
              concentrate; gypsum in the backfill and an auger hole through the pan lower it
  shock       transplanting itself: a few percent, more for a plug planted into dry soil with little water

How the method enters, physically:
  pit depth / pan    roots start at the bottom of the pit; an unbroken kankar pan caps them, and below
                     the depth to rock they follow fissures into the weathered rock at a quarter of the rate
  backfill           FYM / compost raise water held at field capacity; sand opens a clay pit; gypsum as above;
                     hydrogel stores water above field capacity and gives it back as the pit dries
  catchment          basin, half-moon and trench harvest runoff from the ground around: a share of the
                     catchment's rain, scaled by how much of a storm that soil sheds
  sunken planting    collects water and shades the collar; a mound sheds rain but drains
  mulch              cuts evaporation (stone more than organic)

Species tolerances (0 = none, 1 = among the most tolerant) are compiled from the ICFRE / state forest
department species notes and CAZRI and CSSRI bulletins cited in fs_autonomy.silviculture; the hazard
constants are set so the model reproduces the survival those sources report for the regional methods
against ordinary pit planting (see docs/survival.md). It ranks methods and flags risk, it does not
replace a survival count in the field.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from fs_gazebo import climate


@dataclass(frozen=True)
class Traits:
    drought: float
    salt: float
    waterlog: float


SPECIES = {
    "Prosopis cineraria": Traits(0.95, 0.6, 0.2), "Acacia senegal": Traits(0.9, 0.5, 0.2),
    "Capparis decidua": Traits(0.95, 0.5, 0.1), "Tecomella undulata": Traits(0.9, 0.4, 0.2),
    "Ziziphus mauritiana": Traits(0.85, 0.5, 0.3), "Salvadora oleoides": Traits(0.9, 0.9, 0.4),
    "Salvadora persica": Traits(0.85, 0.95, 0.5), "Tamarix aphylla": Traits(0.85, 0.9, 0.6),
    "Acacia nilotica": Traits(0.8, 0.75, 0.7), "Azadirachta indica": Traits(0.8, 0.4, 0.2),
    "Pongamia pinnata": Traits(0.65, 0.6, 0.7), "Ficus religiosa": Traits(0.6, 0.3, 0.3),
    "Dalbergia sissoo": Traits(0.6, 0.4, 0.5), "Holoptelea integrifolia": Traits(0.7, 0.4, 0.3),
    "Eucalyptus camaldulensis": Traits(0.7, 0.6, 0.7), "Terminalia arjuna": Traits(0.5, 0.5, 0.9),
    "Syzygium cumini": Traits(0.5, 0.4, 0.8), "Butea monosperma": Traits(0.75, 0.6, 0.5),
    "Acacia catechu": Traits(0.8, 0.4, 0.2), "Wrightia tinctoria": Traits(0.8, 0.3, 0.2),
    "Anogeissus pendula": Traits(0.85, 0.3, 0.1), "Dalbergia latifolia": Traits(0.5, 0.2, 0.3),
    "Cedrus deodara": Traits(0.4, 0.1, 0.2), "Quercus leucotrichophora": Traits(0.5, 0.1, 0.3),
    "Alnus nepalensis": Traits(0.2, 0.1, 0.9), "Myrica esculenta": Traits(0.5, 0.1, 0.3),
    "Rhododendron arboreum": Traits(0.4, 0.1, 0.3),
}
DEFAULT_TRAITS = Traits(0.6, 0.3, 0.3)

HARVEST = {"none": 1.0, "basin": 2.2, "halfmoon": 3.5, "trench": 3.0, "mound": 0.8}     # catchment / root-zone area
RUNOFF = {"dry_sand": 0.15, "sandy_loam": 0.35, "clayey_soil": 0.5, "heavy_clay": 0.55, "lateritic_gravel": 0.45}
MULCH = {"none": 0.0, "stone": 0.8, "organic": 0.6}
DROUGHT_K = 0.012                       # daily hazard at zero available water for a species with no tolerance
YOUNG = (2.0, 1.0, 0.3)                 # drought multiplier with roots < 0.45 m, < 0.8 m, deeper
ROOT_AREA = math.pi * 0.3 ** 2          # m^2 a first-year root system draws on
HILLS = {"aravalli_hills": 14.0, "himalayan_midhills": 24.0}     # slope, deg: rain runs off a plain spot
PLANTING_DATE = {"thar_arid": "07-20", "eastern_plain": "07-15", "ignp_canal": "07-25", "hadoti_vertisol": "07-10",
                 "aravalli_hills": "07-15", "himalayan_midhills": "07-05"}


@dataclass
class Outlook:
    survival: float                  # expected first-year survival, 0..1
    causes: dict                     # share of the expected losses by cause
    root_depth: float                # m reached by the end of the first monsoon (day 90)
    dry_days: int                    # days the root zone was drier than 25 % available water

    @property
    def risk(self) -> str:
        if not self.causes:
            return ""
        cause, share = max(self.causes.items(), key=lambda kv: kv[1])
        return cause if 1 - self.survival > 0.1 and share > 0.4 else ""


def _kg(backfill, name):
    return sum(kg for m, kg in backfill if m == name)


def predict(region: str, species: str, method, sample: dict, hole_depth: float | None = None,
            water_l: float | None = None, date: str | int | None = None, seed: int = 0, runs: int = 4) -> Outlook:
    """Expected first-year survival of `species` planted by `method` (silviculture.Method) at a spot
    with soil `sample` (soil name, moisture, esp, hardpan, waterlog, rock_depth). hole_depth and
    water_l are what the robot actually achieved (default: as prescribed)."""
    soil = sample["soil"]["name"] if isinstance(sample.get("soil"), dict) else sample.get("soil", "sandy_loam")
    tr = SPECIES.get(species, DEFAULT_TRAITS)
    h0 = climate.SOIL_WATER[soil]
    depth = method.pit_depth if hole_depth is None else hole_depth
    water = (method.water_l + method.pre_water_l) if water_l is None else water_l
    start = climate.doy(date or PLANTING_DATE.get(region, "07-15"))
    rock = float(sample.get("rock_depth", 1.5))
    pan = float(sample.get("hardpan", 9.0))
    pan_broken = depth >= pan + 0.1
    cap = min(1.2, 9.0 if pan_broken else pan)                    # how deep roots can go
    wl = float(sample.get("waterlog", 0.0))

    # backfill: organic matter holds water, sand opens clay, gypsum and depth lower sodium
    fym = _kg(method.backfill, "FYM") + _kg(method.backfill, "compost")
    sand = _kg(method.backfill, "sand")
    gel_g = 1000 * _kg(method.backfill, "hydrogel")
    pit_share = min(1.0, max(depth, 0.15) / 0.5) * min(1.0, method.pit_width / 0.3)
    h = climate.Hydraulics(h0.theta_s, h0.fc + min(0.04, 0.006 * fym) * pit_share, h0.wp, h0.ks * (1 + 0.3 * sand * pit_share), h0.psi)
    esp = float(sample.get("esp", 0.0))
    esp *= max(0.25, 1.0 - _kg(method.backfill, "gypsum") / 4.5)
    if depth >= 0.9 and pan_broken:
        esp *= 0.6                                                   # roots reach the moister, less sodic subsoil
    salt_m = max(0.0, (esp - 15.0) / 35.0) * (1 - tr.salt)

    harvest = 1 + (HARVEST.get(method.catchment, 1.0) - 1) * RUNOFF.get(soil, 0.35) / 0.35
    if region in HILLS:
        # on a slope a plain spot loses part of each storm downhill (more near the crest); a half-moon
        # or trench catches it, which is most of their purpose
        lost = RUNOFF.get(soil, 0.35) * math.sin(math.radians(HILLS[region])) * 1.6
        lost *= 1.3 if float(sample.get("elev_frac", 0.5)) > 0.7 else 1.0
        harvest *= 1 - (lost if method.catchment in ("none", "mound") else 0.2 * lost)
    if method.plant_depth < -0.03:
        harvest += 0.25
    mound = method.plant_depth > 0.15
    _, _, et0 = climate.CLIMATE[region]

    scale = hazard_scale(region)
    total, causes, roots90, dry_total = 0.0, {"drought": 0.0, "waterlog": 0.0, "sodicity": 0.0, "shock": 0.0}, 0.0, 0
    for run in range(runs):
        rain = climate.rainfall(region, 365, seed * 101 + run, start)
        theta0 = float(sample.get("moisture", 0.5 * (h.fc + h.wp)))
        root = max(0.15, depth * 0.8)
        b = climate.Bucket(h, theta0, depth=root, mulch=MULCH.get(method.mulch, 0.0),
                           extra_mm=gel_g * 0.2 / ROOT_AREA)
        b.step(0.0, 0.0, irrigation_mm=water / ROOT_AREA)
        # shock: a plug into dry soil with little water loses more
        avail0 = (b.theta - h.wp) / (h.fc - h.wp)
        haz = {"drought": 0.0, "waterlog": 0.0, "sodicity": 0.0, "shock": 0.04 + 0.08 * max(0.0, 0.4 - avail0)}
        wet_run, dry = 0, 0
        for k in range(365):
            month = climate.month_of((start + k - 1) % 365 + 1)
            r = b.step(rain[k], et0[month], harvest=harvest)
            seep = wl > 0.3 and 6 <= month <= 9 and (k // 7) % 3 != 2 and k > 7
            if seep:
                b.theta = h.theta_s                                      # seepage / perched water table
            avail = (b.theta - h.wp) / max(h.fc - h.wp, 1e-6)
            if avail > 0.3 and root < cap:                               # roots follow water down
                grown = min(cap, root + (0.006 if root < rock else 0.0015))  # slowly in fissured rock
                b.theta = (b.theta * root + climate.SOIL_WATER[soil].fc * 0.8 * (grown - root)) / grown
                root = grown
                b.depth = root
            young = YOUNG[0] if root < 0.45 else (YOUNG[1] if root < 0.8 else YOUNG[2])
            if wl > 0.3 and root > 0.5:
                avail = max(avail, 0.3)                                  # capillary rise from the shallow water
            if avail < 0.25:
                dry += 1
                haz["drought"] += DROUGHT_K * (0.25 - avail) / 0.25 * math.sqrt(1 - tr.drought) * young
            sat = b.theta >= h.theta_s - 0.02
            wet_run = wet_run + 1 if sat else 0
            if wet_run > 2:
                haz["waterlog"] += 0.05 * (1 - tr.waterlog) * (0.25 if mound else 1.0) * (1.5 if not pan_broken and pan < 1 else 1.0)
            if salt_m > 0:
                haz["sodicity"] += salt_m / 365 * (1.6 if avail < 0.3 else 0.8) * (1.3 if root < 0.45 else 1.0)
            if k == 89:
                roots90 += root / runs
        dry_total += dry
        hz = scale * sum(haz.values())
        total += math.exp(-hz) / runs
        for c in causes:
            causes[c] += haz[c] / max(sum(haz.values()), 1e-9) / runs
    return Outlook(total, {c: round(v, 2) for c, v in causes.items()}, roots90, dry_total // runs)


def hazard_scale(region: str) -> float:
    """Field-count calibration (`ros2 run fs_bringup calibrate survival`): the factor on the model's
    hazards that reproduced a year-on survival count in this region."""
    import os
    from pathlib import Path
    path = Path(os.environ.get("FS_CALIBRATION_DIR", Path.home() / ".ros" / "fs_calibration")) / "survival.yaml"
    if not path.exists():
        return 1.0
    import yaml
    return float(((yaml.safe_load(path.read_text()) or {}).get(region) or {}).get("hazard_scale", 1.0))


def plain_pit():
    """Ordinary pit planting for comparison: a 30 cm hole, a litre of water, nothing else."""
    from .silviculture import Method
    return Method(pit_depth=0.30, pit_width=0.15, water_l=1.0)


def compare(region: str, kind: str, sample: dict, seq: int = 0, deep_kit: bool = True) -> tuple:
    """(prescribed method's outlook, plain pit's outlook) for the species the region prescribes."""
    from . import silviculture as sv
    p = sv.prescribe(region, kind, sample, seq, deep_kit)
    return p, predict(region, p.species, p.method, sample), predict(region, p.species, plain_pit(), sample)


def report() -> str:
    """docs/survival.md: each region's microsites, prescribed method against a plain pit."""
    from . import silviculture as sv
    rows = ["# Predicted first-year survival", "",
            "Generated by `python3 -m fs_autonomy.survival` (model in `fs_autonomy/survival.py`): mean of four rain years, "
            "planting on the region's usual date. The plain pit is a 30 cm auger hole with a litre of water.", "",
            "| Region | Microsite | Robot | Species | Prescribed | Plain pit | Main risk |", "|---|---|---|---|---|---|---|"]
    for name, kind, sample in CASES:
        r = sv.REGIONS[name]
        p, a, b = compare(name, kind, sample)
        rows.append(f"| {r.title.split('(')[0].strip()} | {p.microsite} | {kind} | *{p.species}* | {a.survival * 100:.0f} % "
                    f"| {b.survival * 100:.0f} % | {a.risk or '-'} |")
    return "\n".join(rows) + "\n\n" + READING


READING = """## How to read it

The model runs a year of daily root-zone water after planting. It uses the region's rain normals,
Green-Ampt infiltration, drainage and drying (`fs_gazebo/climate.py`). Roots only grow while the
soil is wet enough. Each day adds a death hazard for drought, waterlogging and sodicity, scaled by
the species' tolerance, plus a transplant-shock term. The method changes the physics directly:
- Pit depth decides where the roots start. An unbroken kankar pan or rock caps them.
- Backfill: FYM holds more water, sand drains a clay pit, gypsum lowers sodium, and hydrogel stores water.
- Basins, half-moons and trenches harvest runoff, in proportion to how much of a storm the soil sheds.
- A mound lifts the collar above standing water, and mulch slows drying.

Every planting record carries the prediction for that plant (`survival_1y`, `survival_risk`), using
the hole depth and water actually achieved. The mission summary adds up the trees expected alive
after a year.

The hazard constants were tuned so the regional methods and plain pits land in the ranges the cited
sources report: CAZRI for the Thar, CSSRI auger holes for sodic land, and state forest department
plantation reports. The output is a ranking and a risk flag, not a forecast. Replace the constants
with your own one-year survival counts as soon as you have them (`ros2 run fs_bringup calibrate survival`).

In the Himalayan mid-hills a July planting meets no water stress in the model either way. The
half-moon there is mainly for soil conservation on a 24° slope, which this model does not score.
"""


def _s(soil, moisture, esp=0.0, hardpan=9.0, waterlog=0.0, rock=1.5, elev_frac=0.5):
    return {"soil": {"name": soil}, "moisture": moisture, "esp": esp, "hardpan": hardpan, "waterlog": waterlog,
            "rock_depth": rock, "elev_frac": elev_frac, "elevation": 0.0}


CASES = [
    ("thar_arid", "jaal", _s("dry_sand", 0.05)),
    ("thar_arid", "jaal", _s("dry_sand", 0.05, esp=25)),
    ("eastern_plain", "katar", _s("sandy_loam", 0.13)),
    ("eastern_plain", "jaal", _s("sandy_loam", 0.13)),
    ("ignp_canal", "katar", _s("clayey_soil", 0.19, esp=40, hardpan=0.7, rock=1.6)),
    ("ignp_canal", "katar", _s("clayey_soil", 0.30, waterlog=0.9)),
    ("hadoti_vertisol", "jaal", _s("heavy_clay", 0.22)),
    ("aravalli_hills", "pahadi", _s("lateritic_gravel", 0.09, rock=0.8)),
    ("himalayan_midhills", "pahadi", _s("sandy_loam", 0.26, elev_frac=0.9)),
]


if __name__ == "__main__":
    import sys
    text = report()
    print(text)
    if len(sys.argv) > 2 and sys.argv[1] == "--out":
        open(sys.argv[2], "w").write(text)
