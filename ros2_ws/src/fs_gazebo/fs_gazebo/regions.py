"""Regional site conditions for the generated worlds: the soil, water and hardpan a robot meets in
each agro-climatic zone. The planting prescriptions for the same regions (pit, backfill, species,
water) live in fs_autonomy.silviculture; the names here and there must match.

Five of Rajasthan's agro-climatic zones (the state's Mission Hariyalo Rajasthan plants 10 crore
saplings a monsoon across all of them) and the Himalayan mid-hills, for a robot that has to work
outside the state too.

Layers written into the soil map besides elevation / soil / rock_depth / moisture:
  hardpan   depth to a hard calcic (kankar) pan, m; 9 = none. The auger cuts it at several times
            the soil's boring energy (fs::AugerSoil <hardpan_factor>).
  esp       exchangeable sodium percentage, %: > 15 is sodic; trees need gypsum in the backfill.
  waterlog  0..1 seasonal waterlogging (low patches, canal seepage): plant on a raised mound.
"""
from dataclasses import dataclass, field


@dataclass(frozen=True)
class RegionSite:
    name: str
    title: str
    sites: tuple                     # which generated sites (and so which robots) it applies to
    origin: tuple                    # WGS84 lat, lon, alt of the map origin, for geotagged records
    soil: str                        # dominant soil type (SOILS in sites.py)
    moisture: tuple = (0.12, 0.05)   # mean, spread of volumetric soil moisture at planting time
    rock_depth: tuple = (0.9, 0.3)   # mean, spread of depth to rock, m
    hardpan: tuple | None = None     # (min, max) depth of a kankar pan where present, m
    hardpan_cover: float = 0.0       # share of the ground with a pan
    esp: tuple | None = None         # (background, patch) exchangeable sodium %
    esp_cover: float = 0.0           # share of the ground in sodic patches
    waterlog_cover: float = 0.0      # share of the ground waterlogged in the monsoon
    secondary: dict = field(default_factory=dict)   # soil -> share of the ground
    slope_deg: float | None = None   # hill sites
    rock_cover: float | None = None  # hill sites: share of ground too thin for the auger
    ground_rgb: tuple = (0.66, 0.60, 0.42)


REGIONS = {
    "eastern_plain": RegionSite(
        "eastern_plain", "Semi-arid eastern plain (Rajasthan zone IIIa: Jaipur, Ajmer, Tonk)",
        ("roadside", "open_field"), (26.9124, 75.7873, 431.0), "sandy_loam",
        moisture=(0.13, 0.05), rock_depth=(0.9, 0.3), secondary={"clayey_soil": 0.18, "dry_sand": 0.12},
        waterlog_cover=0.04),
    "thar_arid": RegionSite(
        "thar_arid", "Arid western plain (Rajasthan zone Ia: Jodhpur, Barmer, Nagaur)",
        ("open_field",), (26.2389, 73.0243, 231.0), "dry_sand",
        moisture=(0.055, 0.02), rock_depth=(1.4, 0.3), esp=(4.0, 22.0), esp_cover=0.08,
        hardpan=(0.4, 0.9), hardpan_cover=0.15, ground_rgb=(0.83, 0.70, 0.48)),
    "ignp_canal": RegionSite(
        "ignp_canal", "Irrigated north-western plain, IGNP canal command (zone Ib: Sri Ganganagar, Hanumangarh)",
        ("roadside", "open_field"), (29.9038, 73.8772, 178.0), "clayey_soil",
        moisture=(0.19, 0.06), rock_depth=(1.6, 0.2), hardpan=(0.5, 1.0), hardpan_cover=0.7,
        esp=(12.0, 45.0), esp_cover=0.45, waterlog_cover=0.18, secondary={"sandy_loam": 0.3},
        ground_rgb=(0.68, 0.64, 0.56)),
    "hadoti_vertisol": RegionSite(
        "hadoti_vertisol", "Humid south-eastern plain, black cotton soil (zone V: Kota, Bundi, Baran)",
        ("open_field", "roadside"), (25.2138, 75.8648, 271.0), "heavy_clay",
        moisture=(0.22, 0.06), rock_depth=(1.1, 0.3), waterlog_cover=0.10, secondary={"clayey_soil": 0.15},
        ground_rgb=(0.30, 0.27, 0.24)),
    "aravalli_hills": RegionSite(
        "aravalli_hills", "Sub-humid southern plain and Aravalli hills (zone IVa: Udaipur, Rajsamand)",
        ("hill_slope",), (24.5854, 73.7125, 598.0), "lateritic_gravel",
        moisture=(0.09, 0.03), slope_deg=14.0, rock_cover=0.3, ground_rgb=(0.62, 0.52, 0.40)),
    "himalayan_midhills": RegionSite(
        "himalayan_midhills", "Himalayan mid-hills, moist subtropical to temperate (Garhwal, Uttarakhand)",
        ("hill_slope",), (30.0812, 78.2912, 1480.0), "sandy_loam",
        moisture=(0.26, 0.05), slope_deg=24.0, rock_cover=0.18, secondary={"clayey_soil": 0.3},
        ground_rgb=(0.42, 0.40, 0.30)),
}

DEFAULT = {"roadside": "eastern_plain", "open_field": "eastern_plain", "hill_slope": "aravalli_hills"}


def get(name: str | None, site: str) -> RegionSite:
    name = name or DEFAULT[site]
    if name not in REGIONS:
        raise ValueError(f"unknown region '{name}'; choose from {', '.join(REGIONS)}")
    r = REGIONS[name]
    if site not in r.sites:
        raise ValueError(f"region '{name}' has no {site} site; it covers {', '.join(r.sites)}")
    return r
