"""Planting prescriptions by region: what pit to make, what goes back into it, how deep the plant
sits, what catches water around it, how much water it gets, and which species suits the
microsite. The planting head asks prescribe() for every spot and carries the answer out with the
robot's own mechanisms; the regions' soil, sodicity, hardpan and waterlogging come from the site
generator (fs_gazebo.regions, same names).

The methods follow published Indian practice; SOURCES lists them and each region names the ones
it uses. Where a source gives a range (pit depth, gypsum dose), the midpoint is used. Where the
robots cannot do what a manual crew would (a 60 cm pit for Jaal, a mound for Jaal), prescribe()
says what was done instead and why: the record keeps both.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace

SOURCES = {
    "cazri_pit": ("CAZRI practice on the sandy plains: a 50 x 50 x 50 cm pit per seedling with a saucer-shaped "
                  "basin about 1 m across to catch extra rain", "http://wanatca.org.au/acotanc/papers/tewari-1/"),
    "cazri_bulletin": ("K. D. Muthana, Improved techniques for tree plantation in the arid zone (CAZRI)",
                       "https://www.cazri.res.in/publications/KrishiKosh/8-(IMPROVED%20TECHNIQUES%20FOR%20TREE%20PLANTATION).pdf"),
    "cssri_auger": ("CSSRI Karnal auger-hole technique for sodic land: holes 15-25 cm across and 1.0-1.8 m deep "
                    "through the kankar pan, refilled with sodic soil and sand (1:1) plus 3-4 kg gypsum and 7-8 kg "
                    "FYM; ~80 % survival of Prosopis after 8-10 years",
                    "https://www.researchgate.net/publication/357303756_Afforestation_of_salt_affected_alluvial_soils"),
    "cssri_gap": ("CSSRI, Good Agricultural Practices for salt-affected soils", "https://cssri.res.in/poxumyse/2023/05/GAP.pdf"),
    "icfre_swc": ("ICFRE, A Handbook on Soil Water Conservation: contour trenches 50 x 50 cm (45 x 45 cm also "
                  "used), 2-3 m long, rows 3-5 m apart, staggered", "https://icfre.gov.in/pdf/books-file/book10.pdf"),
    "aravali_plan": ("Aravali plantation project: staggered trenches 2.0 x 0.45 x 0.45 m with a 45 cm pit 15 cm "
                     "from the trench", "https://forestsclearance.nic.in/DownloadPdfFile.aspx?FileName=0_0_71112122612161CA.pdf&FilePath=../writereaddata/Addinfo/"),
    "steep_trench": ("Staggered contour trenches suit 10-25 % slopes; on steeper ground they can trigger "
                     "landslides", "https://www.cllmp.com/wp-content/uploads/2020/06/Structures-for-commons-CCT-SCT-etc..pdf"),
    "irc_sp103": ("IRC:SP:103-2014 roadside plantation: mounding is essential on wet ground; Jamun for "
                  "waterlogged soils, Arjun for loams", "https://law.resource.org/pub/in/bis/irc/irc.gov.in.sp.103.2014.pdf"),
    "icid_ridge": ("ICID bio-drainage report: waterlogged land can be planted on ridges; Eucalyptus as a "
                   "bio-drainer", "https://www.icid.org/report_bmw_n_ss.pdf"),
    "hydrogel": ("Hydrogel 3-4 g per seedling mixed into the planting soil; survival gains are largest without "
                 "irrigation", "http://www.scielo.br/j/rarv/a/YqJtBg6SFLQXWmBqGDHBV6n/?lang=en"),
    "root_collar": ("Container seedlings: root collar level with or slightly below the surrounding ground",
                    "https://p-20.montana.edu/extension/valley/documents/hand%20planting%20trees%20and%20shrubs.pdf"),
    "sor_pits": ("Forest schedules of rates price planting pits at 30 cm or 45 cm cubed for ordinary and hard soils",
                 "https://fmis.telangana.gov.in/Downloads/FSR_2025-26.pdf"),
    "pit_60": ("Horticultural practice: 60 x 60 x 60 cm pits refilled with topsoil, FYM and some sand; for "
               "heavy clay, confirm the mix locally (sand in clay can make a poorly drained pocket)",
               "https://agriculture.institute/floriculture-and-landscaping/preparing-pits-for-planting-trees/"),
    "dhau": ("Anogeissus pendula (Dhau) holds steep rocky Aravalli slopes; Acacia senegal (Kumtha) the gentler "
             "rocky ground", "https://forest.delhi.gov.in/sites/default/files/generic_multiple_files/final_forest_pocket_book.pdf"),
    "garhwal": ("Garhwal Himalaya: Quercus leucotrichophora dominant at lower, Cedrus deodara at middle and upper "
                "altitudes", "https://smujo.id/biodiv/article/view/28"),
}


@dataclass(frozen=True)
class Method:
    pit_depth: float = 0.30          # m below grade, bored
    pit_width: float = 0.15          # m across, made with a cluster of bores where wider than the auger
    backfill: tuple = ()             # ((material, kg), ...) mixed into the refill below the plug
    plant_depth: float = 0.0         # root collar relative to grade, m: negative sunken, positive on a mound
    catchment: str = "none"          # none | basin | halfmoon | trench | mound
    mulch: str = "none"              # none | stone | organic
    water_l: float = 1.0
    pre_water_l: float = 0.0         # into the pit first when the soil is drier than min_moisture
    min_moisture: float = 0.0
    tool: str = "auger"              # auger | pick | seedball
    note: str = ""


@dataclass(frozen=True)
class Rule:
    microsite: str
    when: dict                        # tests on the soil sample, all must hold (see matches())
    species: tuple
    method: dict = field(default_factory=dict)   # overrides of the region's base method


@dataclass(frozen=True)
class Region:
    name: str
    title: str
    rainfall_mm: str
    window: str                       # when to plant
    spacing: dict                     # site -> (spacing m, row spacing m)
    base: Method
    rules: tuple                      # first match wins; the last has when={}
    sources: tuple


# ------------------------------------------------------------------ the six regions
HYDROGEL = ("hydrogel", 0.0035)

REGIONS = {
    "eastern_plain": Region(
        "eastern_plain", "Semi-arid eastern plain (Rajasthan IIIa)", "500-650 mm",
        "July-August, once the monsoon has wetted the pit soil; pre-water in dry spells",
        {"roadside": (5.0, 0.0), "open_field": (3.0, 3.0)},
        Method(pit_depth=0.45, pit_width=0.45, backfill=(("FYM", 2.0), HYDROGEL), plant_depth=-0.02,
               catchment="basin", water_l=3.0, pre_water_l=3.0, min_moisture=0.08),
        (Rule("waterlogged hollow", {"waterlog_gt": 0.5}, ("Syzygium cumini", "Terminalia arjuna"),
              {"catchment": "mound", "plant_depth": 0.25, "backfill": (("FYM", 2.0),), "water_l": 1.0}),
         Rule("sand patch", {"soil_in": ("dry_sand",)}, ("Prosopis cineraria", "Acacia senegal")),
         Rule("clay patch", {"soil_in": ("clayey_soil", "heavy_clay")}, ("Azadirachta indica", "Holoptelea integrifolia"),
              {"catchment": "none"}),
         Rule("sandy loam", {}, ("Azadirachta indica", "Dalbergia sissoo", "Pongamia pinnata", "Ficus religiosa"))),
        ("sor_pits", "aravali_plan", "hydrogel", "root_collar", "irc_sp103")),

    "thar_arid": Region(
        "thar_arid", "Arid western plain (Rajasthan Ia)", "250-350 mm",
        "July-August with the first soaking rains; never into dry sand without pre-watering",
        {"open_field": (5.0, 5.0)},
        Method(pit_depth=0.50, pit_width=0.50, backfill=(("FYM", 2.0), HYDROGEL), plant_depth=-0.08,
               catchment="basin", mulch="stone", water_l=5.0, pre_water_l=5.0, min_moisture=0.06,
               note="sunken in the saucer so runoff collects at the plant"),
        (Rule("saline patch", {"esp_gt": 15}, ("Salvadora oleoides", "Tamarix aphylla"),
              {"backfill": (("FYM", 2.0), ("gypsum", 1.5), HYDROGEL)}),
         Rule("kankar pan", {"hardpan_lt": 0.6}, ("Prosopis cineraria", "Ziziphus mauritiana"),
              {"note": "pit bored through the pan for the taproot"}),
         Rule("interdune hollow", {"moisture_gt": 0.07}, ("Tecomella undulata", "Ziziphus mauritiana")),
         Rule("sandy plain", {}, ("Prosopis cineraria", "Acacia senegal", "Capparis decidua"))),
        ("cazri_pit", "cazri_bulletin", "hydrogel", "root_collar")),

    "ignp_canal": Region(
        "ignp_canal", "IGNP canal command, sodic and waterlogged patches (Rajasthan Ib)", "200-350 mm + canal seepage",
        "July-September; gypsum needs the monsoon to leach the sodium",
        {"roadside": (4.0, 0.0), "open_field": (3.0, 3.0)},
        Method(pit_depth=0.45, pit_width=0.45, backfill=(("FYM", 3.0),), catchment="basin", water_l=2.0),
        (Rule("waterlogged", {"waterlog_gt": 0.5}, ("Eucalyptus camaldulensis", "Syzygium cumini", "Terminalia arjuna"),
              {"catchment": "mound", "plant_depth": 0.3, "pit_depth": 0.3, "water_l": 0.5,
               "note": "ridge/mound planting above the water table; Eucalyptus as a bio-drainer"}),
         # CSSRI's survival figures are for Prosopis juliflora, but it is invasive in Rajasthan (vilayati
         # babool); native salt-tolerant species take its place here.
         Rule("strongly sodic, kankar pan", {"esp_gt": 30}, ("Acacia nilotica", "Salvadora persica", "Tamarix aphylla"),
              {"pit_depth": 1.2, "pit_width": 0.18,
               "backfill": (("sand", 25.0), ("gypsum", 3.5), ("FYM", 7.5)),
               "note": "CSSRI auger hole through the kankar pan, refilled with soil:sand 1:1 + gypsum + FYM"}),
         Rule("moderately sodic", {"esp_gt": 15}, ("Acacia nilotica", "Terminalia arjuna", "Pongamia pinnata"),
              {"pit_depth": 1.0, "pit_width": 0.18, "backfill": (("sand", 20.0), ("gypsum", 2.0), ("FYM", 5.0)),
               "note": "auger hole through the pan, lighter gypsum dose"}),
         Rule("normal alluvium", {}, ("Dalbergia sissoo", "Azadirachta indica", "Acacia nilotica"))),
        ("cssri_auger", "cssri_gap", "irc_sp103", "icid_ridge")),

    "hadoti_vertisol": Region(
        "hadoti_vertisol", "Black cotton soil of the south-eastern plain (Rajasthan V)", "650-1000 mm",
        "Early monsoon, when the clay is moist but not sticky (it cracks when dry, smears when wet)",
        {"open_field": (3.0, 3.0), "roadside": (5.0, 0.0)},
        Method(pit_depth=0.60, pit_width=0.60, backfill=(("FYM", 5.0), ("sand", 8.0)), plant_depth=0.0,
               catchment="none", water_l=2.0, pre_water_l=4.0, min_moisture=0.15,
               note="no basin: water stands on vertisols; pits backfilled for drainage"),
        (Rule("waterlogged", {"waterlog_gt": 0.5}, ("Terminalia arjuna", "Syzygium cumini"),
              {"catchment": "mound", "plant_depth": 0.25, "water_l": 0.5}),
         Rule("lighter clay", {"soil_in": ("clayey_soil",)}, ("Azadirachta indica", "Dalbergia latifolia")),
         Rule("black cotton", {}, ("Azadirachta indica", "Pongamia pinnata", "Acacia nilotica"))),
        ("pit_60", "sor_pits", "irc_sp103")),

    "aravalli_hills": Region(
        "aravalli_hills", "Aravalli hills (Rajasthan IVa)", "600-750 mm",
        "July-August; trenches dug before the monsoon so they fill with the first rains",
        {"hill_slope": (2.5, 4.0)},
        Method(pit_depth=0.45, pit_width=0.45, backfill=(("FYM", 1.0), HYDROGEL), catchment="trench",
               mulch="stone", water_l=2.0, pre_water_l=2.0, min_moisture=0.06,
               note="staggered contour trench 2 m long uphill of the pit"),
        (Rule("rock outcrop", {"rock_lt": 0.12}, ("Acacia senegal", "Anogeissus pendula"),
              {"tool": "seedball", "catchment": "none", "mulch": "none", "water_l": 0.0, "pre_water_l": 0.0,
               "note": "direct seeding: no soil to plant into"}),
         Rule("stony shallow soil", {"rock_lt": 0.30}, ("Anogeissus pendula", "Acacia senegal"),
              {"tool": "pick", "pit_depth": 0.25}),
         Rule("deeper colluvium", {"rock_gt": 0.5}, ("Butea monosperma", "Acacia catechu", "Wrightia tinctoria")),
         Rule("rocky slope", {}, ("Anogeissus pendula", "Acacia catechu"))),
        ("aravali_plan", "icfre_swc", "dhau", "hydrogel")),

    "himalayan_midhills": Region(
        "himalayan_midhills", "Himalayan mid-hills (Garhwal)", "1200-1800 mm",
        "Monsoon (July-August), or winter rains in the higher belt",
        {"hill_slope": (2.5, 3.0)},
        Method(pit_depth=0.45, pit_width=0.45, backfill=(("FYM", 1.0),), catchment="halfmoon", mulch="organic",
               water_l=0.5, note="no contour trenches on slopes this steep; a half-moon eyebrow instead"),
        (Rule("rock outcrop", {"rock_lt": 0.12}, ("Myrica esculenta", "Rhododendron arboreum"),
              {"tool": "seedball", "catchment": "none", "mulch": "none", "water_l": 0.0}),
         Rule("wet gully", {"moisture_gt": 0.31}, ("Alnus nepalensis",), {"catchment": "none"}),
         Rule("upper slope", {"elev_frac_gt": 0.5}, ("Cedrus deodara", "Quercus leucotrichophora")),
         Rule("lower slope", {}, ("Quercus leucotrichophora", "Myrica esculenta", "Rhododendron arboreum"))),
        ("garhwal", "steep_trench", "icfre_swc", "root_collar")),
}

# What each machine can physically do (and what it does instead).
CAPABILITY = {
    "katar": {"depth": 0.55, "deep_kit": 1.25, "auger_d": 0.15, "pattern": "xy", "catchments": ("none", "basin", "mound")},
    "jaal": {"depth": 0.55, "deep_kit": None, "auger_d": 0.12, "pattern": "x", "catchments": ("none", "basin")},
    # Pahadi's 2 kW auger takes ~100 s a bore in laterite: it bores one hole, not a five-bore pit
    "pahadi": {"depth": 0.35, "deep_kit": None, "auger_d": 0.15, "pattern": "single", "catchments": ("none", "halfmoon", "trench")},
}
ROBOT_REGIONS = {"katar": ("eastern_plain", "ignp_canal", "hadoti_vertisol"),
                 "jaal": ("eastern_plain", "thar_arid", "ignp_canal", "hadoti_vertisol"),
                 "pahadi": ("aravalli_hills", "himalayan_midhills")}


@dataclass(frozen=True)
class Prescription:
    region: str
    microsite: str
    species: str
    method: Method
    bores: tuple                      # bore offsets (x, y) in the robot frame, m; the first is the planting hole
    deviations: tuple = ()            # what the robot did differently from the prescription, and why
    sources: tuple = ()

    def summary(self) -> str:
        m = self.method
        parts = []
        if m.tool == "seedball":
            parts.append("seed balls")
        else:
            parts.append(f"{'pick' if m.tool == 'pick' else 'pit'} {m.pit_depth * 100:.0f} cm deep"
                         + (f", {m.pit_width * 100:.0f} cm across ({len(self.bores)} bores)" if len(self.bores) > 1 else ""))
        if m.backfill:
            parts.append(" + ".join(f"{k} {v * 1000:.0f} g" if v < 0.1 else f"{k} {v:g} kg" for k, v in m.backfill))
        if m.plant_depth < -0.01:
            parts.append(f"sunken {-m.plant_depth * 100:.0f} cm")
        elif m.plant_depth > 0.01:
            parts.append(f"on a {m.plant_depth * 100:.0f} cm mound")
        if m.catchment not in ("none", "mound"):
            parts.append({"basin": "saucer basin", "halfmoon": "half-moon", "trench": "contour trench"}[m.catchment])
        if m.mulch != "none":
            parts.append(f"{m.mulch} mulch")
        if m.water_l > 0:
            parts.append(f"{m.water_l:g} L")
        return " · ".join(parts)


def matches(when: dict, s: dict) -> bool:
    for k, v in when.items():
        key, op = k.rsplit("_", 1)
        if op == "in":
            if s["soil"]["name"] not in v:
                return False
            continue
        x = {"esp": s.get("esp", 0.0), "waterlog": s.get("waterlog", 0.0), "hardpan": s.get("hardpan", 9.0),
             "moisture": s["moisture"], "rock": s["rock_depth"], "elev_frac": s.get("elev_frac", 0.5)}[key]
        if (op == "gt" and not x > v) or (op == "lt" and not x < v):
            return False
    return True


def bore_offsets(kind: str, width: float) -> tuple:
    """Bores that open a pit `width` across with the robot's auger; the first is the centre."""
    cap = CAPABILITY[kind]
    d = cap["auger_d"]
    if width <= d * 1.25 or cap["pattern"] == "single":
        return ((0.0, 0.0),)
    o = (width - d) / 2
    if cap["pattern"] == "xy":
        return ((0.0, 0.0), (o, 0.0), (-o, 0.0), (0.0, o), (0.0, -o))
    o = min(o, 0.13)                                  # Jaal: along its shuttle only, which travels 0.3 m
    return ((0.0, 0.0), (-o, 0.0), (-2 * o, 0.0))


def prescribe(region: str, kind: str, sample: dict, seq: int = 0, deep_kit: bool = False) -> Prescription:
    """The method for one spot. sample: SoilMap.sample() plus optional elev_frac (0 bottom .. 1 top of the site)."""
    R = REGIONS[region]
    rule = next(r for r in R.rules if matches(r.when, sample))
    m = replace(R.base, **rule.method)
    cap = CAPABILITY[kind]
    dev = []
    if m.tool != "seedball":
        if m.tool == "auger" and sample["rock_depth"] < m.pit_depth:
            if kind == "pahadi":
                m = replace(m, tool="pick")
            else:
                dev.append(f"rock at {sample['rock_depth'] * 100:.0f} cm: pit only to the rock")
                m = replace(m, pit_depth=max(0.2, sample["rock_depth"] - 0.02))
        limit = cap["deep_kit"] if (deep_kit and cap["deep_kit"]) else cap["depth"]
        if m.pit_depth > limit + 1e-6:
            dev.append(f"{m.pit_depth * 100:.0f} cm prescribed, {limit * 100:.0f} cm is the auger's reach"
                       + ("" if deep_kit or not cap["deep_kit"] else " without the deep-auger kit"))
            m = replace(m, pit_depth=limit)
        need = 0.12 + max(0.0, -m.plant_depth) + 0.02          # the plug, plus any sinking
        if m.pit_depth < need:
            m = replace(m, pit_depth=need)
    if m.catchment not in cap["catchments"]:
        alt = "basin" if (m.catchment == "halfmoon" and "basin" in cap["catchments"]) else "none"
        dev.append(f"{m.catchment} needs another machine; {alt} instead")
        if m.catchment == "mound":
            m = replace(m, plant_depth=0.0)
        m = replace(m, catchment=alt)
    bores = ((0.0, 0.0),) if m.tool in ("seedball", "pick") else bore_offsets(kind, m.pit_width)
    if m.tool not in ("seedball", "pick") and len(bores) == 1 and m.pit_width > cap["auger_d"] * 1.25:
        dev.append(f"one {cap['auger_d'] * 100:.0f} cm bore instead of a {m.pit_width * 100:.0f} cm pit")
    species = rule.species[seq % len(rule.species)]
    return Prescription(region, rule.microsite, species, m, bores, tuple(dev), R.sources)


def species_plan(region: str, kind: str, samples, deep_kit: bool = False) -> list:
    """Species for a sequence of spots (their soil samples, in planting order)."""
    return [prescribe(region, kind, s, i, deep_kit).species for i, s in enumerate(samples)]


def cite(region: str) -> list:
    return [SOURCES[k] for k in REGIONS[region].sources]


def total_backfill(prescriptions) -> dict:
    out = {}
    for p in prescriptions:
        for k, v in p.method.backfill:
            out[k] = out.get(k, 0.0) + v
    return out


def water_per_plant(p: Prescription, moisture: float) -> float:
    m = p.method
    return m.water_l + (m.pre_water_l if moisture < m.min_moisture else 0.0)



SHORT = {
    'cazri_pit': 'Tewari, Plantation forestry in arid tropics (CAZRI practice)',
    'cazri_bulletin': 'Muthana, Improved techniques for tree plantation in the arid zone (CAZRI)',
    'cssri_auger': 'Afforestation of salt-affected alluvial soils (CSSRI auger-hole technique)',
    'cssri_gap': 'CSSRI, Good Agricultural Practices for salt-affected soils',
    'icfre_swc': 'ICFRE, Handbook on Soil Water Conservation',
    'aravali_plan': 'Aravali plantation project plan (Forest clearance)',
    'steep_trench': 'Staggered and continuous contour trenches (CLLMP)',
    'irc_sp103': 'IRC:SP:103-2014, roadside plantation guidelines',
    'icid_ridge': 'ICID, Bio-drainage for waterlogging and salinity',
    'hydrogel': 'Seedling growth under hydrogel doses (Revista Árvore)',
    'root_collar': 'Montana State extension, hand planting trees and shrubs',
    'sor_pits': 'Telangana Forest Department, schedule of rates 2025-26',
    'pit_60': 'Preparing pits for planting trees (Agriculture Institute)',
    'dhau': 'Delhi Forest Department, Plants of the Aravalli',
    'garhwal': 'Dominance and diversity of trees in the lesser Himalaya of Uttarakhand',
}


def field_sheets() -> str:
    """Markdown tables of every region's prescriptions (docs/planting_methods.md is generated from this)."""
    out = []
    for r in REGIONS.values():
        robots = [k for k, names in ROBOT_REGIONS.items() if r.name in names]
        spacing = "; ".join(f"{site.replace('_', ' ')} {s:g} m" + (f" x {rows:g} m" if rows else "")
                            for site, (s, rows) in r.spacing.items())
        out += [f"### {r.title} (`{r.name}`)", "",
                f"Rainfall {r.rainfall_mm}. Plant: {r.window}. Spacing: {spacing}. Robots: {', '.join(robots)}.", "",
                "| Microsite (from the soil map) | Species | Method |", "|---|---|---|"]
        for rule in r.rules:
            m = replace(r.base, **rule.method)
            text = Prescription(r.name, rule.microsite, "", m, ((0.0, 0.0),) * (2 if m.pit_width > 0.2 else 1)).summary()
            text = text.replace(" (2 bores)", "")
            if m.pre_water_l and m.min_moisture:
                text += f" (+{m.pre_water_l:g} L into the pit if moisture < {m.min_moisture:g})"
            cond = ", ".join(f"{k.rsplit('_', 1)[0]} {'>' if k.endswith('gt') else '<' if k.endswith('lt') else 'in'} "
                             f"{v if not isinstance(v, tuple) else '/'.join(v)}" for k, v in rule.when.items()) or "otherwise"
            note = f" {m.note}." if m.note else ""
            out.append(f"| {rule.microsite} ({cond}) | *{', '.join(rule.species)}* | {text}.{note} |")
        out += ["", "Sources: " + "; ".join(f"[{SHORT[k]}]({SOURCES[k][1]})" for k in r.sources), ""]
    return "\n".join(out)
