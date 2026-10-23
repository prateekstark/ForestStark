# Planting methods by region

The robots plant the way the region's soil and climate call for, not the same hole everywhere. For
every spot the planting head reads the soil map under the target (soil type, moisture, depth to
rock, sodicity, a kankar pan, seasonal waterlogging, position on the slope) and looks up the
region's prescription in [`fs_autonomy/silviculture.py`](../ros2_ws/src/fs_autonomy/fs_autonomy/silviculture.py).
The prescription gives the species for that microsite, the pit, what goes back into it, how deep
the plant sits, what catches water round it, mulch and water. The robot then does each step with
its own mechanisms, and the plant record keeps what was prescribed, what was done and why
anything differed.

Five regions are agro-climatic zones of Rajasthan, the state behind Mission Hariyalo Rajasthan's
10 crore saplings a monsoon. The sixth, the Himalayan mid-hills, shows the same machines working
outside the state.

```bash
ros2 launch fs_bringup jaal.launch.py robots:=1 region:=thar_arid
ros2 launch fs_bringup katar.launch.py region:=ignp_canal          # deep-auger kit on by default
ros2 launch fs_bringup pahadi.launch.py region:=himalayan_midhills
```

| Region | Robots | Default for |
|---|---|---|
| `eastern_plain` (Jaipur, Ajmer, Tonk) | Katar, Jaal | Katar, Jaal |
| `thar_arid` (Jodhpur, Barmer, Nagaur) | Jaal | |
| `ignp_canal` (Sri Ganganagar, Hanumangarh) | Katar, Jaal | |
| `hadoti_vertisol` (Kota, Bundi, Baran) | Katar, Jaal | |
| `aravalli_hills` (Udaipur, Rajsamand) | Pahadi | Pahadi |
| `himalayan_midhills` (Garhwal) | Pahadi | |

## What the site generator adds per region

Each region sets the generated site's soil and secondary soil patches, the moisture at planting
time, and three extra soil-map layers:

- `hardpan`: depth to a kankar (calcic) pan. `fs::AugerSoil` cuts it at 3.5 times the soil's
  boring energy, so a pan shows up as a slower, costlier bore.
- `esp`: exchangeable sodium percentage. Above 15 % the soil is sodic and needs gypsum.
- `waterlog`: seasonal waterlogging in low patches and along canal seepage.

The world also changes: the ground colour, salt crust on sodic patches, standing water on
waterlogged ones, and the geotag origin of the records.

## How each step is done by the machines

| Step | Katar (turret head on a boom) | Jaal (gantry + shuttle) | Pahadi (4-axis arm) |
|---|---|---|---|
| Wide pit | Cluster of 5 bores, the boom moves the auger | 3 bores along the shuttle | One bore (each takes ~100 s in laterite) |
| Deep auger hole | 1.2 m with the deep-auger kit (2.0 m feed) | Up to 0.55 m | Up to 0.35 m; percussive pick through stone |
| Pre-watering a dry pit | Dosing pump | Dosing pump | Dosing pump |
| Backfill with FYM, gypsum, sand, hydrogel | Screw feeder from trailer hoppers | Screw feeder from on-board hoppers | FYM and hydrogel only |
| Sunken or raised planting | Duckbill depth | Duckbill depth | Gripper depth |
| Saucer basin | Basin-forming ring (a timed step) | Basin-forming ring (a timed step) | Not available |
| Mound on waterlogged ground | From the auger spoil (a timed step) | Not available: planted flat, recorded | Not available |
| Contour trench / half-moon | Not available | Not available | Arm drags the auger tip along the contour |
| Mulch | Dispenser (a timed step) | Dispenser (a timed step) | Dispenser (a timed step) |
| Species for the microsite | Carousel cup with that species | Drum pocket with that species | Tray cell with that species |

Species matching works like a nursery loading plan. The head knows every upcoming spot's
species from the mission and labels the plugs in the store in that order. Each feeder then picks
the plug the next spot needs. If none of that species is left, the robot plants what it has and
records the substitution.

## Prescriptions

### Semi-arid eastern plain (Rajasthan IIIa) (`eastern_plain`)

Rainfall 500-650 mm. Plant: July-August, once the monsoon has wetted the pit soil; pre-water in dry spells. Spacing: roadside 5 m; open field 3 m x 3 m. Robots: katar, jaal.

| Microsite (from the soil map) | Species | Method |
|---|---|---|
| waterlogged hollow (waterlog > 0.5) | *Syzygium cumini, Terminalia arjuna* | pit 45 cm deep, 45 cm across · FYM 2 kg · on a 25 cm mound · 1 L (+3 L into the pit if moisture < 0.08). |
| sand patch (soil in dry_sand) | *Prosopis cineraria, Acacia senegal* | pit 45 cm deep, 45 cm across · FYM 2 kg + hydrogel 4 g · sunken 2 cm · saucer basin · 3 L (+3 L into the pit if moisture < 0.08). |
| clay patch (soil in clayey_soil/heavy_clay) | *Azadirachta indica, Holoptelea integrifolia* | pit 45 cm deep, 45 cm across · FYM 2 kg + hydrogel 4 g · sunken 2 cm · 3 L (+3 L into the pit if moisture < 0.08). |
| sandy loam (otherwise) | *Azadirachta indica, Dalbergia sissoo, Pongamia pinnata, Ficus religiosa* | pit 45 cm deep, 45 cm across · FYM 2 kg + hydrogel 4 g · sunken 2 cm · saucer basin · 3 L (+3 L into the pit if moisture < 0.08). |

Sources: [Telangana Forest Department, schedule of rates 2025-26](https://fmis.telangana.gov.in/Downloads/FSR_2025-26.pdf); [Aravali plantation project plan (Forest clearance)](https://forestsclearance.nic.in/DownloadPdfFile.aspx?FileName=0_0_71112122612161CA.pdf&FilePath=../writereaddata/Addinfo/); [Seedling growth under hydrogel doses (Revista Árvore)](http://www.scielo.br/j/rarv/a/YqJtBg6SFLQXWmBqGDHBV6n/?lang=en); [Montana State extension, hand planting trees and shrubs](https://p-20.montana.edu/extension/valley/documents/hand%20planting%20trees%20and%20shrubs.pdf); [IRC:SP:103-2014, roadside plantation guidelines](https://law.resource.org/pub/in/bis/irc/irc.gov.in.sp.103.2014.pdf)

### Arid western plain (Rajasthan Ia) (`thar_arid`)

Rainfall 250-350 mm. Plant: July-August with the first soaking rains; never into dry sand without pre-watering. Spacing: open field 5 m x 5 m. Robots: jaal.

| Microsite (from the soil map) | Species | Method |
|---|---|---|
| saline patch (esp > 15) | *Salvadora oleoides, Tamarix aphylla* | pit 50 cm deep, 50 cm across · FYM 2 kg + gypsum 1.5 kg + hydrogel 4 g · sunken 8 cm · saucer basin · stone mulch · 5 L (+5 L into the pit if moisture < 0.06). sunken in the saucer so runoff collects at the plant. |
| kankar pan (hardpan < 0.6) | *Prosopis cineraria, Ziziphus mauritiana* | pit 50 cm deep, 50 cm across · FYM 2 kg + hydrogel 4 g · sunken 8 cm · saucer basin · stone mulch · 5 L (+5 L into the pit if moisture < 0.06). pit bored through the pan for the taproot. |
| interdune hollow (moisture > 0.07) | *Tecomella undulata, Ziziphus mauritiana* | pit 50 cm deep, 50 cm across · FYM 2 kg + hydrogel 4 g · sunken 8 cm · saucer basin · stone mulch · 5 L (+5 L into the pit if moisture < 0.06). sunken in the saucer so runoff collects at the plant. |
| sandy plain (otherwise) | *Prosopis cineraria, Acacia senegal, Capparis decidua* | pit 50 cm deep, 50 cm across · FYM 2 kg + hydrogel 4 g · sunken 8 cm · saucer basin · stone mulch · 5 L (+5 L into the pit if moisture < 0.06). sunken in the saucer so runoff collects at the plant. |

Sources: [Tewari, Plantation forestry in arid tropics (CAZRI practice)](http://wanatca.org.au/acotanc/papers/tewari-1/); [Muthana, Improved techniques for tree plantation in the arid zone (CAZRI)](https://www.cazri.res.in/publications/KrishiKosh/8-(IMPROVED%20TECHNIQUES%20FOR%20TREE%20PLANTATION).pdf); [Seedling growth under hydrogel doses (Revista Árvore)](http://www.scielo.br/j/rarv/a/YqJtBg6SFLQXWmBqGDHBV6n/?lang=en); [Montana State extension, hand planting trees and shrubs](https://p-20.montana.edu/extension/valley/documents/hand%20planting%20trees%20and%20shrubs.pdf)

### IGNP canal command, sodic and waterlogged patches (Rajasthan Ib) (`ignp_canal`)

Rainfall 200-350 mm + canal seepage. Plant: July-September; gypsum needs the monsoon to leach the sodium. Spacing: roadside 4 m; open field 3 m x 3 m. Robots: katar, jaal.

| Microsite (from the soil map) | Species | Method |
|---|---|---|
| waterlogged (waterlog > 0.5) | *Eucalyptus camaldulensis, Syzygium cumini, Terminalia arjuna* | pit 30 cm deep, 45 cm across · FYM 3 kg · on a 30 cm mound · 0.5 L. ridge/mound planting above the water table; Eucalyptus as a bio-drainer. |
| strongly sodic, kankar pan (esp > 30) | *Acacia nilotica, Salvadora persica, Tamarix aphylla* | pit 120 cm deep · sand 25 kg + gypsum 3.5 kg + FYM 7.5 kg · saucer basin · 2 L. CSSRI auger hole through the kankar pan, refilled with soil:sand 1:1 + gypsum + FYM. |
| moderately sodic (esp > 15) | *Acacia nilotica, Terminalia arjuna, Pongamia pinnata* | pit 100 cm deep · sand 20 kg + gypsum 2 kg + FYM 5 kg · saucer basin · 2 L. auger hole through the pan, lighter gypsum dose. |
| normal alluvium (otherwise) | *Dalbergia sissoo, Azadirachta indica, Acacia nilotica* | pit 45 cm deep, 45 cm across · FYM 3 kg · saucer basin · 2 L. |

Sources: [Afforestation of salt-affected alluvial soils (CSSRI auger-hole technique)](https://www.researchgate.net/publication/357303756_Afforestation_of_salt_affected_alluvial_soils); [CSSRI, Good Agricultural Practices for salt-affected soils](https://cssri.res.in/poxumyse/2023/05/GAP.pdf); [IRC:SP:103-2014, roadside plantation guidelines](https://law.resource.org/pub/in/bis/irc/irc.gov.in.sp.103.2014.pdf); [ICID, Bio-drainage for waterlogging and salinity](https://www.icid.org/report_bmw_n_ss.pdf)

### Black cotton soil of the south-eastern plain (Rajasthan V) (`hadoti_vertisol`)

Rainfall 650-1000 mm. Plant: Early monsoon, when the clay is moist but not sticky (it cracks when dry, smears when wet). Spacing: open field 3 m x 3 m; roadside 5 m. Robots: katar, jaal.

| Microsite (from the soil map) | Species | Method |
|---|---|---|
| waterlogged (waterlog > 0.5) | *Terminalia arjuna, Syzygium cumini* | pit 60 cm deep, 60 cm across · FYM 5 kg + sand 8 kg · on a 25 cm mound · 0.5 L (+4 L into the pit if moisture < 0.15). no basin: water stands on vertisols; pits backfilled for drainage. |
| lighter clay (soil in clayey_soil) | *Azadirachta indica, Dalbergia latifolia* | pit 60 cm deep, 60 cm across · FYM 5 kg + sand 8 kg · 2 L (+4 L into the pit if moisture < 0.15). no basin: water stands on vertisols; pits backfilled for drainage. |
| black cotton (otherwise) | *Azadirachta indica, Pongamia pinnata, Acacia nilotica* | pit 60 cm deep, 60 cm across · FYM 5 kg + sand 8 kg · 2 L (+4 L into the pit if moisture < 0.15). no basin: water stands on vertisols; pits backfilled for drainage. |

Sources: [Preparing pits for planting trees (Agriculture Institute)](https://agriculture.institute/floriculture-and-landscaping/preparing-pits-for-planting-trees/); [Telangana Forest Department, schedule of rates 2025-26](https://fmis.telangana.gov.in/Downloads/FSR_2025-26.pdf); [IRC:SP:103-2014, roadside plantation guidelines](https://law.resource.org/pub/in/bis/irc/irc.gov.in.sp.103.2014.pdf)

### Aravalli hills (Rajasthan IVa) (`aravalli_hills`)

Rainfall 600-750 mm. Plant: July-August; trenches dug before the monsoon so they fill with the first rains. Spacing: hill slope 2.5 m x 4 m. Robots: pahadi.

| Microsite (from the soil map) | Species | Method |
|---|---|---|
| rock outcrop (rock < 0.12) | *Acacia senegal, Anogeissus pendula* | seed balls · FYM 1 kg + hydrogel 4 g. direct seeding: no soil to plant into. |
| stony shallow soil (rock < 0.3) | *Anogeissus pendula, Acacia senegal* | pick 25 cm deep, 45 cm across · FYM 1 kg + hydrogel 4 g · contour trench · stone mulch · 2 L (+2 L into the pit if moisture < 0.06). staggered contour trench 2 m long uphill of the pit. |
| deeper colluvium (rock > 0.5) | *Butea monosperma, Acacia catechu, Wrightia tinctoria* | pit 45 cm deep, 45 cm across · FYM 1 kg + hydrogel 4 g · contour trench · stone mulch · 2 L (+2 L into the pit if moisture < 0.06). staggered contour trench 2 m long uphill of the pit. |
| rocky slope (otherwise) | *Anogeissus pendula, Acacia catechu* | pit 45 cm deep, 45 cm across · FYM 1 kg + hydrogel 4 g · contour trench · stone mulch · 2 L (+2 L into the pit if moisture < 0.06). staggered contour trench 2 m long uphill of the pit. |

Sources: [Aravali plantation project plan (Forest clearance)](https://forestsclearance.nic.in/DownloadPdfFile.aspx?FileName=0_0_71112122612161CA.pdf&FilePath=../writereaddata/Addinfo/); [ICFRE, Handbook on Soil Water Conservation](https://icfre.gov.in/pdf/books-file/book10.pdf); [Delhi Forest Department, Plants of the Aravalli](https://forest.delhi.gov.in/sites/default/files/generic_multiple_files/final_forest_pocket_book.pdf); [Seedling growth under hydrogel doses (Revista Árvore)](http://www.scielo.br/j/rarv/a/YqJtBg6SFLQXWmBqGDHBV6n/?lang=en)

### Himalayan mid-hills (Garhwal) (`himalayan_midhills`)

Rainfall 1200-1800 mm. Plant: Monsoon (July-August), or winter rains in the higher belt. Spacing: hill slope 2.5 m x 3 m. Robots: pahadi.

| Microsite (from the soil map) | Species | Method |
|---|---|---|
| rock outcrop (rock < 0.12) | *Myrica esculenta, Rhododendron arboreum* | seed balls · FYM 1 kg. no contour trenches on slopes this steep; a half-moon eyebrow instead. |
| wet gully (moisture > 0.31) | *Alnus nepalensis* | pit 45 cm deep, 45 cm across · FYM 1 kg · organic mulch · 0.5 L. no contour trenches on slopes this steep; a half-moon eyebrow instead. |
| upper slope (elev_frac > 0.5) | *Cedrus deodara, Quercus leucotrichophora* | pit 45 cm deep, 45 cm across · FYM 1 kg · half-moon · organic mulch · 0.5 L. no contour trenches on slopes this steep; a half-moon eyebrow instead. |
| lower slope (otherwise) | *Quercus leucotrichophora, Myrica esculenta, Rhododendron arboreum* | pit 45 cm deep, 45 cm across · FYM 1 kg · half-moon · organic mulch · 0.5 L. no contour trenches on slopes this steep; a half-moon eyebrow instead. |

Sources: [Dominance and diversity of trees in the lesser Himalaya of Uttarakhand](https://smujo.id/biodiv/article/view/28); [Staggered and continuous contour trenches (CLLMP)](https://www.cllmp.com/wp-content/uploads/2020/06/Structures-for-commons-CCT-SCT-etc..pdf); [ICFRE, Handbook on Soil Water Conservation](https://icfre.gov.in/pdf/books-file/book10.pdf); [Montana State extension, hand planting trees and shrubs](https://p-20.montana.edu/extension/valley/documents/hand%20planting%20trees%20and%20shrubs.pdf)


## What the simulations measured

Short Gazebo runs per region (2-5 spots each, one robot), times in simulated seconds per spot:

| Region | Robot | Time per spot | Boring energy | What it showed |
|---|---|---|---|---|
| `eastern_plain` | Katar | 76 s | 264 kJ | 5-bore 45 cm pits, basin |
| `thar_arid` | Jaal | 80 s | 74 kJ | 3-bore 50 cm pits in sand; 10 L per plant with pre-watering |
| `ignp_canal` | Katar | 96-108 s sodic, 55 s waterlogged | 440-490 kJ sodic | 1.0-1.2 m auger holes; kankar pan cut at 64-66 cm; mounds on canal seepage |
| `hadoti_vertisol` | Jaal | 120 s | 330 kJ | heavy clay costs 4x the energy of sand for the same pit |
| `aravalli_hills` | Pahadi | 158-176 s | 133-156 kJ | contour trench and stone mulch per plant |
| `himalayan_midhills` | Pahadi | 48-68 s | 24 kJ | moist loam; half-moon and mulch; pick where the probe finds stone |

Placement error stayed at 0.4-5.4 cm in every region. The method sets the pace more than the
machine does. A proper pit of three to five bores, a backfill and a basin take 3-6 times as long
as the single 25-30 cm hole the robots made before. Throughput and cost figures based on the
single-hole cycle are optimistic for these methods.

## Judgement calls in the prescriptions

- **No *Prosopis juliflora*.** CSSRI's sodic-land survival figures (about 80 % after 8-10 years)
  are for *P. juliflora*, but it is invasive in Rajasthan (vilayati babool). The sodic rules use
  native salt-tolerant species instead.
- **No basins on black cotton soil.** Water stands on vertisols, so the pits are backfilled for
  drainage instead. The sources on vertisol pits are general horticultural guidance; confirm the
  mix with the state department before field use.
- **No contour trenches on the Himalayan slope.** At 24 degrees, above the 10-25 % range for which
  staggered trenches are recommended, trenches can trigger slips. A half-moon is used instead.
- **Planting windows are enforced by soil moisture, not by date.** Where the pit soil is drier than
  the region's threshold, the robot pre-waters it, as is done in arid-zone practice. The
  simulation has no calendar.
