"""A · Katar: tractor, planting trailer, boom sled and carriage, and the planting head.

Planting head (new): a turret under the boom carriage carries the auger and a duckbill planting
jaw 180 deg apart. A 12-cup carousel on the carriage, which the loader keeps filled from the rack
trays, drops one plug down a chute into the closed duckbill while the auger bores. The turret then
swings the duckbill over the hole; it descends, opens, a tamp ring firms the soil, water is dosed.
"""
import math

from .common import (Part, Robot, auger, box, cyl, duckbill_jaw, person, rod, rot, root_trainer_tray, tamp_ring, tray_proxy, tube,
                     wheel_parts)

TURRET_R = 200          # station offset from the turret axis, mm
CAROUSEL = dict(x=-350, z=200, r=150, cups=12)


def build() -> Robot:
    R = Robot("katar")
    # ---------------------------------------------------------------- tractor (base_link: rear axle centre)
    R.add("base_link",
          Part("chassis", box(2500, 600, 300, at=(1050, 0, -80), fillet=20), "graphite", hollow=0.12),
          Part("hood", box(1550, 780, 700, at=(1880, 0, 400), fillet=60), "yellow", hollow=0.03),
          Part("grille", box(60, 680, 580, at=(2670, 0, 360)), "graphite", hollow=0.3),
          Part("lamps", box(40, 140, 90, at=(2700, 250, 580)).union(box(40, 140, 90, at=(2700, -250, 580))), "white"),
          Part("exhaust", cyl(50, 800, at=(2250, 260, 1130)), "graphite", hollow=0.1),
          Part("fenders", box(1150, 500, 60, at=(0, 800, 780), fillet=30).union(box(1150, 500, 60, at=(0, -800, 780), fillet=30)), "yellow", hollow=0.05),
          Part("cab_floor", box(1250, 1200, 80, at=(0, 0, 400)), "graphite", hollow=0.06),
          Part("roof", box(1350, 1300, 80, at=(20, 0, 1920), fillet=40), "yellow", hollow=0.2),
          Part("rops", _rops(), "graphite", hollow=0.2),
          Part("windscreen", box(20, 1080, 1000, at=(600, 0, 1360)), "glass", alpha=0.35),
          Part("seat", box(420, 460, 100, at=(-150, 0, 780)).union(box(80, 460, 500, at=(-360, 0, 1060))), "graphite", hollow=0.3),
          Part("steering", tube(160, 140, 25, at=(340, 0, 1230)).rotate((340, 0, 1230), (340, 1, 1230), -55)
               .union(rod((420, 0, 700), (340, 0, 1230), 25)), "graphite"),
          Part("beacon", cyl(70, 120, at=(300, 420, 2020)), "orange"),
          Part("drawbar", box(500, 140, 120, at=(-850, 0, 0)), "steel"),
          Part("gnss_antenna", cyl(90, 30, at=(100, 0, 2000)), "graphite"))
    R.add("driver_link", *person())
    for side, y in (("left", 800), ("right", -800)):
        R.add(f"rear_{side}_wheel_link", *wheel_parts(720, 400, "yellow", lugs=16))
        R.add(f"front_{side}_steer_link", Part("knuckle", box(120, 90, 260, at=(0, -math.copysign(60, y), 0)), "graphite"))
        R.add(f"front_{side}_wheel_link", *wheel_parts(420, 240, "yellow"))

    # ---------------------------------------------------------------- trailer (origin at the hitch)
    trailer = [
        Part("drawbar", box(1000, 100, 100, at=(-450, 0, 0)), "steel", hollow=0.2),
        Part("deck", box(3700, 2000, 140, at=(-2700, 0, 360)), "graphite", hollow=0.05),
        Part("side_rails", box(3700, 40, 120, at=(-2700, 980, 490)).union(box(3700, 40, 120, at=(-2700, -980, 490))), "yellow", hollow=0.3),
        Part("axles", box(80, 1860, 80, at=(-3100, 0, -80)).union(box(80, 1860, 80, at=(-3950, 0, -80))), "steel", hollow=0.3),
        Part("tank", cyl(550, 1200, at=(-1600, 0, 1000), axis="x"), "blue", hollow=0.02, note="1,000 L water"),
        Part("tank_filler", cyl(120, 80, at=(-1600, 0, 1570)), "white"),
        Part("tank_saddles", box(80, 900, 300, at=(-1150, 0, 580)).union(box(80, 900, 300, at=(-2050, 0, 580))), "graphite", hollow=0.3),
        Part("boom_rail", box(2800, 80, 80, at=(-1600, 1000, 450)), "steel", hollow=0.3),
        Part("shade_net", box(2200, 2000, 20, at=(-3400, 0, 2300)), "shade", alpha=0.6),
        Part("pump", box(300, 260, 240, at=(-2300, -700, 560), fillet=20), "blue", hollow=0.3, note="0.5 L/s dosing pump"),
    ]
    TRAY, PROXY = root_trainer_tray(), tray_proxy()
    racks = None
    for s in (1, -1):
        y = s * 680
        for x in (-2330, -4470):
            for dy in (-220, 220):
                p = box(50, 50, 1860, at=(x, y + dy, 1360))
                racks = p if racks is None else racks.union(p)
        for z in (480, 1000, 1520):
            racks = racks.union(box(2140, 460, 30, at=(-3400, y, z - 30)))
            for k in range(4):
                tx, top = -2610 - k * 530, z + 111            # tray centre; shelf top is at z - 15
                side = 'L' if s > 0 else 'R'
                trailer.append(Part(f"tray_{side}{z}_{k}", rot(TRAY, "z", 90).translate((tx, y, top)), "tray", lod="step"))
                trailer.append(Part(f"tray_{side}{z}_{k}", rot(PROXY, "z", 90).translate((tx, y, top)), "tray", lod="mesh", hollow=0.1))
                trailer.append(Part(f"canopy_{side}{z}_{k}", box(520, 260, 230, at=(tx, y, top + 115)),
                                    "leaf", alpha=0.85, hollow=0.05, note="50 seedlings"))
    trailer.append(Part("racks", racks, "steel", hollow=0.12))
    shade_posts = None
    for x in (-2350, -4450):
        for y in (-980, 980):
            p = box(40, 40, 1800, at=(x, y, 1400))
            shade_posts = p if shade_posts is None else shade_posts.union(p)
    trailer.append(Part("shade_posts", shade_posts, "steel", hollow=0.25))
    R.add("trailer_link", *trailer)
    R.add("loader_link", *person())
    for t in ("fl", "fr", "rl", "rr"):
        R.add(f"trailer_{t}_wheel_link", *wheel_parts(420, 260, "graphite"))

    # ---------------------------------------------------------------- boom
    R.add("sled_link",
          Part("post", box(120, 120, 1150, at=(0, 0, 550)), "yellow", hollow=0.2),
          Part("beam", box(120, 2750, 120, at=(0, 1350, 1070)), "yellow", hollow=0.2),
          Part("brace", rod((0, 0, 300), (0, 900, 1040), 30), "steel", hollow=0.3),
          Part("rail_shoes", box(260, 140, 90, at=(0, 0, 0)), "graphite"),
          Part("hose_track", box(80, 2500, 60, at=(0, 1300, 1170)), "hose", hollow=0.3))
    cups = None
    for i in range(CAROUSEL["cups"]):
        a = 2 * math.pi * i / CAROUSEL["cups"]
        c = tube(30, 26, 140, at=(CAROUSEL["r"] * math.cos(a), CAROUSEL["r"] * math.sin(a), 70))
        cups = c if cups is None else cups.union(c)
    R.add("carriage_link",
          Part("carriage", box(320, 320, 300, fillet=20), "yellow", hollow=0.3),
          Part("motor", cyl(90, 200, at=(0, 0, 250)), "graphite", hollow=0.5),
          Part("turret_drive", cyl(110, 60, at=(0, 0, -150)), "graphite", hollow=0.5),
          Part("carousel_arm", box(400, 120, 40, at=(-250, 0, 150)), "yellow", hollow=0.3),
          Part("chute", tube(34, 30, 330, at=(TURRET_R * -1, 0, 30)), "alu", hollow=1, note="plug drop chute"),
          Part("carousel_motor", cyl(45, 90, at=(CAROUSEL["x"], 0, CAROUSEL["z"] - 70)), "graphite"))
    R.add("carousel_link",
          Part("plate", cyl(CAROUSEL["r"] + 60, 15, at=(0, 0, 0)).cut(cyl(31, 30, at=(CAROUSEL["r"], 0, 0))), "alu"),
          Part("cups", cups, "orange", hollow=1, note="12 plug cups"),
          Part("hub", cyl(40, 60, at=(0, 0, 30)), "steel"))
    R.add("turret_link",
          Part("disc", cyl(280, 40), "yellow", hollow=1),
          Part("auger_station", cyl(90, 120, at=(TURRET_R, 0, -40)), "graphite", hollow=0.4),
          Part("duckbill_station", cyl(80, 120, at=(-TURRET_R, 0, -40)), "graphite", hollow=0.4))
    R.add("auger_feed_link", Part("hydraulic_motor", cyl(75, 160), "graphite", hollow=0.6),
          Part("feed_cylinder", cyl(30, 600, at=(60, 0, 300)), "steel", hollow=0.5))
    R.add("auger_link", *auger(950, 150))
    _head(R, jaw_len=320)
    return R


def _head(R: Robot, jaw_len: int):
    """Duckbill feed, jaws, tamp ring (shared by Katar and Jaal)."""
    R.add("duckbill_feed_link",
          Part("housing", tube(58, 48, 160, at=(0, 0, -40)), "graphite", hollow=1),
          Part("hinge_brackets", box(40, 120, 50, at=(0, 0, -105)), "steel"),
          Part("jaw_actuators", rod((30, 40, -40), (30, 75, -160), 9).union(rod((30, -40, -40), (30, -75, -160), 9)), "steel"),
          Part("water_nozzle", rod((-40, 0, -40), (-70, 0, -300), 8), "brass"),
          Part("feed_rod", cyl(22, 700, at=(0, 0, 330)), "steel", hollow=0.6))
    R.add("jaw_l_link", Part("jaw", duckbill_jaw(+1, length=jaw_len), "steel"))
    R.add("jaw_r_link", Part("jaw", duckbill_jaw(-1, length=jaw_len), "steel"))
    R.add("tamp_link", Part("ring", tamp_ring(), "graphite"))


def _rops():
    out = None
    for x in (-550, 600):
        for y in (-570, 570):
            p = box(60, 60, 1460, at=(x, y, 1170))
            out = p if out is None else out.union(p)
    return out
