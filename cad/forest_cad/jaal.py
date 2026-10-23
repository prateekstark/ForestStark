"""B · Jaal: swerve-drive platform, two rotary sapling drums, Cartesian picker, rear planting head.

Plant handling (new): each drum has 4 tiers of 45 pockets (180 plugs). The drum turns the next full
pocket to the pick window on the left side. A picker on a rail under the solar roof (x travel,
z column, y reach, two fingers) grips the root ball, lifts it out and carries it back to a funnel
that feeds the duckbill on the rear shuttle. While the auger bores, the next plug is already on
its way; the shuttle then swaps the duckbill over the hole.
"""
import math

from .common import Part, Robot, auger, box, cone, cyl, rod, tube, wheel_parts
from .katar import _head

DRUM = dict(tiers=4, pockets=45, r=410, z0=100, dz=400, disc_r=480)
RAIL = dict(y=620, z=2100)
FUNNEL_Y = 530
SHUTTLE_DUCKBILL = 280          # duckbill station ahead of the auger on the shuttle, mm


def h_finger(side):
    """Finger pointing toward the drum (-y), closing along -side*x. Origin at its slide on the palm."""
    return box(14, 90, 36, at=(0, -45, 0)).union(box(6, 50, 30, at=(-side * 9, -70, 0)))


def drum_parts():
    d = DRUM
    hub = cyl(340, 60, at=(0, 0, 30))
    shaft = cyl(50, 1720, at=(0, 0, 860))
    discs = None
    pockets = None
    for k in range(d["tiers"]):
        z = d["z0"] + d["dz"] * k
        disc = tube(d["disc_r"], 60, 25, at=(0, 0, z))
        discs = disc if discs is None else discs.union(disc)
        for i in range(d["pockets"]):
            a = 2 * math.pi * i / d["pockets"]
            x, y = d["r"] * math.cos(a), d["r"] * math.sin(a)
            p = tube(27, 23, 110, at=(x, y, z + 12 + 55))
            slot = box(30, 20, 120, at=(x + 22 * math.cos(a), y + 22 * math.sin(a), z + 12 + 55))
            slot = slot.rotate((x + 22 * math.cos(a), y + 22 * math.sin(a), 0),
                               (x + 22 * math.cos(a), y + 22 * math.sin(a), 1), math.degrees(a))
            p = p.cut(slot)
            pockets = p if pockets is None else pockets.union(p)
    cap = cyl(120, 50, at=(0, 0, 1740))
    return [Part("hub", hub, "graphite", hollow=0.4), Part("shaft", shaft, "steel", hollow=0.4),
            Part("tiers", discs, "tray", hollow=0.5, note="moulded HDPE"), Part("pockets", pockets, "orange", hollow=1, note="4 x 45 pockets"),
            Part("cap", cap, "yellow", hollow=0.4)]


def build() -> Robot:
    R = Robot("jaal")
    posts = None
    for x in (1120, -1120):
        for y in (600, -600):
            p = box(50, 50, 1900, at=(x, y, 1450))
            posts = p if posts is None else posts.union(p)
    gantry = box(80, 80, 800, at=(-1380, 450, 870)).union(box(80, 80, 800, at=(-1380, -450, 870))) \
        .union(box(100, 1000, 100, at=(-1380, 0, 1300)))
    rail_supports = box(60, 60, 300, at=(900, RAIL["y"], 2240)).union(box(60, 60, 300, at=(-1300, RAIL["y"], 2240)))
    # funnel under the gripper's grasp point with the reach retracted (y 0.72 - 0.19 m), feeding the duckbill hose
    funnel = cone(26, 70, 120, at=(-1200, FUNNEL_Y, 1100)).cut(cone(22, 66, 121, at=(-1200, FUNNEL_Y, 1100)))
    feed_hose = rod((-1200, FUNNEL_Y, 1100), (-1370, 120, 1000), 30)
    R.add("base_link",
          Part("body", box(2800, 1300, 450, at=(0, 0, 220), fillet=80), "yellow", hollow=0.02, note="30 kWh LFP inside"),
          Part("skirt", box(2840, 1340, 100, at=(0, 0, 40), fillet=60), "graphite", hollow=0.04),
          Part("deck", box(2700, 1240, 50, at=(0, 0, 470), fillet=60), "graphite", hollow=0.08),
          Part("bumpers", box(100, 1200, 200, at=(1450, 0, 120)).union(box(100, 1200, 200, at=(-1450, 0, 120))), "graphite", hollow=0.1),
          Part("side_lidars", box(20, 1000, 50, at=(1505, 0, 170)).union(box(20, 1000, 50, at=(-1505, 0, 170))), "graphite"),
          Part("posts", posts, "graphite", hollow=0.25),
          Part("roof_frame", box(2540, 1400, 30, at=(0, 0, 2390)), "graphite", hollow=0.08, note="tube frame"),
          Part("solar", box(2500, 1360, 40, at=(0, 0, 2425)), "solar", hollow=0.12),
          Part("mast", cyl(30, 500, at=(1150, 0, 2670)), "graphite", hollow=0.4),
          Part("lidar_3d", cyl(85, 110, at=(1150, 0, 2980)), "graphite"),
          Part("beacon", cyl(60, 100, at=(-1000, -500, 2500)), "orange"),
          Part("gantry", gantry, "graphite", hollow=0.25),
          Part("picker_rail", box(2400, 60, 60, at=(-200, RAIL["y"], RAIL["z"])), "steel", hollow=0.3),
          Part("rail_supports", rail_supports, "graphite", hollow=0.3),
          Part("funnel", funnel, "alu"), Part("feed_hose", feed_hose, "hose", hollow=0.2),
          Part("water_tank", box(600, 900, 300, at=(-600, 0, 0), fillet=40), "blue", hollow=0.05, note="250 L, under the deck"),
          Part("front_lidar", cyl(50, 70, at=(1520, 0, -80)), "graphite"))
    for tag in ("fl", "fr", "rl", "rr"):
        R.add(f"steer_{tag}_link", Part("fork", box(160, 120, 300), "graphite", hollow=0.4),
              Part("steer_motor", cyl(55, 120, at=(0, 0, 200)), "graphite"))
        R.add(f"wheel_{tag}_link", *wheel_parts(380, 220, "graphite", lugs=10))
    R.add("drum_f_link", *drum_parts())
    R.add("drum_r_link", *drum_parts())
    R.add("picker_x_link",
          Part("carriage", box(220, 140, 110, fillet=10), "yellow", hollow=0.4),
          Part("column", box(60, 60, 1560, at=(0, 0, -830)), "alu", hollow=0.3, note="z axis, belt driven"))
    R.add("picker_z_link", Part("head", box(110, 120, 90, fillet=8), "yellow", hollow=0.4),
          Part("reach_guide", box(50, 110, 40, at=(0, -60, -10)), "steel"))
    R.add("picker_y_link", Part("arm", box(40, 120, 40, at=(0, -60, 0)), "alu", hollow=0.4),
          Part("palm", box(90, 30, 70, at=(0, -125, 0)), "graphite"))
    R.add("finger_a_link", Part("finger", h_finger(+1), "steel"))
    R.add("finger_b_link", Part("finger", h_finger(-1), "steel"))
    R.add("shuttle_link",
          Part("plate", box(620, 220, 30, at=(SHUTTLE_DUCKBILL / 2, 0, 20)), "yellow", hollow=1),
          Part("auger_station", cyl(70, 80, at=(0, 0, -20)), "graphite", hollow=0.4),
          Part("duckbill_station", cyl(70, 80, at=(SHUTTLE_DUCKBILL, 0, -20)), "graphite", hollow=0.4))
    R.add("auger_feed_link", Part("motor", cyl(75, 160), "graphite", hollow=0.6))
    R.add("auger_link", *auger(720, 120))
    _head(R, jaw_len=300)
    return R
