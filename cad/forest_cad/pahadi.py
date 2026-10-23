"""C · Pahadi: two tracked modules with steered articulation, 4-axis arm, plant handling.

Plant handling (new): one active 50-cell tray sits on the front module behind the arm base, inside
the arm's reach; three more trays wait in a magazine on the rear module and are swapped in when the
active tray is empty. The wrist carries the auger on its +x face and a parallel gripper underneath:
after boring, the arm pitches the wrist level, picks the next plug from the active tray, lowers it
into the hole, releases it and presses the soil. The seed-ball injector sits on top of the wrist.
"""
import cadquery as cq

from .common import Part, Robot, auger, box, cyl, finger, rod, root_trainer_tray, tray_proxy, tube

ACTIVE_TRAY = dict(x=-300, z=776)     # tray top-face centre in base_link, mm
GRIP = dict(x=70, z=-180)             # grasp point in arm_wrist_link (root-ball centre), mm


def track_belt(y, cx=0.0, cz=0.0):
    outer = (cq.Workplane("XZ").center(cx, cz).slot2D(1520, 420).extrude(140, both=True))
    inner = (cq.Workplane("XZ").center(cx, cz).slot2D(1440, 340).extrude(141, both=True))
    belt = outer.cut(inner)
    for k in range(-6, 7):
        x = cx + k * 110
        belt = belt.union(box(28, 280, 22, at=(x, 0, cz - 220))).union(box(28, 280, 22, at=(x, 0, cz + 220)))
    return belt.translate((0, y, 0))


def build() -> Robot:
    R = Robot("pahadi")
    R.add("base_link",
          Part("chassis", box(1300, 720, 400, at=(0, 0, 420), fillet=40), "yellow", hollow=0.04, note="batteries, hydraulics"),
          Part("top_plate", box(1320, 740, 40, at=(0, 0, 630), fillet=40), "graphite", hollow=0.15),
          Part("lamps", box(30, 120, 70, at=(660, 250, 510)).union(box(30, 120, 70, at=(660, -250, 510))), "white"),
          Part("camera_mast", rod((-450, -260, 650), (-450, -260, 1250), 25).union(cyl(90, 40, at=(-450, -260, 1270))), "graphite"),
          Part("track_left", track_belt(500), "rubber", hollow=1),
          Part("track_right", track_belt(-500), "rubber", hollow=1),
          Part("track_frames", box(1500, 60, 40, at=(0, 500, 240)).union(box(1500, 60, 40, at=(0, -500, 240))), "graphite", hollow=0.4),
          Part("tray_deck", box(340, 600, 20, at=(ACTIVE_TRAY["x"], 0, 640)), "alu"),
          Part("active_tray", root_trainer_tray().translate((ACTIVE_TRAY["x"], 0, ACTIVE_TRAY["z"])), "tray", note="active tray, 50 plugs"))
    R.add("artic_link", Part("yaw_block", cyl(200, 300, axis="x"), "graphite", hollow=0.25),
          Part("kingpin", cyl(70, 500, at=(0, 0, 50)), "steel", hollow=0.6))
    magazine = None
    for z in (430, 650, 870):
        shelf = box(320, 620, 16, at=(-560, 0, z))
        magazine = shelf if magazine is None else magazine.union(shelf)
    for x in (-410, -710):
        for y in (-300, 300):
            magazine = magazine.union(box(30, 30, 560, at=(x, y, 650)))
    rear = [
        Part("chassis", box(1300, 720, 400, at=(-850, 0, 170), fillet=40), "graphite", hollow=0.04, note="150 L water tank inside"),
        Part("top_plate", box(1320, 740, 40, at=(-850, 0, 380), fillet=40), "yellow", hollow=0.15),
        Part("track_left", track_belt(500, cx=-850, cz=-250), "rubber", hollow=1),
        Part("track_right", track_belt(-500, cx=-850, cz=-250), "rubber", hollow=1),
        Part("magazine", magazine, "steel", hollow=0.3, note="3 spare trays"),
        Part("hopper", cyl(180, 420, at=(-1320, 0, 610)), "orange", alpha=0.85, hollow=0.05, note="3,000 seed balls"),
        Part("hopper_lid", cyl(190, 30, at=(-1320, 0, 835)), "graphite"),
        Part("seed_hose", rod((-1320, 0, 400), (-300, 280, 900), 18), "hose", hollow=0.3),
        Part("beacon", cyl(50, 90, at=(-1320, 0, 900)), "orange"),
        Part("amendment_bin", box(280, 520, 330, at=(-1060, 0, 565), fillet=12).cut(box(250, 490, 320, at=(-1060, 0, 575))),
             "orange", hollow=1, note="FYM 25 kg, hydrogel 1 kg"),
    ]
    for k, z in enumerate((430, 650, 870)):
        rear.append(Part(f"spare_tray_{k}", root_trainer_tray().translate((-560, 0, z + 8 + 126)), "tray", lod="step"))
        rear.append(Part(f"spare_tray_{k}", tray_proxy().translate((-560, 0, z + 8 + 126)), "tray", lod="mesh", hollow=0.1))
        rear.append(Part(f"spare_canopy_{k}", box(260, 520, 80, at=(-560, 0, z + 8 + 126 + 45)), "leaf", alpha=0.85, hollow=0.05))
    R.add("rear_link", *rear)
    for side in "lr":
        for i in (1, 2, 3):
            for m in "fr":
                R.add(f"{m}{side}{i}_link", Part("road_wheel", cyl(120, 300, axis="y"), "steel", hollow=0.4))
    R.add("arm_base_link", Part("base", cyl(140, 80, at=(0, 0, 40)), "graphite", hollow=0.6))
    R.add("arm_yaw_link", Part("yaw", box(160, 140, 160, at=(0, 0, 70), fillet=12), "yellow", hollow=0.3))
    R.add("arm_upper_link", Part("clevis", cyl(70, 200, axis="y"), "graphite", hollow=0.5),
          Part("beam", box(850, 90, 110, at=(425, 0, 0), fillet=15), "yellow", hollow=0.2),
          Part("cylinder", rod((80, 60, -40), (600, 60, 20), 22), "steel", hollow=0.5))
    R.add("arm_fore_link", Part("clevis", cyl(60, 180, axis="y"), "graphite", hollow=0.5),
          Part("beam", box(850, 80, 90, at=(425, 0, 0), fillet=12), "yellow", hollow=0.2))
    R.add("arm_wrist_link",
          Part("knuckle", cyl(50, 150, axis="y"), "graphite", hollow=0.5),
          Part("tool_block", box(140, 120, 120, at=(70, 0, 0), fillet=8), "graphite", hollow=0.4),
          Part("gripper_body", box(110, 120, 80, at=(GRIP["x"], 0, -95)), "yellow", hollow=0.4),
          Part("seed_injector", tube(22, 17, 200, at=(80, 0, 120), axis="x"), "orange", hollow=1),
          Part("water_nozzle", rod((120, 50, -40), (160, 60, -200), 7), "brass"),
          # feed rail along +x for the auger carriage (0.45 m stroke)
          Part("feed_rail", box(600, 46, 36, at=(140 + 300, 0, 92)).cut(box(560, 48, 14, at=(140 + 300, 0, 92))), "steel",
               hollow=0.6),
          Part("rail_end", box(30, 70, 70, at=(140 + 600, 0, 80)), "graphite"))
    R.add("finger_l_link", Part("finger", finger(110, +1), "steel"))
    R.add("finger_r_link", Part("finger", finger(110, -1), "steel"))
    R.add("auger_feed_link", Part("motor", cyl(70, 150, axis="x"), "graphite", hollow=0.6),
          Part("carriage", box(130, 80, 40, at=(0, 0, 92)).union(box(40, 40, 40, at=(0, 0, 62))), "yellow", hollow=0.5))
    R.add("auger_link", *[Part(p.name, p.shape.rotate((0, 0, 0), (0, 1, 0), -90), p.material) for p in auger(360, 120)])
    return R
