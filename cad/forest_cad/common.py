"""Shared parts for the ForestStark CAD models (CadQuery, millimetres).

Every robot module returns {link_name: [Part, ...]} with geometry expressed in that link's own
frame, exactly as fs_description defines it (x forward, y left, z up, origin at the joint).
build.py turns that into a STEP assembly for inspection and one glTF mesh per link for Gazebo.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import cadquery as cq

# ------------------------------------------------------------------ colours and materials
RAL = {
    "yellow": (0.886, 0.647, 0.078), "graphite": (0.19, 0.21, 0.20), "steel": (0.62, 0.65, 0.66),
    "alu": (0.78, 0.80, 0.81), "rubber": (0.10, 0.11, 0.11), "blue": (0.18, 0.42, 0.56),
    "leaf": (0.31, 0.60, 0.25), "leaf2": (0.24, 0.50, 0.21), "soil": (0.37, 0.26, 0.17),
    "tray": (0.15, 0.16, 0.16), "orange": (0.88, 0.42, 0.11), "glass": (0.66, 0.80, 0.84),
    "solar": (0.12, 0.20, 0.33), "white": (0.93, 0.93, 0.90), "skin": (0.72, 0.50, 0.35),
    "cloth": (0.23, 0.35, 0.55), "shade": (0.21, 0.37, 0.23), "brass": (0.71, 0.55, 0.25),
    "hose": (0.08, 0.08, 0.09),
}
# density kg/m^3 for the BOM mass estimate
DENSITY = {"steel": 7850, "yellow": 7850, "graphite": 7850, "alu": 2700, "rubber": 1150, "blue": 950,
           "tray": 950, "glass": 2500, "solar": 2400, "white": 1200, "orange": 1200, "hose": 1100,
           "brass": 8500, "shade": 300, "leaf": 600, "leaf2": 600, "soil": 1300, "skin": 1000, "cloth": 1000}


def color(name: str, alpha: float = 1.0) -> cq.Color:
    r, g, b = RAL[name]
    return cq.Color(r, g, b, alpha)


@dataclass
class Part:
    name: str
    shape: cq.Workplane | cq.Shape
    material: str = "steel"
    alpha: float = 1.0
    hollow: float = 1.0          # fraction of the solid that is material (sheet-metal bodies)
    note: str = ""
    lod: str = "both"            # both | step (detail for the STEP only) | mesh (light proxy for Gazebo/GLB only)


@dataclass
class Robot:
    name: str
    links: dict = field(default_factory=dict)     # link -> [Part]
    joints: dict = field(default_factory=dict)    # joint -> (parent, child, xyz mm, rpy, type, axis, default)

    def add(self, link: str, *parts: Part):
        self.links.setdefault(link, []).extend(parts)


def solid(s):
    return s.val() if isinstance(s, cq.Workplane) else s


# ------------------------------------------------------------------ primitives (mm, centred unless noted)
def box(x, y, z, at=(0, 0, 0), fillet=0.0):
    w = cq.Workplane("XY").box(x, y, z)
    if fillet:
        w = w.edges("|Z").fillet(min(fillet, min(x, y) / 2.1))
    return w.translate(at)


def cyl(r, h, at=(0, 0, 0), axis="z", centred=True):
    w = cq.Workplane("XY").circle(r).extrude(h)
    if centred:
        w = w.translate((0, 0, -h / 2))
    if axis == "x":
        w = w.rotate((0, 0, 0), (0, 1, 0), 90)
    elif axis == "y":
        w = w.rotate((0, 0, 0), (1, 0, 0), 90)
    return w.translate(at)


def tube(r_out, r_in, h, at=(0, 0, 0), axis="z"):
    w = cq.Workplane("XY").circle(r_out).circle(r_in).extrude(h).translate((0, 0, -h / 2))
    if axis == "x":
        w = w.rotate((0, 0, 0), (0, 1, 0), 90)
    elif axis == "y":
        w = w.rotate((0, 0, 0), (1, 0, 0), 90)
    return w.translate(at)


def cone(r_bottom, r_top, h, at=(0, 0, 0)):
    """Frustum standing on its base at `at`."""
    return cq.Workplane("XY").add(cq.Solid.makeCone(r_bottom, r_top, h)).translate(at)


def sphere(r, at=(0, 0, 0), scale=(1, 1, 1)):
    s = cq.Workplane("XY").sphere(r)
    if scale != (1, 1, 1):
        m = cq.Matrix([[scale[0], 0, 0, 0], [0, scale[1], 0, 0], [0, 0, scale[2], 0]])
        s = cq.Workplane("XY").add(solid(s).transformGeometry(m))
    return s.translate(at)


def rod(p0, p1, r):
    """Cylinder between two points."""
    v = cq.Vector(*p1) - cq.Vector(*p0)
    return cq.Workplane("XY").add(cq.Solid.makeCylinder(r, v.Length, cq.Vector(*p0), v.normalized()))


def rot(w, axis, deg, origin=(0, 0, 0)):
    a = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[axis]
    return w.rotate(origin, (origin[0] + a[0], origin[1] + a[1], origin[2] + a[2]), deg)


# ------------------------------------------------------------------ drivetrain
def tyre(r, w, lugs=0, lug_h=None):
    """Tyre with its spin axis on y, centre at the origin."""
    t = 0.24 * r
    body = cq.Workplane("XZ").circle(r).circle(r - t).extrude(w / 2, both=True)
    body = body.edges().fillet(min(t, w) * 0.18)
    if lugs:
        lh = lug_h or 0.05 * r
        for i in range(lugs):
            a = 360 * i / lugs
            for side in (-1, 1):
                lug = box(0.07 * r, w * 0.46, lh, at=(0, side * w * 0.25, r + lh / 2 - 2))
                lug = rot(lug, "x", side * 18)
                body = body.union(rot(lug, "y", a))
    return body


def rim(r, w, spokes=6):
    disc = cq.Workplane("XZ").circle(r).circle(r * 0.30).extrude(w * 0.08, both=True)
    hub = cq.Workplane("XZ").circle(r * 0.30).extrude(w * 0.30, both=True)
    barrel = cq.Workplane("XZ").circle(r).circle(r * 0.93).extrude(w * 0.46, both=True)
    out = disc.union(hub).union(barrel)
    for i in range(spokes):
        hole = cyl(r * 0.13, w, at=(r * 0.62, 0, 0), axis="y")
        out = out.cut(rot(hole, "y", 360 * i / spokes))
    for i in range(6):
        out = out.union(rot(cyl(r * 0.035, w * 0.36, at=(r * 0.2, 0, 0), axis="y"), "y", 60 * i))
    return out


def wheel_parts(r, w, rim_mat="yellow", lugs=0):
    return [Part("tyre", tyre(r, w, lugs), "rubber", hollow=0.35),
            Part("rim", rim(r * 0.72, w * 0.95), rim_mat, hollow=0.25)]


# ------------------------------------------------------------------ plant handling
PLUG_H = 120        # root-trainer plug length, mm
PLUG_R_TOP = 21
PLUG_R_BOT = 11


def plug(with_canopy=True):
    """One root-trainer plug, origin at the bottom of the root ball."""
    soil = cone(PLUG_R_BOT, PLUG_R_TOP, PLUG_H)
    parts = [Part("rootball", soil, "soil")]
    if with_canopy:
        stem = cyl(3.5, 210, at=(0, 0, PLUG_H + 105))
        crown = sphere(48, at=(0, 0, PLUG_H + 220), scale=(1, 1, 1.25))
        leaf = sphere(30, at=(28, 10, PLUG_H + 150), scale=(1.4, 0.8, 0.6))
        parts += [Part("stem", stem, "soil"), Part("crown", crown, "leaf"), Part("leaf", leaf, "leaf2")]
    return parts


def root_trainer_tray(nx=5, ny=10, pitch=52):
    """50-cell root-trainer tray, origin at the centre of its top face. Long side on y."""
    lx, ly, h = nx * pitch + 10, ny * pitch + 10, PLUG_H + 6
    body = box(lx, ly, h, at=(0, 0, -h / 2), fillet=6)
    cells = None
    for i in range(nx):
        for j in range(ny):
            x, y = (i - (nx - 1) / 2) * pitch, (j - (ny - 1) / 2) * pitch
            c = cone(PLUG_R_BOT + 1, PLUG_R_TOP + 2, PLUG_H + 2, at=(x, y, -PLUG_H - 1))
            cells = c if cells is None else cells.union(c)
    body = body.cut(cells)
    lip = box(lx + 12, ly + 12, 4, at=(0, 0, -2)).cut(box(lx - 4, ly - 4, 6, at=(0, 0, -2)))
    return body.union(lip)


def tray_proxy(nx=5, ny=10, pitch=52):
    """Light stand-in for a filled tray in the simulation meshes: same envelope, no cells."""
    lx, ly, h = nx * pitch + 10, ny * pitch + 10, PLUG_H + 6
    return box(lx, ly, h, at=(0, 0, -h / 2), fillet=6)


def tray_cells(nx=5, ny=10, pitch=52):
    return [((i - (nx - 1) / 2) * pitch, (j - (ny - 1) / 2) * pitch) for i in range(nx) for j in range(ny)]


def auger(length, dia, pitch=None):
    """Earth auger, origin at the top of the shaft, extending down -z. Real helical flight."""
    pitch = pitch or dia * 0.9
    shaft = cyl(dia * 0.13, length, at=(0, 0, -length / 2))
    flight_len = length * 0.72
    helix = cq.Wire.makeHelix(pitch, flight_len, dia * 0.30)
    profile = cq.Workplane("XZ").center(dia * 0.30, 0).rect(dia * 0.40, 4)
    flight = profile.sweep(cq.Workplane().add(helix), isFrenet=True)
    flight = flight.translate((0, 0, -length + dia * 0.25))
    tip = cq.Workplane("XY").add(cq.Solid.makeCone(dia * 0.13, 1, dia * 0.5, cq.Vector(0, 0, -length), cq.Vector(0, 0, -1)))
    teeth = box(dia * 0.9, 6, 14, at=(0, 0, -length + 6))
    return [Part("auger_shaft", shaft, "steel"), Part("auger_flight", flight, "steel"),
            Part("auger_tip", tip.union(teeth), "graphite")]


def duckbill_jaw(side: int, length=320, r_top=46, r_tip=10, wall=3):
    """Half of a duckbill planting jaw. Origin on the hinge axis (x), at the top; side +1 = +y half."""
    outer = cq.Solid.makeCone(r_top, r_tip, length, cq.Vector(0, 0, 0), cq.Vector(0, 0, -1))
    inner = cq.Solid.makeCone(r_top - wall, r_tip - wall, length, cq.Vector(0, 0, 0), cq.Vector(0, 0, -1))
    shell = cq.Workplane("XY").add(outer).cut(cq.Workplane("XY").add(inner))
    keep = box(2 * r_top + 10, r_top + 5, length + 10, at=(0, side * (r_top + 5) / 2, -length / 2))
    half = shell.intersect(keep)
    lug = box(30, 8, 30, at=(0, side * (r_top + 2), -6))
    # move so that the hinge axis (at y = side*r_top, z = 0) is the origin
    return half.union(lug).translate((0, -side * r_top, 0))


def feed_mast(length, x, y=0.0, z_top=0.0, w=70, d=50):
    """Fixed guide rail for a feed carriage: a steel channel hanging from z_top down `length` at (x, y),
    with a mounting bracket back to the tool axis and a foot at the bottom. Everything that feeds down
    (auger motor, duckbill, tamp ring) runs on one of these, so nothing hangs in the air."""
    rail = box(w, d, length, at=(x, y, z_top - length / 2))
    slot = box(w - 24, d + 2, length - 40, at=(x, y, z_top - length / 2))
    bracket = rod((0, 0, z_top + 10), (x, y, z_top + 10), 24)
    foot = box(w + 50, d + 50, 14, at=(x, y, z_top - length - 7))
    return rail.cut(slot).union(bracket).union(foot)


def feed_carriage(dx, dy=0.0, h=170, w=95, d=80, shift=0.0):
    """Carriage block that rides a mast at (dx, dy), with an arm back to the tool at the link origin.
    shift: sideways offset along the rail face, so two carriages can share one rail."""
    L = math.hypot(dx, dy) or 1.0
    px, py = -dy / L * shift, dx / L * shift
    block = box(w, d, h, at=(dx + px, dy + py, 0))
    arm = rod((0, 0, 0), (dx + px, dy + py, 0), 22)
    return block.union(arm)


def tamp_ring(r_in=62, r_out=102, h=26, rods=3, rod_len=230):
    ring = tube(r_out, r_in, h, at=(0, 0, -h / 2))
    out = ring
    for i in range(rods):
        a = 2 * math.pi * i / rods + math.pi / 6
        out = out.union(cyl(7, rod_len, at=((r_in + r_out) / 2 * math.cos(a), (r_in + r_out) / 2 * math.sin(a), rod_len / 2)))
    return out


def finger(length=110, side=1):
    """Gripper finger, origin at its slide, closing along -side*y."""
    f = box(18, 12, length, at=(0, 0, -length / 2))
    pad = box(26, 6, 50, at=(0, -side * 7, -length + 25))
    return f.union(pad)


def person(height=1700):
    s = height / 1700
    legs = box(140 * s, 320 * s, 820 * s, at=(0, 0, 410 * s))
    torso = cyl(170 * s, 620 * s, at=(0, 0, 1130 * s))
    arms = box(100 * s, 500 * s, 560 * s, at=(0, 0, 1130 * s))
    head = sphere(115 * s, at=(0, 0, 1580 * s))
    return [Part("legs", legs, "graphite", hollow=0.3), Part("torso", torso, "cloth", hollow=0.3),
            Part("arms", arms, "cloth", hollow=0.2), Part("head", head, "skin", hollow=0.5)]
