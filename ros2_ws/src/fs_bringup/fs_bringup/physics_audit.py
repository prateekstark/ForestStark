"""Physics audit of the three robots as Gazebo sees them (URDF with the CAD inertials).

    ros2 run fs_bringup physics_audit                 # prints the report
    ros2 run fs_bringup physics_audit --out docs/physics_audit.md

Checks, per robot:
  inertials    every link: positive mass, positive-definite inertia satisfying the triangle inequality
  mass and CoM total running mass, centre of mass, static load on every wheel or road wheel
  stability    static tip-over angles from the support polygon, against the slopes the robot works on
  sinkage      static Bekker sinkage of the most loaded contact in each soil, against 15 % of its radius
  gradeability straight-up slope the drive torque can hold and the soil's shear can carry (40 % slip)
  tool loads   auger thrust against the machine's weight; Pahadi's arm joint torques at full reach
Each check is PASS, WARN (works, with a caveat) or FAIL.
"""
from __future__ import annotations

import argparse
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from fs_autonomy import terramechanics as tm
from fs_bringup import sim

G = 9.81
WHEELS = {   # contact links, driven or not, and the slope each robot must hold (deg): (working, lateral)
    # a road verge rises ~6 deg at most along the way; the rig must not tip on a 15 deg embankment
    "katar": dict(contacts=r"wheel", driven=("rear_left_wheel_link", "rear_right_wheel_link"), slope=(6.0, 15.0),
                  thrust=8000.0, soils=("sandy_loam", "clayey_soil", "heavy_clay"), trailer=("trailer_link", "hitch_joint")),
    "jaal": dict(contacts=r"wheel_", driven=None, slope=(5.0, 10.0), thrust=None,
                 soils=("dry_sand", "sandy_loam", "clayey_soil", "heavy_clay")),
    # Pahadi drives along contours; it climbs only between rows (diagonally, ~12 deg on a 25 deg hill)
    "pahadi": dict(contacts=r"^[fr][lr][123]_link$", driven=None, slope=(12.0, 25.0), thrust=None, track_l=0.45,
                   soils=("lateritic_gravel", "sandy_loam", "clayey_soil")),
}


@dataclass
class Check:
    robot: str
    name: str
    status: str
    detail: str


WET = 0.32   # volumetric moisture of a loam or clay a day after monsoon rain


def soils():
    from fs_gazebo.sites import SOILS
    return {s[0]: dict(name=s[0], n=s[1], kc=s[2], kphi=s[3], c=s[4], phi=s[5], K=s[6], eb=s[7]) for s in SOILS}


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


class Model:
    """URDF links, inertials and forward kinematics at given joint values (zero by default)."""

    def __init__(self, xml: str, q: dict | None = None):
        self.root = ET.fromstring(xml)
        self.q = q or {}
        self.joints = {}
        for j in self.root.iter("joint"):
            o = j.find("origin")
            a = j.find("axis")
            lim = j.find("limit")
            self.joints[j.find("child").get("link")] = dict(
                name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"),
                xyz=np.array([float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]),
                rpy=[float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()],
                axis=np.array([float(v) for v in (a.get("xyz", "1 0 0") if a is not None else "1 0 0").split()]),
                effort=float(lim.get("effort", "0")) if lim is not None else 0.0)
        self.links = {}
        for l in self.root.iter("link"):
            i = l.find("inertial")
            if i is None:
                continue
            o = i.find("origin")
            c = np.array([float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()])
            t = i.find("inertia")
            I = np.array([[float(t.get("ixx")), float(t.get("ixy", 0)), float(t.get("ixz", 0))],
                          [float(t.get("ixy", 0)), float(t.get("iyy")), float(t.get("iyz", 0))],
                          [float(t.get("ixz", 0)), float(t.get("iyz", 0)), float(t.get("izz"))]])
            self.links[l.get("name")] = dict(m=float(i.find("mass").get("value")), c=c, I=I, el=l)
        self._T = {}

    def T(self, link):
        if link in self._T:
            return self._T[link]
        j = self.joints.get(link)
        if j is None:
            M = np.eye(4)
        else:
            M = self.T(j["parent"]).copy()
            A = np.eye(4)
            A[:3, :3] = rpy_matrix(*j["rpy"])
            A[:3, 3] = j["xyz"]
            M = M @ A
            q = self.q.get(j["name"], 0.0)
            J = np.eye(4)
            ax = j["axis"] / (np.linalg.norm(j["axis"]) or 1)
            if j["type"] in ("revolute", "continuous"):
                K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
                J[:3, :3] = np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K
            elif j["type"] == "prismatic":
                J[:3, 3] = ax * q
            M = M @ J
        self._T[link] = M
        return M

    def com(self, links=None):
        names = links or self.links
        m = sum(self.links[n]["m"] for n in names)
        c = sum(self.links[n]["m"] * (self.T(n)[:3, :3] @ self.links[n]["c"] + self.T(n)[:3, 3]) for n in names) / m
        return m, c

    def radius(self, link):
        el = self.links[link]["el"] if link in self.links else None
        for g in (el.iter("collision") if el is not None else []):
            for shape in g.iter():
                if shape.tag in ("cylinder", "sphere"):
                    return float(shape.get("radius")), float(shape.get("length", 2 * float(shape.get("radius"))))
        return None, None


def least_norm_loads(points, weight, com):
    """Vertical contact loads that balance the weight with the smallest norm (statically indeterminate)."""
    P = np.asarray(points)
    A = np.vstack([np.ones(len(P)), P[:, 1], P[:, 0]])          # sum F, sum y F, sum x F
    b = np.array([weight, weight * com[1], weight * com[0]])
    F, *_ = np.linalg.lstsq(A, b, rcond=None)
    return F


def tip_angles(points, com, h):
    """Smallest tilt (deg) that takes the CoM over an edge of the support polygon, for each edge."""
    from fs_autonomy.geometry import convex_hull
    hull = np.asarray(convex_hull([tuple(p[:2]) for p in points]))
    out = []
    for a, b in zip(hull, np.roll(hull, -1, axis=0)):
        e = b - a
        n = np.array([e[1], -e[0]]) / np.linalg.norm(e)
        d = abs(np.dot(com[:2] - a, n))
        out.append((math.degrees(math.atan2(d, h)), e))
    return out


def audit(kind: str) -> list:
    cfg = WHEELS[kind]
    xml = sim.robot_description(kind, kind, Path("/tmp/none.soilmap"), lidar=False)
    m = Model(xml)
    out = []
    # ------------------------------------------------------------- inertials
    bad = []
    for n, l in m.links.items():
        w = np.linalg.eigvalsh(l["I"])
        tri = w.sum() - 2 * w.max() >= -1e-6 * max(w.max(), 1e-9)
        if l["m"] <= 0 or w.min() <= 0 or not tri:
            bad.append(n)
    out.append(Check(kind, "inertials", "FAIL" if bad else "PASS",
                     f"{len(m.links)} links; " + (f"invalid: {', '.join(bad)}" if bad else "all positive-definite, triangle inequality holds")))
    # ------------------------------------------------------------- mass, CoM, loads
    import re
    contacts = [n for n in m.links if re.search(cfg["contacts"], n)]
    lift = sim.BASE_HEIGHT[kind]
    pts = []
    for n in contacts:
        r, w = m.radius(n)
        p = m.T(n)[:3, 3]
        pts.append((p[0], p[1], p[2] - (r or 0.2), r or 0.2, w or 0.25))
    ground_z = min(p[2] for p in pts)
    mass, c = m.com()
    h = c[2] - ground_z
    if cfg.get("trailer"):
        # articulated: the trailer stands on its wheels and the hitch; the hitch load goes onto the tractor
        tl, hj = cfg["trailer"]
        on_trailer = [n for n in m.links if _below(m, n, tl)]
        mt, ct = m.com(on_trailer)
        hitch = m.T(tl)[:3, 3]
        tw = [i for i, n in enumerate(contacts) if n in on_trailer]
        fw = [i for i in range(len(contacts)) if i not in tw]
        Ft = least_norm_loads([(pts[i][0], pts[i][1]) for i in tw] + [(hitch[0], hitch[1])], mt * G, ct)
        h_load = Ft[-1]
        tractor = [n for n in m.links if n not in on_trailer]
        mtr, ctr = m.com(tractor)
        W = mtr * G + h_load
        cc = (mtr * G * ctr + h_load * hitch) / W
        Ff = least_norm_loads([(pts[i][0], pts[i][1]) for i in fw], W, cc)
        loads = np.zeros(len(contacts))
        loads[tw], loads[fw] = Ft[:-1], Ff
    else:
        loads = least_norm_loads([(p[0], p[1]) for p in pts], mass * G, c)
    out.append(Check(kind, "mass and CoM", "PASS",
                     f"{mass:,.0f} kg running order; CoM {c[0]:+.2f} m, {c[1]:+.2f} m, {h:.2f} m above ground; "
                     f"contact loads {loads.min() / G:,.0f}-{loads.max() / G:,.0f} kg"))
    if loads.min() < 0:
        out.append(Check(kind, "load distribution", "FAIL", "a contact would have to pull the ground"))
    # ------------------------------------------------------------- stability
    working, lateral = cfg["slope"]
    angles = tip_angles([(p[0], p[1]) for p in pts], c, h)
    lat = min(a for a, e in angles if abs(e[0]) > abs(e[1]))
    lon = min(a for a, e in angles if abs(e[0]) <= abs(e[1]))
    need = lateral + 10
    out.append(Check(kind, "static stability", "PASS" if lat >= need and lon >= working + 10 else
                     ("WARN" if lat >= lateral else "FAIL"),
                     f"tips at {lat:.0f} deg sideways, {lon:.0f} deg fore-aft; works on {lateral:.0f} deg side "
                     f"slopes (needs >= {need:.0f} with a 10 deg margin)"))
    # ------------------------------------------------------------- sinkage
    S = soils()
    worst = int(np.argmax(loads))
    W = loads[worst]
    r, b = pts[worst][3], pts[worst][4]

    def patch(soil, W, r, b):
        """Sinkage and contact length: a wheel's chord at that sinkage, or a track's road-wheel pitch."""
        if cfg.get("track_l"):
            return tm.sinkage(soil, W, b, cfg["track_l"]), cfg["track_l"]
        l = 0.25 * r
        for _ in range(30):
            z = tm.sinkage(soil, W, b, l)
            l = max(0.05, 2 * math.sqrt(max(z, 1e-4) * (2 * r - min(z, r))))
        return z, l
    rows = []
    status = "PASS"
    for sname in cfg["soils"]:
        soil = S[sname]
        z, _ = patch(soil, W, r, b)
        limit = 0.15 * r if kind != "pahadi" else 0.06
        st = "PASS" if z <= limit else ("WARN" if z <= 2 * limit else "FAIL")
        status = {"PASS": status, "WARN": "WARN" if status == "PASS" else status, "FAIL": "FAIL"}[st]
        rows.append(f"{sname} {z * 100:.1f} cm")
    out.append(Check(kind, "static sinkage", status,
                     f"most loaded contact {W / G:,.0f} kg on {b * 100:.0f} cm: " + ", ".join(rows)))
    # ------------------------------------------------------------- gradeability
    driven = cfg["driven"] or contacts
    torque = sum(m.joints[n]["effort"] / m.radius(n)[0] for n in driven if n in m.joints and m.radius(n)[0])
    motor_deg = math.degrees(math.asin(min(1.0, torque / (mass * G)))) if torque else 0.0
    idx = {n: i for i, n in enumerate(contacts)}

    def climbs(soil, th, slip=0.4):
        """Driven contacts' thrust at 40 % slip against compaction resistance of all contacts plus gravity."""
        thrust = resist = 0.0
        for n, i in idx.items():
            Wi = max(0.0, loads[i]) * math.cos(th)
            z, l = patch(soil, Wi, pts[i][3], pts[i][4])
            resist += tm.compaction_resistance(soil, z, pts[i][4])
            if n in driven:
                thrust += tm.shear_capacity(soil, Wi, pts[i][4], l) * tm.shear_fraction(slip, l, soil["K"])
        return thrust >= resist + mass * G * math.sin(th)

    def soil_limit(soil):
        lo, hi = 0.0, 45.0
        if not climbs(soil, 0.0):
            return 0.0
        for _ in range(30):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if climbs(soil, math.radians(mid)) else (lo, mid)
        return lo
    soil_deg = {s: soil_limit(S[s]) for s in cfg["soils"]}
    worst_soil = min(soil_deg, key=soil_deg.get)
    limit = min(motor_deg, soil_deg[worst_soil])
    st = "PASS" if limit >= working + 5 else ("WARN" if limit >= working else "FAIL")
    out.append(Check(kind, "gradeability", st,
                     f"drive torque holds {motor_deg:.0f} deg ({torque / 1000:.1f} kN at the contacts); soil carries "
                     + ", ".join(f"{s} {d:.0f} deg" for s, d in soil_deg.items()) + f"; works on {working:.0f} deg"))
    # monsoon: the same soils near field capacity (fs::soil::AtMoisture), an operating limit, not a design one
    wet = {s: soil_limit(tm.at_moisture(S[s], WET)) for s in cfg["soils"]}
    short = [s for s, d in wet.items() if d < working]
    out.append(Check(kind, "monsoon traction", "WARN" if short else "PASS",
                     f"at {WET:.2f} moisture the soil carries " + ", ".join(f"{s} {d:.0f} deg" for s, d in wet.items())
                     + (f"; keep off wet {', '.join(short)} steeper than " + ", ".join(f"{wet[s]:.0f}" for s in short)
                        + " deg" if short else "")))
    # ------------------------------------------------------------- tool loads
    if kind == "pahadi":
        q = {"arm_yaw_joint": -1.57, "arm_shoulder_joint": 0.0, "arm_elbow_joint": 0.0, "arm_wrist_joint": 0.0}
        ma = Model(xml, q)
        notes, st = [], "PASS"
        for joint, child in (("arm_shoulder_joint", "arm_upper_link"), ("arm_elbow_joint", "arm_fore_link"),
                             ("arm_wrist_joint", "arm_wrist_link")):
            # links outboard of the joint
            outb = [n for n in ma.links if _below(ma, n, child)]
            mo, co = ma.com(outb)
            pivot = ma.T(child)[:3, 3]
            tau = mo * G * math.hypot(co[0] - pivot[0], co[1] - pivot[1])
            eff = ma.joints[child]["effort"]
            s = "PASS" if tau < 0.7 * eff else ("WARN" if tau < eff else "FAIL")
            st = s if s == "FAIL" or (s == "WARN" and st == "PASS") else st
            notes.append(f"{joint.split('_')[1]} {tau:,.0f} of {eff:,.0f} N m")
        out.append(Check(kind, "arm torque at full reach", st, "arm level and fully out: " + "; ".join(notes)))
    if cfg["thrust"]:
        out.append(Check(kind, "auger reaction", "PASS" if cfg["thrust"] < 0.25 * mass * G else "WARN",
                         f"{cfg['thrust'] / 1000:.0f} kN thrust is {cfg['thrust'] / (mass * G) * 100:.0f} % of the rig's weight"))
    return out


def _below(m: Model, link: str, root: str) -> bool:
    while link is not None:
        if link == root:
            return True
        j = m.joints.get(link)
        link = j["parent"] if j else None
    return False


def report(checks) -> str:
    lines = ["# Physics audit", "", "Generated by `ros2 run fs_bringup physics_audit` from the URDF Gazebo loads "
             "(link inertials computed from the CAD by `cad/build.py`).", "",
             "| Robot | Check | Result | Detail |", "|---|---|---|---|"]
    for c in checks:
        lines.append(f"| {c.robot} | {c.name} | **{c.status}** | {c.detail} |")
    return "\n".join(lines) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="")
    a, _ = ap.parse_known_args()
    checks = [c for k in WHEELS for c in audit(k)]
    text = report(checks)
    print(text)
    if a.out:
        Path(a.out).write_text(text)
