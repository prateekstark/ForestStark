#!/usr/bin/env python3
"""Build the ForestStark CAD outputs from forest_cad/*.py.

    cad/.venv/bin/python cad/build.py              # all robots
    cad/.venv/bin/python cad/build.py jaal         # one robot
    cad/.venv/bin/python cad/build.py --joints     # only refresh the *_joints.json files

Outputs
  cad/out/<robot>.step               full assembly, every link placed by its URDF joint (display pose)
  cad/out/<robot>_<subsystem>.step   the plant-handling subsystem on its own, for close inspection
  cad/out/<robot>.glb                the assembly for the web viewer (glTF, y-up, millimetres)
  cad/out/<robot>_bom.csv            parts, materials, volumes and estimated masses
  cad/out/<robot>_joints.json        the URDF joint tree and display pose, for posing the GLB in the viewer
  ros2_ws/src/fs_description/meshes/<robot>/<link>.obj/.mtl   per-link visuals for Gazebo and RViz (metres)
  ros2_ws/src/fs_description/meshes/plug.obj                   the plug that fs::PlugInventory places

Link frames come from the robot's xacro (expanded with xacro), so the CAD and the simulation
always share one kinematic definition.
"""
import csv
import json
import math
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import cadquery as cq
import numpy as np
from OCP.gp import gp_Trsf

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from forest_cad import common, jaal, katar, pahadi  # noqa: E402

WS = ROOT.parent / "ros2_ws"
MESHES = WS / "src" / "fs_description" / "meshes"
OUT = ROOT / "out"
ROBOTS = {"katar": katar, "jaal": jaal, "pahadi": pahadi}
POSES = {   # joint values for the display pose of the STEP/GLB assemblies
    "katar": {"carriage_joint": 1.5},
    "jaal": {"picker_x_joint": -1.2, "picker_z_joint": 0.74},
    "pahadi": {"arm_shoulder_joint": -1.35, "arm_elbow_joint": 2.6, "arm_wrist_joint": 0.3},
}
SUBSYSTEMS = {
    "katar": ("planting_head", ["carriage_link", "carousel_link", "turret_link", "auger_feed_link", "auger_link",
                                "duckbill_feed_link", "jaw_l_link", "jaw_r_link", "tamp_link"]),
    "jaal": ("plant_handling", ["drum_f_link", "drum_r_link", "picker_x_link", "picker_z_link", "picker_y_link",
                                "finger_a_link", "finger_b_link", "shuttle_link", "auger_feed_link", "auger_link",
                                "duckbill_feed_link", "jaw_l_link", "jaw_r_link", "tamp_link"]),
    "pahadi": ("arm_and_gripper", ["arm_base_link", "arm_yaw_link", "arm_upper_link", "arm_fore_link", "arm_wrist_link",
                                   "finger_l_link", "finger_r_link", "auger_feed_link", "auger_link"]),
}
TOL, ANG = 0.8, 0.3   # tessellation: mm, rad


# ------------------------------------------------------------------ URDF kinematics
def urdf_tree(robot: str):
    cmd = (f"source /opt/ros/lyrical/setup.bash && source {WS}/install/setup.bash && "
           f"xacro {WS}/src/fs_description/urdf/{robot}.urdf.xacro soil_map:=/tmp/x.soilmap lidar:=false")
    xml = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True, check=True).stdout
    root = ET.fromstring(xml)
    joints = {}
    for j in root.iter("joint"):
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        a = j.find("axis")
        axis = [float(v) for v in (a.get("xyz", "1 0 0") if a is not None else "1 0 0").split()]
        lim = j.find("limit")
        joints[j.find("child").get("link")] = dict(
            name=j.get("name"), parent=j.find("parent").get("link"), type=j.get("type"), xyz=xyz, rpy=rpy, axis=axis,
            lower=float(lim.get("lower", 0)) if lim is not None else None,
            upper=float(lim.get("upper", 0)) if lim is not None else None)
    return joints


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) \
        @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])


def axis_angle(axis, q):
    a = np.asarray(axis, float)
    a /= np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K


def link_poses(joints, pose):
    cache = {}

    def T(link):
        if link in cache:
            return cache[link]
        if link not in joints:
            cache[link] = np.eye(4)
            return cache[link]
        j = joints[link]
        M = np.eye(4)
        M[:3, :3] = rpy_matrix(*j["rpy"])
        M[:3, 3] = j["xyz"]
        q = pose.get(j["name"], 0.0)
        J = np.eye(4)
        if j["type"] in ("revolute", "continuous"):
            J[:3, :3] = axis_angle(j["axis"], q)
        elif j["type"] == "prismatic":
            J[:3, 3] = np.asarray(j["axis"]) * q
        cache[link] = T(j["parent"]) @ M @ J
        return cache[link]
    return T


def location(M):
    t = gp_Trsf()
    t.SetValues(M[0, 0], M[0, 1], M[0, 2], M[0, 3] * 1000, M[1, 0], M[1, 1], M[1, 2], M[1, 3] * 1000,
                M[2, 0], M[2, 1], M[2, 2], M[2, 3] * 1000)
    return cq.Location(t)


# ------------------------------------------------------------------ OBJ writer (metres, z-up)
def write_obj(path: Path, parts):
    mtl = path.with_suffix(".mtl")
    lines, mlines, vbase, nbase, used = [f"mtllib {mtl.name}"], [], 1, 1, set()
    tris = 0
    for i, p in enumerate(parts):
        if p.lod == "step":
            continue
        verts, faces = common.solid(p.shape).tessellate(TOL, ANG)
        if not faces:
            continue
        mname = f"{p.material}_{int(p.alpha * 100)}"
        if mname not in used:
            used.add(mname)
            r, g, b = common.RAL[p.material]
            mlines += [f"newmtl {mname}", f"Ka {r * .4:.3f} {g * .4:.3f} {b * .4:.3f}", f"Kd {r:.3f} {g:.3f} {b:.3f}",
                       "Ks 0.15 0.15 0.15", "Ns 30", f"d {p.alpha:.2f}", ""]
        lines += [f"o {p.name}_{i}", f"usemtl {mname}"]
        V = np.array([[v.x, v.y, v.z] for v in verts]) / 1000.0
        lines += [f"v {x:.5f} {y:.5f} {z:.5f}" for x, y, z in V]
        F = np.array(faces)
        n = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
        n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
        lines += [f"vn {a:.4f} {b:.4f} {c:.4f}" for a, b, c in n]
        for k, (a, b, c) in enumerate(F):
            lines.append(f"f {a + vbase}//{k + nbase} {b + vbase}//{k + nbase} {c + vbase}//{k + nbase}")
        vbase += len(V)
        nbase += len(F)
        tris += len(F)
    path.write_text("\n".join(lines) + "\n")
    mtl.write_text("\n".join(mlines) + "\n")
    return tris


# ------------------------------------------------------------------ main
def bom_rows(robot, R):
    rows = []
    for link, parts in R.links.items():
        for p in parts:
            if p.lod == "mesh":
                continue
            vol = common.solid(p.shape).Volume() / 1e9          # m^3
            mass = vol * common.DENSITY.get(p.material, 1000) * p.hollow
            rows.append([robot, link, p.name, p.material, round(vol * 1000, 3), round(mass, 2), p.note])
    return rows


def write_joints(robot, joints=None):
    joints = joints or urdf_tree(robot)
    OUT.mkdir(exist_ok=True)
    (OUT / f"{robot}_joints.json").write_text(json.dumps({"joints": joints, "pose": POSES[robot]}, indent=1))


def build(robot):
    t0 = time.time()
    R = ROBOTS[robot].build()
    print(f"{robot}: {len(R.links)} links built in {time.time() - t0:.0f} s")
    (MESHES / robot).mkdir(parents=True, exist_ok=True)
    tris = 0
    for link, parts in R.links.items():
        tris += write_obj(MESHES / robot / f"{link}.obj", parts)
    print(f"  meshes: {tris:,} triangles -> {MESHES / robot}")

    joints = urdf_tree(robot)
    write_joints(robot, joints)
    T = link_poses(joints, POSES[robot])
    missing = [l for l in R.links if l != "base_link" and l not in joints]
    if missing:
        print(f"  WARNING: CAD links not in the URDF: {missing}")
    sub_name, sub_links = SUBSYSTEMS[robot]
    def assembly(name, links, lods):
        a = cq.Assembly(name=name)
        for link in links:
            la = cq.Assembly(name=link, loc=location(T(link)))
            for i, p in enumerate(R.links[link]):
                if p.lod in lods:
                    la.add(common.solid(p.shape), name=f"{p.name}_{i}", color=common.color(p.material, p.alpha))
            a.add(la)
        return a
    OUT.mkdir(exist_ok=True)
    assembly(robot, R.links, ("both", "step")).export(str(OUT / f"{robot}.step"))
    assembly(f"{robot}_{sub_name}", [l for l in R.links if l in sub_links], ("both", "step")).export(
        str(OUT / f"{robot}_{sub_name}.step"))
    assembly(robot, R.links, ("both", "mesh")).export(str(OUT / f"{robot}.glb"))
    rows = bom_rows(robot, R)
    with open(OUT / f"{robot}_bom.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["robot", "link", "part", "material", "volume_L", "est_mass_kg", "note"])
        w.writerows(rows)
    total = sum(r[5] for r in rows)
    print(f"  STEP, GLB, BOM -> {OUT}  (estimated structural mass {total:,.0f} kg, {len(rows)} parts) "
          f"in {time.time() - t0:.0f} s")


def plug_mesh():
    MESHES.mkdir(parents=True, exist_ok=True)
    n = write_obj(MESHES / "plug.obj", common.plug())
    print(f"plug mesh: {n} triangles")


if __name__ == "__main__":
    args = sys.argv[1:]
    if "--joints" in args:
        for r in ROBOTS:
            write_joints(r)
        sys.exit(0)
    plug_mesh()
    for r in (args or list(ROBOTS)):
        build(r)
