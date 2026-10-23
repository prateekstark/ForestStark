"""Site generator: one seeded description per site produces everything that has to agree.

For each site it writes, into an output directory:

  <site>.soilmap   elevation, soil type, depth to rock, moisture (read by the Gazebo soil systems
                   and by the autonomy stack as its prior terrain/soil survey)
  <site>.yaml      survey data for the autonomy stack: planting line and no-dig zones (roadside),
                   field, tender and obstacles (open field), terrain and boulders (hill slope)
  <site>.sdf       the Gazebo world
  <site>_height.png  16-bit heightmap (hill slope only)

Sites
  roadside    Katar: 360 m of curving district road, gravel shoulder, verge with trees, a culvert,
              a buried telecom cable with marker posts and a field access track.
  open_field  Jaal: 1.0 ha block of degraded land, stones, stumps and termite mounds, a tree line
              on the north edge that degrades RTK, a tender truck at the west headland.
  hill_slope  Pahadi: 96 m x 96 m hillside at a chosen slope, lateritic gravel over rock with
              outcrops and boulders.

Generated at launch time (fs_bringup) so that slope, soil and seed are launch arguments, and
once into fs_gazebo/worlds for the defaults.
"""
from __future__ import annotations

import math
import struct
import zlib
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

# Bekker constants (Wong) + specific boring energy, MJ/m^3. Same table as SoilModel.hh.
SOILS = [
    ("dry_sand", 1.10, 0.99, 1528.43, 1.04, 28.0, 0.025, 2.5),
    ("sandy_loam", 0.90, 52.53, 1127.97, 4.83, 20.0, 0.025, 4.0),
    ("clayey_soil", 0.50, 13.19, 692.15, 4.14, 13.0, 0.020, 9.0),
    ("heavy_clay", 0.13, 12.70, 1555.95, 68.95, 34.0, 0.010, 14.0),
    ("lateritic_gravel", 0.80, 30.00, 4000.00, 10.00, 35.0, 0.015, 22.0),
]
SOIL_INDEX = {s[0]: i for i, s in enumerate(SOILS)}

ORIGINS = {   # WGS84 origin of each site's map frame (ENU)
    "roadside": (26.9124, 75.7873, 431.0),     # district road near Jaipur
    "open_field": (27.2181, 76.4305, 312.0),   # degraded block, Aravalli foothills
    "hill_slope": (30.0812, 78.2912, 880.0),   # hillside above Rishikesh
}


def smooth_noise(x, y, seed=0):
    """Deterministic smooth noise in about [-1, 1] (sum of incommensurate sines)."""
    r = np.random.default_rng(seed)
    out = np.zeros(np.broadcast(x, y).shape)
    amp = 0.55
    for k in range(5):
        fx, fy = r.uniform(0.15, 0.45) * (1.9 ** k), r.uniform(0.15, 0.45) * (1.9 ** k)
        px, py = r.uniform(0, 2 * math.pi, 2)
        out = out + amp * np.sin(x * fx + px) * np.cos(y * fy + py)
        amp *= 0.5
    return out


@dataclass
class Grid:
    x0: float
    y0: float
    res: float
    nx: int
    ny: int
    layers: dict = field(default_factory=dict)

    def coords(self):
        xs = self.x0 + np.arange(self.nx) * self.res
        ys = self.y0 + np.arange(self.ny) * self.res
        return np.meshgrid(xs, ys)        # arrays [ny, nx]

    def write(self, path: Path, name: str):
        with open(path, "w") as f:
            f.write("# fs_soilmap v1\n")
            f.write(f"name {name}\norigin {self.x0:.4f} {self.y0:.4f}\nresolution {self.res:.6f}\n")
            f.write(f"size {self.nx} {self.ny}\n")
            for i, s in enumerate(SOILS):
                f.write(f"soil {i} " + " ".join(str(v) for v in s) + "\n")
            for lname, fmt in (("elevation", "%.3f"), ("soil", "%d"), ("rock_depth", "%.3f"), ("moisture", "%.3f")):
                f.write(f"layer {lname}\n")
                np.savetxt(f, self.layers[lname], fmt=fmt)


def write_terrain_obj(path: Path, X: np.ndarray, Y: np.ndarray, Z: np.ndarray, rgb=(0.66, 0.55, 0.40)):
    """Grid surface as an OBJ with smooth normals: the terrain visual. Ogre2's heightmap shader
    does not compile under headless EGL, and a mesh also matches the soil map vertex for vertex."""
    ny, nx = Z.shape
    gy, gx = np.gradient(Z, Y[:, 0], X[0, :])
    N = np.dstack([-gx, -gy, np.ones_like(Z)])
    N /= np.linalg.norm(N, axis=2, keepdims=True)
    mtl = path.with_suffix(".mtl")
    lines = [f"mtllib {mtl.name}", "o terrain", "usemtl soil"]
    lines += [f"v {x:.3f} {y:.3f} {z:.3f}" for x, y, z in zip(X.ravel(), Y.ravel(), Z.ravel())]
    lines += [f"vn {a:.4f} {b:.4f} {c:.4f}" for a, b, c in N.reshape(-1, 3)]
    idx = np.arange(nx * ny).reshape(ny, nx) + 1
    a, b, c, d = idx[:-1, :-1].ravel(), idx[:-1, 1:].ravel(), idx[1:, 1:].ravel(), idx[1:, :-1].ravel()
    lines += [f"f {i}//{i} {j}//{j} {k}//{k}" for i, j, k in zip(a, b, c)]
    lines += [f"f {i}//{i} {j}//{j} {k}//{k}" for i, j, k in zip(a, c, d)]
    path.write_text("\n".join(lines) + "\n")
    r, g, bl = rgb
    mtl.write_text(f"newmtl soil\nKa {r * .5:.3f} {g * .5:.3f} {bl * .5:.3f}\nKd {r} {g} {bl}\nKs 0.02 0.02 0.02\nNs 5\n")


def write_png16(path: Path, img: np.ndarray):
    """Minimal 16-bit grayscale PNG writer (no Pillow needed). Row 0 is the top of the image."""
    h, w = img.shape
    raw = b"".join(b"\x00" + img[r].astype(">u2").tobytes() for r in range(h))
    def chunk(t, d):
        c = struct.pack(">I", len(d)) + t + d
        return c + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)
    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 16, 0, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b"")
    path.write_bytes(png)


# --------------------------------------------------------------------------- SDF helpers
def _pose(x, y, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    return f"<pose>{x:.3f} {y:.3f} {z:.3f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>"


def _include(uri, name, x, y, z=0.0, yaw=0.0):
    return f"<include><uri>model://{uri}</uri><name>{name}</name>{_pose(x, y, z, 0, 0, yaw)}</include>"


def _box_visual(name, x, y, z, sx, sy, sz, yaw, rgb):
    r, g, b = rgb
    return (f"<visual name='{name}'>{_pose(x, y, z, 0, 0, yaw)}<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}"
            f"</size></box></geometry><material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>"
            f"</material></visual>")


OBSTACLE_RGB = {"termite_mound": (0.62, 0.42, 0.27), "stump": (0.40, 0.29, 0.19), "stone": (0.55, 0.53, 0.49),
                "boulder": (0.52, 0.50, 0.46)}


def obstacle_model(kind, name, x, y, z, r, h, yaw):
    """Static obstacle with collision, sized by footprint radius r and height h."""
    rgb = " ".join(str(c) for c in OBSTACLE_RGB[kind])
    # Collisions are primitives (DART meshes every ellipsoid, which is slow); stones have none:
    # the robots roll over them, and the soil map tells the auger there is rock there.
    if kind == "stump":
        geo = f"<cylinder><radius>{r:.3f}</radius><length>{h:.3f}</length></cylinder>"
        col, gz = geo, h / 2
    else:
        geo = f"<ellipsoid><radii>{r:.3f} {r * 0.85:.3f} {h:.3f}</radii></ellipsoid>"
        col, gz = f"<cylinder><radius>{r * 0.9:.3f}</radius><length>{h:.3f}</length></cylinder>", 0.0
    mat = f"<material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material>"
    collision = "" if kind == "stone" else \
        f"<collision name='c'><pose>0 0 {h / 2 - gz:.3f} 0 0 0</pose><geometry>{col}</geometry></collision>"
    return (f"    <model name='{name}'><static>true</static>{_pose(x, y, z, 0, 0, yaw)}<link name='link'>"
            f"<pose>0 0 {gz:.3f} 0 0 0</pose>{collision}"
            f"<visual name='v'><geometry>{geo}</geometry>{mat}</visual></link></model>\n")


def world_header(name, origin, step=0.004):
    lat, lon, alt = origin
    return f"""<?xml version="1.0"?>
<!-- Generated by fs_gazebo/sites.py. Do not edit; change the generator and regenerate. -->
<sdf version="1.10">
  <world name="{name}">
    <physics name="4ms" type="dart">
      <max_step_size>{step}</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{lat}</latitude_deg>
      <longitude_deg>{lon}</longitude_deg>
      <elevation>{alt}</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    <scene><ambient>0.55 0.56 0.52 1</ambient><background>0.78 0.85 0.87 1</background><shadows>true</shadows></scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 60 0 0 0</pose>
      <diffuse>1.0 0.96 0.88 1</diffuse><specular>0.2 0.2 0.2 1</specular>
      <direction>-0.45 0.3 -0.85</direction>
    </light>
"""


def ground_plane(rgb=(0.62, 0.58, 0.40), size=2000):
    r, g, b = rgb
    return f"""    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>{size} {size}</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>{size} {size}</size></plane></geometry>
          <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material></visual>
      </link>
    </model>
"""


# =========================================================================== roadside (Katar)
def shoulder_y(x):
    return 6.0 * np.sin(x / 45.0) + 2.5 * np.sin(x / 19.0 + 1.0)


def build_roadside(out: Path, seed=7, soil="sandy_loam", spacing=3.0):
    name = "roadside"
    rng = np.random.default_rng(seed)
    xs = np.arange(-30.0, 382.0, 2.0)
    cl = np.stack([xs, shoulder_y(xs)], axis=1)
    tang = np.gradient(cl, axis=0)
    tang /= np.linalg.norm(tang, axis=1, keepdims=True)
    left = np.stack([-tang[:, 1], tang[:, 0]], axis=1)   # +y side when driving east
    s_cl = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(cl, axis=0), axis=1))])
    plant_offset = 2.5

    def at_station(s, lat):
        i = int(np.clip(np.searchsorted(s_cl, s) - 1, 0, len(s_cl) - 2))
        u = (s - s_cl[i]) / (s_cl[i + 1] - s_cl[i])
        p = cl[i] + u * (cl[i + 1] - cl[i])
        return p + left[i] * lat, math.atan2(tang[i, 1], tang[i, 0])

    s_start, s_end = 40.0, 380.0      # planting runs over stations 40..380 m of the centreline
    no_dig = [
        {"name": "culvert", "kind": "culvert", "s0": 150.0, "s1": 158.0, "lat0": -1.5, "lat1": 6.0, "visible": True},
        {"name": "telecom cable", "kind": "cable", "s0": 232.0, "s1": 290.0, "lat0": 1.9, "lat1": 3.2, "visible": False},
        {"name": "field access", "kind": "access", "s0": 318.0, "s1": 324.0, "lat0": 1.3, "lat1": 12.0, "visible": True},
    ]
    marker_posts = [s for s in (232.0, 251.0, 270.0, 289.0)]
    trees = []
    for s in np.arange(55.0, 370.0, 13.0):
        lat = rng.uniform(4.0, 10.0) if rng.random() > 0.18 else rng.uniform(2.2, 2.9)
        if any(z["s0"] - 4 < s < z["s1"] + 4 for z in no_dig):
            continue
        p, _ = at_station(s + rng.uniform(-3, 3), lat)
        trees.append({"x": float(p[0]), "y": float(p[1]), "r": 0.35, "clearance": 2.0})

    # Soil map 1 m grid over the corridor.
    g = Grid(-32.0, -40.0, 1.0, 416, 81)
    X, Y = g.coords()
    n1 = smooth_noise(X * 0.05, Y * 0.05, seed)
    g.layers["elevation"] = np.zeros_like(X)
    soil_idx = np.full(X.shape, SOIL_INDEX[soil])
    soil_idx[(X > 150) & (X < 225)] = SOIL_INDEX["clayey_soil"]
    soil_idx[(X > 280) & (X < 335) & (n1 > 0.0)] = SOIL_INDEX["lateritic_gravel"]
    # Carriageway and gravel shoulder are compacted: lateral offset from the shoulder centreline
    # (left positive) below +1.4 m. The verge, where planting happens, keeps the natural soil.
    lateral = (Y - shoulder_y(X)) * np.cos(np.arctan(np.gradient(shoulder_y(X), axis=1)))
    soil_idx[lateral < 1.4] = SOIL_INDEX["lateritic_gravel"]
    g.layers["soil"] = soil_idx
    g.layers["rock_depth"] = np.clip(0.9 + 0.3 * n1, 0.45, 1.5)
    g.layers["moisture"] = np.clip(0.14 + 0.06 * smooth_noise(X * 0.03, Y * 0.03, seed + 1), 0.04, 0.35)
    g.write(out / f"{name}.soilmap", name)

    # World.
    parts = [world_header(name, ORIGINS[name]), ground_plane((0.60, 0.58, 0.38))]
    vis = []
    for i in range(len(cl) - 1):
        a, b = cl[i], cl[i + 1]
        mid = (a + b) / 2
        yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        L = float(np.linalg.norm(b - a)) + 0.05
        nrm = left[i]
        for lab, off, wid, h, rgb in (("asph", -4.9, 7.2, 0.012, (0.29, 0.30, 0.31)),
                                      ("shld", 0.0, 2.6, 0.008, (0.66, 0.61, 0.53)),
                                      ("edge", -1.42, 0.12, 0.016, (0.92, 0.92, 0.88))):
            c = mid + nrm * off
            vis.append(_box_visual(f"{lab}{i}", c[0], c[1], h / 2, L, wid, h, yaw, rgb))
        if i % 3 == 0:
            c = mid + nrm * -4.9
            vis.append(_box_visual(f"dash{i}", c[0], c[1], 0.008, 2.2, 0.12, 0.016, yaw, (0.92, 0.92, 0.88)))
    parts.append("    <model name='road'><static>true</static><link name='link'>" + "".join(vis) + "</link></model>\n")
    for k, t in enumerate(trees):
        parts.append("    " + _include("fs_tree_big", f"tree_{k}", t["x"], t["y"], 0, rng.uniform(0, 6.28)) + "\n")
    for k, s in enumerate(range(100, 380, 100)):
        p, yaw = at_station(float(s), 1.6)
        parts.append("    " + _include("fs_milestone", f"milestone_{k}", p[0], p[1], 0, yaw) + "\n")
    for k, s in enumerate(marker_posts):
        p, yaw = at_station(s, 1.75)
        parts.append("    " + _include("fs_marker_post", f"cable_marker_{k}", p[0], p[1], 0, yaw) + "\n")
    p, yaw = at_station(154.0, 2.0)
    parts.append("    " + _include("fs_culvert", "culvert", p[0], p[1], 0, yaw) + "\n")
    p, yaw = at_station(321.0, 6.0)
    vis = [_box_visual("track", 0, 0, 0.006, 5.0, 11.0, 0.012, 0, (0.70, 0.64, 0.52))]
    parts.append(f"    <model name='access_track'><static>true</static>{_pose(p[0], p[1], 0, 0, 0, yaw)}"
                 f"<link name='link'>{''.join(vis)}</link></model>\n")
    parts.append("  </world>\n</sdf>\n")
    (out / f"{name}.sdf").write_text("".join(parts))

    p0, yaw0 = at_station(8.0, 0.0)
    site = {
        "name": name, "robot": "katar",
        "origin": dict(zip(("lat", "lon", "alt"), ORIGINS[name])),
        "soil_map": f"{name}.soilmap",
        "spawn": [{"name": "katar", "x": float(p0[0]), "y": float(p0[1]), "z": 0.8, "yaw": float(yaw0)}],
        "roadside": {
            "shoulder_centerline": [[round(float(a), 3), round(float(b), 3)] for a, b in cl],
            "planting_offset": plant_offset,
            "station_start": s_start, "station_end": s_end,
            "default_spacing": spacing,
            "no_dig": no_dig,
            "marker_posts": marker_posts,
            "trees": trees,
        },
    }
    (out / f"{name}.yaml").write_text(yaml.safe_dump(site, sort_keys=False))
    return site


# =========================================================================== open field (Jaal)
def build_open_field(out: Path, seed=11, soil="sandy_loam", fleet=3):
    name = "open_field"
    rng = np.random.default_rng(seed)
    fx0, fy0, fx1, fy1 = 0.0, 0.0, 120.0, 84.0           # 1.01 ha block
    tender = {"x": -14.0, "y": 10.0, "yaw": 0.0}
    obstacles = []

    def free(x, y, r):
        return all(math.hypot(o["x"] - x, o["y"] - y) > o["r"] + r + 1.0 for o in obstacles)
    for kind, count, rr, hh in (("termite_mound", 7, (0.8, 1.4), (0.9, 1.6)), ("stump", 16, (0.25, 0.35), (0.3, 0.5)),
                                ("stone", 70, (0.15, 0.32), (0.12, 0.25))):
        n = 0
        while n < count:
            x, y = rng.uniform(fx0 + 2, fx1 - 2), rng.uniform(fy0 + 2, fy1 - 2)
            r = rng.uniform(*rr)
            if free(x, y, r):
                obstacles.append({"kind": kind, "x": round(float(x), 3), "y": round(float(y), 3),
                                  "r": round(float(r), 3), "h": round(float(rng.uniform(*hh)), 3)})
                n += 1

    g = Grid(-40.0, -30.0, 0.5, 361, 301)
    X, Y = g.coords()
    n1 = smooth_noise(X * 0.06, Y * 0.06, seed)
    g.layers["elevation"] = np.zeros_like(X)
    soil_idx = np.full(X.shape, SOIL_INDEX[soil])
    soil_idx[(n1 > 0.45)] = SOIL_INDEX["clayey_soil"]
    soil_idx[(n1 < -0.55)] = SOIL_INDEX["dry_sand"]
    g.layers["soil"] = soil_idx
    rock = np.clip(0.8 + 0.35 * n1, 0.35, 1.4)
    for o in obstacles:   # a stone or stump means no soil to bore where it sits
        if o["kind"] in ("stone", "stump"):
            m = (X - o["x"]) ** 2 + (Y - o["y"]) ** 2 < (o["r"] + 0.05) ** 2
            rock[m] = 0.03
    g.layers["rock_depth"] = rock
    g.layers["moisture"] = np.clip(0.11 + 0.05 * smooth_noise(X * 0.04, Y * 0.04, seed + 3), 0.03, 0.3)
    g.write(out / f"{name}.soilmap", name)

    parts = [world_header(name, ORIGINS[name]), ground_plane((0.66, 0.60, 0.42))]
    # field boundary posts
    m = 2.5   # survey posts stand clear of the headland lanes
    for k, (x, y) in enumerate([(fx0 - m, fy0 - m), (fx1 + m, fy0 - m), (fx1 + m, fy1 + m), (fx0 - m, fy1 + m)]):
        parts.append(f"    <model name='corner_{k}'><static>true</static>{_pose(x, y)}<link name='l'>"
                     + _box_visual("v", 0, 0, 0.6, 0.1, 0.1, 1.2, 0, (0.45, 0.33, 0.22)) + "</link></model>\n")
    for k, o in enumerate(obstacles):
        parts.append(obstacle_model(o["kind"], f"{o['kind']}_{k}", o["x"], o["y"], 0.0, o["r"], o["h"],
                                    rng.uniform(0, 6.28)))
    # tree line along the north edge
    for k, x in enumerate(np.arange(-10, 135, 7.5)):
        parts.append("    " + _include("fs_tree_big", f"treeline_{k}", x + rng.uniform(-2, 2), fy1 + 9 + rng.uniform(0, 4),
                                       0, rng.uniform(0, 6.28)) + "\n")
    parts.append("    " + _include("fs_tender_truck", "tender_truck", tender["x"] - 4.5, tender["y"] + 3.2, 0, 0) + "\n")
    cows = [{"name": f"cow_{k}", "x": float(x), "y": float(y)} for k, (x, y) in enumerate([(70.0, 30.0), (30.0, 60.0)])]
    for c in cows:
        parts.append("    " + _include("fs_cow", c["name"], c["x"], c["y"], 0, rng.uniform(0, 6.28)) + "\n")
    parts.append("  </world>\n</sdf>\n")
    (out / f"{name}.sdf").write_text("".join(parts))

    # robots park in a column between the tender and the field, 4.5 m apart
    spawn = [{"name": f"jaal_{i + 1}", "x": -6.0, "y": 2.0 + 4.5 * i, "z": 0.45, "yaw": 0.0} for i in range(6)]
    site = {
        "name": name, "robot": "jaal",
        "origin": dict(zip(("lat", "lon", "alt"), ORIGINS[name])),
        "soil_map": f"{name}.soilmap",
        "spawn": spawn,
        "field": {
            "polygon": [[fx0, fy0], [fx1, fy0], [fx1, fy1], [fx0, fy1]],
            "demo_polygon": [[0.0, 0.0], [36.0, 0.0], [36.0, 24.0], [0.0, 24.0]],
            "headland": 4.0,
        },
        # queue slots run south from (-9, 4), between the field and the bay, so nobody passes the bay to queue
        "tender": {"bay": [tender["x"], tender["y"], 0.0], "wait": [-9.0, 4.0, 0.0],
                   "spare_drums": 6, "plugs_per_drum": 180, "water_l": 3000, "restock_s": 2400},
        "rtk_degraded_zones": [{"name": "north tree line", "polygon": [[-20, fy1 - 10], [140, fy1 - 10], [140, fy1 + 30],
                                                                        [-20, fy1 + 30]], "mode": "float", "sigma": 0.35}],
        "obstacles": obstacles,
        "livestock": cows,
    }
    (out / f"{name}.yaml").write_text(yaml.safe_dump(site, sort_keys=False))
    return site


# =========================================================================== hill slope (Pahadi)
def hill_height(X, Y, slope_deg, seed):
    t = math.tan(math.radians(slope_deg))
    base = Y * t
    rough = 0.35 * smooth_noise(X * 0.35, Y * 0.35, seed) + 0.9 * smooth_noise(X * 0.06, Y * 0.06, seed + 5)
    gully = -1.2 * np.exp(-((X - 31.0) / 3.0) ** 2) * (1 / (1 + np.exp(-(Y + 10) / 4)))
    return base + rough + gully


def build_hill_slope(out: Path, seed=23, slope_deg=14.0, soil="lateritic_gravel", rock_cover=0.25):
    name = "hill_slope"
    rng = np.random.default_rng(seed)
    size, px = 96.0, 257
    res = size / (px - 1)
    g = Grid(-size / 2, -size / 2, res, px, px)
    X, Y = g.coords()
    H = hill_height(X, Y, slope_deg, seed)
    hmin, hmax = float(H.min()), float(H.max())
    g.layers["elevation"] = H
    # Depth to rock: deep soil, a stony band (pick needed) and outcrops (seed balls only), with
    # rock_cover the share of ground whose soil is too thin for the auger.
    nrock = smooth_noise(X * 0.22, Y * 0.22, seed + 9)
    q_hi = np.quantile(nrock, 1.0 - 0.45 * rock_cover)
    q_mid = np.quantile(nrock, 1.0 - rock_cover)
    deep = np.clip(0.45 + 0.35 * smooth_noise(X * 0.12, Y * 0.12, seed + 4), 0.3, 0.9)
    stony = 0.13 + 0.11 * np.clip((q_hi - nrock) / max(q_hi - q_mid, 1e-6), 0, 1)
    outcrop = 0.03 + 0.08 * np.clip(1 - (nrock - q_hi) / 0.3, 0, 1)
    depth = np.where(nrock > q_hi, outcrop, np.where(nrock > q_mid, stony, deep))
    thr = q_mid
    g.layers["rock_depth"] = depth
    soil_idx = np.full(X.shape, SOIL_INDEX[soil])
    soil_idx[(smooth_noise(X * 0.09, Y * 0.09, seed + 2) > 0.5) & (nrock < thr)] = SOIL_INDEX["sandy_loam"]
    g.layers["soil"] = soil_idx
    g.layers["moisture"] = np.clip(0.10 + 0.05 * smooth_noise(X * 0.05, Y * 0.05, seed + 8) - 0.002 * (Y + 48),
                                   0.03, 0.3)
    g.write(out / f"{name}.soilmap", name)

    # Heightmap image: Gazebo puts image row 0 at +y (north). Values span the full 16-bit range.
    img = np.round((H - hmin) / (hmax - hmin) * 65535).astype(np.uint16)[::-1, :]
    write_png16(out / f"{name}_height.png", img)

    boulders = []
    while len(boulders) < 45:
        x, y = rng.uniform(-44, 44), rng.uniform(-44, 44)
        r = 0.3 + rng.random() ** 2 * 1.1
        if all(math.hypot(b["x"] - x, b["y"] - y) > b["r"] + r + 1.5 for b in boulders):
            boulders.append({"x": round(float(x), 3), "y": round(float(y), 3), "r": round(float(r), 3)})

    def h_at(x, y):
        return float(hill_height(np.array(x), np.array(y), slope_deg, seed))

    png = (out / f"{name}_height.png").resolve()
    mesh = (out / f"{name}_terrain.obj").resolve()
    write_terrain_obj(mesh, X, Y, H - hmin)
    parts = [world_header(name, ORIGINS[name])]
    # DART ignores <heightmap><pos>; lift the whole model by hmin instead (honoured by physics and rendering).
    parts.append(f"""    <model name="terrain">
      <static>true</static>
      <pose>0 0 {hmin:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><heightmap><uri>file://{png}</uri><size>{size} {size} {hmax - hmin:.4f}</size>
          <pos>0 0 0</pos></heightmap></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
        <visual name="v"><geometry><mesh><uri>file://{mesh}</uri></mesh></geometry></visual>
      </link>
    </model>
""")
    for k, b in enumerate(boulders):
        parts.append(obstacle_model("boulder", f"boulder_{k}", b["x"], b["y"], h_at(b["x"], b["y"]) - 0.2 * b["r"],
                                    b["r"], 0.75 * b["r"], rng.uniform(0, 6.28)))
    for k in range(14):
        x, y = rng.uniform(-44, 44), rng.uniform(-44, 44)
        parts.append("    " + _include("fs_shrub", f"shrub_{k}", x, y, h_at(x, y) - 0.05, rng.uniform(0, 6.28)) + "\n")
    parts.append("  </world>\n</sdf>\n")
    (out / f"{name}.sdf").write_text("".join(parts))

    sx, sy = -40.0, -4.0       # parked below the first contour, facing uphill
    site = {
        "name": name, "robot": "pahadi",
        "origin": dict(zip(("lat", "lon", "alt"), ORIGINS[name])),
        "soil_map": f"{name}.soilmap",
        "heightmap": f"{name}_height.png",
        "slope_deg": slope_deg,
        "spawn": [{"name": "pahadi", "x": sx, "y": sy, "z": round(h_at(sx, sy) + 0.45, 3), "yaw": round(math.pi / 2, 4)}],
        "hill": {"bounds": [-44.0, -44.0, 44.0, 44.0], "work_area": [-34.0, -14.0, 30.0, 10.0],
                 "uphill": [0.0, 1.0]},
        "boulders": boulders,
    }
    (out / f"{name}.yaml").write_text(yaml.safe_dump(site, sort_keys=False))
    return site


BUILDERS = {"roadside": build_roadside, "open_field": build_open_field, "hill_slope": build_hill_slope}


def generate(site: str, out: Path, **kw) -> dict:
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    return BUILDERS[site](out, **kw)
