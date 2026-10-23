"""Scenery and planting models, written as Gazebo model directories (model.config + model.sdf).

Planting results (sapling, seed balls, half-moon, skip marker) are also returned as SDF strings
by `planting_sdf()`, which the planting head sends to Gazebo's /world/<w>/create service, so a
spawn never depends on the resource path.
"""
from __future__ import annotations

from pathlib import Path


def _mat(r, g, b):
    return f"<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>"


def _vis(name, geo, pose, rgb):
    return f"<visual name='{name}'><pose>{pose}</pose><geometry>{geo}</geometry>{_mat(*rgb)}</visual>"


def _col(name, geo, pose):
    return f"<collision name='{name}'><pose>{pose}</pose><geometry>{geo}</geometry></collision>"


def cyl(r, l):
    return f"<cylinder><radius>{r}</radius><length>{l}</length></cylinder>"


def box(x, y, z):
    return f"<box><size>{x} {y} {z}</size></box>"


def sph(r):
    return f"<sphere><radius>{r}</radius></sphere>"


def ell(a, b, c):
    return f"<ellipsoid><radii>{a} {b} {c}</radii></ellipsoid>"


LEAF = (0.31, 0.60, 0.25)
LEAF2 = (0.24, 0.50, 0.21)
BARK = (0.35, 0.26, 0.19)
SOIL = (0.37, 0.26, 0.17)


def _model(name, body, static=True):
    return (f"<?xml version='1.0'?>\n<sdf version='1.10'><model name='{name}'><static>{str(static).lower()}</static>"
            f"{body}</model></sdf>\n")


def _link(name, inner, pose="0 0 0 0 0 0"):
    return f"<link name='{name}'><pose>{pose}</pose>{inner}</link>"


MODELS = {
    "fs_tree_big": _model("fs_tree_big", _link("trunk",
        _col("trunk", cyl(0.3, 2.4), "0 0 1.2 0 0 0") +
        _vis("trunk", cyl(0.22, 2.4), "0 0 1.2 0 0 0", BARK) +
        _vis("c1", ell(2.2, 2.0, 1.4), "0 0 3.3 0 0 0", (0.36, 0.49, 0.24)) +
        _vis("c2", ell(1.4, 1.3, 1.0), "0.9 0.4 2.9 0 0 0", (0.33, 0.46, 0.22)) +
        _vis("c3", ell(1.3, 1.4, 1.1), "-0.8 -0.5 3.0 0 0 0", (0.39, 0.52, 0.26)))),
    "fs_shrub": _model("fs_shrub", _link("l",
        _vis("s", cyl(0.05, 0.6), "0 0 0.3 0 0 0", BARK) +
        _vis("c", ell(0.8, 0.7, 0.45), "0 0 0.75 0 0 0", (0.40, 0.48, 0.25)))),
    "fs_milestone": _model("fs_milestone", _link("l",
        _col("c", box(0.14, 0.32, 0.55), "0 0 0.275 0 0 0") +
        _vis("b", box(0.14, 0.32, 0.55), "0 0 0.275 0 0 0", (0.93, 0.93, 0.9)) +
        _vis("t", cyl(0.16, 0.145), "0 0 0.55 1.5708 0 1.5708", (0.91, 0.70, 0.13)))),
    "fs_marker_post": _model("fs_marker_post", _link("l",
        _col("c", cyl(0.05, 1.0), "0 0 0.5 0 0 0") +
        _vis("p", cyl(0.05, 1.0), "0 0 0.5 0 0 0", (0.88, 0.42, 0.11)) +
        _vis("h", box(0.12, 0.02, 0.22), "0 0 0.85 0 0 0", (0.95, 0.95, 0.9)))),
    "fs_culvert": _model("fs_culvert", _link("l",
        _col("w1", box(0.4, 7.0, 0.9), "-4.0 0 0.45 0 0 0") + _col("w2", box(0.4, 7.0, 0.9), "4.0 0 0.45 0 0 0") +
        _vis("w1", box(0.4, 7.0, 0.9), "-4.0 0 0.45 0 0 0", (0.72, 0.71, 0.68)) +
        _vis("w2", box(0.4, 7.0, 0.9), "4.0 0 0.45 0 0 0", (0.72, 0.71, 0.68)) +
        _vis("ditch", box(7.6, 7.0, 0.02), "0 0 0.01 0 0 0", (0.33, 0.30, 0.24)))),
    "fs_tender_truck": _model("fs_tender_truck", _link("body",
        _col("chassis", box(6.0, 2.1, 1.0), "0 0 0.8 0 0 0") + _col("cab", box(1.8, 2.1, 1.9), "2.3 0 1.9 0 0 0") +
        _vis("chassis", box(6.0, 2.0, 0.3), "0 0 0.8 0 0 0", (0.19, 0.21, 0.20)) +
        _vis("cab", box(1.8, 2.1, 1.8), "2.3 0 1.85 0 0 0", (0.91, 0.93, 0.91)) +
        _vis("deck", box(3.9, 2.1, 0.1), "-0.6 0 1.0 0 0 0", (0.19, 0.21, 0.20)) +
        "".join(_vis(f"drum{i}", cyl(0.44, 1.5), f"{x} {y} 1.8 0 0 0", (0.15, 0.16, 0.16)) +
                _vis(f"top{i}", cyl(0.44, 0.06), f"{x} {y} 2.58 0 0 0", LEAF)
                for i, (x, y) in enumerate([(a, b) for a in (-2.05, -1.1, -0.15) for b in (-0.5, 0.5)])) +
        _vis("tank", cyl(0.6, 1.6), "-2.4 0 1.2 0 1.5708 0", (0.18, 0.42, 0.56)) +
        "".join(_vis(f"w{i}", cyl(0.5, 0.3), f"{x} {y} 0.5 1.5708 0 0", (0.1, 0.1, 0.1))
                for i, (x, y) in enumerate([(a, b) for a in (2.2, -1.0, -2.2) for b in (-1.0, 1.0)])))),
    "fs_cow": _model("fs_cow", _link("body",
        "<inertial><mass>450</mass><inertia><ixx>60</ixx><iyy>160</iyy><izz>160</izz></inertia>"
        "<pose>0 0 0.9 0 0 0</pose></inertial>" +
        _col("torso", box(1.6, 0.6, 0.7), "0 0 1.05 0 0 0") +
        "".join(_col(f"leg{i}", box(0.12, 0.12, 0.7), f"{x} {y} 0.35 0 0 0")
                for i, (x, y) in enumerate([(0.6, 0.2), (0.6, -0.2), (-0.6, 0.2), (-0.6, -0.2)])) +
        _vis("torso", box(1.6, 0.6, 0.7), "0 0 1.05 0 0 0", (0.85, 0.82, 0.76)) +
        _vis("head", box(0.5, 0.3, 0.32), "1.0 0 1.35 0 0.3 0", (0.85, 0.82, 0.76)) +
        _vis("hump", ell(0.25, 0.22, 0.2), "0.55 0 1.45 0 0 0", (0.8, 0.77, 0.7)) +
        "".join(_vis(f"leg{i}", box(0.12, 0.12, 0.7), f"{x} {y} 0.35 0 0 0", (0.7, 0.67, 0.6))
                for i, (x, y) in enumerate([(0.6, 0.2), (0.6, -0.2), (-0.6, 0.2), (-0.6, -0.2)]))),
        static=False),
}


def planting_sdf(kind: str, name: str, x: float, y: float, z: float, yaw: float = 0.0) -> str:
    """SDF for something left behind by the planting head. Visual only, no collision."""
    pose = f"<pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.3f}</pose>"
    if kind == "sapling":
        inner = (_vis("disc", cyl(0.25, 0.03), "0 0 0.015 0 0 0", SOIL) +
                 _vis("stem", cyl(0.014, 0.48), "0 0 0.24 0 0 0", BARK) +
                 _vis("c1", sph(0.14), "0 0 0.5 0 0 0", LEAF) +
                 _vis("c2", sph(0.1), "0.07 0.04 0.38 0 0 0", LEAF2))
    elif kind == "seedballs":
        inner = ("".join(_vis(f"b{i}", sph(0.04), f"{dx} {dy} 0.03 0 0 0", (0.48, 0.32, 0.21))
                         for i, (dx, dy) in enumerate([(0, 0), (0.12, 0.06), (-0.08, 0.1)])) +
                 _vis("stick", cyl(0.008, 0.5), "-0.15 -0.1 0.25 0 0 0", BARK) +
                 _vis("flag", box(0.13, 0.01, 0.08), "-0.085 -0.1 0.46 0 0 0", (0.88, 0.42, 0.11)))
    elif kind == "halfmoon":
        segs = []
        import math
        for i in range(7):
            a = -math.pi / 2 + i * math.pi / 6
            segs.append(_vis(f"s{i}", box(0.26, 0.12, 0.1), f"{0.45 * math.cos(a):.3f} {0.45 * math.sin(a):.3f} 0.04 "
                                                              f"0 0 {a + math.pi / 2:.3f}", SOIL))
        inner = "".join(segs)
    elif kind == "skip":
        inner = (_vis("cone", "<cone><radius>0.12</radius><length>0.3</length></cone>", "0 0 0.15 0 0 0",
                      (0.88, 0.42, 0.11)))
    else:
        raise ValueError(f"unknown planting kind '{kind}'")
    return (f"<?xml version='1.0'?><sdf version='1.10'><model name='{name}'><static>true</static>{pose}"
            f"<link name='l'>{inner}</link></model></sdf>")


def write_models(models_dir: Path):
    models_dir = Path(models_dir)
    for name, sdf in MODELS.items():
        d = models_dir / name
        d.mkdir(parents=True, exist_ok=True)
        (d / "model.sdf").write_text(sdf)
        (d / "model.config").write_text(
            f"<?xml version='1.0'?>\n<model><name>{name}</name><version>1.0</version>"
            f"<sdf version='1.10'>model.sdf</sdf><description>ForestStark scenery</description></model>\n")
