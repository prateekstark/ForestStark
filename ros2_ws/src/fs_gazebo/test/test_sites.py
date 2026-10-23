"""The site generator writes a consistent world, soil map and survey for each site."""
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

from fs_gazebo.models import MODELS, planting_sdf
from fs_gazebo.sites import BUILDERS, generate, hill_height


@pytest.mark.parametrize("site", sorted(BUILDERS))
def test_site_files(site, tmp_path):
    data = generate(site, tmp_path)
    for ext in (".sdf", ".soilmap", ".yaml"):
        assert (tmp_path / f"{site}{ext}").stat().st_size > 1000
    ET.parse(tmp_path / f"{site}.sdf")                      # valid XML
    survey = yaml.safe_load((tmp_path / f"{site}.yaml").read_text())
    assert survey["name"] == site and survey["spawn"]
    lines = (tmp_path / f"{site}.soilmap").read_text().splitlines()
    nx, ny = map(int, next(ln for ln in lines if ln.startswith("size")).split()[1:])
    assert sum(ln.startswith("layer") for ln in lines) == 4
    assert len(lines) > 4 * ny
    assert data["origin"]


def test_hill_heightmap_matches_soil_map(tmp_path):
    generate("hill_slope", tmp_path, slope_deg=18.0)
    png = (tmp_path / "hill_slope_height.png").read_bytes()
    w, h = struct.unpack(">II", png[16:24])
    assert w == h == 257                                   # Gazebo wants 2^n + 1
    world = (tmp_path / "hill_slope.sdf").read_text()
    assert "hill_slope_height.png" in world and "<pos>0 0 0</pos>" in world
    xs = np.linspace(-48, 48, 257)
    X, Y = np.meshgrid(xs, xs)
    H = hill_height(X, Y, 18.0, 23)
    assert np.degrees(np.arctan(np.gradient(H, xs[1] - xs[0], axis=0).mean())) == pytest.approx(18.0, abs=1.5)


def test_roadside_has_hidden_cable_and_visible_culvert(tmp_path):
    s = generate("roadside", tmp_path)
    zones = {z["name"]: z for z in s["roadside"]["no_dig"]}
    assert zones["telecom cable"]["visible"] is False and zones["culvert"]["visible"] is True


def test_models_and_planting_sdf():
    assert {"fs_tree_big", "fs_tender_truck", "fs_cow"} <= set(MODELS)
    for kind in ("sapling", "seedballs", "halfmoon", "skip"):
        ET.fromstring(planting_sdf(kind, "x", 1.0, 2.0, 0.0))
