"""The site generator writes a consistent world, soil map and survey for each site."""
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

from fs_gazebo.models import MODELS, planting_sdf
from fs_gazebo import regions
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
    layers = [ln.split()[1] for ln in lines if ln.startswith("layer")]
    assert layers[:4] == ["elevation", "soil", "rock_depth", "moisture"]
    assert len(lines) > 4 * ny
    assert data["origin"] and data["region"]


@pytest.mark.parametrize("region", sorted(regions.REGIONS))
def test_every_region_generates_its_sites(region, tmp_path):
    R = regions.REGIONS[region]
    for site in R.sites:
        out = tmp_path / site
        data = generate(site, out, region=region)
        assert data["region"] == region and data["origin"]["lat"] == pytest.approx(R.origin[0])
        text = (out / f"{site}.soilmap").read_text()
        layers = [ln.split()[1] for ln in text.splitlines() if ln.startswith("layer")]
        assert ("hardpan" in layers) == (R.hardpan is not None) and ("esp" in layers) == (R.esp is not None)
        ET.parse(out / f"{site}.sdf")


def test_region_layers_hit_their_cover(tmp_path):
    from fs_autonomy.site import SoilMap
    generate("open_field", tmp_path, region="ignp_canal")
    sm = SoilMap(tmp_path / "open_field.soilmap")
    R = regions.REGIONS["ignp_canal"]
    assert (sm.layers["esp"] > 15).mean() == pytest.approx(R.esp_cover + 0.1, abs=0.15)   # patches plus their margins
    assert (sm.layers["hardpan"] < 9).mean() == pytest.approx(R.hardpan_cover, abs=0.05)
    assert (sm.layers["waterlog"] > 0.5).mean() == pytest.approx(R.waterlog_cover, abs=0.08)


def test_unknown_region_or_site_is_refused(tmp_path):
    with pytest.raises(ValueError):
        generate("hill_slope", tmp_path, region="thar_arid")


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
