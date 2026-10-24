"""The site generator writes a consistent world, soil map and survey for each site."""
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

from fs_gazebo.models import MODELS, planting_sdf
from fs_gazebo import climate, regions
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
    generate("open_field", tmp_path, region="ignp_canal")
    layers = {k: sites_read_layer(tmp_path / "open_field.soilmap", k) for k in ("esp", "hardpan", "waterlog")}
    R = regions.REGIONS["ignp_canal"]
    assert (layers["esp"] > 15).mean() == pytest.approx(R.esp_cover + 0.1, abs=0.15)   # patches plus their margins
    assert (layers["hardpan"] < 9).mean() == pytest.approx(R.hardpan_cover, abs=0.05)
    assert (layers["waterlog"] > 0.5).mean() == pytest.approx(R.waterlog_cover, abs=0.08)


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


# ------------------------------------------------------------------ season and ruts



def test_monsoon_wets_the_soil_and_the_dry_season_dries_it():
    for region, soil in (("eastern_plain", "sandy_loam"), ("hadoti_vertisol", "heavy_clay"), ("thar_arid", "dry_sand")):
        dry, wet = climate.moisture_on(region, soil, "05-15"), climate.moisture_on(region, soil, "08-15")
        h = climate.SOIL_WATER[soil]
        assert wet > dry and h.wp * 0.3 <= dry and wet <= h.theta_s, (region, dry, wet)
    # clay holds far more water than sand in the same rains
    assert climate.moisture_on("eastern_plain", "heavy_clay", "08-15") > 3 * climate.moisture_on("eastern_plain", "dry_sand", "08-15")


def test_rainfall_matches_the_normals_over_years():
    for region, (rain, _, _) in climate.CLIMATE.items():
        mean = sum(climate.rainfall(region, 365, s).sum() for s in range(20)) / 20
        assert 0.75 * sum(rain) < mean < 1.25 * sum(rain), (region, mean, sum(rain))


def test_clay_sheds_a_storm_sand_takes_it():
    assert climate.infiltrate(climate.SOIL_WATER["dry_sand"], 0.05, 60) == pytest.approx(60)
    assert climate.infiltrate(climate.SOIL_WATER["heavy_clay"], 0.40, 60) < 15


def test_dose_wetting_depth():
    # 5 L into a 0.5 m basin of dry sand wets ~25 cm (Green-Ampt piston to field capacity)
    assert climate.pit_wetting("dry_sand", 0.03, 5.0, 0.2) == pytest.approx(0.005 / 0.2 / 0.061, rel=1e-6)


def test_site_moisture_follows_the_date_and_ruts_persist(tmp_path):
    a = generate("open_field", tmp_path / "a", region="eastern_plain", date="05-01")
    b = generate("open_field", tmp_path / "b", region="eastern_plain", date="08-15")
    m = lambda d: float(np.mean(sites_read_layer(d / "open_field.soilmap", "moisture")))
    assert m(tmp_path / "b") > m(tmp_path / "a") + 0.05
    (tmp_path / "c").mkdir()
    (tmp_path / "c" / "hill_slope.soilmap.ruts").write_text("cell 0.1\n" + "".join(f"{x / 10:.2f} 0.05 0.06\n" for x in range(-20, 20)))
    generate("hill_slope", tmp_path / "c")
    flat = generate("hill_slope", tmp_path / "d")
    sdf = (tmp_path / "c" / "hill_slope.sdf").read_text()
    assert "hill_slope_ruts.obj" in sdf and "hill_slope_ruts.obj" not in (tmp_path / "d" / "hill_slope.sdf").read_text()
    e_c = sites_read_layer(tmp_path / "c" / "hill_slope.soilmap", "elevation")
    e_d = sites_read_layer(tmp_path / "d" / "hill_slope.soilmap", "elevation")
    assert 0.01 < float((e_d - e_c).max()) <= 0.06 + 1e-6


def sites_read_layer(path, name):
    lines = path.read_text().splitlines()
    nx, ny = next(map(int, l.split()[1:3]) for l in lines if l.startswith("size"))
    k = lines.index(f"layer {name}")
    return np.array([[float(v) for v in lines[k + 1 + r].split()] for r in range(ny)])
