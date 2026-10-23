"""Regional planting prescriptions: the science they encode and what each robot can carry out."""
import pytest
from fs_autonomy import silviculture as sv
from fs_gazebo import regions


def sample(soil="sandy_loam", moisture=0.15, rock=1.0, esp=0.0, hardpan=9.0, waterlog=0.0, elev_frac=0.5):
    return {"soil": {"name": soil}, "moisture": moisture, "rock_depth": rock, "esp": esp, "hardpan": hardpan,
            "waterlog": waterlog, "elev_frac": elev_frac, "elevation": 0.0}


def test_regions_agree_between_site_generator_and_prescriptions():
    assert set(sv.REGIONS) == set(regions.REGIONS)
    for kind, names in sv.ROBOT_REGIONS.items():
        site = {"katar": "roadside", "jaal": "open_field", "pahadi": "hill_slope"}[kind]
        for n in names:
            assert site in regions.REGIONS[n].sites, (kind, n)
            assert site in sv.REGIONS[n].spacing, (kind, n)
    for r in sv.REGIONS.values():
        assert r.rules[-1].when == {}                           # every region has a default microsite
        assert all(k in sv.SOURCES for k in r.sources)


def test_thar_pit_basin_and_sunken_planting_follow_cazri():
    p = sv.prescribe("thar_arid", "jaal", sample("dry_sand", moisture=0.05))
    m = p.method
    assert m.pit_depth == pytest.approx(0.50) and m.catchment == "basin" and m.plant_depth < 0
    assert dict(m.backfill)["hydrogel"] == pytest.approx(0.0035)
    assert sv.water_per_plant(p, 0.05) == pytest.approx(10.0)    # 5 L pre-water into a dry pit + 5 L
    assert len(p.bores) == 3 and p.species in ("Prosopis cineraria", "Acacia senegal", "Capparis decidua")


def test_saline_patch_in_the_thar_gets_salt_tolerant_species_and_gypsum():
    p = sv.prescribe("thar_arid", "jaal", sample("dry_sand", esp=25))
    assert p.microsite == "saline patch" and p.species in ("Salvadora oleoides", "Tamarix aphylla")
    assert "gypsum" in dict(p.method.backfill)


def test_cssri_auger_hole_needs_the_deep_kit():
    s = sample("clayey_soil", esp=40, hardpan=0.7, rock=1.6)
    deep = sv.prescribe("ignp_canal", "katar", s, deep_kit=True)
    assert deep.method.pit_depth == pytest.approx(1.2) and not deep.deviations
    b = dict(deep.method.backfill)
    assert b["gypsum"] == pytest.approx(3.5) and b["FYM"] == pytest.approx(7.5)
    assert "Prosopis juliflora" not in sv.REGIONS["ignp_canal"].rules[1].species     # invasive in Rajasthan
    std = sv.prescribe("ignp_canal", "katar", s, deep_kit=False)
    assert std.method.pit_depth == pytest.approx(0.55) and "deep-auger kit" in std.deviations[0]


def test_waterlogged_ground_is_planted_on_a_mound_where_the_robot_can_make_one():
    s = sample("clayey_soil", waterlog=0.9, moisture=0.3)
    k = sv.prescribe("ignp_canal", "katar", s)
    assert k.method.catchment == "mound" and k.method.plant_depth > 0.2
    j = sv.prescribe("ignp_canal", "jaal", s)
    assert j.method.catchment == "none" and j.method.plant_depth == 0.0 and "mound" in j.deviations[0]


def test_vertisol_pit_is_clamped_to_jaals_reach_and_has_no_basin():
    p = sv.prescribe("hadoti_vertisol", "jaal", sample("heavy_clay", moisture=0.2))
    assert p.method.pit_depth == pytest.approx(0.55) and p.method.catchment == "none"
    assert any("reach" in d for d in p.deviations)


def test_hills_trench_on_aravalli_halfmoon_in_the_himalaya():
    a = sv.prescribe("aravalli_hills", "pahadi", sample("lateritic_gravel", rock=0.8))
    assert a.method.catchment == "trench" and a.species in ("Butea monosperma", "Acacia catechu", "Wrightia tinctoria")
    h = sv.prescribe("himalayan_midhills", "pahadi", sample(elev_frac=0.9, moisture=0.25))
    assert h.method.catchment == "halfmoon" and h.species in ("Cedrus deodara", "Quercus leucotrichophora")
    assert sv.prescribe("himalayan_midhills", "pahadi", sample(moisture=0.35)).species == "Alnus nepalensis"


def test_rock_decides_the_tool_on_the_hills():
    assert sv.prescribe("aravalli_hills", "pahadi", sample("lateritic_gravel", rock=0.08)).method.tool == "seedball"
    assert sv.prescribe("aravalli_hills", "pahadi", sample("lateritic_gravel", rock=0.2)).method.tool == "pick"
    assert len(sv.prescribe("aravalli_hills", "pahadi", sample("lateritic_gravel", rock=0.8)).bores) == 1


def test_bore_clusters_fit_each_machine():
    assert len(sv.bore_offsets("katar", 0.45)) == 5
    jaal = sv.bore_offsets("jaal", 0.5)
    assert all(-0.3 <= x <= 0.0 and y == 0.0 for x, y in jaal)          # along the shuttle only
    assert sv.bore_offsets("katar", 0.15) == ((0.0, 0.0),)


def test_species_are_deterministic_per_spot_and_mixed_along_a_row():
    s = sample("sandy_loam")
    plan = [sv.prescribe("eastern_plain", "jaal", s, i).species for i in range(8)]
    assert plan == [sv.prescribe("eastern_plain", "jaal", s, i).species for i in range(8)]
    assert len(set(plan)) == 4


def test_summary_reads_like_a_field_sheet():
    text = sv.prescribe("thar_arid", "jaal", sample("dry_sand")).summary()
    assert "pit 50 cm deep" in text and "saucer basin" in text and "sunken 8 cm" in text and "5 L" in text


def test_field_sheets_cover_every_region_and_source():
    text = sv.field_sheets()
    assert set(sv.SHORT) == set(sv.SOURCES)
    for r in sv.REGIONS.values():
        assert f"`{r.name}`" in text
        for rule in r.rules:
            assert rule.microsite in text
