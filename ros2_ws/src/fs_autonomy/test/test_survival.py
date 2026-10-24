"""First-year survival: the regional methods must beat a plain pit where the sources say they do, for the reasons they give."""
import dataclasses

import pytest
from fs_autonomy import silviculture as sv
from fs_autonomy import survival as su


def case(region, microsite=None):
    for name, kind, s in su.CASES:
        p = sv.prescribe(name, kind, s, 0, True)
        if name == region and (microsite is None or p.microsite == microsite):
            return kind, s, p
    raise KeyError(region)


@pytest.mark.parametrize("region", ["thar_arid", "ignp_canal", "hadoti_vertisol", "aravalli_hills", "eastern_plain"])
def test_prescribed_method_beats_a_plain_pit(region):
    kind, s, p = case(region)
    good = su.predict(region, p.species, p.method, s)
    plain = su.predict(region, p.species, su.plain_pit(), s)
    assert good.survival > plain.survival + 0.08, (good.survival, plain.survival)
    assert 0.6 < good.survival < 0.95


def test_sodic_land_needs_the_auger_hole_through_the_pan_and_gypsum():
    kind, s, p = case("ignp_canal", "strongly sodic, kankar pan")
    full = su.predict("ignp_canal", p.species, p.method, s)
    shallow = su.predict("ignp_canal", p.species, dataclasses.replace(p.method, pit_depth=0.55), s)
    no_gypsum = su.predict("ignp_canal", p.species, dataclasses.replace(p.method, backfill=()), s)
    assert full.survival > shallow.survival + 0.2 and full.root_depth > shallow.root_depth + 0.3
    assert full.survival > no_gypsum.survival


def test_mound_saves_a_plant_on_waterlogged_ground():
    kind, s, p = case("ignp_canal", "waterlogged")
    mound = su.predict("ignp_canal", p.species, p.method, s)
    flat = su.predict("ignp_canal", p.species, dataclasses.replace(p.method, catchment="none", plant_depth=0.0), s)
    assert mound.survival > flat.survival + 0.2 and flat.risk == "waterlog"


def test_thar_basin_and_hydrogel_carry_the_plant_into_the_dry_season():
    kind, s, p = case("thar_arid", "sandy plain")
    full = su.predict("thar_arid", p.species, p.method, s)
    bare = su.predict("thar_arid", p.species, dataclasses.replace(p.method, catchment="none", backfill=(), mulch="none"), s)
    assert full.survival > bare.survival and full.dry_days < bare.dry_days


def test_tolerant_species_survive_where_others_do_not():
    kind, s, p = case("thar_arid", "sandy plain")
    assert su.predict("thar_arid", "Prosopis cineraria", p.method, s).survival > \
        su.predict("thar_arid", "Dalbergia latifolia", p.method, s).survival + 0.1
    salty = dict(s, esp=40.0)
    assert su.predict("thar_arid", "Salvadora persica", p.method, salty).survival > \
        su.predict("thar_arid", "Azadirachta indica", p.method, salty).survival


def test_what_the_robot_achieved_counts():
    kind, s, p = case("thar_arid", "sandy plain")
    full = su.predict("thar_arid", p.species, p.method, s)
    dry = su.predict("thar_arid", p.species, p.method, s, water_l=1.0, hole_depth=0.2)
    assert full.survival > dry.survival


def test_every_prescribed_species_has_traits():
    for r in sv.REGIONS.values():
        for rule in r.rules:
            for sp in rule.species:
                assert sp in su.SPECIES, sp


def test_report_lists_every_case():
    text = su.report()
    assert text.count("\n| ") == len(su.CASES) + 1
