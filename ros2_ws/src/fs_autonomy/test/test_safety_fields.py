"""The certified layer's fields and PLC logic, and how the comfort layer sits above it."""
import numpy as np
import pytest
from fs_autonomy import safety, safety_fields as sf


def test_protective_length_is_iso13855_style():
    ch = sf.StopChain(scanner_s=0.1, plc_s=0.02, drive_s=0.15, decel=0.8, z=0.2)
    assert sf.protective_length(1.0, ch) == pytest.approx(1.0 * 0.27 + 1.1 / 1.6 + 0.2)
    assert sf.protective_length(0.0, ch) == pytest.approx(0.2)


@pytest.mark.parametrize("kind", ["jaal", "pahadi"])
def test_comfort_layer_always_stops_before_the_protective_field(kind):
    cfg = safety.SafetyConfig.for_robot(kind)
    for s in sf.field_sets(kind):
        v = s.band
        # a static object dead ahead, just outside the protective field: the comfort layer has already stopped
        ahead = cfg.footprint.xmax + sf.protective_length(v, sf.CHAINS[kind]) + 0.05
        tr = type("T", (), {"moving": False, "kind": "object", "r": 0.1, "id": 1})()
        A = safety.assess([(tr, ahead + 0.1, 0.0, 0.0, 0.0)], (v, 0.0, 0.0), cfg)
        assert A.scale == 0.0, (kind, v, A.reason)


def test_plc_selects_band_and_direction_and_trips_on_its_field():
    plc = sf.PlcModel("jaal")
    k, name, prot, _ = plc.select(0.7, 0.0)
    assert name == "+x_0.8"
    assert plc.select(0.0, -0.25)[1] == "-y_0.3"
    assert plc.select(0.0, 0.0)[1] == "standstill"
    fp = safety.FOOTPRINTS["jaal"]
    inside_pt = [[fp.xmax + 0.5, 0.0]]
    s = plc.update(0.0, inside_pt, 0.7, 0.0)
    assert s.protective_stop and s.sto_drive
    s = plc.update(0.5, [], 0.0, 0.0)
    assert s.protective_stop                       # held for the reset time
    s = plc.update(1.6, [], 0.0, 0.0)
    assert not s.protective_stop
    assert plc.update(2.0, [[fp.xmax + 1.0, 0.0]], 0.3, 0.0).warning and not plc.state.protective_stop


def test_plc_tool_zone_and_estop():
    plc = sf.PlcModel("pahadi")
    s = plc.update(0.0, [[2.0, 1.0]], 0.0, 0.0, tool_xy=(1.5, 1.2), tool_active=True)
    assert s.tool_zone and s.sto_tool
    s = plc.update(0.1, [[2.0, 1.0]], 0.0, 0.0, tool_xy=(1.5, 1.2), tool_active=False)
    assert not s.tool_zone
    s = plc.update(0.2, [], 0.0, 0.0, estop=True)
    assert s.estop and s.sto_drive and s.sto_tool


def test_katar_guards_the_tool_only():
    assert sf.field_sets("katar") == []
    assert safety.SafetyConfig.for_robot("katar").chain is None
