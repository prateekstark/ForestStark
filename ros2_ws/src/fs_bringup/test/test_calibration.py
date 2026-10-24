"""Calibration fits recover known parameters from synthetic tests, and what they write reaches the models."""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
from fs_bringup import calibration as cal
from fs_bringup import physics_audit as pa
from fs_bringup import sim

RNG = np.random.default_rng(3)


def test_plate_sinkage_recovers_bekker_constants():
    n, kc, kphi = 0.9, 52.53, 1127.97
    rows = []
    for b in (0.10, 0.20, 0.30):
        for z in np.linspace(0.01, 0.08, 8):
            p = (kc / b + kphi) * z ** n * (1 + 0.01 * RNG.normal())
            rows.append({"b_m": b, "z_m": z, "p_kpa": p})
    fit = cal.fit_plate(rows)
    assert fit["n"] == pytest.approx(n, abs=0.03)
    assert fit["kphi"] == pytest.approx(kphi, rel=0.1) and fit["kc"] == pytest.approx(kc, rel=0.35)
    with pytest.raises(ValueError):
        cal.fit_plate([r for r in rows if r["b_m"] == 0.1])


def test_shear_ring_recovers_c_phi_K():
    c, phi, K = 4.83, 20.0, 0.025
    rows = [{"sigma_kpa": s, "j_m": j, "tau_kpa": (c + s * math.tan(math.radians(phi))) * (1 - math.exp(-j / K))
             + 0.05 * RNG.normal()} for s in (10, 30, 60) for j in np.linspace(0.002, 0.15, 15)]
    fit = cal.fit_shear(rows)
    assert fit["c"] == pytest.approx(c, abs=0.4) and fit["phi"] == pytest.approx(phi, abs=0.7)
    assert fit["K"] == pytest.approx(K, rel=0.15)


def test_vane_recovers_moisture_response():
    ref, s, t0 = 0.22, 7.0, 40.0
    rows = []
    for th in np.linspace(0.08, 0.42, 25):
        d = th - ref
        f = max(0.3, math.exp(-s * d)) if d > 0 else min(1.4, 1 - 0.5 * s * d)
        rows.append({"moisture": th, "tau_kpa": t0 * f * (1 + 0.02 * RNG.normal())})
    fit = cal.fit_vane(rows)
    assert fit["ref_moisture"] == pytest.approx(ref, abs=0.03) and fit["wet_softening"] == pytest.approx(s, abs=1.5)


def test_weights_give_mass_scale_and_com_shift():
    pts = {"a": (1.0, 1.0), "b": (1.0, -1.0), "c": (-1.0, 1.0), "d": (-1.0, -1.0)}
    res = cal.mass_from_weights(pts, {"a": 300, "b": 300, "c": 200, "d": 200}, 900.0, (0.0, 0.0))
    assert res["mass_scale"] == pytest.approx(1000 / 900) and res["com_shift"] == pytest.approx([0.2, 0.0])
    links = {"base_link": dict(m=800.0, c=[0, 0, 0], I=np.eye(3)), "arm": dict(m=100.0, c=[0, 0, 0], I=np.eye(3))}
    out = cal.apply_mass(links, res)
    total = sum(v["m"] for v in out.values())
    com_x = sum(v["m"] * v["c"][0] for v in out.values()) / total
    assert total == pytest.approx(1000.0) and com_x == pytest.approx(0.2)


def test_mass_calibration_reaches_the_urdf(tmp_path, monkeypatch):
    monkeypatch.setenv("FS_CALIBRATION_DIR", str(tmp_path))
    before = pa.Model(sim.robot_description("jaal", "jaal", Path("/tmp/none.soilmap"), lidar=False)).com()
    (tmp_path / "jaal_mass.yaml").write_text("mass_scale: 1.1\ncom_shift: [0.05, 0.0]\n")
    after = pa.Model(sim.robot_description("jaal", "jaal", Path("/tmp/none.soilmap"), lidar=False)).com()
    assert after[0] == pytest.approx(1.1 * before[0], rel=1e-3)
    assert after[1][0] - before[1][0] == pytest.approx(0.05, abs=2e-3)


def test_calibrated_soil_reaches_the_soil_map(tmp_path, monkeypatch):
    from fs_gazebo import sites
    monkeypatch.setenv("FS_CALIBRATION_DIR", str(tmp_path / "cal"))
    (tmp_path / "cal").mkdir()
    (tmp_path / "cal" / "soils.yaml").write_text("sandy_loam: {c: 7.5, wet_softening: 5.5}\n")
    sites.generate("open_field", tmp_path / "site")
    from fs_autonomy.site import SoilMap
    sm = SoilMap(tmp_path / "site" / "open_field.soilmap")
    loam = next(s for s in sm.soils if s["name"] == "sandy_loam")
    assert loam["c"] == 7.5 and loam["wet_softening"] == 5.5 and loam["n"] == 0.9


def test_survival_hazard_scale_matches_the_count():
    plants = [{"robot": "j", "id": str(i), "outcome": "planted", "region": "thar_arid", "survival_1y": "0.8"} for i in range(100)]
    counts = [{"robot": "j", "id": str(i), "alive": "1" if i < 60 else "0"} for i in range(100)]
    res = cal.fit_survival(plants, counts)["thar_arid"]
    assert 0.8 ** res["hazard_scale"] == pytest.approx(0.6, abs=0.01)
