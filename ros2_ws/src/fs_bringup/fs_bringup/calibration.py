"""Calibrate the models against measurements from the real machine and site.

    ros2 run fs_bringup calibrate weights  --robot jaal --csv wheel_scales.csv
    ros2 run fs_bringup calibrate plate    --soil sandy_loam --csv plate_sinkage.csv
    ros2 run fs_bringup calibrate shear    --soil sandy_loam --csv shear_ring.csv
    ros2 run fs_bringup calibrate vane     --soil sandy_loam --csv vane_moisture.csv
    ros2 run fs_bringup calibrate pull     --robot jaal --soil sandy_loam --csv drawbar.csv
    ros2 run fs_bringup calibrate survival --plants ~/.ros/fs_logs/<run>/plants.csv --counts survey.csv

Every result goes to the calibration directory ($FS_CALIBRATION_DIR, default ~/.ros/fs_calibration),
which the rest of the stack reads on the next launch:
  <robot>_mass.yaml   mass scale and CoM shift   -> the URDF Gazebo loads (sim.apply_inertials), the audit
  soils.yaml          fitted soil constants      -> the soil maps (sites.py), Gazebo systems, autonomy, audit
  survival.yaml       hazard scale per region    -> fs_autonomy.survival

Tests and their CSV columns:
  weights   one row per wheel / track segment on a scale: link, kg            (machine level, running order)
  plate     Bekker plate-sinkage, two or more plate widths: b_m, z_m, p_kpa   -> n, kc, kphi
  shear     shear ring or annular bevameter at several normal loads: sigma_kpa, j_m, tau_kpa -> c, phi, K
  vane      field shear vane against moisture (TDR probe beside it): moisture, tau_kpa -> ref moisture, softening
  pull      drawbar pull on site: slip, pull_n                               (compared with the model; reports fit)
  survival  field count a year on: id (the plant record id), robot, alive (1/0)
"""
from __future__ import annotations

import argparse
import csv
import math
import re
from pathlib import Path

import numpy as np
import yaml

G = 9.81


def calibration_dir() -> Path:
    from fs_gazebo.sites import calibration_dir as d
    return d()


def _merge(path: Path, key: str, values: dict):
    data = yaml.safe_load(path.read_text()) if path.exists() else {}
    data = data or {}
    data.setdefault(key, {}).update(values)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, sort_keys=True))
    return data[key]


def read_csv(path) -> list:
    with open(path, newline="") as f:
        return [{k.strip(): v.strip() for k, v in row.items()} for row in csv.DictReader(f)]


# ---------------------------------------------------------------- mass and centre of mass
def mass_from_weights(points: dict, kg: dict, model_mass: float, model_com) -> dict:
    """Whole-machine mass and CoM from wheel scales (CoM = load-weighted contact positions, valid for a
    tractor-trailer too: the hitch force is internal), as a scale on the model's masses and a shift of
    its CoM in the horizontal plane."""
    W = np.array([kg[n] for n in points])
    P = np.array([points[n][:2] for n in points])
    m = float(W.sum())
    c = (W[:, None] * P).sum(axis=0) / m
    return {"mass_scale": m / model_mass, "com_shift": [float(c[0] - model_com[0]), float(c[1] - model_com[1])],
            "measured_kg": m, "measured_com": [float(c[0]), float(c[1])]}


def apply_mass(links: dict, cal: dict, base: str = "base_link") -> dict:
    """links: name -> dict(m, c (3), I (3x3)) in link frames (base_link frame for the base). Masses and
    inertias scale; the base link's CoM moves so the whole machine's CoM moves by com_shift."""
    k = float(cal.get("mass_scale", 1.0))
    dx, dy = cal.get("com_shift", [0.0, 0.0])
    out = {n: dict(m=v["m"] * k, c=np.array(v["c"], float), I=np.array(v["I"], float) * k) for n, v in links.items()}
    total = sum(v["m"] for v in out.values())
    if base in out and out[base]["m"] > 0:
        out[base]["c"] = out[base]["c"] + np.array([dx, dy, 0.0]) * total / out[base]["m"]
    return out


# ---------------------------------------------------------------- soil
def fit_plate(rows) -> dict:
    """Bekker p = (kc/b + kphi) z^n from plate tests at two or more widths: n from the slope of log p
    on log z (common to all plates), then kc, kphi by least squares on the plate moduli."""
    b = np.array([float(r["b_m"]) for r in rows])
    z = np.array([float(r["z_m"]) for r in rows])
    p = np.array([float(r["p_kpa"]) for r in rows])
    widths = sorted(set(b))
    if len(widths) < 2:
        raise ValueError("plate test needs at least two plate widths to separate kc from kphi")
    # common n, one intercept per plate: log p = n log z + log k_b
    A = np.column_stack([np.log(z)] + [(b == w).astype(float) for w in widths])
    sol, *_ = np.linalg.lstsq(A, np.log(p), rcond=None)
    n, kb = float(sol[0]), np.exp(sol[1:])
    M = np.column_stack([1.0 / np.array(widths), np.ones(len(widths))])
    (kc, kphi), *_ = np.linalg.lstsq(M, kb, rcond=None)
    return {"n": round(n, 4), "kc": round(float(kc), 3), "kphi": round(float(kphi), 2)}


def fit_shear(rows) -> dict:
    """Janosi-Hanamoto tau = (c + sigma tan phi)(1 - e^(-j/K)): K by a 1-D search, then c and tan phi
    by linear least squares (tau is linear in them for a given K)."""
    s = np.array([float(r["sigma_kpa"]) for r in rows])
    j = np.array([float(r["j_m"]) for r in rows])
    t = np.array([float(r["tau_kpa"]) for r in rows])
    best = None
    for K in np.geomspace(0.002, 0.1, 400):
        f = 1 - np.exp(-j / K)
        A = np.column_stack([f, s * f])
        sol, *_ = np.linalg.lstsq(A, t, rcond=None)
        err = float(((A @ sol - t) ** 2).sum())
        if best is None or err < best[0]:
            best = (err, K, sol)
    _, K, (c, tphi) = best
    return {"c": round(float(c), 3), "phi": round(math.degrees(math.atan(tphi)), 2), "K": round(float(K), 4)}


def fit_vane(rows) -> dict:
    """Strength against moisture: tau = tau_ref f(theta) with fs::soil::StrengthFactor's shape
    (exponential softening above theta_ref, linear hard-setting below, 0.3..1.4)."""
    th = np.array([float(r["moisture"]) for r in rows])
    tau = np.array([float(r["tau_kpa"]) for r in rows])

    def f(theta, ref, s):
        d = theta - ref
        return np.where(d > 0, np.maximum(0.3, np.exp(-s * d)), np.minimum(1.4, 1 - 0.5 * s * d))
    best = None
    for ref in np.linspace(0.03, 0.40, 75):
        for s in np.linspace(0.0, 15.0, 151):
            g = f(th, ref, s)
            tref = float((g * tau).sum() / (g * g).sum())
            err = float(((tref * g - tau) ** 2).sum())
            if best is None or err < best[0]:
                best = (err, ref, s, tref)
    _, ref, s, tref = best
    return {"ref_moisture": round(float(ref), 3), "wet_softening": round(float(s), 2), "vane_tau_ref_kpa": round(tref, 2)}


def soil_from_table(name: str) -> dict:
    """The soil's current constants (calibrated where they have been)."""
    from fs_gazebo.sites import soil_rows
    for row in soil_rows():
        if row[0] == name:
            keys = ("n", "kc", "kphi", "c", "phi", "K", "eb", "ref_moisture", "wet_softening", "capillary_c", "true_cohesion")
            return dict(zip(keys, row[1:]), name=name)
    raise KeyError(name)


def pull_check(soil: dict, loads, b: float, l: float, rows) -> dict:
    """Measured drawbar pull against the model's at the same slips (fs_autonomy.terramechanics)."""
    from fs_autonomy import terramechanics as tm
    slip = np.array([float(r["slip"]) for r in rows])
    meas = np.array([float(r["pull_n"]) for r in rows])
    pred = np.array([tm.drawbar_pull(soil, loads, b, l, i) for i in slip])
    scale = float((pred * meas).sum() / max((pred * pred).sum(), 1e-9))
    return {"rms_error_n": round(float(np.sqrt(((pred - meas) ** 2).mean())), 1),
            "measured_over_model": round(scale, 3), "peak_measured_n": round(float(meas.max()), 1),
            "peak_model_n": round(float(pred.max()), 1)}


# ---------------------------------------------------------------- survival
def fit_survival(plants, counts) -> dict:
    """Per region, the scale m on the model's hazards that matches the counted survival:
    observed = mean(pred_i ** m)  (pred = exp(-H), so the hazard H becomes m H), solved by bisection."""
    alive = {(r.get("robot", ""), int(r["id"])): int(r["alive"]) for r in counts}
    by_region = {}
    for r in plants:
        key = (r.get("robot", ""), int(r["id"]))
        if r.get("outcome") != "planted" or key not in alive or not r.get("survival_1y"):
            continue
        by_region.setdefault(r["region"], []).append((max(min(float(r["survival_1y"]), 0.999), 1e-3), alive[key]))
    out = {}
    for region, rows in by_region.items():
        pred = np.array([p for p, _ in rows])
        obs = float(np.mean([a for _, a in rows]))
        lo, hi = 0.05, 20.0
        for _ in range(60):
            mid = math.sqrt(lo * hi)
            if np.mean(pred ** mid) > obs:
                lo = mid
            else:
                hi = mid
        out[region] = {"hazard_scale": round(math.sqrt(lo * hi), 3), "plants": len(rows),
                       "observed": round(obs, 3), "model": round(float(pred.mean()), 3)}
    return out


# ---------------------------------------------------------------- CLI
def _model(robot: str):
    from fs_bringup import physics_audit as pa
    from fs_bringup import sim
    xml = sim.robot_description(robot, robot, Path("/tmp/none.soilmap"), lidar=False)
    m = pa.Model(xml)
    contacts = [n for n in m.links if re.search(pa.WHEELS[robot]["contacts"], n)]
    pts = {n: m.T(n)[:3, 3] for n in contacts}
    return m, pts


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("test", choices=["weights", "plate", "shear", "vane", "pull", "survival"])
    ap.add_argument("--robot", default="jaal")
    ap.add_argument("--soil", default="sandy_loam")
    ap.add_argument("--csv", default="")
    ap.add_argument("--plants", default="")
    ap.add_argument("--counts", default="")
    ap.add_argument("--dir", default="")
    a, _ = ap.parse_known_args()
    d = Path(a.dir) if a.dir else calibration_dir()
    if a.test == "weights":
        m, pts = _model(a.robot)
        kg = {r["link"]: float(r["kg"]) for r in read_csv(a.csv)}
        missing = [n for n in pts if n not in kg]
        if missing:
            raise SystemExit(f"no scale reading for {', '.join(missing)}")
        mass, c = m.com()
        res = mass_from_weights(pts, kg, mass, c)
        # the model already carries any earlier calibration: compose with it
        path = d / f"{a.robot}_mass.yaml"
        old = (yaml.safe_load(path.read_text()) or {}) if path.exists() else {}
        res["mass_scale"] *= float(old.get("mass_scale", 1.0))
        res["com_shift"] = [x + y for x, y in zip(res["com_shift"], old.get("com_shift", [0.0, 0.0]))]
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(yaml.safe_dump(res, sort_keys=True))
        print(f"{a.robot}: measured {res['measured_kg']:,.0f} kg (model {mass:,.0f}), CoM shift "
              f"{res['com_shift'][0]:+.3f} {res['com_shift'][1]:+.3f} m -> {d / f'{a.robot}_mass.yaml'}")
    elif a.test in ("plate", "shear", "vane"):
        res = {"plate": fit_plate, "shear": fit_shear, "vane": fit_vane}[a.test](read_csv(a.csv))
        res.pop("vane_tau_ref_kpa", None)
        merged = _merge(d / "soils.yaml", a.soil, res)
        print(f"{a.soil}: {res} -> {d / 'soils.yaml'} ({len(merged)} constants set)")
    elif a.test == "pull":
        from fs_bringup import physics_audit as pa
        m, pts = _model(a.robot)
        mass, c = m.com()
        loads = pa.least_norm_loads([p[:2] for p in pts.values()], mass * G, c)
        r, w = m.radius(next(iter(pts)))
        res = pull_check(soil_from_table(a.soil), loads, w or 0.25, 0.25 * (r or 0.3), read_csv(a.csv))
        print(f"{a.robot} on {a.soil}: {res}")
        if abs(res["measured_over_model"] - 1) > 0.15:
            print("  more than 15 % off: run the shear and plate tests on this ground and calibrate the soil")
    else:
        res = fit_survival(read_csv(a.plants), read_csv(a.counts))
        for region, v in res.items():
            _merge(d / "survival.yaml", region, v)
        print(f"survival: {res} -> {d / 'survival.yaml'}")


if __name__ == "__main__":
    main()
