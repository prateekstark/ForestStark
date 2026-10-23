"""Unit tests for the fs_autonomy library (no ROS needed)."""
import math

import numpy as np
import pytest

from fs_autonomy import ekf, geometry, head_model, ik, planners, stability, terramechanics as tm
from fs_autonomy.site import Site

SAND = {"name": "dry_sand", "n": 1.1, "kc": 0.99, "kphi": 1528.43, "c": 1.04, "phi": 28.0, "K": 0.025, "eb": 2.5}
LOAM = {"name": "sandy_loam", "n": 0.9, "kc": 52.53, "kphi": 1127.97, "c": 4.83, "phi": 20.0, "K": 0.025, "eb": 4.0}


@pytest.fixture(scope="module")
def sites(tmp_path_factory):
    from fs_gazebo.sites import generate
    d = tmp_path_factory.mktemp("sites")
    for s in ("roadside", "open_field", "hill_slope"):
        generate(s, d)
    return {s: Site(d / f"{s}.yaml") for s in ("roadside", "open_field", "hill_slope")}


# ------------------------------------------------------------------ geometry
def test_polyline_projection_and_offset():
    p = geometry.Polyline([(0, 0), (10, 0), (10, 10)])
    assert p.length == pytest.approx(20)
    s, lat, _ = p.project(5, 2)
    assert s == pytest.approx(5) and lat == pytest.approx(2)
    s, lat, _ = p.project(12, 5)
    assert s == pytest.approx(15) and lat == pytest.approx(-2)
    off = p.offset(1.0)
    assert off.project(5, 1)[1] == pytest.approx(0, abs=1e-9)


def test_hull_and_polygon():
    h = geometry.convex_hull([(0, 0), (1, 0), (1, 1), (0, 1), (0.5, 0.5)])
    assert len(h) == 4
    assert geometry.point_in_polygon(0.5, 0.5, h)
    assert not geometry.point_in_polygon(1.5, 0.5, h)


# ------------------------------------------------------------------ terramechanics (matches C++)
def test_sinkage_matches_cpp_value():
    assert tm.sinkage(SAND, 3000, 0.28, 1.1) == pytest.approx(0.01007, rel=0.01)


def test_drawbar_and_slip():
    loads = [3000] * 4
    assert tm.drawbar_pull(LOAM, loads, 0.28, 1.1, 0.0) < 0           # standing still: only resistance
    assert tm.drawbar_pull(LOAM, loads, 0.28, 1.1, 0.3) > 0
    i = tm.slip_for_pull(LOAM, loads, 0.28, 1.1, 2000)
    assert 0 < i < 0.3
    assert tm.slip_for_pull(LOAM, loads, 0.28, 1.1, 1e6) == 1.0
    g = tm.max_gradeability_deg(SAND, 1200, 12, 0.28, 0.37)
    assert 15 < g < 35


# ------------------------------------------------------------------ stability
def test_force_angle_flat_and_tilted():
    contacts = np.array([[0.65, 0.64, 0], [0.65, -0.64, 0], [-0.65, 0.64, 0], [-0.65, -0.64, 0]])
    cg = np.array([0, 0, 0.75])
    _, m = stability.force_angle_margins(contacts, cg, [0, 0, -1000])
    assert min(m) == pytest.approx(math.atan2(0.64, 0.75), abs=1e-6)
    # Same machine on a 20 deg side slope (rotate gravity): the downhill edge loses 20 deg.
    th = math.radians(20)
    _, m2 = stability.force_angle_margins(contacts, cg, [0, -1000 * math.sin(th), -1000 * math.cos(th)])
    assert min(m2) == pytest.approx(math.atan2(0.64, 0.75) - th, abs=1e-6)


def test_urdf_mass_parse():
    xml = "<robot name='r'><link name='a'><inertial><origin xyz='1 0 0'/><mass value='5'/></inertial></link>" \
          "<link name='b'/></robot>"
    m = stability.parse_urdf_masses(xml)
    assert m["a"][0] == 5 and m["a"][1][0] == 1 and "b" not in m


# ------------------------------------------------------------------ IK
@pytest.mark.parametrize("target", [(0.3, 1.25, -0.35), (0.6, -1.2, 0.0), (0.25, 1.0, 0.3)])
def test_ik_round_trip(target):
    q = ik.solve(target, clearance=0.08)
    x, y, z = ik.forward(q)
    assert (x, y, z) == pytest.approx((target[0], target[1], target[2] + 0.08), abs=1e-6)
    assert q[1] + q[2] + q[3] == pytest.approx(math.pi / 2)


def test_ik_unreachable():
    with pytest.raises(ik.Unreachable):
        ik.solve((0.25, 3.0, -1.0))


# ------------------------------------------------------------------ EKF
def test_ekf_converges_with_gnss_and_rejects_outlier():
    rng = np.random.default_rng(0)
    f = ekf.PoseEKF(0, 0, 0.3, lever=(1.0, 0.0))
    truth = np.array([0.0, 0.0])
    yaw = 0.0
    for k in range(400):
        truth += 0.05 * np.array([math.cos(yaw), math.sin(yaw)])
        f.predict(0.05)
        f.update_odom(1.0 * 1.05, 0.0)        # 5 % slip in the odometry
        if k % 4 == 0:
            ant = truth + np.array([1.0, 0.0])
            f.update_gnss(*(ant + rng.normal(0, 0.02, 2)), sigma=0.02)
    x, y, th = f.pose
    assert math.hypot(x - truth[0], y - truth[1]) < 0.05
    assert abs(th) < math.radians(2)
    assert not f.update_gnss(x + 5.0, y, sigma=0.02)   # 5 m jump is gated


def test_ekf_follows_crab_motion_and_recovers_from_divergence():
    rng = np.random.default_rng(1)
    f = ekf.PoseEKF(0, 0, 0.0)
    truth = np.array([0.0, 0.0])
    for k in range(300):                       # crab sideways at 0.5 m/s, heading fixed
        truth += np.array([0.0, 0.5 * 0.05])
        f.predict(0.05)
        f.update_odom(0.0, 0.0, vy=0.5)
        if k % 4 == 0:
            f.update_gnss(*(truth + rng.normal(0, 0.02, 2)), sigma=0.02)
    assert math.hypot(f.pose[0] - truth[0], f.pose[1] - truth[1]) < 0.05
    f.x[1] -= 20.0                             # corrupt the estimate: gate rejects, then re-centres
    for k in range(10):
        f.predict(0.05)
        f.update_gnss(*(truth + rng.normal(0, 0.02, 2)), sigma=0.02)
    assert f.resets >= 1 and abs(f.pose[1] - truth[1]) < 0.2


# ------------------------------------------------------------------ head model
def test_head_model_orders_soils():
    a = head_model.predict("jaal", LOAM, 0.15, depth=0.25)
    b = head_model.predict("jaal", {**LOAM, "eb": 22.0}, 0.15, depth=0.25)
    assert b.bore_s > a.bore_s and 2 < a.total_s < 20


# ------------------------------------------------------------------ planners on generated sites
def test_roadside_survey_skips_hidden_cable(sites):
    s = sites["roadside"]
    surveyed = planners.roadside_spots(s, 3.0, surveyed=True)
    blind = planners.roadside_spots(s, 3.0, surveyed=False)
    assert len(surveyed) == len(blind) > 90
    assert not any(sp.hidden_hazard for sp in surveyed if not sp.skip)
    assert any(sp.hidden_hazard == "telecom cable" and not sp.skip for sp in blind)
    assert any("culvert" in sp.note for sp in surveyed)


def test_grid_split_balanced(sites):
    s = sites["open_field"]
    rows, dy = planners.grid_rows(s["field"]["polygon"], 3.0, "square")
    spots = planners.resolve_obstacles(rows, s["obstacles"])
    blocks = planners.split_rows(spots, 3)
    counts = [sum(1 for j in b for sp in spots[j] if not sp.skip) for b in blocks]
    assert len(blocks) == 3 and min(counts) > 0
    assert max(counts) - min(counts) <= len(spots[0]) + 1
    flat = [sp for r in spots for sp in r]
    assert any(sp.note.startswith("shifted") for sp in flat)
    tri, dyt = planners.grid_rows(s["field"]["polygon"], 3.0, "triangular")
    assert sum(len(r) for r in tri) > sum(len(r) for r in rows)          # 1,283 vs 1,111 per ha


def test_contour_rows_are_level(sites):
    s = sites["hill_slope"]
    rows, spots = planners.contour_plan(s, n_rows=2)
    assert len(rows) == 2 and len(spots) > 20
    sm = s.soil_map
    for line in rows:
        h = [float(sm.elevation(x, y)) for x, y in line.p[::8]]
        assert np.std(h) < 0.35                 # follows the contour (boulder detours allowed)
    tools = {sp.tool for sp in spots}
    assert "auger" in tools and len(tools) >= 2


@pytest.mark.parametrize("gap", [3.0, 4.0, 8.0])
def test_u_turn_respects_turning_radius(gap):
    p = planners.u_turn((0, 0), 0.3, (gap * math.sin(0.3), -gap * math.cos(0.3)), 3.4)
    assert p[-1] == pytest.approx((gap * math.sin(0.3), -gap * math.cos(0.3)))
    assert 1 / planners.max_curvature(p) >= 3.4 - 0.05


def test_downhill_spots_are_below_the_row(sites):
    s = sites["hill_slope"]
    rows, spots = planners.contour_plan(s, n_rows=1, side="downhill")
    sm = s.soil_map
    below = [sm.elevation(sp.x, sp.y) < sm.elevation(*rows[0].at(sp.station)[:2]) for sp in spots]
    assert sum(below) > 0.9 * len(below)


@pytest.mark.parametrize("ahead", [-0.8, 0.0, 0.8])
def test_u_turn_absorbs_along_track_offset(ahead):
    p = planners.u_turn((0, 0), 0.0, (ahead, -4.0), 3.4)
    assert p[-1] == pytest.approx((ahead, -4.0))
    assert 1 / planners.max_curvature(p) >= 3.4 - 0.05


@pytest.mark.parametrize("target", [(-0.30, 0.15, 0.715), (-0.40, -0.23, 0.715), (0.35, 1.25, -0.40), (0.6, -1.1, 0.1)])
def test_gripper_ik_round_trip(target):
    q = ik.solve_gripper(target)
    assert ik.forward_gripper(q) == pytest.approx(target, abs=1e-6)
    assert q[1] + q[2] + q[3] == pytest.approx(0.0, abs=1e-9)
