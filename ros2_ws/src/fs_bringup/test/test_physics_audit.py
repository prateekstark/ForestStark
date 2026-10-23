"""Physics audit of the models Gazebo loads: CAD inertials, stability, sinkage, traction, arm torque."""
import numpy as np
import pytest
from fs_bringup import physics_audit as pa


@pytest.mark.parametrize("kind", sorted(pa.WHEELS))
def test_no_check_fails(kind):
    checks = pa.audit(kind)
    assert {c.name for c in checks} >= {"inertials", "mass and CoM", "static stability", "gradeability"}
    failed = [f"{c.name}: {c.detail}" for c in checks if c.status == "FAIL"]
    assert not failed, failed


def test_least_norm_loads_balance_weight_and_moments():
    pts = [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)]
    f = pa.least_norm_loads(pts, 1000.0, np.array([0.5, 0.0, 1.0]))
    assert f.sum() == pytest.approx(1000.0)
    assert sum(fi * p[0] for fi, p in zip(f, pts)) == pytest.approx(500.0)
    assert f[0] == pytest.approx(f[1]) and f[0] > f[2]


def test_tip_angle_of_a_box():
    edges = pa.tip_angles([(1.0, 0.5), (1.0, -0.5), (-1.0, 0.5), (-1.0, -0.5)], np.zeros(3), 0.5)
    angles = sorted(round(a, 1) for a, _ in edges)
    assert angles == [45.0, 45.0, 63.4, 63.4]
