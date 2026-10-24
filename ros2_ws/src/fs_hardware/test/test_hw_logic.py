"""Hardware-side logic: drive kinematics, the auger's phases, the plug books, the ros2_control description."""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from fs_hardware import description, kinematics as kin
from fs_hardware.auger import Auger, AugerParams
from fs_hardware.inventory import Inventory


@pytest.mark.parametrize("twist", [(0.8, 0.0, 0.0), (0.0, 0.5, 0.0), (0.3, -0.2, 0.4), (0.0, 0.0, -0.6)])
def test_swerve_inverse_then_forward_recovers_the_twist(twist):
    steer = {m: 0.0 for m in kin.JAAL_MODULES}
    for _ in range(3):                       # let the modules turn to their angles
        sol = kin.swerve_inverse(*twist, steer)
        steer = {m: a for m, (a, _) in sol.items()}
    sol = kin.swerve_inverse(*twist, steer)
    v = kin.swerve_forward({m: a for m, (a, _) in sol.items()}, {m: w for m, (_, w) in sol.items()})
    assert v == pytest.approx(twist, abs=1e-9)
    assert all(abs(a) <= 1.9 + 1e-9 for a, _ in sol.values())


def test_swerve_does_not_drive_a_module_still_turning():
    sol = kin.swerve_inverse(0.0, 0.8, 0.0, {m: 0.0 for m in kin.JAAL_MODULES})
    assert all(abs(w) < 1e-6 for _, w in sol.values())       # 90 deg off: wait for the steering


def test_diff_drive_round_trip():
    wl, wr = kin.diff_inverse(0.3, 0.2)
    assert kin.diff_forward(wl, wr) == pytest.approx((0.3, 0.2))


def drive(a, ground=0.2, rock=None, steps=4000, dt=0.01):
    """Feed the auger against a ground at `ground` m of stroke, refusing at `rock` m."""
    q = 0.0
    for _ in range(steps):
        force = 0.0 if q < ground else 2000.0
        stalled = rock is not None and q >= ground + rock
        if stalled:
            force = a.p.max_thrust
        feed, _ = a.step(dt, q, 0.0 if stalled else a.feed_cmd, force, 300.0 if q > ground else 20.0, 12.0)
        q = max(0.0, q + (0.0 if stalled and feed > 0 else feed) * dt)
        if a.phase in ("bored", "refusal", "no_ground", "idle") and a.phase_t > 0.05:
            break
    return q


def test_auger_bores_to_depth_from_ground_contact():
    a = Auger(AugerParams(max_stroke=1.3, max_thrust=8000, spin_torque_limit=800))
    a.command("bore 0.45")
    q = drive(a)
    assert a.phase == "bored" and a.depth == pytest.approx(0.45, abs=0.01) and q == pytest.approx(0.65, abs=0.02)
    s = a.status(q, 2000, 300, 12)
    assert s["phase"] == "bored" and s["energy_kj"] > 0 and s["open_depth"] == s["depth"]
    a.command("retract")
    drive(a)
    assert a.phase == "idle"


def test_auger_refuses_on_rock_and_reports_no_ground():
    a = Auger(AugerParams(max_stroke=1.3, max_thrust=8000))
    a.command("bore 0.5")
    drive(a, rock=0.1)
    assert a.phase == "refusal" and a.depth == pytest.approx(0.1, abs=0.02)
    b = Auger(AugerParams(max_stroke=0.5))
    b.command("bore 0.3")
    drive(b, ground=9.0)
    assert b.phase == "no_ground"


def test_auger_hold_and_resume():
    a = Auger(AugerParams())
    a.command("bore 0.3")
    a.step(0.01, 0.0, 0.0, 0.0, 0.0, 0.0)
    a.command("hold")
    assert a.step(0.01, 0.05, 0.0, 0.0, 0.0, 0.0) == (0.0, 0.0)
    a.command("resume")
    assert a.phase == "lowering" and a.step(0.01, 0.05, 0.0, 0.0, 0.0, 0.0)[0] > 0


def urdf(kind):
    import xacro
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory("fs_description")) / "urdf" / f"{kind}.urdf.xacro"
    return xacro.process_file(str(path), mappings={"ns": kind, "soil_map": "/tmp/none.soilmap", "lidar": "false"}).toxml()


def commanded(urdf_xml):
    """Joints the simulator drives with position controllers: the ones the planting head commands."""
    root = ET.fromstring(urdf_xml)
    return [p.findtext("joint_name") for p in root.iter("plugin") if p.get("name") == "gz::sim::systems::JointPositionController"]


@pytest.mark.parametrize("kind", ["katar", "jaal", "pahadi"])
def test_inventory_reads_the_simulators_carriers(kind):
    inv = Inventory(urdf(kind))
    s = inv.status(0.0)
    assert s["carriers"] and all(c["count"] > 0 and len(c["occ"]) == c["cells"] for c in s["carriers"].values())
    name = next(iter(s["carriers"]))
    holder = next(iter(inv.holders))
    inv.command(f"pick {name} next {holder}")
    s2 = inv.status(1.0)
    assert s2["carriers"][name]["count"] == s["carriers"][name]["count"] - 1 and s2["holders"][holder]
    inv.sensed(holder, False)
    assert not inv.status(2.0)["holders"][holder] and "missed" in inv.status(2.0)["error"]


@pytest.mark.parametrize("kind", ["katar", "jaal", "pahadi"])
def test_description_covers_every_commanded_joint(kind):
    cfg = description.load(kind)
    g = description.groups(cfg)
    cmds = commanded(urdf(kind))
    assert cmds
    for j in cmds:
        assert j in g["position"], j
    assert g["auger"] == ["auger_feed_joint", "auger_spin_joint"]
    nodes = [j["node"] for j in cfg["joints"]]
    assert len(set(nodes)) == len(nodes) and all(1 <= n <= 127 for n in nodes)
    full = description.with_ros2_control(urdf(kind), kind, bus="mock")
    root = ET.fromstring(full)
    rc = root.find("ros2_control")
    assert rc.find("hardware/plugin").text == "fs_hardware/CanopenSystem"
    joints = {j.get("name") for j in root.iter("joint") if j.get("type")}
    assert all(j.get("name") in joints for j in rc.findall("joint"))
    assert "drive_controller" in description.controllers(kind) or kind == "katar"


def test_safety_plc_pdo_round_trip():
    from fs_hardware.safety_bridge import BITS, decode, encode
    st = {k: False for k in BITS} | {"estop": True, "tool_zone": True, "heartbeat_ok": True}
    d = decode(encode(st, field_set=5, alive=200))
    assert d["estop"] and d["tool_zone"] and d["heartbeat_ok"] and not d["protective_stop"]
    assert d["field_set"] == 5 and d["alive"] == 200
