"""Each robot model expands with xacro, converts to SDF, and has the joints and plugins the
autonomy stack and the soil systems refer to."""
import os
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

URDF = Path(__file__).resolve().parents[1] / "urdf"
NEEDS = {
    "katar": {"joints": ["hitch_joint", "sled_joint", "carriage_joint", "auger_feed_joint", "auger_spin_joint",
                         "front_left_steer_joint", "rear_left_wheel_joint"],
              "plugins": ["AckermannSteering", "fs::Terramechanics", "fs::AugerSoil"], "wheels": 8},
    "jaal": {"joints": ["steer_fl_joint", "wheel_rr_joint", "auger_feed_joint"],
             "plugins": ["fs::SwerveDrive", "fs::Terramechanics", "fs::AugerSoil"], "wheels": 4},
    "pahadi": {"joints": ["artic_yaw_joint", "artic_roll_joint", "arm_yaw_joint", "arm_shoulder_joint",
                          "arm_elbow_joint", "arm_wrist_joint", "auger_feed_joint"],
               "plugins": ["DiffDrive", "fs::Terramechanics", "fs::AugerSoil"], "wheels": 12},
}


def expand(robot, tmp_path):
    env = dict(os.environ)
    share = URDF.parent.parent
    env["ROS_PACKAGE_PATH"] = f"{share}:{env.get('ROS_PACKAGE_PATH', '')}"
    out = subprocess.run(["xacro", str(URDF / f"{robot}.urdf.xacro"), "soil_map:=/tmp/none.soilmap", "lidar:=true"],
                         capture_output=True, text=True, env=env, check=True).stdout
    p = tmp_path / f"{robot}.urdf"
    p.write_text(out)
    return p


@pytest.mark.parametrize("robot", sorted(NEEDS))
def test_model(robot, tmp_path):
    urdf = expand(robot, tmp_path)
    root = ET.parse(urdf).getroot()
    joints = {j.get("name") for j in root.iter("joint")}
    for j in NEEDS[robot]["joints"]:
        assert j in joints, f"{robot}: missing joint {j}"
    plugins = [p.get("name") for p in root.iter("plugin")]
    for p in NEEDS[robot]["plugins"]:
        assert any(p in n for n in plugins), f"{robot}: missing plugin {p}"
    tm = next(p for p in root.iter("plugin") if p.get("name") == "fs::Terramechanics")
    assert len(tm.findall("wheel")) == NEEDS[robot]["wheels"]
    sdf = subprocess.run(["gz", "sdf", "-p", str(urdf)], capture_output=True, text=True)
    assert sdf.returncode == 0, sdf.stderr
    assert "<model name" in sdf.stdout
    mass = sum(float(m.get("value")) for m in root.iter("mass"))
    assert 800 < mass < 6000, f"{robot}: total mass {mass} kg is implausible"
