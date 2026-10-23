"""Launch helpers: every commanded joint exists in its model and the plant-handling topics are bridged."""
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from fs_bringup import sim


@pytest.mark.parametrize("kind", sorted(sim.JOINT_CMDS))
def test_commanded_joints_exist(kind):
    urdf = ET.fromstring(sim.robot_description(kind, kind, Path("/tmp/none.soilmap"), lidar=False))
    joints = {j.get("name") for j in urdf.iter("joint")}
    missing = [j for j in sim.JOINT_CMDS[kind] if j not in joints]
    assert not missing, missing


@pytest.mark.parametrize("kind", sorted(sim.JOINT_CMDS))
def test_plug_topics_bridged(kind):
    entries = sim.bridge_entries(kind, "r1", lidar=False)
    by_topic = {e["ros_topic_name"]: e["direction"] for e in entries if "ros_topic_name" in e}
    assert by_topic["/r1/plugs/status"] == "GZ_TO_ROS"
    assert by_topic["/r1/plugs/cmd"] == "ROS_TO_GZ"
    for j in sim.JOINT_CMDS[kind]:
        assert by_topic[f"/r1/joint/{j}/cmd_pos"] == "ROS_TO_GZ"


def test_inspect_camera_optional():
    plain = sim.robot_description("jaal", "jaal_1", Path("/tmp/none.soilmap"), lidar=False)
    cam = sim.robot_description("jaal", "jaal_1", Path("/tmp/none.soilmap"), lidar=False, inspect=True)
    assert "inspect_cam" not in plain and "/model/jaal_1/inspect_cam" in cam
