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


def test_safety_lidars_are_bridged():
    topics = lambda kind, lidar: {e["ros_topic_name"] for e in sim.bridge_entries(kind, "r1", lidar=lidar)
                                  if "ros_topic_name" in e}
    assert {"/r1/scan", "/r1/scan_rear"} <= topics("jaal", True)
    assert "/r1/scan" in topics("katar", False)          # Katar's boom-side lidar is always on
    assert "/r1/scan" not in topics("pahadi", False)


def test_every_robot_runs_the_safety_monitor():
    for kind in sim.JOINT_CMDS:
        nodes = sim.robot_core(kind, kind, Path("/tmp/x.yaml"), "w", {})
        assert any(getattr(n, "_Node__node_executable", None) == "safety_monitor" for n in nodes)


@pytest.mark.parametrize("kind", sorted(sim.JOINT_CMDS))
def test_hardware_contract_covers_every_command_with_limits(kind):
    from fs_bringup import interface_contract as ic
    text = ic.contract(kind)
    for j in sim.JOINT_CMDS[kind]:
        assert f"| `{j}` | ?" not in text, j          # every commanded joint has URDF limits for the driver
    assert "`gnss/fix_raw`" in text and "`cmd_vel`" in text and "ground_truth" not in text


def test_robot_config_feeds_both_sim_and_hardware():
    assert sim.head_params("katar")["plugs"] == 1200
    assert sim.head_params("jaal", plugs=18)["plugs"] == 18
    assert sim.robot_config("jaal", "navigator")["mass_kg"] > 1500     # running order from the CAD
