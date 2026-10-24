"""Launch actions for a machine's drives: the controller manager with fs_hardware/CanopenSystem, the
controllers, and the nodes that serve the hardware interface. Used by fs_bringup robot.launch.py
(hardware:=true) and fs_hardware hardware.launch.py (the drives alone, e.g. on a desk with bus:=mock)."""
from __future__ import annotations

import re
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from launch_ros.actions import Node

from . import description

PASSIVE = {   # joints no drive measures, shown by copying a measured one ("zero": held at 0)
    "katar": {r"front_(left|right)_steer_joint": "zero", r"front_.*_wheel_joint|trailer_.*_wheel_joint": "rear_left_wheel_joint"},
    "jaal": {},
    "pahadi": {r"([fr][lr])[13]_joint": r"\g<1>2_joint"},
}


def mirrors(kind: str, urdf: str, cfg: dict) -> tuple:
    have = {j["name"] for j in cfg["joints"]}
    joints, sources = [], []
    for j in ET.fromstring(urdf).findall("joint"):
        n = j.get("name")
        if j.get("type") == "fixed" or n in have:
            continue
        for pat, src in PASSIVE[kind].items():
            if re.fullmatch(pat, n):
                joints.append(n)
                sources.append(re.sub(pat, src, n) if src != "zero" else "zero")
                break
    return joints or [""], sources or [""]


def wheel_loads(urdf: str, wheels: list) -> list:
    """Static share of the machine's weight on each driven wheel (equal split; a weighed machine's
    calibrated mass is already in the URDF)."""
    from fs_bringup.physics_audit import Model
    m, _ = Model(urdf).com()
    return [m * 9.81 / max(len(wheels), 1)] * len(wheels)


def hardware_nodes(kind: str, ns: str, urdf_hw: str, sim_time: bool = False) -> list:
    """Everything below the autonomy stack on a machine. urdf_hw: the URDF with its ros2_control block
    (description.with_ros2_control), as published by robot_state_publisher on /<ns>/robot_description."""
    cfg = description.load(kind)
    g = description.groups(cfg)
    ctl = Path(tempfile.mkdtemp(prefix="fs_hw_")) / f"{kind}_controllers.yaml"
    ctl.write_text(description.controllers_yaml(kind, ns, cfg))
    common = {"use_sim_time": sim_time}
    mj, ms = mirrors(kind, urdf_hw, cfg)
    drive = g["drive"] or [j for j in ("rear_left_wheel_joint", "rear_right_wheel_joint") if j in g["sensor"]]
    radius = {"jaal": 0.38, "pahadi": 0.21, "katar": 0.72}[kind]
    return [
        Node(package="controller_manager", executable="ros2_control_node", namespace=ns, output="screen",
             parameters=[str(ctl), common]),
        Node(package="controller_manager", executable="spawner", namespace=ns, output="screen",
             arguments=description.controllers(kind, cfg) + ["--controller-manager", f"/{ns}/controller_manager",
                                                             "--param-file", str(ctl)]),
        Node(package="fs_hardware", executable="hw_bridge", namespace=ns, output="screen",
             parameters=[{**common, "robot_type": kind, "mirror_joints": mj, "mirror_sources": ms,
                          "robot_description": urdf_hw}]),
        Node(package="fs_hardware", executable="auger_driver", namespace=ns, output="screen",
             parameters=[{**common, "robot_description": urdf_hw}]),
        Node(package="fs_hardware", executable="plug_inventory", namespace=ns, output="screen",
             parameters=[{**common, "robot_description": urdf_hw}]),
        Node(package="fs_hardware", executable="safety_bridge", namespace=ns, output="screen",
             parameters=[{**common, "bus": description.ros2_control_bus(urdf_hw)}]),
        Node(package="fs_hardware", executable="traction_estimator", namespace=ns, output="screen",
             parameters=[{**common, "wheels": drive, "loads_n": wheel_loads(urdf_hw, drive), "radius": radius}]),
    ]
