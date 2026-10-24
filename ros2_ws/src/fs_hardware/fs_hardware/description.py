"""The ros2_control description and controller set for a robot, from its CAN map (config/<robot>_can.yaml).

robot.launch.py hardware:=true inserts ros2_control_xml() into the URDF the controller manager and
robot_state_publisher load, and writes controllers_yaml() for the controller manager:

  joint_state_broadcaster   every drive's position, velocity, effort -> joint_state_broadcaster/joint_states
  position_controller       head, arm, steering (position-mode drives), fed by hw_bridge
  drive_controller          wheels / tracks (velocity-mode drives), fed by hw_bridge from cmd_vel
  auger_controller          auger feed and spin (velocity), fed by auger_driver
"""
from __future__ import annotations

from pathlib import Path
from xml.sax.saxutils import escape

import yaml

AUGER = ("auger_feed_joint", "auger_spin_joint")


def config_path(kind: str) -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("fs_hardware")) / "config" / f"{kind}_can.yaml"
    except Exception:       # source tree (tests)
        return Path(__file__).resolve().parents[1] / "config" / f"{kind}_can.yaml"


def load(kind: str, path: str | Path | None = None) -> dict:
    return yaml.safe_load(Path(path or config_path(kind)).read_text())


def groups(cfg: dict) -> dict:
    j = cfg["joints"]
    return {"position": [x["name"] for x in j if x["mode"] == "position"],
            "drive": [x["name"] for x in j if x["mode"] == "velocity" and x["name"] not in AUGER],
            "auger": [x["name"] for x in j if x["name"] in AUGER],
            "sensor": [x["name"] for x in j if x["mode"] == "sensor"]}


def ros2_control_xml(kind: str, bus: str | None = None, cfg: dict | None = None, starts: dict | None = None) -> str:
    """starts: joint -> position the mock bus's drives start at (inside the URDF range)."""
    cfg = cfg or load(kind)
    bus = bus or cfg.get("bus", "can0")
    starts = starts or {}
    out = [f'  <ros2_control name="{kind}_hw" type="system">', "    <hardware>",
           "      <plugin>fs_hardware/CanopenSystem</plugin>",
           f'      <param name="bus">{escape(bus)}</param>',
           f'      <param name="cycle_s">{cfg.get("cycle_s", 0.01)}</param>',
           f'      <param name="timeout_s">{cfg.get("timeout_s", 0.1)}</param>', "    </hardware>"]
    for j in cfg["joints"]:
        out.append(f'    <joint name="{j["name"]}">')
        if j["mode"] in ("position", "velocity"):
            # the drives enforce their own soft limits and hw_bridge clamps and ramps the targets, so
            # ros2_control's limiter (which clamps against the measured position) is not run on top
            out.append(f'      <command_interface name="{j["mode"]}"><limits enable="false"/></command_interface>')
        out += [f'      <state_interface name="{s}"/>' for s in ("position", "velocity", "effort")]
        for k in ("node", "counts_per_unit", "offset", "rated", "mock_contact"):
            if k in j:
                out.append(f'      <param name="{"node_id" if k == "node" else k}">{j[k]}</param>')
        if bus == "mock" and starts.get(j["name"]):
            out.append(f'      <param name="mock_start">{starts[j["name"]]}</param>')
        out.append("    </joint>")
    out.append("  </ros2_control>")
    return "\n".join(out)


def with_ros2_control(urdf_xml: str, kind: str, bus: str | None = None) -> str:
    """The robot's URDF with its ros2_control block, before </robot>."""
    import xml.etree.ElementTree as ET
    starts = {}
    for j in ET.fromstring(urdf_xml).findall("joint"):
        lim = j.find("limit")
        if lim is not None and j.get("type") in ("prismatic", "revolute"):
            lo, hi = float(lim.get("lower", "0")), float(lim.get("upper", "0"))
            if not lo <= 0.0 <= hi:
                starts[j.get("name")] = lo if lo > 0 else hi
    i = urdf_xml.rfind("</robot>")
    return urdf_xml[:i] + ros2_control_xml(kind, bus, starts=starts) + "\n" + urdf_xml[i:]


def ros2_control_bus(urdf_xml: str) -> str:
    """The bus a ros2_control block in the URDF uses (can0, mock...)."""
    import xml.etree.ElementTree as ET
    p = ET.fromstring(urdf_xml).find("ros2_control/hardware/param[@name='bus']")
    return p.text if p is not None else "can0"


def controllers_yaml(kind: str, ns: str, cfg: dict | None = None) -> str:
    cfg = cfg or load(kind)
    g = groups(cfg)
    rate = int(round(1.0 / float(cfg.get("cycle_s", 0.01))))
    cm = {"update_rate": rate, "enforce_command_limits": False, "joint_state_broadcaster": {"type": "joint_state_broadcaster/JointStateBroadcaster"},
          "position_controller": {"type": "forward_command_controller/ForwardCommandController"},
          "auger_controller": {"type": "forward_command_controller/ForwardCommandController"}}
    params = {f"/{ns}/controller_manager": {"ros__parameters": cm},
              f"/{ns}/joint_state_broadcaster": {"ros__parameters": {"use_local_topics": True}},
              f"/{ns}/position_controller": {"ros__parameters": {"joints": g["position"], "interface_name": "position"}},
              f"/{ns}/auger_controller": {"ros__parameters": {"joints": g["auger"], "interface_name": "velocity"}}}
    if g["drive"]:
        cm["drive_controller"] = {"type": "forward_command_controller/ForwardCommandController"}
        params[f"/{ns}/drive_controller"] = {"ros__parameters": {"joints": g["drive"], "interface_name": "velocity"}}
    return yaml.safe_dump(params, sort_keys=False)


def controllers(kind: str, cfg: dict | None = None) -> list:
    g = groups(cfg or load(kind))
    return ["joint_state_broadcaster", "position_controller", "auger_controller"] + (["drive_controller"] if g["drive"] else [])
