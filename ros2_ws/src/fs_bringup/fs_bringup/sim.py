"""Building blocks for the ForestStark simulation launch files.

Each launch regenerates its site (world, soil map, survey YAML) into ~/.ros/fs_sites/<site>-<hash>
from the launch arguments, starts Gazebo on it, spawns the robots from xacro, and bridges each
robot's gz topics to ROS under /<robot>/...
"""
from __future__ import annotations

import hashlib
import json
import os
import signal
import subprocess
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

GZ_TAG = "FS_GZ_LAUNCH"
DUCKBILL = ["duckbill_feed_joint", "jaw_l_joint", "jaw_r_joint", "tamp_joint"]
JOINT_CMDS = {"katar": ["sled_joint", "carriage_joint", "turret_joint", "carousel_joint"] + DUCKBILL,
              "jaal": ["drum_f_joint", "drum_r_joint", "picker_x_joint", "picker_z_joint", "picker_y_joint",
                       "finger_a_joint", "finger_b_joint", "shuttle_joint"] + DUCKBILL,
              "pahadi": ["artic_yaw_joint", "arm_yaw_joint", "arm_shoulder_joint", "arm_elbow_joint", "arm_wrist_joint",
                         "finger_l_joint", "finger_r_joint"]}
BASE_HEIGHT = {"katar": 0.72, "jaal": 0.38, "pahadi": 0.21}
GNSS_LEVER = {"katar": [0.1, 0.0, 2.0], "jaal": [1.15, 0.0, 2.95], "pahadi": [-0.45, -0.26, 1.27]}  # base_link -> antenna


def on(v) -> bool:
    return str(v).lower() in ("1", "true", "yes", "on")


def make_site(site: str, **kw) -> Path:
    from fs_gazebo.sites import generate
    key = hashlib.sha1(json.dumps(kw, sort_keys=True).encode()).hexdigest()[:8]
    out = Path(os.environ.get("FS_SITES_DIR", Path.home() / ".ros" / "fs_sites")) / f"{site}-{key}"
    generate(site, out, **kw)
    return out


def gazebo(world: Path, headless: bool, rtf: float, verbosity: int = 2, step: float = 0.004):
    """Gazebo server (+ GUI unless headless). rtf > 1 runs faster than real time if the CPU allows."""
    rate = int(round(rtf / step)) if rtf > 0 else 0
    args = f"-r -v {verbosity} -z {rate} {world}"
    if headless:
        args = "-s --headless-rendering " + args
    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_gz_sim"), "launch",
                                                   "gz_sim.launch.py")),
        launch_arguments={"gz_args": args, "on_exit_shutdown": "true"}.items())
    return [include, RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=_stop_gazebo)]))]


def _stop_gazebo(context, *args, **kwargs):
    """`ros2 launch` only signals the `gz sim` wrapper; make sure this launch's server and GUI exit."""
    tag = f"{GZ_TAG}={os.getpid()}".encode()
    pids = []
    for d in Path("/proc").iterdir():
        if not d.name.isdigit():
            continue
        try:
            if (d / "comm").read_text().startswith("gz-sim") and tag in (d / "environ").read_bytes().split(b"\0"):
                pids.append(int(d.name))
        except OSError:
            continue
    for p in pids:
        try:
            os.kill(p, signal.SIGINT)
        except OSError:
            pass
    if pids:
        subprocess.Popen(["sh", "-c", f"sleep 8; kill -9 {' '.join(map(str, pids))} 2>/dev/null"],
                         start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return []


def robot_description(kind: str, ns: str, soil_map: Path, lidar: bool, gnss_sigma: float = 0.02,
                      inspect: bool = False) -> str:
    """URDF from xacro. inspect adds a fixed camera beside the plant handling that saves frames to
    /tmp/fs_inspect (topic /model/<ns>/inspect_cam)."""
    path = Path(get_package_share_directory("fs_description")) / "urdf" / f"{kind}.urdf.xacro"
    doc = xacro.process_file(str(path), mappings={"ns": ns, "soil_map": str(soil_map),
                                                  "lidar": "true" if lidar else "false",
                                                  "gnss_sigma": str(gnss_sigma),
                                                  "inspect_cam": "true" if inspect else "false"})
    return doc.toxml()


def spawn_robot(kind: str, ns: str, site_dir: Path, site: dict, lidar: bool, inspect: bool = False):
    pose = next(s for s in site["spawn"] if s["name"] == ns)
    desc = robot_description(kind, ns, site_dir / site["soil_map"], lidar, inspect=inspect)
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher", namespace=ns, output="log",
             parameters=[{"robot_description": desc, "frame_prefix": f"{ns}/", "use_sim_time": True}]),
        Node(package="ros_gz_sim", executable="create", output="log",
             arguments=["-world", site["name"], "-topic", f"/{ns}/robot_description", "-name", ns,
                        "-x", str(pose["x"]), "-y", str(pose["y"]), "-z", str(pose["z"]), "-Y", str(pose["yaw"])]),
    ]


def bridge_entries(kind: str, ns: str, lidar: bool) -> list:
    g2r = [("odometry", "odom", "nav_msgs/msg/Odometry", "gz.msgs.Odometry"),
           ("imu", "imu", "sensor_msgs/msg/Imu", "gz.msgs.IMU"),
           ("gnss", "gnss/fix_raw", "sensor_msgs/msg/NavSatFix", "gz.msgs.NavSat"),
           ("joint_states", "joint_states", "sensor_msgs/msg/JointState", "gz.msgs.Model"),
           ("ground_truth", "ground_truth", "nav_msgs/msg/Odometry", "gz.msgs.Odometry"),
           ("auger/status", "auger/status", "std_msgs/msg/String", "gz.msgs.StringMsg"),
           ("plugs/status", "plugs/status", "std_msgs/msg/String", "gz.msgs.StringMsg"),
           ("terramechanics", "terramechanics", "std_msgs/msg/String", "gz.msgs.StringMsg")]
    if kind == "pahadi":
        g2r.append(("imu_rear", "imu_rear", "sensor_msgs/msg/Imu", "gz.msgs.IMU"))
    if lidar:
        g2r.append(("scan", "scan", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan"))
    r2g = [("cmd_vel", "cmd_vel", "geometry_msgs/msg/Twist", "gz.msgs.Twist"),
           ("auger/cmd", "auger/cmd", "std_msgs/msg/String", "gz.msgs.StringMsg"),
           ("plugs/cmd", "plugs/cmd", "std_msgs/msg/String", "gz.msgs.StringMsg")]
    r2g += [(f"joint/{j}/cmd_pos", f"joint/{j}/cmd_pos", "std_msgs/msg/Float64", "gz.msgs.Double")
            for j in JOINT_CMDS[kind]]
    out = [{"ros_topic_name": f"/{ns}/{r}", "gz_topic_name": f"/model/{ns}/{g}", "ros_type_name": rt,
            "gz_type_name": gt, "direction": "GZ_TO_ROS"} for g, r, rt, gt in g2r]
    out += [{"ros_topic_name": f"/{ns}/{r}", "gz_topic_name": f"/model/{ns}/{g}", "ros_type_name": rt,
             "gz_type_name": gt, "direction": "ROS_TO_GZ"} for g, r, rt, gt in r2g]
    return out


def bridge(world: str, entries: list, out_dir: Path):
    cfg = [{"ros_topic_name": "/clock", "gz_topic_name": "/clock", "ros_type_name": "rosgraph_msgs/msg/Clock",
            "gz_type_name": "gz.msgs.Clock", "direction": "GZ_TO_ROS"},
           {"service_name": f"/world/{world}/create", "ros_type_name": "ros_gz_interfaces/srv/SpawnEntity",
            "gz_req_type_name": "gz.msgs.EntityFactory", "gz_rep_type_name": "gz.msgs.Boolean"},
           {"service_name": f"/world/{world}/set_pose", "ros_type_name": "ros_gz_interfaces/srv/SetEntityPose",
            "gz_req_type_name": "gz.msgs.Pose", "gz_rep_type_name": "gz.msgs.Boolean"}] + entries
    path = out_dir / "bridge.yaml"
    path.write_text(yaml.safe_dump(cfg, sort_keys=False))
    return Node(package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge", output="log",
                parameters=[{"config_file": str(path), "use_sim_time": True}])


def robot_core(kind: str, ns: str, site_file: Path, world: str, head: dict, gnss_mode: str = "rtk",
               dropouts: float = 0.0, seed: int = 1):
    """Nodes every robot runs: GNSS model, localization, planting head, traction monitor."""
    common = {"site_file": str(site_file), "robot": ns, "use_sim_time": True}
    return [
        Node(package="fs_autonomy", executable="gnss_degrader", namespace=ns, output="screen",
             parameters=[{**common, "mode": gnss_mode, "dropouts_per_hour": float(dropouts), "seed": seed}]),
        Node(package="fs_autonomy", executable="localization", namespace=ns, output="screen",
             parameters=[{**common, "gnss_lever": GNSS_LEVER[kind], "base_height": BASE_HEIGHT[kind]}]),
        Node(package="fs_autonomy", executable="planting_head", namespace=ns, output="screen",
             parameters=[{**common, "robot_type": kind, "world": world, "seed": seed, **head}]),
        Node(package="fs_autonomy", executable="traction_monitor", namespace=ns, output="screen",
             parameters=[common]),
    ]


def common_nodes(site_file: Path, rviz: bool, rviz_config: str, log_dir: str = ""):
    nodes = [
        Node(package="fs_autonomy", executable="mission_logger", output="screen",
             parameters=[{"site_file": str(site_file), "out_dir": log_dir, "use_sim_time": True}]),
        Node(package="fs_autonomy", executable="viz_markers", output="log",
             parameters=[{"site_file": str(site_file), "use_sim_time": True}]),
    ]
    if rviz:
        nodes.append(Node(package="rviz2", executable="rviz2", output="log",
                          arguments=["-d", os.path.join(get_package_share_directory("fs_bringup"), "rviz", rviz_config)],
                          parameters=[{"use_sim_time": True}]))
    return nodes


def load_site_yaml(site_dir: Path, site: str) -> dict:
    return yaml.safe_load((site_dir / f"{site}.yaml").read_text())
