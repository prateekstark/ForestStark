"""C · Pahadi on the hill-slope site: contour planting with tool choice and tip-over monitoring.

    ros2 launch fs_bringup pahadi.launch.py
    ros2 launch fs_bringup pahadi.launch.py slope:=22 side:=uphill rock_cover:=0.4
    ros2 launch fs_bringup pahadi.launch.py headless:=true rviz:=false max_spots:=6

Arguments
  slope       hillside slope, degrees (14)
  soil        lateritic_gravel | sandy_loam | dry_sand | clayey_soil | heavy_clay
  rock_cover  share of ground too thin for the auger, 0-0.8 (0.25)
  side        downhill | uphill: which side the arm plants (stability trade-off)
  rows, row_spacing, spacing, half_moon, speed, max_spots (0 = all)
  headless, rtf, rviz, seed, lidar
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node

from fs_bringup import sim

ARGS = {"slope": "14", "soil": "lateritic_gravel", "rock_cover": "0.25", "side": "downhill", "rows": "3",
        "row_spacing": "4.0", "spacing": "2.5", "half_moon": "true", "speed": "0.35", "max_spots": "0",
        "headless": "false", "rtf": "1.0", "rviz": "true", "seed": "23", "lidar": "false", "log_dir": ""}

CONTACTS = [f"{m}{s}{i}_link" for m in "fr" for s in "lr" for i in (1, 2, 3)]


def setup(context):
    c = context.launch_configurations
    seed = int(c["seed"])
    site_dir = sim.make_site("hill_slope", seed=seed, slope_deg=float(c["slope"]), soil=c["soil"],
                             rock_cover=float(c["rock_cover"]))
    site = sim.load_site_yaml(site_dir, "hill_slope")
    site_file = site_dir / "hill_slope.yaml"
    ns = "pahadi"
    common = {"site_file": str(site_file), "robot": ns, "use_sim_time": True}
    head = {"plugs": 200, "water_l": 150.0, "seedballs": 3000, "water_dose_l": 1.0, "pump_lps": 0.3,
            "hole_depth": 0.25, "place_s": 2.0, "close_s": 1.5}
    lidar = sim.on(c["lidar"])
    actions = sim.gazebo(site_dir / "hill_slope.sdf", sim.on(c["headless"]), float(c["rtf"]))
    actions += sim.spawn_robot("pahadi", ns, site_dir, site, lidar)
    actions.append(sim.bridge(site["name"], sim.bridge_entries("pahadi", ns, lidar), site_dir))
    actions += sim.robot_core("pahadi", ns, site_file, site["name"], head, seed=seed)
    actions += [
        Node(package="fs_autonomy", executable="stability_monitor", namespace=ns, output="screen",
             parameters=[{**common, "contact_links": CONTACTS,
                          "contact_parents": ["base_link" if n.startswith("f") else "rear_link" for n in CONTACTS],
                          "wheel_radius": 0.21}]),
        Node(package="fs_autonomy", executable="pahadi_navigator", namespace=ns, output="screen",
             parameters=[{**common, "rows": int(c["rows"]), "row_spacing": float(c["row_spacing"]),
                          "spacing": float(c["spacing"]), "side": c["side"], "half_moon": sim.on(c["half_moon"]),
                          "speed": float(c["speed"]), "max_spots": int(c["max_spots"]), "seed": seed}]),
    ]
    actions += sim.common_nodes(site_file, sim.on(c["rviz"]), "pahadi.rviz", c["log_dir"])
    return actions


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS",
                               EnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", default_value="UDPv4")),
        SetEnvironmentVariable(sim.GZ_TAG, str(os.getpid())),
        *[DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()],
        OpaqueFunction(function=setup),
    ])
