"""A · Katar on the roadside site: tractor-trailer planter, simulated driver, planter, planting head.

    ros2 launch fs_bringup katar.launch.py
    ros2 launch fs_bringup katar.launch.py mode:=on_the_move rail_min:=-1.8 rail_max:=0.6
    ros2 launch fs_bringup katar.launch.py surveyed:=false gnss:=standard headless:=true rtf:=3

Arguments
  mode          stop_and_go | on_the_move
  spacing       m between plants (3.0)
  speed_kmh     driver's set speed (2.5)
  weave_m       driver's lateral weave, 1 sigma (0.25)
  reaction_s    driver reaction to the cab STOP cue (0.8)
  surveyed      utility survey available (true); false shows buried-cable strikes
  gnss          rtk | standard | none (odometry only)
  soil          verge soil: sandy_loam | dry_sand | clayey_soil | heavy_clay | lateritic_gravel
  people        safety scenarios: all (= tool,stand:2.5) or a list such as tool,cross; off by default
  headless, rtf, rviz, seed, inspect
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node

from fs_bringup import sim

ARGS = {"mode": "stop_and_go", "spacing": "3.0", "speed_kmh": "2.5", "weave_m": "0.25", "reaction_s": "0.8",
        "surveyed": "true", "gnss": "rtk", "soil": "sandy_loam", "rail_min": "-1.8", "rail_max": "0.6",
        "headless": "false", "rtf": "1.0", "rviz": "true", "seed": "7", "log_dir": "", "inspect": "false", "people": ""}


def setup(context):
    c = context.launch_configurations
    seed = int(c["seed"])
    site_dir = sim.make_site("roadside", seed=seed, soil=c["soil"], spacing=float(c["spacing"]))
    site = sim.load_site_yaml(site_dir, "roadside")
    site_file = site_dir / "roadside.yaml"
    ns = "katar"
    common = {"site_file": str(site_file), "robot": ns, "use_sim_time": True}
    head = {"plugs": 1200, "water_l": 1000.0, "water_dose_l": 1.0, "pump_lps": 0.5, "hole_depth": 0.30,
            "rail_min": float(c["rail_min"]), "rail_max": float(c["rail_max"])}
    actions = sim.gazebo(site_dir / "roadside.sdf", sim.on(c["headless"]), float(c["rtf"]))
    actions += sim.spawn_robot("katar", ns, site_dir, site, lidar=False, inspect=sim.on(c["inspect"]))
    actions.append(sim.bridge(site["name"], sim.bridge_entries("katar", ns, lidar=False), site_dir))
    actions += sim.robot_core("katar", ns, site_file, site["name"], head, gnss_mode=c["gnss"], seed=seed)
    actions += sim.people(site["name"], ns, site_file, "tool,stand:2.5" if c["people"] == "all" else c["people"])
    actions += [
        Node(package="fs_autonomy", executable="katar_driver", namespace=ns, output="screen",
             parameters=[{**common, "speed_kmh": float(c["speed_kmh"]), "weave_m": float(c["weave_m"]),
                          "reaction_s": float(c["reaction_s"]), "seed": seed}]),
        Node(package="fs_autonomy", executable="katar_planter", namespace=ns, output="screen",
             parameters=[{**common, "spacing": float(c["spacing"]), "mode": c["mode"], "surveyed": sim.on(c["surveyed"]),
                          "speed_kmh": float(c["speed_kmh"]), "driver_reaction_s": float(c["reaction_s"]),
                          "rail_min": float(c["rail_min"]), "rail_max": float(c["rail_max"])}]),
    ]
    actions += sim.common_nodes(site_file, sim.on(c["rviz"]), "katar.rviz", c["log_dir"])
    return actions


def generate_launch_description():
    import os
    return LaunchDescription([
        SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS",
                               EnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", default_value="UDPv4")),
        SetEnvironmentVariable(sim.GZ_TAG, str(os.getpid())),
        *[DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()],
        OpaqueFunction(function=setup),
    ])
