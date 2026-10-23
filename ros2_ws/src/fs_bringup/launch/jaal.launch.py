"""B · Jaal fleet on the open-field site: N swerve-drive planters, fleet manager and tender truck.

    ros2 launch fs_bringup jaal.launch.py                         # 3 robots, 0.09 ha demo block
    ros2 launch fs_bringup jaal.launch.py robots:=4 field:=full   # 1 ha, 1,111 spots
    ros2 launch fs_bringup jaal.launch.py rtk_policy:=wait dropouts:=6 lidar:=false

Arguments
  robots        fleet size, 1-6 (3)
  field         demo (36 x 24 m) | full (120 x 84 m)
  pattern       square | triangular;   spacing (3.0)
  plugs         plugs per drum load (demo default 18, so each robot visits the tender once)
  bays          service bays at the tender (1)
  stock_plugs   plugs on the tender at the start (demo 36: the third robot waits for a restock)
  rtk_policy    continue | wait (stop while RTK is lost);  dropouts: random RTK losses per hour
  cows          cattle wandering in the field (2)
  lidar         front and rear safety lidars (true; needs GPU rendering)
  people        safety scenarios for jaal_1: all (= cross,tool,stand,obstacle) or a list; off by default
  headless, rtf, rviz, seed, soil
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node

from fs_bringup import sim

ARGS = {"robots": "3", "field": "demo", "pattern": "square", "spacing": "3.0", "plugs": "", "stock_plugs": "",
        "bays": "1", "rtk_policy": "continue", "dropouts": "0", "cows": "2", "lidar": "true", "cruise": "1.0",
        "headless": "false", "rtf": "1.0", "rviz": "true", "seed": "11", "soil": "sandy_loam", "log_dir": "", "inspect": "false", "people": ""}


def setup(context):
    c = context.launch_configurations
    n = max(1, min(6, int(c["robots"])))
    seed = int(c["seed"])
    lidar = sim.on(c["lidar"])
    demo = c["field"] == "demo"
    plugs = int(c["plugs"]) if c["plugs"] else (18 if demo else 360)
    stock = int(c["stock_plugs"]) if c["stock_plugs"] else (36 if demo else -1)
    site_dir = sim.make_site("open_field", seed=seed, soil=c["soil"])
    site = sim.load_site_yaml(site_dir, "open_field")
    site_file = site_dir / "open_field.yaml"
    robots = [f"jaal_{i + 1}" for i in range(n)]
    world = site["name"]
    out_dir = c["log_dir"]

    actions = sim.gazebo(site_dir / "open_field.sdf", sim.on(c["headless"]), float(c["rtf"]))
    entries = []
    for i, ns in enumerate(robots):
        actions += sim.spawn_robot("jaal", ns, site_dir, site, lidar, inspect=sim.on(c["inspect"]))
        entries += sim.bridge_entries("jaal", ns, lidar)
        head = {"plugs": plugs, "water_l": 250.0, "water_dose_l": 0.5, "pump_lps": 0.4, "hole_depth": 0.25}
        actions += sim.robot_core("jaal", ns, site_file, world, head, dropouts=float(c["dropouts"]), seed=seed + i,
                                  lidar=lidar, peers=robots)
        actions.append(Node(package="fs_autonomy", executable="jaal_navigator", namespace=ns, output="screen",
                            parameters=[{"site_file": str(site_file), "robot": ns, "robots": robots,
                                         "field": c["field"], "use_lidar": lidar, "rtk_policy": c["rtk_policy"],
                                         "plugs_capacity": plugs, "cruise_speed": float(c["cruise"]), "bays": int(c["bays"]),
                                         "use_sim_time": True}]))
    actions.append(sim.bridge(world, entries, site_dir))
    actions += [
        Node(package="fs_autonomy", executable="jaal_fleet_manager", output="screen",
             parameters=[{"site_file": str(site_file), "robots": robots, "field": c["field"], "pattern": c["pattern"],
                          "spacing": float(c["spacing"]), "plugs_per_robot": plugs,
                          "stock_plugs": stock, "bays": int(c["bays"]),
                          "restock_s": 600.0 if demo else 2400.0, "out_dir": out_dir, "use_sim_time": True}]),
        Node(package="fs_autonomy", executable="livestock", output="log",
             parameters=[{"site_file": str(site_file), "world": world, "count": int(c["cows"]), "field": c["field"],
                          "seed": seed, "use_sim_time": True}]),
    ]
    actions += sim.people(world, robots[0], site_file, c["people"])
    actions += sim.common_nodes(site_file, sim.on(c["rviz"]), "jaal.rviz", out_dir)
    return actions


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS",
                               EnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", default_value="UDPv4")),
        SetEnvironmentVariable(sim.GZ_TAG, str(os.getpid())),
        *[DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()],
        OpaqueFunction(function=setup),
    ])
