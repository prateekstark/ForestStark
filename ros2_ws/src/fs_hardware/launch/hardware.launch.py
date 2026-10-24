"""A machine's drives on their own: controller manager, controllers and the hardware-interface nodes.

    ros2 launch fs_hardware hardware.launch.py robot:=jaal bus:=mock     (on a desk: simulated drives)
    ros2 launch fs_hardware hardware.launch.py robot:=pahadi bus:=can0   (on the machine, autonomy off:
                                                                          commissioning, joint by joint)
Then e.g.  ros2 topic pub -1 /jaal/joint/shuttle_joint/cmd_pos std_msgs/msg/Float64 "{data: -0.2}"
           ros2 topic pub -r 10 /jaal/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
           ros2 topic pub -1 /jaal/auger/cmd std_msgs/msg/String "{data: bore 0.3}"
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

ARGS = {"robot": "jaal", "ns": "", "bus": "mock"}


def setup(context):
    from fs_bringup import sim
    from fs_hardware import bringup, description
    c = context.launch_configurations
    kind, ns = c["robot"], c["ns"] or ("jaal_1" if c["robot"] == "jaal" else c["robot"])
    urdf = description.with_ros2_control(sim.robot_description(kind, ns, Path("/tmp/none.soilmap"), lidar=True), kind, c["bus"])
    return [Node(package="robot_state_publisher", executable="robot_state_publisher", namespace=ns, output="log",
                 parameters=[{"robot_description": urdf, "frame_prefix": f"{ns}/"}])] + \
        bringup.hardware_nodes(kind, ns, urdf)


def generate_launch_description():
    return LaunchDescription([*[DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()],
                              OpaqueFunction(function=setup)])
