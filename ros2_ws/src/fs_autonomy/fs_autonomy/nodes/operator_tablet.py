"""Operator tablet, command-line edition: send a command to a robot's navigator.

    ros2 run fs_autonomy operator pause            # robot defaults to pahadi
    ros2 run fs_autonomy operator resume
    ros2 run fs_autonomy operator skip
    ros2 run fs_autonomy operator estop pahadi
    ros2 run fs_autonomy operator release pahadi
"""
import sys
import time

import rclpy
from std_msgs.msg import String

COMMANDS = ("pause", "resume", "skip", "estop", "release")


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if not args or args[0] not in COMMANDS:
        print(__doc__)
        sys.exit(1)
    cmd, robot = args[0], (args[1] if len(args) > 1 else "pahadi")
    rclpy.init()
    node = rclpy.create_node("operator_tablet")
    pub = node.create_publisher(String, f"/{robot}/operator", 10)
    t_end = time.time() + 1.5
    while time.time() < t_end and pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    pub.publish(String(data=cmd))
    rclpy.spin_once(node, timeout_sec=0.3)
    print(f"{robot}: {cmd} ({pub.get_subscription_count()} receiver(s))")
    node.destroy_node()
    rclpy.try_shutdown()
