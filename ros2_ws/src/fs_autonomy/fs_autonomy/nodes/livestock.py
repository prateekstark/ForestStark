"""Simulation only: cattle wandering across the field (graze, walk, graze), moved in Gazebo with
/world/<w>/set_pose so the robots' lidar sees them. Tests the safety field, which the concept
sheet flags as a watch-out for unattended robots."""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose

from ..geometry import quat_from_yaw
from ..ros_common import load_site, param, sim_now


class Livestock(Node):
    def __init__(self):
        super().__init__("livestock")
        self.site = load_site(self)
        world = param(self, "world", self.site.name)
        self.speed = param(self, "speed", 0.4)
        self.rng = np.random.default_rng(param(self, "seed", 4))
        n = int(param(self, "count", len(self.site.get("livestock", []))))
        f = self.site["field"]
        poly = f["demo_polygon"] if param(self, "field", "demo") == "demo" else f["polygon"]
        xs, ys = [p[0] for p in poly], [p[1] for p in poly]
        self.box = (min(xs) - 4, min(ys) - 4, max(xs) + 4, max(ys) + 4)
        self.cows = []
        for c in self.site.get("livestock", [])[:n]:
            x = float(np.clip(c["x"], self.box[0], self.box[2]))
            y = float(np.clip(c["y"], self.box[1], self.box[3]))
            self.cows.append({"name": c["name"], "x": x, "y": y, "yaw": self.rng.uniform(-3.14, 3.14),
                              "walk": False, "until": 0.0})
        self.cli = self.create_client(SetEntityPose, f"/world/{world}/set_pose")
        self.pub = self.create_publisher(PoseArray, "/sim/livestock", 10)
        self.last = None
        self.create_timer(0.25, self.tick)
        self.get_logger().info(f"{len(self.cows)} cattle in {self.box}")

    def tick(self):
        t = sim_now(self)
        dt = 0.25 if self.last is None else max(1e-3, t - self.last)
        self.last = t
        arr = PoseArray()
        arr.header.frame_id = "map"
        arr.header.stamp = self.get_clock().now().to_msg()
        for c in self.cows:
            if t > c["until"]:
                c["walk"] = not c["walk"]
                c["until"] = t + (self.rng.uniform(15, 40) if c["walk"] else self.rng.uniform(20, 90))
                c["yaw"] += self.rng.normal(0, 0.8)
            if c["walk"]:
                c["yaw"] += self.rng.normal(0, 0.05)
                nx = c["x"] + self.speed * math.cos(c["yaw"]) * dt
                ny = c["y"] + self.speed * math.sin(c["yaw"]) * dt
                if not (self.box[0] < nx < self.box[2] and self.box[1] < ny < self.box[3]):
                    c["yaw"] += math.pi
                else:
                    c["x"], c["y"] = nx, ny
            p = Pose()
            p.position.x, p.position.y = c["x"], c["y"]
            _, _, p.orientation.z, p.orientation.w = quat_from_yaw(c["yaw"])
            arr.poses.append(p)
            if self.cli.service_is_ready():
                req = SetEntityPose.Request()
                req.entity.name, req.entity.type = c["name"], Entity.MODEL
                req.pose = p
                self.cli.call_async(req)
        self.pub.publish(arr)


def main():
    rclpy.init()
    n = Livestock()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
