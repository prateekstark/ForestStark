"""Simulated tractor driver for Katar (simulation only: on the real machine a person drives).

Keeps the tractor on the gravel shoulder with a human weave (Ornstein-Uhlenbeck lateral offset),
holds the speed the operator chose, and obeys the planter's cab display (speed_limit topic:
<0 no limit, 0 stop, >0 cap) after a reaction delay. Reads ground truth, because a driver
uses their eyes, not the robot's localization.
"""
import collections
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64, String

from ..control import OrnsteinUhlenbeck, RateLimiter
from ..geometry import Polyline, wrap, yaw_from_quat
from ..ros_common import load_site, param, sim_now


class KatarDriver(Node):
    def __init__(self):
        super().__init__("katar_driver")
        self.site = load_site(self)
        rs = self.site["roadside"]
        self.path = Polyline(rs["shoulder_centerline"])
        self.end_s = rs["station_end"] + 8.0
        self.v_set = param(self, "speed_kmh", 2.5) / 3.6
        self.ld = param(self, "lookahead", 5.0)
        self.reaction = param(self, "reaction_s", 0.8)
        weave = param(self, "weave_m", 0.25)
        self.weave = OrnsteinUhlenbeck(weave, param(self, "weave_tau_s", 8.0), np.random.default_rng(param(self, "seed", 5)))
        self.accel = RateLimiter(1.0)
        self.acc_up, self.acc_down = param(self, "accel", 0.4), param(self, "decel", 0.9)
        self.limits = collections.deque([(-1e9, -1.0)])
        self.pose = None
        self.hint = None
        self.done = False
        self.last = None
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.done_pub = self.create_publisher(String, "driver/state", 10)
        self.create_subscription(Odometry, "ground_truth", self.on_gt, 10)
        self.create_subscription(Float64, "speed_limit", self.on_limit, 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f"driver: {self.v_set * 3.6:.1f} km/h, weave {weave} m, "
                               f"reaction {self.reaction} s")

    def on_gt(self, m: Odometry):
        q = m.pose.pose.orientation
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_from_quat(q.x, q.y, q.z, q.w))

    def on_limit(self, m: Float64):
        self.limits.append((sim_now(self), m.data))

    def effective_limit(self, t):
        lim = self.limits[0][1]
        for ts, v in self.limits:
            if ts <= t - self.reaction:
                lim = v
        while len(self.limits) > 2 and self.limits[1][0] <= t - self.reaction:
            self.limits.popleft()
        return lim

    def tick(self):
        if self.pose is None:
            return
        t = sim_now(self)
        dt = 0.05 if self.last is None else max(1e-3, t - self.last)
        self.last = t
        x, y, yaw = self.pose
        s, lat, self.hint = self.path.project(x, y, self.hint)
        w = self.weave(dt)
        gx, gy, gh = self.path.at(s + self.ld)
        gx, gy = gx - math.sin(gh) * w, gy + math.cos(gh) * w
        alpha = wrap(math.atan2(gy - y, gx - x) - yaw)
        kappa = 2 * math.sin(alpha) / max(math.hypot(gx - x, gy - y), 1e-3)
        lim = self.effective_limit(t)
        target = self.v_set if lim < 0 else min(self.v_set, lim)
        if s >= self.end_s:
            target = 0.0
            if not self.done:
                self.done = True
                self.done_pub.publish(String(data="done"))
                self.get_logger().info("end of the planting stretch")
        self.accel.rate = self.acc_up if target > self.accel.value else self.acc_down
        v = self.accel(target, dt)
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = v * kappa
        self.pub.publish(cmd)


def main():
    rclpy.init()
    n = KatarDriver()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
