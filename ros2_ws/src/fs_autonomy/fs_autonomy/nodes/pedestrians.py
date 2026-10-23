"""Simulation only: people and an unsurveyed obstacle that test a robot's safety behaviour.
The models (walker_0, walker_1, crate_0) are parked off-site in every world; this node moves them
with /world/<w>/set_pose, relative to where the target robot is and what it is doing.

Events, run in order with gap_s between them (parameter `events`, e.g. "cross,tool,stand,obstacle"):
  cross        once the robot is driving, a person walks across its path 8 m ahead at 1.3 m/s
  stand[:off]  a person walks into the path 7 m ahead (or `off` m to the left of it) and stands
               there for stand_s, then walks off
  tool         while the auger is working, a person walks up to 1.3 m from it, waits 8 s, leaves
  obstacle     a crate appears on the path 6 m ahead and stays until the robot is past it
Expected: stop and wait for people (never a detour close to them), tool hold at the auger,
detour round the crate after the navigator's wait.
"""
import math

import rclpy
from fs_interfaces.msg import HeadStatus
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from tf2_ros import Buffer, TransformListener

from ..geometry import quat_from_yaw, yaw_from_quat
from ..ros_common import load_site, lookup, param, sim_now

PARK = (0.0, -400.0)


class Walker:
    def __init__(self, name):
        self.name = name
        self.x, self.y = PARK
        self.yaw = 0.0
        self.path = []                 # [(x, y, speed, pause_s)]
        self.pause_until = 0.0

    @property
    def busy(self):
        return bool(self.path)


class Pedestrians(Node):
    def __init__(self):
        super().__init__("pedestrians")
        self.site = load_site(self)
        world = param(self, "world", self.site.name)
        self.robot = param(self, "robot", "jaal_1")
        ev = param(self, "events", "cross,tool,stand,obstacle")
        self.events = [e.strip() for e in (ev.split(",") if isinstance(ev, str) else ev) if e.strip()]
        self.start = float(param(self, "start_delay_s", 25.0))
        self.gap = float(param(self, "gap_s", 20.0))
        self.speed = float(param(self, "walk_speed", 1.3))
        self.stand_s = float(param(self, "stand_s", 35.0))
        self.walkers = [Walker("walker_0"), Walker("walker_1")]
        self.crate = Walker("crate_0")
        self.gt = None
        self.phase = ""
        self.active = None             # (name, started_at, data)
        self.next_t = None
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.create_subscription(Odometry, f"/{self.robot}/ground_truth", lambda m: setattr(self, "gt", m), 10)
        self.create_subscription(HeadStatus, f"/{self.robot}/head/status", lambda m: setattr(self, "phase", m.phase), 10)
        self.cli = self.create_client(SetEntityPose, f"/world/{world}/set_pose")
        self.create_timer(0.1, self.tick)
        self.get_logger().info(f"safety scenarios for {self.robot}: {', '.join(self.events) or 'none'}")

    # ------------------------------------------------------------------ helpers
    def robot_state(self):
        if self.gt is None:
            return None
        p, q, v = self.gt.pose.pose.position, self.gt.pose.pose.orientation, self.gt.twist.twist.linear
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        vx, vy = math.cos(yaw) * v.x - math.sin(yaw) * v.y, math.sin(yaw) * v.x + math.cos(yaw) * v.y   # world frame
        return p.x, p.y, yaw, vx, vy

    def ahead(self, st, along, left):
        x, y, yaw, vx, vy = st
        sp = math.hypot(vx, vy)
        ux, uy = (vx / sp, vy / sp) if sp > 0.1 else (math.cos(yaw), math.sin(yaw))
        return x + ux * along - uy * left, y + uy * along + ux * left

    def place(self, w, x, y, yaw):
        w.x, w.y, w.yaw = x, y, yaw
        if not self.cli.service_is_ready():
            return
        req = SetEntityPose.Request()
        req.entity.name, req.entity.type = w.name, Entity.MODEL
        p = Pose()
        p.position.x, p.position.y = float(x), float(y)
        p.position.z = float(self.site.soil_map.elevation(x, y)) if (x, y) != PARK else 0.0
        _, _, p.orientation.z, p.orientation.w = quat_from_yaw(yaw)
        req.pose = p
        self.cli.call_async(req)

    def walk(self, w, dt):
        t = sim_now(self)
        if not w.path or t < w.pause_until:
            return
        tx, ty, sp, pause = w.path[0]
        d = math.hypot(tx - w.x, ty - w.y)
        if d < 0.05 or sp <= 0:
            if sp <= 0:                                # teleport (spawn or park)
                self.place(w, tx, ty, w.yaw)
            w.path.pop(0)
            w.pause_until = t + pause
            return
        step = min(d, sp * dt)
        yaw = math.atan2(ty - w.y, tx - w.x)
        self.place(w, w.x + (tx - w.x) / d * step, w.y + (ty - w.y) / d * step, yaw)

    # ------------------------------------------------------------------ scenarios
    def begin(self, ev, st, t):
        name, _, arg = ev.partition(":")
        w = self.walkers[0]
        if name == "cross":
            a, b = self.ahead(st, 8.0, 5.0), self.ahead(st, 8.0, -6.0)
            w.path = [(a[0], a[1], 0, 0.0), (b[0], b[1], self.speed, 0.0), (*PARK, 0, 0.0)]
            self.get_logger().warning(f"scenario cross: a person walks across {self.robot}'s path 8 m ahead")
        elif name == "stand":
            off = float(arg) if arg else 0.0
            a, b = self.ahead(st, 7.0, off + 4.0), self.ahead(st, 7.0, off)
            c = self.ahead(st, 7.0, off + 5.0)
            w.path = [(a[0], a[1], 0, 0.0), (b[0], b[1], self.speed * 0.8, self.stand_s), (c[0], c[1], self.speed, 0.0),
                      (*PARK, 0, 0.0)]
            self.get_logger().warning(f"scenario stand: a person stands in {self.robot}'s path for {self.stand_s:.0f} s")
        elif name == "tool":
            T = lookup(self.tf, "map", f"{self.robot}/auger_link", timeout=0.2)
            if T is None:
                return False
            tx, ty = T.transform.translation.x, T.transform.translation.y
            x, y = st[0], st[1]
            ox, oy = tx - x, ty - y                     # come from outside, along the robot-to-tool line
            n = math.hypot(ox, oy) or 1.0
            ox, oy = ox / n, oy / n
            a, b = (tx + 7 * ox - 2 * oy, ty + 7 * oy + 2 * ox), (tx + 1.3 * ox, ty + 1.3 * oy)
            w.path = [(a[0], a[1], 0, 0.0), (b[0], b[1], self.speed * 0.7, 8.0), (a[0], a[1], self.speed, 0.0), (*PARK, 0, 0.0)]
            self.get_logger().warning(f"scenario tool: a person walks up to {self.robot}'s auger while it is working")
        elif name == "obstacle":
            a = self.ahead(st, 6.0, 0.2)
            self.crate.path = [(a[0], a[1], 0, 0.0)]
            self.active = ("obstacle", t, a)
            self.get_logger().warning(f"scenario obstacle: a crate is left on {self.robot}'s path 6 m ahead")
            return True
        else:
            self.get_logger().error(f"unknown scenario '{ev}'")
            return True
        self.active = (name, t, None)
        return True

    def tick(self):
        t = sim_now(self)
        for w in self.walkers + [self.crate]:
            self.walk(w, 0.1)
        st = self.robot_state()
        if st is None:
            return
        if self.next_t is None:
            self.next_t = t + self.start
        if self.active is not None:
            name, t0, data = self.active
            if name == "obstacle":
                x, y = st[0], st[1]
                passed = math.hypot(x - data[0], y - data[1]) > 6.0 and t - t0 > 15.0
                if passed or t - t0 > 240.0:
                    self.crate.path = [(*PARK, 0, 0.0)]
                    self.get_logger().info(f"scenario obstacle: crate removed ({'robot went past it' if passed else 'timeout'})")
                    self.active, self.next_t = None, t + self.gap
            elif not self.walkers[0].busy:
                self.get_logger().info(f"scenario {name}: done after {t - t0:.0f} s")
                self.active, self.next_t = None, t + self.gap
            return
        if not self.events or t < self.next_t:
            return
        ev = self.events[0]
        moving = math.hypot(st[3], st[4]) > 0.2
        name = ev.partition(":")[0]
        ready = (self.phase in ("bore", "pick")) if name == "tool" else moving
        if ready and self.begin(ev, st, t):
            self.events.pop(0)


def main():
    rclpy.init()
    n = Pedestrians()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
