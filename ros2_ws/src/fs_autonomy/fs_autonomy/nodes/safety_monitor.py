"""Safety monitor: the last stage between a robot's planner and its wheels, one per robot.

  navigator / driver --cmd_vel_nav--> safety_monitor --cmd_vel--> Gazebo (or the drive controller)

Each cycle it clusters the lidar scans (front and rear on Jaal, the boom side on Katar, front on
Pahadi), drops hits on the surveyed ground, tracks the clusters in the wheel-odometry frame (smooth
over seconds, unlike the map frame, which jumps when GNSS corrections arrive) and applies
fs_autonomy.safety:

  person within 1.2 m of the footprint           stop (any direction, even pivoting)
  person in the path or crossing it within 3 s   stop; slow from 6 m
  static object in the path                      stop at 1.5 m; slow from 5 m
  person within 2 m of the tool                  tool_hold: the planting head freezes auger and joints
  e-stop (/fleet/estop true, or operator "estop") latched stop until "release" / false
  no command for 0.4 s                           zero output (watchdog)
  localization older than 1 s, or off the site   stop

A stop is released only after the zone has been clear for clear_s (default 2 s), so a person
stepping in and out does not make the robot lurch. Everything is published on safety/status
(fs_interfaces/SafetyStatus) for the navigator, the planting head and RViz.
"""
import math

import numpy as np
import rclpy
from fs_interfaces.msg import PlantRecord, RobotState, SafetyStatus
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from .. import safety
from ..geometry import rpy_from_quat, transform_to_matrix, yaw_from_quat
from ..ros_common import load_site, lookup, param, sim_now

SELF_LINKS = {   # moving parts of the robot itself that a lidar can see; points near them are dropped
    "jaal": ["auger_link", "duckbill_feed_link", "tamp_link", "shuttle_link"],
    "katar": ["auger_link", "duckbill_feed_link", "tamp_link", "jaw_l_link", "jaw_r_link", "carriage_link"],
    "pahadi": ["auger_link", "auger_feed_link", "arm_wrist_link", "arm_fore_link", "finger_l_link", "rear_link"],
}

BASE_HEIGHT = {"katar": 0.72, "jaal": 0.38, "pahadi": 0.21}     # base_link above the ground beneath it
# ground-filter margin: fixed + per metre of range. Jaal's and Katar's lidars sit 0.3 m up (a margin that
# grows would hide a person's shins); Pahadi's sits 0.7 m up on a tilting machine on a slope.
GROUND_MARGIN = {"katar": (0.12, 0.0), "jaal": (0.12, 0.0), "pahadi": (0.15, 0.04)}
SELF_BOXES = {   # the machine's own body as its lidars see it, one box per link frame
    "jaal": [("base_link", safety.Footprint(-1.7, 1.7, -1.35, 1.35))],     # steered 90 deg, the tyres reach y = +-1.2
    "katar": [("base_link", safety.Footprint(-1.6, 2.6, -1.35, 1.35)),     # tractor, front weights
              ("trailer_link", safety.Footprint(-5.0, 0.3, -1.4, 1.4))],  # trailer: swings on the hitch
    "pahadi": [("base_link", safety.Footprint(-2.4, 0.85, -0.75, 0.75))],
}


class SafetyMonitor(Node):
    def __init__(self):
        super().__init__("safety_monitor")
        self.site = load_site(self)
        self.ns = param(self, "robot", "jaal_1")
        self.kind = param(self, "robot_type", "jaal")
        self.cfg = safety.SafetyConfig.for_robot(self.kind)
        self.clear_s = float(param(self, "clear_s", 2.0))
        self.tool_clear_s = float(param(self, "tool_clear_s", 3.0))
        self.tool_link = param(self, "tool_link", "auger_link")
        self.self_links = list(param(self, "self_links", SELF_LINKS[self.kind]))
        self.self_r = float(param(self, "self_radius", 0.4))
        self.self_boxes = SELF_BOXES[self.kind]
        self.base_h = float(param(self, "base_height", BASE_HEIGHT[self.kind]))
        self.ground_margin = GROUND_MARGIN[self.kind]
        self.rp = (0.0, 0.0)
        scans = list(param(self, "scans", ["scan"]))
        peers = [p for p in param(self, "peers", [""]) if p and p != self.ns]

        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        surveyed = ((self.site.get("obstacles", []) or []) + (self.site.get("boulders", []) or [])
                    + ((self.site.get("roadside") or {}).get("trees", []) or []))
        self.zones_map = [(float(o["x"]), float(o["y"]), float(o.get("r", 0.3)) + 1.0) for o in surveyed]
        self.tracker = safety.Tracker()
        self.opose, self.odom_t0 = None, None
        self.warmup_s = float(param(self, "warmup_s", 5.0))
        self.scans = {}
        self.pose = None
        self.pose_t = -1.0
        self.peer_pos = {}
        self.cmd = Twist()
        self.cmd_t = -1.0
        self.estop = False
        self.A = safety.Assessment()
        self.out_scale = 1.0
        self.stop_reason = ""
        self.clear_since = None
        self.hold = False
        self.hold_clear_since = None
        self.block_id, self.block_since = None, None
        self.last_log = -1e9

        for s in scans:
            self.create_subscription(LaserScan, s, lambda m, s=s: self.scans.__setitem__(s, m), qos_profile_sensor_data)
        self.create_subscription(Odometry, "localization/odom", self.on_odom, 20)
        self.create_subscription(Odometry, "odom", self.on_wheel_odom, 20)
        self.create_subscription(Imu, "imu", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(PlantRecord, "/plants/records", self.on_plant, 50)
        self.create_subscription(Twist, "cmd_vel_nav", self.on_cmd, 20)
        self.create_subscription(Bool, "/fleet/estop", lambda m: self.set_estop(m.data, "fleet"), 10)
        self.create_subscription(String, "operator", self.on_operator, 10)
        for p in peers:
            self.create_subscription(RobotState, f"/{p}/state", lambda m, p=p: self.peer_pos.__setitem__(p, (m.x, m.y)), 10)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.status_pub = self.create_publisher(SafetyStatus, "safety/status", 10)
        self.create_timer(0.1, self.tick)
        sm = self.site.soil_map
        self.bounds = (sm.x0 + 1.0, sm.y0 + 1.0, sm.x0 + (sm.nx - 1) * sm.res - 1.0, sm.y0 + (sm.ny - 1) * sm.res - 1.0)
        self.get_logger().info(f"{self.kind} safety monitor: scans {scans}, footprint {self.cfg.footprint}, "
                               f"peers {peers or 'none'}")

    # ------------------------------------------------------------------ inputs
    def on_odom(self, m: Odometry):
        q = m.pose.pose.orientation
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_from_quat(q.x, q.y, q.z, q.w))
        self.pose_t = sim_now(self)

    def on_wheel_odom(self, m: Odometry):
        q = m.pose.pose.orientation
        self.opose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_from_quat(q.x, q.y, q.z, q.w))
        if self.odom_t0 is None:
            self.odom_t0 = sim_now(self)

    def frame_change(self, src, dst, x, y):
        """Point from the frame where the robot is at pose src to the frame where it is at dst."""
        c, s = math.cos(src[2]), math.sin(src[2])
        dx, dy = x - src[0], y - src[1]
        bx, by = c * dx + s * dy, -s * dx + c * dy
        c, s = math.cos(dst[2]), math.sin(dst[2])
        return dst[0] + c * bx - s * by, dst[1] + s * bx + c * by

    def on_plant(self, m: PlantRecord):
        """A sapling just planted is a static object: its track can never become a person. The zone is
        small (0.5 m) so a person walking along a planted row is still seen as one."""
        if m.outcome == "planted":
            self.zones_map.append((float(m.actual.x), float(m.actual.y), 0.5))

    def on_imu(self, m: Imu):
        q = m.orientation
        r, p, _ = rpy_from_quat(q.x, q.y, q.z, q.w)
        self.rp = (r, p)

    def on_operator(self, m: String):
        if m.data == "estop":
            self.set_estop(True, "operator")
        elif m.data == "release":
            self.set_estop(False, "operator")

    def set_estop(self, on: bool, src: str):
        if on != self.estop:
            (self.get_logger().error if on else self.get_logger().info)(f"e-stop {'ENGAGED' if on else 'released'} ({src})")
        self.estop = on
        if on:
            self.cmd_pub.publish(Twist())

    def on_cmd(self, m: Twist):
        self.cmd, self.cmd_t = m, sim_now(self)
        self.publish_cmd()

    # ------------------------------------------------------------------ geometry
    def base_M(self, frame):
        """4x4 transform base_link <- frame."""
        t = lookup(self.tf, f"{self.ns}/base_link", frame, timeout=0.0)
        return None if t is None else transform_to_matrix(t.transform)

    def base_T(self, frame):
        """3x3 2-D transform base_link <- frame (looked up each time: some links move)."""
        M = self.base_M(frame)
        if M is None:
            return None
        return np.array([[M[0, 0], M[0, 1], M[0, 3]], [M[1, 0], M[1, 1], M[1, 3]], [0, 0, 1]])

    def link_xy(self, link):
        T = self.base_T(f"{self.ns}/{link}")
        return None if T is None else (float(T[0, 2]), float(T[1, 2]))

    def detections(self):
        sets = []
        selfs = [p for p in (self.link_xy(l) for l in self.self_links) if p is not None]
        for name, s in self.scans.items():
            M = self.base_M(s.header.frame_id)         # looked up each time: Katar's lidar rides the trailer
            if M is None:
                continue
            q = safety.scan_points(s.ranges, s.angle_min, s.angle_increment, s.range_min, s.range_max)
            P = np.c_[q, np.zeros(len(q))] @ M[:3, :3].T + M[:3, 3] if len(q) else np.zeros((0, 3))
            p = P[:, :2]
            if len(p):
                keep = np.ones(len(p), bool)
                for link, box in self.self_boxes:                                  # own body and wheels
                    L = np.eye(3) if link == "base_link" else self.base_T(f"{self.ns}/{link}")
                    if L is None:
                        continue
                    Li = np.linalg.inv(L)
                    q2 = p @ Li[:2, :2].T + Li[:2, 2]
                    keep &= ~box.contains(q2[:, 0], q2[:, 1])
                if self.pose is not None:                                          # hits on the ground itself
                    keep &= safety.above_ground(P, self.rp[0], self.rp[1], self.pose[2], self.pose[:2], self.base_h,
                                                self.site.soil_map.elevation, *self.ground_margin)
                for sx, sy in selfs:
                    keep &= np.hypot(p[:, 0] - sx, p[:, 1] - sy) > self.self_r      # own moving tools
                sets.append(p[keep])
        return safety.clusters(sets)

    # ------------------------------------------------------------------ main loop
    def tick(self):
        t = sim_now(self)
        if self.pose is not None and self.opose is not None:
            x, y, th = self.opose
            c, s = math.cos(th), math.sin(th)
            dets = [(x + c * bx - s * by, y + s * bx + c * by, r) for bx, by, r in self.detections()]
            to_odom = lambda px, py: self.frame_change(self.pose, self.opose, px, py)
            self.tracker.zones = [(*to_odom(zx, zy), zr) for zx, zy, zr in self.zones_map
                                  if abs(zx - self.pose[0]) < 25 and abs(zy - self.pose[1]) < 25]
            before = {tr.id for tr in self.tracker.tracks if tr.moving}
            self.tracker.update(t, dets, peers=[to_odom(*p) for p in self.peer_pos.values()], robot=(x, y, th))
            if t - self.odom_t0 < self.warmup_s:          # physics and localization still settling
                for tr in self.tracker.tracks:
                    tr.moving, tr.hist = False, [(t, tr.x, tr.y)]
            for tr in self.tracker.tracks:
                if tr.moving and tr.id not in before and tr.kind == "object":
                    h = tr.hist[0]
                    self.get_logger().info(f"track {tr.id}: moving object at odom ({tr.x:.1f}, {tr.y:.1f}), r {tr.r:.2f} m, "
                                           f"{math.hypot(tr.x - h[1], tr.y - h[2]):.2f} m in {t - h[0]:.1f} s, "
                                           f"first seen {t - tr.t_first:.1f} s ago at ({tr.x0:.1f}, {tr.y0:.1f})")
        self.evaluate(t)
        self.publish_status(t)
        if t - self.cmd_t > 0.4:                      # watchdog: the planner went quiet
            self.cmd_pub.publish(Twist())

    def evaluate(self, t):
        """Run the zones for the current command and apply the hysteresis."""
        reason = ""
        A = safety.Assessment()
        if self.estop:
            reason = "estop"
        elif self.pose is None or t - self.pose_t > 1.0 or self.opose is None:
            reason = "localization"
        else:
            x, y, _ = self.pose
            v = self.cmd
            objs = safety.to_base(self.tracker.tracks, self.opose)
            A = safety.assess(objs, (v.linear.x, v.linear.y, v.angular.z), self.cfg, self.link_xy(self.tool_link))
            out = not (self.bounds[0] < x < self.bounds[2] and self.bounds[1] < y < self.bounds[3])
            if out:
                reason = "geofence"
            elif A.scale <= 0.0:
                reason = A.reason
        self.A = A
        scale = 0.0 if reason else A.scale
        if scale <= 0.0:
            self.clear_since = None
            if reason != self.stop_reason and t - self.last_log > 2.0:
                self.last_log = t
                where = f" at ({A.blocker_base[0]:+.1f}, {A.blocker_base[1]:+.1f}) m" if A.blocker else ""
                self.get_logger().warning(f"protective stop: {reason}{where}")
            self.stop_reason = reason
        elif self.stop_reason:                          # stopped: wait for the zone to stay clear
            self.clear_since = self.clear_since if self.clear_since is not None else t
            if t - self.clear_since >= self.clear_s and self.stop_reason != "estop":
                self.get_logger().info(f"zone clear for {self.clear_s:.0f} s after {self.stop_reason}: resuming")
                self.stop_reason = ""
            else:
                scale = 0.0
        self.out_scale = scale
        # blocked-by-the-same-object timer, for the navigator's reroute decision
        b = A.blocker if scale <= 0.0 and A.blocker is not None and A.reason in ("obstacle", "person_in_path", "person_crossing") else None
        if b is None:
            self.block_id, self.block_since = None, None
        elif b.id != self.block_id:
            self.block_id, self.block_since = b.id, t
        # tool zone
        if A.tool_hold or self.estop:
            if not self.hold:
                self.get_logger().warning(f"tool hold: person in the tool zone ({A.notes[0]})" if A.tool_hold and A.notes
                                          else "tool hold: e-stop" if self.estop else "tool hold: person in the tool zone")
            self.hold, self.hold_clear_since = True, None
        elif self.hold:
            self.hold_clear_since = self.hold_clear_since if self.hold_clear_since is not None else t
            if t - self.hold_clear_since >= self.tool_clear_s:
                self.hold = False
                self.get_logger().info("tool zone clear: tool released")

    def publish_cmd(self):
        v = self.cmd
        t = sim_now(self)
        if self.opose is not None and not self.estop:
            # re-check against this command's direction (the navigator may have turned away from the blocker)
            objs = safety.to_base(self.tracker.tracks, self.opose)
            A = safety.assess(objs, (v.linear.x, v.linear.y, v.angular.z), self.cfg)
            k = 0.0 if self.stop_reason else min(1.0, A.scale)
        else:
            k = 0.0
        if k <= 0.0 and self.out_scale > 0.0 and not self.stop_reason:
            self.evaluate(t)                            # new stop: let the hysteresis and logging see it
            k = 0.0
        out = Twist()
        out.linear.x, out.linear.y, out.angular.z = v.linear.x * k, v.linear.y * k, v.angular.z * (k if k > 0 else 0.0)
        self.cmd_pub.publish(out)

    def publish_status(self, t):
        A = self.A
        m = SafetyStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = f"{self.ns}/base_link"
        m.robot = self.ns
        m.speed_scale = float(self.out_scale)
        m.level = (SafetyStatus.ESTOP if self.estop else SafetyStatus.STOP if self.out_scale <= 0.0
                   else SafetyStatus.SLOW if self.out_scale < 1.0 else SafetyStatus.CLEAR)
        m.reason = self.stop_reason or (A.reason if self.out_scale < 1.0 else "")
        m.tool_hold = bool(self.hold)
        m.nearest_person_m = float(A.nearest_person)
        if self.block_id is not None and A.blocker is not None:
            m.blocked = True
            m.blocker_x, m.blocker_y = float(A.blocker_base[0]), float(A.blocker_base[1])
            m.blocker_r, m.blocker_moving = float(A.blocker.r), bool(A.blocker.moving)
            m.blocked_s = float(t - self.block_since)
        m.free_left, m.free_right = bool(A.free_left), bool(A.free_right)
        for tr in self.tracker.tracks:
            if tr.moving and tr.kind == "object" and self.pose is not None and self.opose is not None:
                mx, my = self.frame_change(self.opose, self.pose, tr.x, tr.y)
                m.people.append(Point(x=float(mx), y=float(my), z=float(tr.speed)))
        self.status_pub.publish(m)


def main():
    rclpy.init()
    n = SafetyMonitor()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
