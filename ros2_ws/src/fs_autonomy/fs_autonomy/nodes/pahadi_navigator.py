"""Pahadi navigator: contour rows on the hillside, local autonomy under an operator's tablet.

Plan   contour lines from the terrain survey (iso-elevation, boulders pushed out of the way),
       driven in alternating directions and joined by U-turns; a spot every <spacing> m beside
       the line at the arm's reach, on the downhill (or uphill) side. The tool for each spot
       comes from the surveyed depth to rock: auger, percussive pick, or seed balls.
Drive  pure pursuit on the front module (skid steer, passive articulation). Speed is cut on a
       stability WARN and the machine holds on CRITICAL; if traction reports immobilised it
       backs off and tries again.
Stop   with the arm base abreast of the spot, then the planting head does the rest
       (IK, bore or pick or seed, plug, water, half-moon).
Tablet <ns>/operator: pause | resume | skip | estop | release. `ros2 run fs_autonomy operator pause`.
"""
import math

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from fs_interfaces.action import PlantSpot
from fs_interfaces.msg import Mission, RobotState, Spot, StabilityStatus, TractionStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float64, String

from ..control import PurePursuit
from ..geometry import Polyline, yaw_from_quat
from ..planners import contour_plan, max_curvature, u_turn
from ..ros_common import LATCHED, load_site, param, pt, sim_now


def bezier(p0, h0, p3, h3, k=4.0, n=40):
    p0, p3 = np.asarray(p0, float), np.asarray(p3, float)
    p1 = p0 + k * np.array([math.cos(h0), math.sin(h0)])
    p2 = p3 - k * np.array([math.cos(h3), math.sin(h3)])
    t = np.linspace(0, 1, n)[:, None]
    return ((1 - t) ** 3) * p0 + 3 * ((1 - t) ** 2) * t * p1 + 3 * (1 - t) * t * t * p2 + t ** 3 * p3


class PahadiNavigator(Node):
    def __init__(self):
        super().__init__("navigator")
        self.site = load_site(self)
        self.ns = param(self, "robot", "pahadi")
        self.speed = param(self, "speed", 0.35)
        self.arm_x = param(self, "arm_x", 0.25)
        self.half_moon = param(self, "half_moon", True)
        n_rows = int(param(self, "rows", 3))
        max_spots = int(param(self, "max_spots", 0))
        rows, spots = contour_plan(self.site, n_rows=n_rows, row_spacing=param(self, "row_spacing", 4.0),
                                   spacing=param(self, "spacing", 2.5), reach=param(self, "reach", 1.25),
                                   side=param(self, "side", "downhill"), seed=int(param(self, "seed", 1)))
        if max_spots > 0:
            spots = spots[:max_spots]
        spawn = self.site.spawn(self.ns)
        # stitch: spawn -> row 0 -> turn -> row 1 ...; turns respect the articulation's radius
        self.turn_radius = param(self, "turn_radius", 3.4)
        x0, y0, h0 = rows[0].at(0)
        approach = min((bezier((spawn["x"], spawn["y"]), spawn["yaw"], (x0, y0), h0, k=k) for k in np.arange(2, 9, 0.5)),
                       key=max_curvature)
        pts = [tuple(p) for p in approach]
        for i, line in enumerate(rows):
            pts += [tuple(p) for p in line.p[1:]]
            if i + 1 < len(rows):
                ex, ey, eh = line.at(line.length)
                sx, sy, _ = rows[i + 1].at(0)
                pts += u_turn((ex, ey), eh, (sx, sy), self.turn_radius)[1:]
        self.path = Polyline(pts).resample(0.25)
        self.get_logger().info(f"tightest curve on the path: radius {1 / max(max_curvature(self.path.p), 1e-6):.1f} m")
        self.spots = []
        for sp in spots:
            sx, sy, _ = rows[sp.row].at(sp.station)
            s_glob, _, _ = self.path.project(sx, sy)
            self.spots.append((s_glob, sp))
        self.spots.sort(key=lambda t: t[0])
        self.pp = PurePursuit(self.path, lookahead=1.6, min_lookahead=1.2, k_speed=0.5)
        self.pp.hint = 0          # track progress locally from the start: rows double back near each other
        self.k = 0
        self.pose = None
        self.stab = None
        self.trac = None
        self.paused = False
        self.estop = False
        self.state = "idle"
        self.goal_fut = self.res_fut = None
        self.backoff_until = 0.0
        self.planted = self.skipped = 0
        self.t0 = None

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.artic_pub = self.create_publisher(Float64, "joint/artic_yaw_joint/cmd_pos", 10)
        self.half_length = param(self, "module_offset", 0.85)
        self.state_pub = self.create_publisher(RobotState, "state", 10)
        self.mission_pub = self.create_publisher(Mission, "mission", LATCHED)
        self.client = ActionClient(self, PlantSpot, "plant_spot")
        self.create_subscription(Odometry, "localization/odom", self.on_odom, 10)
        self.create_subscription(StabilityStatus, "stability", lambda m: setattr(self, "stab", m), 10)
        self.create_subscription(TractionStatus, "traction", lambda m: setattr(self, "trac", m), 10)
        self.create_subscription(String, "operator", self.on_operator, 10)
        self.create_timer(0.05, self.tick)
        self.create_timer(0.5, self.publish_state)
        self.publish_mission()
        tools = {}
        for _, sp in self.spots:
            tools[sp.tool] = tools.get(sp.tool, 0) + 1
        self.get_logger().info(f"{len(rows)} contour rows, path {self.path.length:.0f} m, {len(self.spots)} spots "
                               f"{tools}, slope {self.site.get('slope_deg')} deg, arm {param(self, 'side', 'downhill')}")

    def publish_mission(self):
        m = Mission()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.robot, m.pattern, m.spacing_m = self.ns, "contour", float(param(self, "spacing", 2.5))
        for _, sp in self.spots:
            s = Spot()
            s.id, s.position, s.row, s.station, s.tool = sp.id, pt(sp.x, sp.y), sp.row, sp.station, sp.tool
            m.spots.append(s)
        self.mission_pub.publish(m)

    def on_odom(self, m: Odometry):
        q = m.pose.pose.orientation
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_from_quat(q.x, q.y, q.z, q.w))

    def on_operator(self, m: String):
        c = m.data.strip().lower()
        if c == "pause":
            self.paused = True
        elif c == "resume":
            self.paused = False
        elif c == "estop":
            self.estop = True
        elif c == "release":
            self.estop = False
        elif c == "skip" and self.k < len(self.spots):
            self.get_logger().warning(f"operator skipped spot {self.spots[self.k][1].id}")
            self.k += 1
            self.skipped += 1
        self.get_logger().info(f"operator: {c}")

    def cmd(self, v=0.0, w=0.0):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.cmd_pub.publish(t)

    def tick(self):
        if self.pose is None:
            return
        now = sim_now(self)
        if self.t0 is None:
            self.t0 = now
        if self.estop or self.paused:
            self.cmd()
            self.state = "estop" if self.estop else "paused"
            return
        if self.goal_fut is not None or self.res_fut is not None:
            self.cmd()
            self.poll_plant()
            return
        x, y, th = self.pose
        if now < self.backoff_until:
            self.cmd(-0.2, 0.0)
            self.state = "back_off"
            return
        if self.trac is not None and self.trac.immobilised:
            self.get_logger().warning("immobilised: backing off 1 m and trying again")
            self.backoff_until = now + 5.0
            self.trac = None
            return
        if self.stab is not None and self.stab.level == StabilityStatus.CRITICAL:
            self.cmd()
            self.state = "stability_hold"
            return
        kappa = self.pp.step(x, y, th, self.speed)
        s = self.pp.s
        if self.k < len(self.spots):
            s_spot, sp = self.spots[self.k]
            gap = s_spot - (s + self.arm_x)
            if gap < -1.5:
                self.get_logger().warning(f"passed spot {sp.id} by {-gap:.1f} m; skipping it")
                self.k += 1
                self.skipped += 1
                return
            if gap <= 0.05:
                self.cmd()
                self.send(sp)
                return
            v = min(self.speed, max(0.08, 0.5 * gap))
        else:
            gap = self.path.length - s
            if gap < 0.3:
                self.cmd()
                if self.state != "done":
                    dt = (now - self.t0) / 60
                    self.get_logger().info(f"contours complete: {self.planted} planted/seeded, {self.skipped} "
                                           f"skipped in {dt:.1f} min")
                self.state = "done"
                return
            v = min(self.speed, max(0.08, 0.5 * gap))
        if self.stab is not None and self.stab.level == StabilityStatus.WARN:
            v *= 0.5
        self.state = "drive"
        # articulated steering: both modules on the same arc -> joint angle -2 atan(L kappa)
        gamma = max(-0.5, min(0.5, -2.0 * math.atan(self.half_length * kappa)))
        self.artic_pub.publish(Float64(data=gamma))
        self.cmd(v, max(-0.5, min(0.5, v * kappa)))

    def send(self, sp):
        g = PlantSpot.Goal()
        g.spot_id, g.target, g.tool, g.half_moon = sp.id, pt(sp.x, sp.y), sp.tool, bool(self.half_moon)
        self.goal_fut = self.client.send_goal_async(g)
        self.state = "plant"

    def poll_plant(self):
        if self.goal_fut is not None and self.goal_fut.done():
            gh = self.goal_fut.result()
            self.goal_fut = None
            if gh.accepted:
                self.res_fut = gh.get_result_async()
        if self.res_fut is not None and self.res_fut.done():
            res = self.res_fut.result()
            self.res_fut = None
            ok = res.status == GoalStatus.STATUS_SUCCEEDED and res.result.success
            self.planted += int(ok)
            self.skipped += int(not ok)
            self.k += 1

    def publish_state(self):
        if self.pose is None:
            return
        s = RobotState()
        s.header.stamp = self.get_clock().now().to_msg()
        s.robot, s.state = self.ns, self.state
        s.x, s.y, s.yaw = self.pose
        s.planted, s.skipped = self.planted, self.skipped
        self.state_pub.publish(s)


def main():
    rclpy.init()
    n = PahadiNavigator()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
