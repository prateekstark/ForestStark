"""Jaal navigator: executes one robot's mission on the swerve-drive base.

Rows   The robot drives along the row facing the direction of travel, so the rear gantry trails
       and never passes over a planted sapling. It stops with the gantry over the spot
       (base = spot + 1.65 m along the row), holonomically: velocity toward the stop point plus
       heading hold, slowing on a sqrt(2 a d) profile and finishing on a P-law to +-3 cm.
Turns  At the headland it pivots in place to the new row heading and crabs sideways onto the
       next row: no three-point turns, the 4WS base makes this cheap.
Detour Surveyed obstacles (termite mounds, stumps) on a row are passed by crabbing toward the
       unplanted side.
Stones An auger refusal means an unsurveyed stone: shift 0.3 m along the row and retry once.
Tender Plugs, water or battery low -> leave the row through the lane between rows, reserve a bay,
       wait in its own slot if the bay is taken, service, and come back to the same spot.
Safety Front lidar: anything inside the travel corridor slows the robot from 5 m and stops it at
       1.5 m (people, livestock, other robots). Other robots also share their positions.
GNSS   rtk_policy wait: stop while RTK is lost (up to rtk_wait_s), then continue on odometry.
Battery A power model (rolling resistance, scrub, auger, hotel load, solar) gives the SoC.
"""
import math

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from fs_interfaces.action import PlantSpot, TenderService
from fs_interfaces.msg import HeadStatus, Mission, PlantRecord, RobotState, TractionStatus
from fs_interfaces.srv import ReserveBay
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, String

from ..control import approach_speed
from ..geometry import wrap, yaw_from_quat
from ..ros_common import LATCHED, SENSOR, load_site, param, pt, sim_now

G = 9.81


class Task:
    def __init__(self, kind, **kw):
        self.kind = kind
        self.__dict__.update(kw)

    def relabel(self, label):
        fields = {k: v for k, v in self.__dict__.items() if k != "kind"}
        return Task(self.kind, **{**fields, "label": label})

    def __repr__(self):
        return f"Task({self.kind})"


class JaalNavigator(Node):
    def __init__(self):
        super().__init__("navigator")
        self.site = load_site(self)
        self.ns = param(self, "robot", "jaal_1")
        self.peers = [r for r in param(self, "robots", ["jaal_1", "jaal_2", "jaal_3"]) if r != self.ns]
        self.cruise = param(self, "cruise_speed", 1.0)
        self.decel = param(self, "decel", 0.5)
        self.gantry = param(self, "gantry_offset", 1.65)
        self.headland = param(self, "headland_ext", 2.8)
        self.use_lidar = param(self, "use_lidar", True)
        self.slow_d, self.stop_d = param(self, "slow_distance", 5.0), param(self, "stop_distance", 1.5)
        self.rtk_policy = param(self, "rtk_policy", "continue")
        self.rtk_wait = param(self, "rtk_wait_s", 60.0)
        self.dose = param(self, "water_dose_l", 0.5)
        self.capacity = int(param(self, "plugs_capacity", 360))
        self.water_capacity = param(self, "water_capacity_l", 250.0)
        self.kwh = param(self, "battery_kwh", 30.0)
        self.soc = param(self, "initial_soc", 0.9)
        self.mass = param(self, "mass_kg", 1000.0)
        self.crr = param(self, "rolling_resistance", 0.07)
        self.hotel_w = param(self, "hotel_w", 350.0)
        self.solar_w = param(self, "solar_w", 450.0)
        f = self.site["field"]
        poly = f["demo_polygon"] if param(self, "field", "demo") == "demo" else f["polygon"]
        self.field_x0 = min(p[0] for p in poly)
        self.obstacles = [o for o in self.site.get("obstacles", []) if o["kind"] != "stone"]

        self.pose = None
        self.v_body = (0.0, 0.0, 0.0)
        self.scan = None
        self.gnss = "rtk_fix"
        self.head = None
        self.peer_pos = {}
        self.tasks = []
        self.task = None
        self.task_t = 0.0
        self.state = "idle"
        self.planted = self.skipped = 0
        self.mission = None
        self.row_dy = 3.0
        self.goal_fut = self.res_fut = None
        self.lost_fix_since = None
        self.last_t = None
        self.last_cmd = Twist()
        self.bay = None
        self.blocker = ("none", 0.0, 0.0)
        self.blocked_since = None
        self.detour_after = param(self, "detour_after_s", 20.0)
        self.n_bays = int(param(self, "bays", 1))

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.state_pub = self.create_publisher(RobotState, "state", 10)
        self.refill_pub = self.create_publisher(Float64MultiArray, "head/refill", 10)
        self.record_pub = self.create_publisher(PlantRecord, "/plants/records", 50)
        self.plant_cli = ActionClient(self, PlantSpot, "plant_spot")
        self.tender_cli = ActionClient(self, TenderService, "/fleet/tender_service")
        self.bay_cli = self.create_client(ReserveBay, "/fleet/reserve_bay")
        self.create_subscription(Mission, "mission", self.on_mission, LATCHED)
        self.create_subscription(Odometry, "localization/odom", self.on_odom, 10)
        self.create_subscription(LaserScan, "scan", lambda m: setattr(self, "scan", m), SENSOR)
        self.create_subscription(String, "gnss/status", lambda m: setattr(self, "gnss", m.data), 10)
        self.create_subscription(HeadStatus, "head/status", lambda m: setattr(self, "head", m), 10)
        self.create_subscription(TractionStatus, "traction", self.on_traction, 10)
        for p in self.peers:
            self.create_subscription(RobotState, f"/{p}/state", lambda m, p=p: self.peer_pos.__setitem__(p, (m.x, m.y)), 10)
        self.create_timer(0.05, self.tick)
        self.create_timer(0.5, self.publish_state)

    # ------------------------------------------------------------------ inputs
    def on_odom(self, m: Odometry):
        q = m.pose.pose.orientation
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_from_quat(q.x, q.y, q.z, q.w))

    def on_mission(self, m: Mission):
        if self.mission is not None:
            return
        self.mission = m
        self.row_dy = m.row_spacing_m or 3.0
        self.tasks = self.build(m)
        n = sum(1 for t in self.tasks if t.kind == "plant")
        self.get_logger().info(f"mission: {len(m.spots)} spots ({n} to plant), {len(self.tasks)} tasks")

    def on_traction(self, m: TractionStatus):
        """Wheels spinning without progress: something the lidar cannot see is in the way. Back off
        and go round it."""
        if not m.immobilised or self.task is None or self.task.kind != "goto" or self.pose is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - getattr(self, "_last_recovery", -1e9) < 15:
            return
        self._last_recovery = now
        sp = math.hypot(self.last_cmd.linear.x, self.last_cmd.linear.y) or 1.0
        ux, uy = self.last_cmd.linear.x / sp, self.last_cmd.linear.y / sp
        x, y, th = self.pose
        bx, by = -ux * 1.0, -uy * 1.0                                     # one metre back along the way we came
        back = (x + math.cos(th) * bx - math.sin(th) * by, y + math.sin(th) * bx + math.cos(th) * by)
        self.blocker = ("lidar", 1.6 * ux, 1.6 * uy)                      # treat it as straight ahead
        cmd = Twist()
        cmd.linear.x, cmd.linear.y = ux, uy
        t = self.task
        self.task = Task("goto", x=back[0], y=back[1], yaw=th, tol=0.2, label="detour")
        self.tasks.insert(0, t)
        self.detour_around_from(back, th, ux, uy, t)
        self.get_logger().warning("immobilised against something the lidar cannot see: backing off and going round")

    def detour_around_from(self, origin, th, ux, uy, resume):
        x, y = origin
        def world(a, l):
            px, py = ux * a - uy * l, uy * a + ux * l
            return x + math.cos(th) * px - math.sin(th) * py, y + math.sin(th) * px + math.cos(th) * py
        p1, p2 = world(0.0, 1.8), world(4.0, 1.8)
        self.tasks[0:0] = [Task("goto", x=p1[0], y=p1[1], yaw=th, tol=0.2, label="detour"),
                           Task("goto", x=p2[0], y=p2[1], yaw=th, tol=0.2, label="detour")]

    # ------------------------------------------------------------------ mission -> tasks
    def build(self, m: Mission):
        rows = []
        for s in m.spots:
            if not rows or rows[-1][0] != s.row:
                rows.append((s.row, []))
            rows[-1][1].append(s)
        tasks = []
        for i, (row, spots) in enumerate(rows):
            xs = [s.position.x for s in spots]
            d = 1.0 if len(spots) < 2 or xs[-1] >= xs[0] else -1.0
            if len(spots) < 2:
                d = 1.0 if i % 2 == 0 else -1.0
            yaw = 0.0 if d > 0 else math.pi
            y = spots[0].position.y
            entry_x = spots[0].position.x - d * (self.headland - self.gantry)
            tasks.append(Task("goto", x=entry_x, y=y, yaw=yaw, tol=0.15, label="turn"))
            prev_x = entry_x
            for s in spots:
                stop_x = s.position.x + d * self.gantry
                tasks += self.detours(prev_x, stop_x, y, d, yaw)
                if s.tool == "skip":
                    tasks.append(Task("skip", spot=s))
                    continue
                tasks.append(Task("goto", x=stop_x, y=s.position.y, yaw=yaw, tol=0.03, label="drive"))
                tasks.append(Task("plant", spot=s, x=s.position.x, y=s.position.y, d=d, yaw=yaw, retry=0))
                prev_x = stop_x
            exit_x = spots[-1].position.x + d * (self.gantry + self.headland)
            tasks += self.detours(prev_x, exit_x, y, d, yaw)
            tasks.append(Task("goto", x=exit_x, y=y, yaw=yaw, tol=0.2, label="drive"))
        tasks.append(Task("park"))
        return tasks

    def detours(self, x0, x1, y, d, yaw):
        """Crab around surveyed obstacles in the corridor between x0 and x1 on row y."""
        out = []
        lo, hi = min(x0, x1), max(x0, x1)
        hits = [o for o in self.obstacles if lo - o["r"] - 1.5 < o["x"] < hi + o["r"] + 1.5
                and abs(o["y"] - y) < o["r"] + 1.2]
        for o in sorted(hits, key=lambda o: d * o["x"]):
            side = 1.0 if o["y"] <= y else -1.0          # pass on the far side of the obstacle's centre
            off = o["y"] + side * (o["r"] + 1.4)
            a = o["x"] - d * (o["r"] + 2.0)
            b = o["x"] + d * (o["r"] + 2.0)
            out += [Task("goto", x=a, y=y, yaw=yaw, tol=0.15, label="detour"),
                    Task("goto", x=a, y=off, yaw=yaw, tol=0.15, label="detour"),
                    Task("goto", x=b, y=off, yaw=yaw, tol=0.15, label="detour"),
                    Task("goto", x=b, y=y, yaw=yaw, tol=0.1, label="detour")]
        return out

    def tender_trip(self, resume):
        """Leave through the lane, service at the tender, come back to `resume` (a goto task)."""
        x, y, _ = self.pose
        lane = y - self.row_dy / 2
        head_x = self.field_x0 - 3.0
        bx, by, _ = self.site["tender"]["bay"]
        exit_y = by + 4.5 + 4.0 * (self.n_bays - 1)
        # lane legs are routed round surveyed obstacles just like the rows
        out_leg = [t.relabel("to_tender") for t in self.detours(x, head_x, lane, -1.0, math.pi)]
        back_leg = [t.relabel("return") for t in self.detours(-1.5, resume.x, lane, 1.0, 0.0)]
        go = [Task("goto", x=x, y=lane, yaw=self.pose[2], tol=0.2, label="to_tender"),
              Task("goto", x=x, y=lane, yaw=math.pi, tol=0.2, label="to_tender"), *out_leg,
              Task("goto", x=head_x, y=lane, yaw=math.pi, tol=0.3, label="to_tender"),
              Task("reserve"), Task("service"), Task("release"),
              # one-way loop: in from the queue column to the south, out to the north, back down x = -1.5
              Task("goto", x=bx, y=exit_y, yaw=0.0, tol=0.3, label="return"),
              Task("goto", x=-1.5, y=exit_y, yaw=0.0, tol=0.3, label="return"),
              Task("goto", x=-1.5, y=lane, yaw=0.0, tol=0.3, label="return"), *back_leg,
              Task("goto", x=resume.x, y=lane, yaw=resume.yaw, tol=0.2, label="return"),
              Task("goto", x=resume.x, y=resume.y, yaw=resume.yaw, tol=0.03, label="return")]
        return go

    # ------------------------------------------------------------------ control
    def safety_scale(self, vx, vy):
        """0..1 speed factor from obstacles in the direction of travel."""
        sp = math.hypot(vx, vy)
        if sp < 1e-3:
            return 1.0
        ux, uy = vx / sp, vy / sp
        nearest = 1e9
        if self.use_lidar and self.scan is not None:
            s = self.scan
            ang = s.angle_min + np.arange(len(s.ranges)) * s.angle_increment
            r = np.asarray(s.ranges)
            ok = np.isfinite(r) & (r > s.range_min) & (r < s.range_max)
            px, py = 1.52 + r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])
            body = (np.abs(px) < 1.55) & (np.abs(py) < 1.18)       # own wheels (steered) and frame
            px, py = px[~body], py[~body]
            along = px * ux + py * uy
            across = np.abs(-px * uy + py * ux)
            front = 1.45 if abs(ux) > 0.7 else 0.95
            m = (across < 1.1) & (along > 0)
            if m.any():
                i = int(np.argmin(np.where(m, along, np.inf)))
                nearest = float(along[i]) - front
                self.blocker = ("lidar", float(px[i]), float(py[i]))
        if self.pose is not None:
            x, y, th = self.pose
            for p, (qx, qy) in self.peer_pos.items():
                dx, dy = qx - x, qy - y
                bx, by = math.cos(-th) * dx - math.sin(-th) * dy, math.sin(-th) * dx + math.cos(-th) * dy
                along = bx * ux + by * uy
                if along > 0 and abs(-bx * uy + by * ux) < 2.2 and p < self.ns and along - 3.0 < nearest:
                    nearest = along - 3.0
                    self.blocker = (p, bx, by)
        if nearest < self.stop_d:
            return 0.0
        if nearest < self.slow_d:
            return 0.2 + 0.8 * (nearest - self.stop_d) / (self.slow_d - self.stop_d)
        return 1.0

    def drive_to(self, t: Task, dt: float) -> bool:
        x, y, th = self.pose
        dx, dy = t.x - x, t.y - y
        dist = math.hypot(dx, dy)
        err = wrap(t.yaw - th)
        cmd = Twist()
        if abs(err) > 0.3 and dist > 0.25:
            cmd.angular.z = max(-0.6, min(0.6, 1.5 * err))          # pivot in place first
        else:
            sp = approach_speed(dist, self.cruise, self.decel, 0.03)
            if dist < 0.4:
                sp = min(sp, max(0.03, 1.0 * dist))
            vxm, vym = (dx / dist * sp, dy / dist * sp) if dist > 1e-3 else (0.0, 0.0)
            cmd.linear.x = math.cos(-th) * vxm - math.sin(-th) * vym
            cmd.linear.y = math.sin(-th) * vxm + math.cos(-th) * vym
            cmd.angular.z = max(-0.5, min(0.5, 2.0 * err))
            k = self.safety_scale(cmd.linear.x, cmd.linear.y)
            now = self.get_clock().now().nanoseconds * 1e-9
            if k == 0.0:
                self.blocked_since = self.blocked_since or now
                if now - self.blocked_since > self.detour_after and self.detour_around(t, cmd):
                    return False
                self.set_state("safety_stop")
                if self.get_clock().now().nanoseconds * 1e-9 - getattr(self, "_blog", 0) > 10:
                    self._blog = self.get_clock().now().nanoseconds * 1e-9
                    self.get_logger().warning(f"safety stop: {self.blocker[0]} at ({self.blocker[1]:.2f}, {self.blocker[2]:.2f}) "
                                              f"in the body frame, moving ({cmd.linear.x:.2f}, {cmd.linear.y:.2f})")
            else:
                self.blocked_since = None
            cmd.linear.x *= k
            cmd.linear.y *= k
        self.publish_cmd(cmd)
        return dist < t.tol and abs(err) < 0.06

    def detour_around(self, t: Task, cmd: Twist) -> bool:
        """Blocked for a while by something that is not moving off: crab around it on the free side."""
        if getattr(t, "label", "") == "detour":
            return False
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.blocker[0] != "lidar" and now - self.blocked_since < 2 * self.detour_after:
            return False                      # a peer usually moves off; give it longer before going round
        x, y, th = self.pose
        sp = math.hypot(cmd.linear.x, cmd.linear.y)
        ux, uy = cmd.linear.x / sp, cmd.linear.y / sp                        # travel direction, body frame
        bx, by = self.blocker[1], self.blocker[2]
        blocker_left = bx * -uy + by * ux                                    # its offset to the left of travel
        lat = -1.6 if blocker_left > 0 else 1.6                              # pass on the other side
        ahead = (bx * ux + by * uy) + 2.5
        def world(a, l):
            px, py = ux * a - uy * l, uy * a + ux * l
            return x + math.cos(th) * px - math.sin(th) * py, y + math.sin(th) * px + math.cos(th) * py
        p1, p2 = world(0.0, lat), world(ahead, lat)
        self.tasks.insert(0, t)
        self.tasks[0:0] = [Task("goto", x=p1[0], y=p1[1], yaw=th, tol=0.2, label="detour"),
                           Task("goto", x=p2[0], y=p2[1], yaw=th, tol=0.2, label="detour")]
        self.task = None
        self.blocked_since = None
        self.get_logger().warning(f"blocked by {'a stationary ' + self.blocker[0] if self.blocker[0] != 'lidar' else 'a static object'}; detouring "
                                  f"{abs(lat):.1f} m to the {'left' if lat > 0 else 'right'}")
        return True

    def publish_cmd(self, cmd: Twist):
        self.last_cmd = cmd
        self.cmd_pub.publish(cmd)

    def stop(self):
        self.publish_cmd(Twist())

    def set_state(self, s):
        self.state = s

    def battery(self, dt):
        v = math.hypot(self.last_cmd.linear.x, self.last_cmd.linear.y)
        p = self.crr * self.mass * G * v / 0.85 + abs(self.last_cmd.angular.z) * 1500.0 + self.hotel_w
        if self.head is not None:
            p += self.head.auger_power_w
        p -= self.solar_w
        self.soc = max(0.0, self.soc - p * dt / (self.kwh * 3.6e6))

    def needs_service(self) -> bool:
        if self.head is None:
            return False
        return self.head.plugs_left < 1 or self.head.water_left_l < self.dose or self.soc < 0.2

    # ------------------------------------------------------------------ main loop
    def tick(self):
        t = sim_now(self)
        dt = 0.05 if self.last_t is None else max(1e-3, t - self.last_t)
        self.last_t = t
        if self.pose is None or self.mission is None:
            return
        self.battery(dt)
        if self.task is None:
            if not self.tasks:
                self.stop()
                self.set_state("done")
                return
            self.task = self.tasks.pop(0)
            self.task_t = t
        k = self.task

        if self.gnss != "rtk_fix" and k.kind == "goto" and self.rtk_policy == "wait":
            self.lost_fix_since = self.lost_fix_since or t
            if t - self.lost_fix_since < self.rtk_wait:
                self.stop()
                self.set_state("wait_fix")
                return
        elif self.gnss == "rtk_fix":
            self.lost_fix_since = None

        if k.kind == "goto":
            self.set_state(k.label)
            if self.drive_to(k, dt):
                self.stop()
                self.task = None
        elif k.kind == "skip":
            self.record_skip(k.spot, k.spot.note or "planned skip")
            self.task = None
        elif k.kind == "plant":
            self.do_plant(k)
        elif k.kind == "reserve":
            self.do_reserve(k)
        elif k.kind == "service":
            self.do_service(k)
        elif k.kind == "release":
            req = ReserveBay.Request()
            req.robot, req.release = self.ns, True
            self.bay_cli.call_async(req)
            self.task = None
        elif k.kind == "park":
            spawn = self.site.spawn(self.ns)
            slot = Task("goto", x=spawn["x"], y=spawn["y"], yaw=0.0, tol=0.3, label="return")   # back where it started
            self.tasks = [slot]
            self.task = None
            self.get_logger().info(f"mission complete: {self.planted} planted, {self.skipped} skipped; parking")

    def do_plant(self, k: Task):
        self.stop()
        if self.goal_fut is None and self.res_fut is None:
            if self.needs_service():
                resume = Task("goto", x=k.x + k.d * self.gantry, y=k.y, yaw=k.yaw, tol=0.03, label="return")
                self.tasks = self.tender_trip(resume) + [k] + self.tasks
                self.task = None
                self.get_logger().info(f"plugs {self.head.plugs_left}, water {self.head.water_left_l:.0f} L, "
                                       f"SoC {self.soc:.0%}: going to the tender")
                return
            g = PlantSpot.Goal()
            g.spot_id, g.target, g.tool = k.spot.id, pt(k.x, k.y), "auger"
            self.goal_fut = self.plant_cli.send_goal_async(g)
            self.set_state("plant")
            return
        if self.goal_fut is not None and self.goal_fut.done():
            gh = self.goal_fut.result()
            self.goal_fut = None
            if not gh.accepted:
                return
            self.res_fut = gh.get_result_async()
        if self.res_fut is not None and self.res_fut.done():
            res = self.res_fut.result()
            self.res_fut = None
            ok = res.status == GoalStatus.STATUS_SUCCEEDED and res.result.success
            reason = res.result.record.reason if res.result else ""
            self.task = None
            if ok:
                self.planted += 1
            elif "refusal" in reason and k.retry == 0:
                nx = k.x + 0.3 * k.d
                self.tasks = [Task("goto", x=nx + k.d * self.gantry, y=k.y, yaw=k.yaw, tol=0.03, label="drive"),
                              Task("plant", spot=k.spot, x=nx, y=k.y, d=k.d, yaw=k.yaw, retry=1)] + self.tasks
            else:
                self.skipped += 1

    def do_reserve(self, k: Task):
        """Ask for a bay every few seconds; go to the bay when granted, else wait in our slot."""
        self.set_state("queue")
        self.stop()
        now = sim_now(self)
        fut = getattr(k, "fut", None)
        if fut is None:
            if now < getattr(k, "next_try", 0.0) or not self.bay_cli.service_is_ready():
                return
            req = ReserveBay.Request()
            req.robot = self.ns
            k.fut = self.bay_cli.call_async(req)
            return
        if not fut.done():
            return
        r = fut.result()
        k.fut = None
        if r.granted:
            b, q = r.bay_pose.position, r.bay_pose.orientation
            self.tasks.insert(0, Task("goto", x=b.x, y=b.y, yaw=yaw_from_quat(q.x, q.y, q.z, q.w), tol=0.3,
                                      label="to_tender"))
            self.task = None
            return
        k.next_try = now + 3.0
        w = r.wait_pose.position
        if math.hypot(w.x - self.pose[0], w.y - self.pose[1]) > 0.5:
            self.tasks.insert(0, Task("goto", x=w.x, y=w.y, yaw=0.0, tol=0.3, label="queue"))
            self.tasks.insert(1, k)
            self.task = None

    def do_service(self, k: Task):
        self.stop()
        self.set_state("service")
        if getattr(k, "gfut", None) is None and getattr(k, "rfut", None) is None:
            if not self.tender_cli.server_is_ready():
                return
            g = TenderService.Goal()
            g.robot = self.ns
            g.plugs_needed = max(0, self.capacity - (self.head.plugs_left if self.head else 0))
            g.water_needed_l = max(0.0, self.water_capacity - (self.head.water_left_l if self.head else 0.0))
            g.soc = float(self.soc)
            k.gfut = self.tender_cli.send_goal_async(g)
            k.rfut = None
            return
        if getattr(k, "gfut", None) is not None and k.gfut.done():
            k.rfut = k.gfut.result().get_result_async()
            k.gfut = None
        if getattr(k, "rfut", None) is not None and k.rfut.done():
            r = k.rfut.result().result
            self.refill_pub.publish(Float64MultiArray(data=[float(r.plugs_loaded), float(r.water_loaded_l), 0.0]))
            self.soc = r.soc_after
            self.get_logger().info(f"tender: {r.note}")
            self.task = None

    def record_skip(self, s, reason):
        r = PlantRecord()
        r.header.stamp = self.get_clock().now().to_msg()
        r.header.frame_id = "map"
        r.robot, r.spot_id, r.tool, r.outcome, r.reason = self.ns, s.id, "none", "skipped", reason
        r.intended = r.actual = pt(s.position.x, s.position.y)
        r.latitude, r.longitude = self.site.enu_to_geo(s.position.x, s.position.y)
        self.record_pub.publish(r)
        self.skipped += 1

    def publish_state(self):
        if self.pose is None:
            return
        s = RobotState()
        s.header.stamp = self.get_clock().now().to_msg()
        s.robot, s.state = self.ns, self.state
        s.x, s.y, s.yaw = self.pose
        if self.head is not None:
            s.plugs_left, s.water_left_l = self.head.plugs_left, self.head.water_left_l
        s.soc, s.planted, s.skipped = float(self.soc), self.planted, self.skipped
        s.gnss = self.gnss
        self.state_pub.publish(s)


def main():
    rclpy.init()
    n = JaalNavigator()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
