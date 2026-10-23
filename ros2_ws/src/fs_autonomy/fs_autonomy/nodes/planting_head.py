"""Planting head: one action, one sapling (or one seed-ball spot), every robot.

Action plant_spot (fs_interfaces/PlantSpot). The cycle is the one on the concept sheet:

  locate   put the bit over the target   Katar: sled + carriage joints from TF
                                          Jaal:  fixed gantry (navigation did the work)
                                          Pahadi: 4-axis arm IK in base_link
  bore     auger (or percussive pick) through the soil; the Gazebo fs::AugerSoil system
           decides how fast, from soil type, moisture and depth to rock
  retract, place the plug, close and tamp, water (dose / pump rate), half-moon (Pahadi),
  log      fs_interfaces/PlantRecord on /plants/records, with the true hole position from the
           simulator so placement error is measured, not assumed.

Tool choice (tool="auto") reads the surveyed depth to rock under the target:
>= hole depth: auger;  >= 0.12 m: pick;  else seed balls. A stone the survey missed shows up
as an auger refusal; Pahadi then switches to the pick, the others report the spot skipped so
the navigator can shift and retry.
"""
import math
import threading

import numpy as np
import rclpy
from fs_interfaces.action import PlantSpot
from fs_interfaces.msg import HeadStatus, PlantRecord
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, String
from tf2_ros import Buffer, TransformListener

from .. import ik
from ..geometry import transform_to_matrix
from ..planners import choose_tool
from ..ros_common import load_site, lookup, param, parse_json, pt, sim_now, wait_sim

try:
    from fs_gazebo.models import planting_sdf
except ImportError:            # running against hardware without the sim package
    planting_sdf = None

DONE_PHASES = ("bored", "refusal", "no_ground")


class Skip(Exception):
    """The spot cannot be planted; the message is the reason in the record."""


class PlantingHead(Node):
    def __init__(self):
        super().__init__("planting_head")
        self.site = load_site(self)
        self.ns = param(self, "robot", "jaal_1")
        self.kind = param(self, "robot_type", "jaal")
        self.world = param(self, "world", self.site.name)
        self.plugs = int(param(self, "plugs", 360))
        self.water = float(param(self, "water_l", 250.0))
        self.seedballs = int(param(self, "seedballs", 0))
        self.dose = float(param(self, "water_dose_l", 0.5))
        self.pump = float(param(self, "pump_lps", 0.4))
        self.depth = float(param(self, "hole_depth", 0.25))
        self.place_s = float(param(self, "place_s", 1.0))
        self.close_s = float(param(self, "close_s", 1.2))
        self.species = list(param(self, "species", ["Azadirachta indica", "Dalbergia sissoo", "Acacia nilotica"]))
        self.spawn_models = bool(param(self, "spawn_models", True))
        self.rail = (float(param(self, "rail_min", -1.8)), float(param(self, "rail_max", 0.6)))
        self.probe_sigma = float(param(self, "probe_sigma", 0.03))
        self.rng = np.random.default_rng(int(param(self, "seed", 3)))

        self.cb = ReentrantCallbackGroup()
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self, spin_thread=False)
        self.joints = {}
        self.auger = {}
        self.auger_t = -1.0
        self.phase = "idle"
        self.phase_t = sim_now(self)
        self.planted = 0
        self.seq = 0
        self.busy = threading.Lock()
        self.track_target = None

        self.create_subscription(JointState, "joint_states", self.on_joints, 20, callback_group=self.cb)
        self.create_subscription(String, "auger/status", self.on_auger, 20, callback_group=self.cb)
        self.create_subscription(Float64MultiArray, "head/refill", self.on_refill, 10, callback_group=self.cb)
        self.auger_pub = self.create_publisher(String, "auger/cmd", 10)
        self.status_pub = self.create_publisher(HeadStatus, "head/status", 10)
        self.record_pub = self.create_publisher(PlantRecord, "/plants/records", 50)
        self.joint_pubs = {}
        self.spawn_cli = self.create_client(SpawnEntity, f"/world/{self.world}/create", callback_group=self.cb)
        self.create_timer(0.2, self.publish_status, callback_group=self.cb)
        self.create_timer(0.05, self.track, callback_group=self.cb)
        self.server = ActionServer(self, PlantSpot, "plant_spot", execute_callback=self.execute,
                                   goal_callback=self.on_goal, cancel_callback=lambda g: CancelResponse.ACCEPT,
                                   callback_group=self.cb)
        self.get_logger().info(f"{self.kind} head ready: {self.plugs} plugs, {self.water:.0f} L, "
                               f"{self.seedballs} seed balls, dose {self.dose} L")

    # ------------------------------------------------------------------ io
    def on_joints(self, m: JointState):
        for n, p in zip(m.name, m.position):
            self.joints[n] = p

    def on_auger(self, m: String):
        d = parse_json(m.data)
        if d:
            self.auger = d
            self.auger_t = sim_now(self)

    def on_refill(self, m: Float64MultiArray):
        v = list(m.data) + [0, 0, 0]
        self.plugs += int(v[0])
        self.water += float(v[1])
        self.seedballs += int(v[2])
        self.get_logger().info(f"refilled: {self.plugs} plugs, {self.water:.0f} L water")

    def cmd_joint(self, joint: str, value: float):
        if joint not in self.joint_pubs:
            self.joint_pubs[joint] = self.create_publisher(Float64, f"joint/{joint}/cmd_pos", 10)
        self.joint_pubs[joint].publish(Float64(data=float(value)))

    def auger_cmd(self, text: str):
        self.auger_pub.publish(String(data=text))

    def set_phase(self, p: str):
        self.phase = p
        self.phase_t = sim_now(self)

    def publish_status(self):
        s = HeadStatus()
        s.header.stamp = self.get_clock().now().to_msg()
        s.robot, s.phase = self.ns, self.phase
        s.phase_elapsed_s = sim_now(self) - self.phase_t
        a = self.auger
        s.auger_depth_m = float(a.get("depth", 0.0))
        s.auger_torque_nm = float(a.get("torque", 0.0))
        s.auger_power_w = float(a.get("power", 0.0))
        s.auger_thrust_n = float(a.get("thrust", 0.0))
        s.plugs_left, s.seedballs_left = max(self.plugs, 0), max(self.seedballs, 0)
        s.water_left_l, s.planted = float(self.water), self.planted
        self.status_pub.publish(s)

    def spawn(self, kind: str, x: float, y: float, z: float, yaw: float = 0.0):
        if not (self.spawn_models and planting_sdf and self.spawn_cli.service_is_ready()):
            return
        req = SpawnEntity.Request()
        name = f"{kind}_{self.ns}_{self.seq}"
        f = req.entity_factory
        f.name, f.allow_renaming = name, True
        # Gazebo applies the request's pose over the one in the SDF, so the pose goes here.
        f.sdf = planting_sdf(kind, name, 0.0, 0.0, 0.0, 0.0)
        f.pose.position.x, f.pose.position.y, f.pose.position.z = float(x), float(y), float(z)
        f.pose.orientation.z, f.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.spawn_cli.call_async(req)

    def wait_joints(self, targets: dict, tol: float, timeout: float) -> bool:
        return wait_sim(self, timeout, lambda: all(abs(self.joints.get(j, 1e9) - v) < tol for j, v in targets.items()))

    # ------------------------------------------------------------------ geometry helpers
    def frame(self, child: str):
        t = lookup(self.tf, "map", f"{self.ns}/{child}", timeout=0.5)
        return None if t is None else transform_to_matrix(t.transform)

    def ground(self, x, y) -> float:
        return float(self.site.soil_map.elevation(x, y))

    def downhill(self, x, y):
        n = self.site.soil_map.normal(x, y)
        h = n[:2]                      # the terrain normal leans downhill
        nh = np.linalg.norm(h)
        return (h / nh) if nh > 1e-3 else np.array([1.0, 0.0])

    # ------------------------------------------------------------------ robot adapters
    def locate(self, target):
        if self.kind == "katar":
            q = self.katar_targets(target)
            for j, v in q.items():
                self.cmd_joint(j, v)
            if not self.wait_joints(q, 0.03, 8.0):
                self.get_logger().warning("boom slow to reach target, continuing")
        elif self.kind == "pahadi":
            T = self.frame("base_link")
            if T is None:
                raise Skip("no pose")
            p = np.linalg.inv(T) @ np.array([target[0], target[1], self.ground(*target[:2]), 1.0])
            try:
                q = ik.solve(p[:3], clearance=0.06)
            except ik.Unreachable as e:
                raise Skip(f"out of arm reach: {e}")
            names = ("arm_yaw_joint", "arm_shoulder_joint", "arm_elbow_joint", "arm_wrist_joint")
            # yaw first with the arm folded, then unfold
            self.cmd_joint(names[0], q[0])
            self.wait_joints({names[0]: q[0]}, 0.05, 6.0)
            for n, v in zip(names[1:], q[1:]):
                self.cmd_joint(n, v)
            if not self.wait_joints(dict(zip(names, q)), 0.04, 10.0):
                self.get_logger().warning("arm slow to reach target, continuing")
            wait_sim(self, 0.8)            # let the arm settle before cutting

    def katar_targets(self, target):
        T = self.frame("trailer_link")
        if T is None:
            raise Skip("no trailer pose")
        p = np.linalg.inv(T) @ np.array([target[0], target[1], 0.0, 1.0])
        q_s = p[0] + 0.85
        q_c = p[1] - 1.0
        if not (self.rail[0] <= q_s <= self.rail[1]):
            raise Skip(f"outside sled travel ({q_s:+.2f} m)")
        if not (0.5 <= q_c <= 2.5):
            raise Skip(f"outside boom reach ({q_c + 1.0:.2f} m from trailer axis)")
        return {"sled_joint": float(q_s), "carriage_joint": float(q_c)}

    def track(self):
        """Katar on-the-move: keep the bit over the hole while the trailer rolls on."""
        if self.track_target is None:
            return
        try:
            for j, v in self.katar_targets(self.track_target).items():
                self.cmd_joint(j, v)
        except Skip:
            pass

    def stow(self):
        if self.kind == "pahadi":
            names = ("arm_yaw_joint", "arm_shoulder_joint", "arm_elbow_joint", "arm_wrist_joint")
            stow = ik.ArmGeometry().stow
            for n, v in zip(names[1:], stow[1:]):
                self.cmd_joint(n, v)
            self.wait_joints({names[1]: stow[1], names[2]: stow[2]}, 0.1, 8.0)
            self.cmd_joint(names[0], stow[0])
            self.wait_joints(dict(zip(names, stow)), 0.08, 6.0)
        elif self.kind == "katar":
            self.cmd_joint("sled_joint", 0.0)

    def half_moon(self, hole):
        d = self.downhill(*hole)
        c = np.array(hole[:2]) + d * 0.45
        T = self.frame("base_link")
        if T is not None:
            p = np.linalg.inv(T) @ np.array([c[0], c[1], self.ground(*c), 1.0])
            try:
                q = ik.solve(p[:3], clearance=-0.03)
                names = ("arm_yaw_joint", "arm_shoulder_joint", "arm_elbow_joint", "arm_wrist_joint")
                for n, v in zip(names, q):
                    self.cmd_joint(n, v)
                self.wait_joints(dict(zip(names, q)), 0.06, 8.0)
                for sweep in (0.25, -0.25, 0.0):
                    self.cmd_joint(names[0], q[0] + sweep)
                    wait_sim(self, 1.2)
            except ik.Unreachable:
                wait_sim(self, 5.0)
        self.spawn("halfmoon", hole[0], hole[1], self.ground(*hole), math.atan2(d[1], d[0]))

    # ------------------------------------------------------------------ action
    def on_goal(self, goal):
        return GoalResponse.REJECT if self.busy.locked() else GoalResponse.ACCEPT

    def bore(self, verb: str, depth: float):
        t0 = sim_now(self)
        self.auger_cmd(f"{verb} {depth:.3f}")
        wait_sim(self, 0.3)
        ok = wait_sim(self, 400.0, lambda: self.auger_t > t0 and self.auger.get("phase") in DONE_PHASES)
        return self.auger.get("phase") if ok else "timeout"

    def retract(self):
        t0 = sim_now(self)
        self.auger_cmd("retract")
        wait_sim(self, 20.0, lambda: self.auger_t > t0 + 0.2 and self.auger.get("phase") == "idle")

    def execute(self, gh):
        with self.busy:
            return self._execute(gh)

    def _execute(self, gh):
        g = gh.request
        t0 = sim_now(self)
        target = (g.target.x, g.target.y)
        rec = PlantRecord()
        rec.robot, rec.spot_id = self.ns, g.spot_id
        rec.intended = pt(g.target.x, g.target.y, self.ground(*target))
        rec.species = self.species[self.seq % len(self.species)]
        smp = self.site.soil_map.sample(*target)
        rec.soil = smp["soil"]["name"]
        rec.rock_depth_m = smp["rock_depth"]
        tool = g.tool
        if tool in ("", "auto"):
            probe = smp["rock_depth"] + self.rng.normal(0, self.probe_sigma)
            tool = choose_tool(probe, self.depth) if self.kind == "pahadi" else "auger"
        energy = 0.0
        hole = None
        fb = PlantSpot.Feedback()

        def feedback(phase):
            self.set_phase(phase)
            fb.phase, fb.elapsed_s = phase, sim_now(self) - t0
            fb.auger_depth_m = float(self.auger.get("depth", 0.0))
            gh.publish_feedback(fb)

        try:
            if tool != "seedball" and self.plugs <= 0:
                raise Skip("out of plugs")
            if tool == "seedball" and self.seedballs < 3:
                raise Skip("out of seed balls")
            feedback("locate")
            self.locate(target)
            if g.track_ground and self.kind == "katar":
                self.track_target = target
            if tool == "seedball":
                feedback("seed")
                wait_sim(self, 4.0)
                T = self.frame("auger_link")
                hole = (target[0], target[1]) if T is None else (T[0, 3], T[1, 3])
                self.seedballs -= 3
                self.spawn("seedballs", hole[0], hole[1], self.ground(*hole))
                rec.outcome, rec.tool = "seeded", "seedball"
            else:
                feedback("pick" if tool == "pick" else "bore")
                result = self.bore("pick" if tool == "pick" else "bore", self.depth)
                energy += float(self.auger.get("energy_kj", 0.0))
                if result == "refusal" and self.kind == "pahadi":
                    feedback("pick")
                    tool = "pick"
                    result = self.bore("pick", self.depth)
                    energy += float(self.auger.get("energy_kj", 0.0))
                hole = tuple(self.auger.get("hole", target))[:2]
                reached = float(self.auger.get("depth", 0.0))
                rec.hole_depth_m = reached
                if result != "bored":
                    feedback("retract")
                    self.retract()
                    if self.kind == "pahadi" and self.seedballs >= 3:
                        feedback("seed")
                        wait_sim(self, 4.0)
                        self.seedballs -= 3
                        self.spawn("seedballs", hole[0], hole[1], self.ground(*hole))
                        rec.outcome, rec.tool, rec.reason = "seeded", "seedball", f"{result} at {reached * 100:.0f} cm"
                    else:
                        raise Skip(f"auger {result} at {reached * 100:.0f} cm (stone or rock)")
                else:
                    feedback("retract")
                    self.retract()
                    feedback("place")
                    wait_sim(self, self.place_s)
                    self.plugs -= 1
                    self.spawn("sapling", hole[0], hole[1], self.ground(*hole))
                    feedback("close")
                    wait_sim(self, self.close_s)
                    feedback("water")
                    dose = min(self.dose, self.water)
                    wait_sim(self, dose / self.pump)
                    self.water -= dose
                    rec.water_l = dose
                    rec.outcome, rec.tool = "planted", tool
                    self.planted += 1
                    if g.half_moon:
                        feedback("halfmoon")
                        self.half_moon(hole)
            self.track_target = None
            feedback("stow")
            self.stow()
            success = True
        except Skip as e:
            self.track_target = None
            rec.outcome, rec.tool, rec.reason = "skipped", "none", str(e)
            self.stow()
            success = False

        feedback("log")
        if hole is None:
            hole = target
        rec.actual = pt(hole[0], hole[1], self.ground(*hole))
        rec.error_m = math.hypot(hole[0] - target[0], hole[1] - target[1]) if rec.outcome != "skipped" else 0.0
        rec.latitude, rec.longitude = self.site.enu_to_geo(hole[0], hole[1])
        rec.bore_energy_kj = energy
        rec.cycle_time_s = sim_now(self) - t0
        rec.header.stamp = self.get_clock().now().to_msg()
        rec.header.frame_id = "map"
        rec.id = self.seq
        self.seq += 1
        self.record_pub.publish(rec)
        self.set_phase("idle")
        lvl = self.get_logger().info if success else self.get_logger().warning
        lvl(f"spot {g.spot_id}: {rec.outcome} ({rec.tool}) err {rec.error_m * 100:.1f} cm, "
            f"{rec.cycle_time_s:.1f} s, {rec.bore_energy_kj:.1f} kJ {rec.reason}")
        gh.succeed()
        res = PlantSpot.Result()
        res.record, res.success = rec, success
        return res


def main():
    rclpy.init()
    n = PlantingHead()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
