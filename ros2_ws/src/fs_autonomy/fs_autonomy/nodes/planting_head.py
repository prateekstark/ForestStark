"""Planting head: one action, one sapling (or one seed-ball spot), every robot, with the plant
handling done by each machine's own mechanisms.

Action plant_spot (fs_interfaces/PlantSpot):

  locate   put the bit over the target     Katar: sled + carriage (auger station on the turret)
                                            Jaal:  fixed gantry (navigation did the work)
                                            Pahadi: 4-axis arm IK, auger face down
  bore     auger (or percussive pick); the Gazebo fs::AugerSoil system decides how fast from
           soil type, moisture and depth to rock
  deliver  get a plug into the hole        Katar: turret swings the loaded duckbill over the hole
                                            Jaal:  shuttle swaps the loaded duckbill over the hole
                                            Pahadi: gripper picks from the active tray, places it
           then open, close, tamp, water (dose / pump rate), half-moon (Pahadi)
  log      fs_interfaces/PlantRecord on /plants/records, with the true hole position from the
           simulator so placement error is measured, not assumed.

Plugs are tracked by the fs::PlugInventory system (plugs/status, plugs/cmd). A feeder thread keeps
the next plug staged while the robot drives and bores: Katar's carousel drops one into the
duckbill, Jaal's picker fetches one from a drum and drops it down the funnel into the duckbill.

Tool choice (tool="auto") reads the surveyed depth to rock under the target:
>= hole depth: auger;  >= 0.12 m: pick;  else seed balls. A stone the survey missed shows up
as an auger refusal; Pahadi then switches to the pick, the others report the spot skipped so
the navigator can shift and retry.
"""
import math
import threading
import time

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
from ..geometry import transform_to_matrix, wrap
from ..planners import choose_tool
from ..ros_common import load_site, lookup, param, parse_json, pt, sim_now, wait_sim

try:
    from fs_gazebo.models import planting_sdf
except ImportError:            # running against hardware without the sim package
    planting_sdf = None

DONE_PHASES = ("bored", "refusal", "no_ground")
PLUG_BOTTOM = 0.12             # plug root ball is 0.12 m long; its top ends flush with the ground
JAW_TIP = 0.42                 # duckbill_feed_link origin to the closed jaw tip
TAMP_RING = 0.026              # tamp_link origin to the underside of the ring
READY = 0.10                   # duckbill tip and tamp ring wait this far above the ground
CLEAR = 0.35                   # above the seedling's crown: safe to close the jaws and swing away


class Skip(Exception):
    """The spot cannot be planted; the message is the reason in the record."""


# ====================================================================== mechanisms
class Mechanism:
    """Robot-specific plant handling. feeder() runs in its own thread, the rest in the action.

    CARRIERS hold plugs on the machine where the mechanism can reach them (fs::PlugInventory
    tracks every cell); `reserve` counts the ones further back (Katar's rack trays, Pahadi's
    spare-tray magazine) that a loader or tray swap brings forward."""
    CARRIERS: tuple = ()
    HOLDERS: tuple = ()

    def __init__(self, head):
        self.h = head
        self.reserve = 0
        self.synced = False

    def stock(self) -> int:
        if not self.synced:
            return self.h.plugs
        h = self.h
        return (self.reserve + sum(h.carrier(c).get("count", 0) for c in self.CARRIERS)
                + sum(h.holding(k) for k in self.HOLDERS))

    def load(self, n: int) -> int:
        """Put up to n plugs into free cells; returns how many did not fit."""
        for c in self.CARRIERS:
            car = self.h.carrier(c)
            k = min(n, car.get("cells", 0) - car.get("count", 0))
            if k > 0:
                self.h.inv_cmd(f"fill {c} {k}")
                n -= k
        return n

    def sync(self):
        """The world starts with every cell full; match the load to the launch's plug count."""
        for c in self.CARRIERS:
            self.h.inv_cmd(f"empty {c}")
        self.reserve = self.load(self.h.plugs)
        self.synced = True
        self.h.get_logger().info(f"plugs: {self.stock() - self.reserve} on the mechanism, {self.reserve} in reserve")

    def feeder(self):
        time.sleep(0.5)

    def refill(self, plugs):
        self.reserve += self.load(int(plugs)) if self.synced else int(plugs)

    def locate(self, target):
        pass

    def deliver(self, hole):
        raise Skip("no plant handling")

    def stow(self):
        pass

    def half_moon(self, hole):
        pass

    def stage(self, target):
        """While the auger bores: lower the loaded duckbill and the tamp ring to READY."""

    # duckbill heads (Katar, Jaal): a feed rod lowers the jaws into the hole, a ring round them tamps
    def lower_to(self, joint, link, offset, z):
        """Joint target that puts the point `offset` below `link` at height z (feed axes point down)."""
        zl = self.h.link_z(link)
        if zl is None:
            return None
        return self.h.joints.get(joint, 0.0) + (zl - offset) - z

    def stage_duckbill(self, target):
        h = self.h
        if not h.holding("duckbill"):
            return                        # the feeder still needs it up under the chute
        g = h.ground(*target)
        for joint, link, off in (("duckbill_feed_joint", "duckbill_feed_link", JAW_TIP), ("tamp_joint", "tamp_link", TAMP_RING)):
            q = self.lower_to(joint, link, off, g + READY)
            if q is not None:
                h.cmd_joint(joint, max(0.0, q))

    def plant_with_duckbill(self, hole):
        """Duckbill over the hole: push it in, open, drop the plug, withdraw past the stem, tamp."""
        h = self.h
        g = h.ground(*hole)
        h.feedback("place")
        q = self.lower_to("duckbill_feed_joint", "duckbill_feed_link", JAW_TIP, g - PLUG_BOTTOM - 0.02)
        if q is None:
            raise Skip("no duckbill pose")
        h.move({"duckbill_feed_joint": q}, 0.015, 8.0)
        h.move({"jaw_l_joint": 0.5, "jaw_r_joint": 0.5}, 0.05, 3.0)
        h.inv_cmd("drop duckbill")
        h.spawn("sapling", hole[0], hole[1], g)
        # withdraw with the jaws open round the seedling, close them once clear of its crown
        h.move({"duckbill_feed_joint": q - PLUG_BOTTOM - 0.02 - CLEAR}, 0.03, 4.0)
        h.cmd_joint("duckbill_feed_joint", 0.0)
        h.cmd_joint("jaw_l_joint", 0.0)
        h.cmd_joint("jaw_r_joint", 0.0)
        h.feedback("close")
        qt = self.lower_to("tamp_joint", "tamp_link", TAMP_RING, g - 0.01)
        if qt is not None:
            h.move({"tamp_joint": qt}, 0.015, 6.0)
            wait_sim(h, 0.4)
            h.move({"tamp_joint": qt - 0.01 - CLEAR}, 0.03, 4.0)
        h.cmd_joint("tamp_joint", 0.0)    # the rest of the way up while the head swings back


class KatarMechanism(Mechanism):
    """Turret head: auger at +x, duckbill at -x. The carousel over the chute feeds the duckbill;
    a loader refills its cups from the rack trays on the trailer."""
    CUPS = 12
    CARRIERS, HOLDERS = ("carousel",), ("duckbill",)

    def __init__(self, head):
        super().__init__(head)
        self.last_load = -1e9
        self.load_every = float(param(head, "loader_s_per_plug", 4.0))

    def feeder(self):
        h = self.h
        car = h.carrier("carousel")
        t = sim_now(h)
        if car and car.get("count", 0) < self.CUPS and self.reserve > 0 and t - self.last_load > self.load_every:
            h.inv_cmd("fill carousel 1")                         # the loader tops up one cup from the racks
            self.reserve -= 1
            self.last_load = t
        home = abs(h.joints.get("turret_joint", 0.0)) < 0.05 and h.joints.get("duckbill_feed_joint", 0.0) < 0.02
        if home and car.get("count", 0) > 0 and car.get("next", -1) >= 0 and not h.holding("duckbill"):
            k = car["next"]
            cur = h.joints.get("carousel_joint", 0.0)
            want = cur + wrap(-2 * math.pi * k / self.CUPS - cur)       # cup k over the chute (+x)
            h.move({"carousel_joint": want}, 0.02, 4.0)
            h.inv_cmd(f"pick carousel {k} duckbill")
            wait_sim(h, 0.6, lambda: h.holding("duckbill"))
        time.sleep(0.1)

    def targets(self, target):
        T = self.h.frame("trailer_link")
        if T is None:
            raise Skip("no trailer pose")
        p = np.linalg.inv(T) @ np.array([target[0], target[1], 0.0, 1.0])
        q_s = p[0] + 0.85 - 0.20          # the auger station is 0.20 m ahead of the carriage
        q_c = p[1] - 1.0
        if not (self.h.rail[0] <= q_s <= self.h.rail[1]):
            raise Skip(f"outside sled travel ({q_s:+.2f} m)")
        if not (0.5 <= q_c <= 2.5):
            raise Skip(f"outside boom reach ({q_c + 1.0:.2f} m from trailer axis)")
        return {"sled_joint": float(q_s), "carriage_joint": float(q_c)}

    def locate(self, target):
        if not self.h.move(self.targets(target), 0.03, 8.0):
            self.h.get_logger().warning("boom slow to reach target, continuing")

    def stage(self, target):
        self.stage_duckbill(target)

    def deliver(self, hole):
        h = self.h
        if not wait_sim(h, 20.0, lambda: h.holding("duckbill")):
            raise Skip("no plug in the duckbill (carousel empty)")
        h.feedback("turret")
        h.move({"turret_joint": math.pi}, 0.03, 5.0)        # the duckbill takes the auger's place
        self.plant_with_duckbill(hole)

    def stow(self):
        self.h.move({"turret_joint": 0.0}, 0.05, 5.0)
        self.h.cmd_joint("sled_joint", 0.0)


class JaalMechanism(Mechanism):
    """Two drums, a Cartesian picker that carries plugs to the funnel, duckbill on the rear shuttle."""
    DRUMS = {"drum_f": ("drum_f_joint", 0.55), "drum_r": ("drum_r_joint", -0.43)}
    POCKETS = 45
    FUNNEL_X, PARK_Z, REACH, SHUTTLE = -1.2, 0.74, 0.12, -0.28
    CARRIERS, HOLDERS = tuple(DRUMS), ("gripper", "duckbill")

    def feeder(self):
        h = self.h
        if not h.inv:
            time.sleep(0.2)
            return
        if not h.holding("gripper"):
            drum = next((d for d in self.DRUMS if h.carrier(d).get("count", 0) > 0), None)
            if drum is None:
                time.sleep(0.5)
                return
            self.fetch(drum)
        elif (not h.holding("duckbill") and abs(h.joints.get("shuttle_joint", 0.0)) < 0.02
              and h.joints.get("duckbill_feed_joint", 0.0) < 0.02):
            h.move({"finger_a_joint": 0.0, "finger_b_joint": 0.0}, 0.003, 2.0)
            h.inv_cmd("move gripper duckbill")              # down the funnel into the duckbill
            wait_sim(h, 0.6, lambda: h.holding("duckbill"))
        time.sleep(0.1)

    def fetch(self, drum):
        h = self.h
        c = h.carrier(drum)
        i, a = c["next"], c["next_angle"]
        joint, x = self.DRUMS[drum]
        cur = h.joints.get(joint, 0.0)
        turn = cur + wrap(math.pi / 2 - a)                   # pocket i to the pick window (+y)
        qz = 1.303 - 0.4 * (i // self.POCKETS)               # palm level with the root ball's middle
        h.cmd_joint(joint, turn)
        h.move({"finger_a_joint": 0.0, "finger_b_joint": 0.0, "picker_y_joint": 0.0}, 0.005, 3.0)
        h.move({"picker_x_joint": x}, 0.01, 6.0)
        h.move({"picker_z_joint": qz, joint: turn}, 0.01, 6.0)
        h.move({"picker_y_joint": self.REACH}, 0.004, 3.0)
        h.move({"finger_a_joint": 0.012, "finger_b_joint": 0.012}, 0.002, 2.0)
        h.inv_cmd(f"pick {drum} {i} gripper")
        wait_sim(h, 0.4, lambda: h.holding("gripper"))
        h.move({"picker_z_joint": qz - 0.05}, 0.01, 2.0)    # lift out of the pocket
        h.move({"picker_y_joint": 0.0}, 0.004, 3.0)
        h.move({"picker_z_joint": self.PARK_Z}, 0.01, 6.0)
        h.move({"picker_x_joint": self.FUNNEL_X}, 0.01, 6.0)

    def stage(self, target):
        self.stage_duckbill(target)

    def deliver(self, hole):
        h = self.h
        if not wait_sim(h, 45.0, lambda: h.holding("duckbill")):
            raise Skip("no plug reached the duckbill (drums empty?)")
        h.feedback("shuttle")
        h.move({"shuttle_joint": self.SHUTTLE}, 0.01, 4.0)  # the duckbill takes the auger's place
        self.plant_with_duckbill(hole)

    def stow(self):
        self.h.move({"shuttle_joint": 0.0}, 0.01, 4.0)


class PahadiMechanism(Mechanism):
    """Arm with auger and gripper; active tray on the front module, spare trays in the magazine."""
    NAMES = ("arm_yaw_joint", "arm_shoulder_joint", "arm_elbow_joint", "arm_wrist_joint")
    FOLD = (-0.9, 2.2, -1.3)
    TRAY = 50
    GRIP_OPEN, GRIP_CLOSED = 0.0, 0.016
    CARRIERS, HOLDERS = ("tray",), ("gripper",)

    def __init__(self, head):
        super().__init__(head)
        self.swap_s = float(param(head, "tray_swap_s", 45.0))

    def to_base(self, p):
        T = self.h.frame("base_link")
        if T is None:
            raise Skip("no pose")
        return (np.linalg.inv(T) @ np.array([p[0], p[1], p[2], 1.0]))[:3]

    def arm(self, q, tol=0.04, timeout=10.0):
        h = self.h
        cur = h.joints.get(self.NAMES[0], 0.0)
        yaw = cur + wrap(q[0] - cur)
        if abs(yaw) > 3.25:                           # stay inside the slip ring's +-3.3 rad
            yaw -= math.copysign(2 * math.pi, yaw)
        if abs(yaw - cur) > 0.4:                      # fold, turn, unfold so the arm clears the deck
            h.move(dict(zip(self.NAMES[1:], self.FOLD)), 0.1, 6.0)
            h.move({self.NAMES[0]: yaw}, 0.05, 8.0)
        return h.move(dict(zip(self.NAMES, (yaw, *q[1:]))), tol, timeout)

    def grip(self, closed):
        v = self.GRIP_CLOSED if closed else self.GRIP_OPEN
        self.h.move({"finger_l_joint": v, "finger_r_joint": v}, 0.003, 2.0)

    def grip_to(self, p_world, up=0.0):
        p = self.to_base((p_world[0], p_world[1], p_world[2] + up))
        try:
            return self.arm(ik.solve_gripper(p), 0.03, 10.0)
        except ik.Unreachable as e:
            raise Skip(f"gripper cannot reach: {e}")

    def locate(self, target):
        p = self.to_base((target[0], target[1], self.h.ground(*target)))
        try:
            q = ik.solve(p, clearance=0.06)
        except ik.Unreachable as e:
            raise Skip(f"out of arm reach: {e}")
        if not self.arm(q):
            self.h.get_logger().warning("arm slow to reach target, continuing")
        wait_sim(self.h, 0.8)                         # let the arm settle before cutting

    def swap_tray(self):
        h = self.h
        h.feedback("tray_swap")
        stow = ik.ArmGeometry().stow
        h.move(dict(zip(self.NAMES[1:], stow[1:])), 0.1, 8.0)
        wait_sim(h, self.swap_s)                      # the tray changer brings a full tray forward
        n = min(self.TRAY, self.reserve)
        h.inv_cmd(f"fill tray {n}")
        self.reserve -= n
        wait_sim(h, 2.0, lambda: h.carrier("tray").get("count", 0) > 0)

    def deliver(self, hole):
        h = self.h
        if h.carrier("tray").get("count", 0) == 0:
            if self.reserve <= 0:
                raise Skip("out of plugs")
            self.swap_tray()
        cell = h.carrier("tray")["next_world"]
        grasp = (cell[0], cell[1], cell[2] + 0.06)
        h.feedback("pick_plug")
        self.grip(False)
        self.grip_to(grasp, up=0.15)
        self.grip_to(grasp)
        self.grip(True)
        h.inv_cmd("pick tray next gripper")
        wait_sim(h, 0.5, lambda: h.holding("gripper"))
        self.grip_to(grasp, up=0.25)
        g = h.ground(*hole)
        into = (hole[0], hole[1], g - PLUG_BOTTOM + 0.06)
        h.feedback("place")
        self.grip_to(into, up=0.3)
        self.grip_to(into)
        self.grip(False)
        h.inv_cmd("drop gripper")
        h.spawn("sapling", hole[0], hole[1], g)
        h.feedback("close")
        self.grip_to(into, up=0.15)
        self.grip(True)
        self.grip_to((hole[0], hole[1], g + 0.05))   # firm the soil round the plug with the closed fingers
        self.grip_to(into, up=0.3)

    def stow(self):
        stow = ik.ArmGeometry().stow
        h = self.h
        h.move(dict(zip(self.NAMES[1:], stow[1:])), 0.1, 8.0)
        h.move({self.NAMES[0]: stow[0]}, 0.08, 8.0)
        h.move(dict(zip(self.NAMES, stow)), 0.08, 6.0)

    def half_moon(self, hole):
        h = self.h
        d = h.downhill(*hole)
        c = np.array(hole[:2]) + d * 0.45
        try:
            q = ik.solve(self.to_base((c[0], c[1], h.ground(*c))), clearance=-0.03)
            self.arm(q, 0.06, 8.0)
            for sweep in (0.25, -0.25, 0.0):
                h.cmd_joint(self.NAMES[0], h.joints.get(self.NAMES[0], q[0]) + sweep)
                wait_sim(h, 1.2)
        except (ik.Unreachable, Skip):
            wait_sim(h, 5.0)
        h.spawn("halfmoon", hole[0], hole[1], h.ground(*hole), math.atan2(d[1], d[0]))


MECHANISMS = {"katar": KatarMechanism, "jaal": JaalMechanism, "pahadi": PahadiMechanism}


# ====================================================================== node
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
        self.inv = {}
        self.phase = "idle"
        self.phase_t = sim_now(self)
        self.planted = 0
        self.seq = 0
        self.busy = threading.Lock()
        self.pub_lock = threading.Lock()
        self.track_target = None
        self.gh = None
        self.t0 = 0.0

        self.create_subscription(JointState, "joint_states", self.on_joints, 20, callback_group=self.cb)
        self.create_subscription(String, "auger/status", self.on_auger, 20, callback_group=self.cb)
        self.create_subscription(String, "plugs/status", self.on_plugs, 10, callback_group=self.cb)
        self.create_subscription(Float64MultiArray, "head/refill", self.on_refill, 10, callback_group=self.cb)
        self.auger_pub = self.create_publisher(String, "auger/cmd", 10)
        self.plug_pub = self.create_publisher(String, "plugs/cmd", 20)
        self.status_pub = self.create_publisher(HeadStatus, "head/status", 10)
        self.record_pub = self.create_publisher(PlantRecord, "/plants/records", 50)
        self.joint_pubs = {}
        self.spawn_cli = self.create_client(SpawnEntity, f"/world/{self.world}/create", callback_group=self.cb)
        self.mech = MECHANISMS[self.kind](self)
        self.create_timer(0.2, self.publish_status, callback_group=self.cb)
        self.create_timer(0.05, self.track, callback_group=self.cb)
        self.server = ActionServer(self, PlantSpot, "plant_spot", execute_callback=self.execute,
                                   goal_callback=self.on_goal, cancel_callback=lambda g: CancelResponse.ACCEPT,
                                   callback_group=self.cb)
        threading.Thread(target=self.feeder_loop, daemon=True).start()
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

    def on_plugs(self, m: String):
        d = parse_json(m.data)
        if d:
            self.inv = d

    def carrier(self, name) -> dict:
        return self.inv.get("carriers", {}).get(name, {})

    def holding(self, holder) -> bool:
        return bool(self.inv.get("holders", {}).get(holder, False))

    def inv_cmd(self, text: str):
        """Tell fs::PlugInventory what the mechanism just did, then wait for the status to show it."""
        t = self.inv.get("t", 0.0)
        self.plug_pub.publish(String(data=text))
        wait_sim(self, 1.0, lambda: self.inv.get("t", 0.0) > t + 0.15)

    def on_refill(self, m: Float64MultiArray):
        v = list(m.data) + [0, 0, 0]
        self.plugs += int(v[0])
        self.mech.refill(int(v[0]))
        self.water += float(v[1])
        self.seedballs += int(v[2])
        self.get_logger().info(f"refilled: {self.mech.stock()} plugs, {self.water:.0f} L water")

    def cmd_joint(self, joint: str, value: float):
        with self.pub_lock:
            if joint not in self.joint_pubs:
                self.joint_pubs[joint] = self.create_publisher(Float64, f"joint/{joint}/cmd_pos", 10)
            pub = self.joint_pubs[joint]
        pub.publish(Float64(data=float(value)))

    def move(self, targets: dict, tol: float, timeout: float) -> bool:
        for j, v in targets.items():
            self.cmd_joint(j, v)
        return wait_sim(self, timeout, lambda: all(abs(self.joints.get(j, 1e9) - v) < tol for j, v in targets.items()))

    def auger_cmd(self, text: str):
        self.auger_pub.publish(String(data=text))

    def set_phase(self, p: str):
        self.phase = p
        self.phase_t = sim_now(self)

    def feedback(self, phase: str):
        self.set_phase(phase)
        gh = self.gh
        if gh is not None:
            fb = PlantSpot.Feedback()
            fb.phase, fb.elapsed_s = phase, sim_now(self) - self.t0
            fb.auger_depth_m = float(self.auger.get("depth", 0.0))
            gh.publish_feedback(fb)

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
        s.plugs_left = max(int(self.mech.stock()), 0)
        s.seedballs_left = max(self.seedballs, 0)
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

    def feeder_loop(self):
        while rclpy.ok():
            try:
                if not self.mech.synced:
                    if self.inv.get("carriers"):
                        self.mech.sync()
                    else:
                        time.sleep(0.2)
                    continue
                self.mech.feeder()
            except Exception as e:  # noqa: BLE001 - keep feeding after a transient error
                self.get_logger().warning(f"feeder: {e}")
                time.sleep(1.0)

    # ------------------------------------------------------------------ geometry helpers
    def frame(self, child: str):
        t = lookup(self.tf, "map", f"{self.ns}/{child}", timeout=0.5)
        return None if t is None else transform_to_matrix(t.transform)

    def link_z(self, child: str):
        T = self.frame(child)
        return None if T is None else float(T[2, 3])

    def ground(self, x, y) -> float:
        return float(self.site.soil_map.elevation(x, y))

    def downhill(self, x, y):
        n = self.site.soil_map.normal(x, y)
        h = n[:2]                      # the terrain normal leans downhill
        nh = np.linalg.norm(h)
        return (h / nh) if nh > 1e-3 else np.array([1.0, 0.0])

    def track(self):
        """Katar on-the-move: keep the working station over the hole while the trailer rolls on."""
        if self.track_target is None or self.kind != "katar":
            return
        try:
            for j, v in self.mech.targets(self.track_target).items():
                self.cmd_joint(j, v)
        except Skip:
            pass

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

    def seed(self, hole):
        self.feedback("seed")
        wait_sim(self, 4.0)
        self.seedballs -= 3
        self.spawn("seedballs", hole[0], hole[1], self.ground(*hole))

    def execute(self, gh):
        with self.busy:
            self.gh = gh
            try:
                return self._execute(gh)
            finally:
                self.gh = None

    def _execute(self, gh):
        g = gh.request
        self.t0 = t0 = sim_now(self)
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
        try:
            if tool != "seedball" and self.mech.stock() <= 0:
                raise Skip("out of plugs")
            if tool == "seedball" and self.seedballs < 3:
                raise Skip("out of seed balls")
            self.feedback("locate")
            self.mech.locate(target)
            if g.track_ground and self.kind == "katar":
                self.track_target = target
            if tool == "seedball":
                T = self.frame("auger_link")
                hole = (target[0], target[1]) if T is None else (T[0, 3], T[1, 3])
                self.seed(hole)
                rec.outcome, rec.tool = "seeded", "seedball"
            else:
                self.feedback("pick" if tool == "pick" else "bore")
                self.mech.stage(target)
                result = self.bore("pick" if tool == "pick" else "bore", self.depth)
                energy += float(self.auger.get("energy_kj", 0.0))
                if result == "refusal" and self.kind == "pahadi":
                    self.feedback("pick")
                    tool = "pick"
                    result = self.bore("pick", self.depth)
                    energy += float(self.auger.get("energy_kj", 0.0))
                hole = tuple(self.auger.get("hole", target))[:2]
                reached = float(self.auger.get("depth", 0.0))
                rec.hole_depth_m = reached
                self.feedback("retract")
                self.retract()
                if result != "bored":
                    if self.kind == "pahadi" and self.seedballs >= 3:
                        self.seed(hole)
                        rec.outcome, rec.tool, rec.reason = "seeded", "seedball", f"{result} at {reached * 100:.0f} cm"
                    else:
                        raise Skip(f"auger {result} at {reached * 100:.0f} cm (stone or rock)")
                else:
                    self.mech.deliver(hole)
                    self.plugs -= 1
                    self.feedback("water")
                    dose = min(self.dose, self.water)
                    wait_sim(self, dose / self.pump)
                    self.water -= dose
                    rec.water_l = dose
                    rec.outcome, rec.tool = "planted", tool
                    self.planted += 1
                    if g.half_moon:
                        self.feedback("halfmoon")
                        self.mech.half_moon(hole)
            self.track_target = None
            self.feedback("stow")
            self.mech.stow()
            success = True
        except Skip as e:
            self.track_target = None
            rec.outcome, rec.tool, rec.reason = "skipped", "none", str(e)
            self.mech.stow()
            success = False

        self.feedback("log")
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
            f"{rec.cycle_time_s:.1f} s, {rec.bore_energy_kj:.1f} kJ {rec.reason} | {self.mech.stock()} plugs left")
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
