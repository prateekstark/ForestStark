"""Nodes that serve the hardware interface (docs/hardware_interface.md) on a machine, on top of
ros2_control (fs_hardware/CanopenSystem). Each replaces one piece of the Gazebo bridge:

  hw_bridge           joint/<j>/cmd_pos and cmd_vel -> the controllers; odom from the encoders;
                      joint_states (the drives' states plus passive joints mirrored)
  auger_driver        auger/cmd -> feed and spin, auger/status   (fs_hardware.auger)
  plug_inventory      plugs/cmd -> the books, plugs/status       (fs_hardware.inventory)
  traction_estimator  terramechanics: wheel loads, slip against GNSS, drive forces from torque
  safety_bridge       safety/hw: the safety PLC's diagnostic PDO; the heartbeat and mode back (non-safety)

All run in the robot's namespace; the robot description comes from the robot_state_publisher's
parameter, passed in by robot.launch.py.
"""
from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, NavSatFix
from std_msgs.msg import Bool, Float64, Float64MultiArray, String

from fs_interfaces.msg import HwSafety

from . import description, kinematics as kin
from .auger import Auger, AugerParams
from .inventory import Inventory

HW_STATES = "joint_state_broadcaster/joint_states"     # the drives (use_local_topics); joint_states is the bridge's


def _param(node, name, default):
    node.declare_parameter(name, default)
    return node.get_parameter(name).value


def _spin(cls):
    rclpy.init()
    n = cls()
    try:
        rclpy.spin(n)
    except Exception:            # Ctrl-C / launch shutdown invalidates the context mid-spin
        if rclpy.ok():
            raise
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


class HwBridge(Node):
    def __init__(self):
        super().__init__("hw_bridge")
        self.kind = _param(self, "robot_type", "jaal")
        self.ns = self.get_namespace().strip("/") or self.kind
        self.timeout = _param(self, "cmd_timeout", 0.5)
        self.mirror = dict(zip(_param(self, "mirror_joints", [""]), _param(self, "mirror_sources", [""])))
        self.mirror.pop("", None)
        g = description.groups(description.load(self.kind, _param(self, "can_config", "") or None))
        self.pos_joints, self.drive_joints = g["position"], g["drive"]
        self.steer = {j: j.split("_")[1] for j in self.pos_joints if j.startswith("steer_")}
        self.cmd = {j: math.nan for j in self.pos_joints}
        self.js = {}
        self.twist, self.twist_t = (0.0, 0.0, 0.0), -1e9
        self.gyro_z = 0.0
        self.odom = kin.Odometry()
        self.last = None
        self.pos_pub = self.create_publisher(Float64MultiArray, "position_controller/commands", 10)
        self.drive_pub = self.create_publisher(Float64MultiArray, "drive_controller/commands", 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.js_pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_subscription(JointState, HW_STATES, self.on_js, qos_profile_sensor_data)
        self.create_subscription(Twist, "cmd_vel", self.on_twist, 10)
        self.create_subscription(Imu, "imu", lambda m: setattr(self, "gyro_z", m.angular_velocity.z), qos_profile_sensor_data)
        for j in self.pos_joints:
            if j not in self.steer:
                self.create_subscription(Float64, f"joint/{j}/cmd_pos", lambda m, j=j: self.cmd.__setitem__(j, m.data), 10)
        self.period = float(_param(self, "period", 0.01))      # the controller manager's cycle
        self.create_timer(self.period, self.tick)
        self.limits = _limits(_param(self, "robot_description", ""), self.pos_joints)
        self.sent = {}
        self.get_logger().info(f"{self.kind}: {len(self.pos_joints)} position joints, drive {self.drive_joints or 'none'}")

    def on_js(self, m: JointState):
        for i, n in enumerate(m.name):
            self.js[n] = (m.position[i] if i < len(m.position) else 0.0, m.velocity[i] if i < len(m.velocity) else 0.0,
                          m.effort[i] if i < len(m.effort) else 0.0)
        out = JointState(header=m.header, name=list(m.name), position=list(m.position), velocity=list(m.velocity),
                         effort=list(m.effort))
        for j, src in self.mirror.items():           # passive joints the drives do not measure
            p, v, _ = self.js.get(src, (0.0, 0.0, 0.0)) if src != "zero" else (0.0, 0.0, 0.0)
            out.name.append(j), out.position.append(p), out.velocity.append(v), out.effort.append(0.0)
        self.js_pub.publish(out)

    def on_twist(self, m: Twist):
        self.twist, self.twist_t = (m.linear.x, m.linear.y, m.angular.z), self.now_s()

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        if not self.js:
            return
        now = self.now_s()
        vx, vy, wz = self.twist if now - self.twist_t < self.timeout else (0.0, 0.0, 0.0)
        drive = []
        if self.kind == "jaal":
            steer_now = {m: self.js.get(j, (0.0,))[0] for j, m in self.steer.items()}
            sol = kin.swerve_inverse(vx, vy, wz, steer_now)
            for j, m in self.steer.items():
                self.cmd[j] = sol[m][0]
            drive = [sol[j.split("_")[1]][1] for j in self.drive_joints]
        elif self.kind == "pahadi":
            wl, wr = kin.diff_inverse(vx, wz)
            drive = [wl if j[1] == "l" else wr for j in self.drive_joints]
        # unset position commands hold the measured position; targets are clamped to the joint's range
        # and ramped at its velocity limit, so the controller manager's limiter never has to cut them
        pos = []
        for j in self.pos_joints:
            want = self.cmd[j] if math.isfinite(self.cmd[j]) else self.js.get(j, (0.0,))[0]
            lo, hi, vmax = self.limits.get(j, (-math.inf, math.inf, math.inf))
            want = min(max(want, lo), hi)
            meas = self.js.get(j, (want,))[0]
            step = 0.9 * vmax * self.period
            last = self.sent.get(j, meas)
            if abs(last - meas) > max(0.5 * vmax, 20 * step):     # controller not running yet, or the joint held back
                last = meas
            self.sent[j] = last + max(-step, min(step, want - last))
            pos.append(self.sent[j])
        self.pos_pub.publish(Float64MultiArray(data=pos))
        if self.drive_joints:
            self.drive_pub.publish(Float64MultiArray(data=drive))
        self.publish_odom(now)

    def publish_odom(self, now):
        dt = 0.02 if self.last is None else max(1e-3, min(0.2, now - self.last))
        self.last = now
        if self.kind == "jaal":
            st = {m: self.js.get(f"steer_{m}_joint", (0.0,))[0] for m in kin.JAAL_MODULES}
            wh = {m: self.js.get(f"wheel_{m}_joint", (0.0, 0.0))[1] for m in kin.JAAL_MODULES}
            vx, vy, wz = kin.swerve_forward(st, wh)
        elif self.kind == "pahadi":
            l = [self.js.get(j, (0, 0))[1] for j in self.drive_joints if j[1] == "l"]
            r = [self.js.get(j, (0, 0))[1] for j in self.drive_joints if j[1] == "r"]
            vx, wz = kin.diff_forward(sum(l) / max(len(l), 1), sum(r) / max(len(r), 1))
            vy = 0.0
        else:
            w = [self.js.get(j, (0, 0))[1] for j in ("rear_left_wheel_joint", "rear_right_wheel_joint")]
            vx, vy, wz = kin.KATAR_R * sum(w) / 2, 0.0, self.gyro_z
        x, y, yaw = self.odom.step(vx, vy, wz, dt)
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id, o.child_frame_id = f"{self.ns}/odom", f"{self.ns}/base_link"
        o.pose.pose.position.x, o.pose.pose.position.y = x, y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = vx, vy, wz
        self.odom_pub.publish(o)


def _limits(urdf: str, joints) -> dict:
    """(lower, upper, velocity) of each joint from the URDF; continuous joints have no range."""
    out = {}
    if not urdf:
        return out
    for j in ET.fromstring(urdf).findall("joint"):
        lim = j.find("limit")
        if j.get("name") in joints and lim is not None:
            cont = j.get("type") == "continuous"
            out[j.get("name")] = (-math.inf if cont else float(lim.get("lower", "-inf")),
                                  math.inf if cont else float(lim.get("upper", "inf")), float(lim.get("velocity", "inf")))
    return out


def _plugin(urdf: str, name: str):
    root = ET.fromstring(urdf)
    return next((p for p in root.iter("plugin") if p.get("name") == name), None)


class AugerDriver(Node):
    def __init__(self):
        super().__init__("auger_driver")
        self.ns = self.get_namespace().strip("/")
        urdf = _param(self, "robot_description", "")
        pl = _plugin(urdf, "fs::AugerSoil") if urdf else None
        f = (lambda k, d: float(pl.findtext(k, str(d)))) if pl is not None else (lambda k, d: d)
        rpm = f("rpm", 120.0)
        p = AugerParams(max_stroke=f("max_stroke", 1.0), free_speed=f("free_speed", 0.25), max_feed=f("max_feed", 0.15),
                        retract_speed=f("retract_speed", 0.3), rpm=rpm, max_thrust=f("max_thrust", 3000.0),
                        spin_torque_limit=f("max_power", 4000.0) / (rpm * 2 * math.pi / 60))
        p.contact_force = _param(self, "contact_force", 0.15 * p.max_thrust)
        self.tip = pl.findtext("tip_link", "auger_link") if pl is not None else "auger_link"
        self.a = Auger(p)
        self.js = {}
        self.base_force, self.n_base = 0.0, 0
        from tf2_ros import Buffer, TransformListener
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.cmd_pub = self.create_publisher(Float64MultiArray, "auger_controller/commands", 10)
        self.st_pub = self.create_publisher(String, "auger/status", 10)
        self.create_subscription(String, "auger/cmd", lambda m: self.a.command(m.data), 10)
        self.create_subscription(JointState, HW_STATES, self.on_js, qos_profile_sensor_data)
        self.create_timer(0.01, self.tick)
        self.create_timer(0.05, self.publish)
        self.get_logger().info(f"auger: stroke {p.max_stroke} m, thrust {p.max_thrust:.0f} N, torque limit {p.spin_torque_limit:.0f} N m")

    def on_js(self, m):
        for i, n in enumerate(m.name):
            self.js[n] = (m.position[i], m.velocity[i] if m.velocity else 0.0, m.effort[i] if m.effort else 0.0)

    def tip_xy(self):
        try:
            t = self.tf.lookup_transform("map", f"{self.ns}/{self.tip}", rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None

    def readings(self):
        q, qd, force = self.js.get("auger_feed_joint", (0.0, 0.0, 0.0))
        _, w, torque = self.js.get("auger_spin_joint", (0.0, 0.0, 0.0))
        return q, qd, force, torque, w

    def tick(self):
        if not self.js:
            return
        q, qd, force, torque, w = self.readings()
        if self.a.phase == "lowering" and self.a.phase_t < 0.5:      # no-load force of the moving feed
            self.n_base += 1
            self.base_force += (force - self.base_force) / self.n_base
        elif self.a.phase != "lowering":
            self.n_base = 0
        feed, spin = self.a.step(0.01, q, qd, force - self.base_force, abs(torque), abs(w),
                                 self.tip_xy() if self.a.phase == "lowering" else None)
        self.cmd_pub.publish(Float64MultiArray(data=[feed, spin]))

    def publish(self):
        q, qd, force, torque, w = self.readings()
        self.st_pub.publish(String(data=json.dumps(self.a.status(q, force - self.base_force, abs(torque), abs(w)))))


class PlugInventory(Node):
    def __init__(self):
        super().__init__("plug_inventory")
        self.inv = Inventory(_param(self, "robot_description", ""))
        self.pub = self.create_publisher(String, "plugs/status", 10)
        self.create_subscription(String, "plugs/cmd", lambda m: self.inv.command(m.data), 50)
        for h in self.inv.holders:
            self.create_subscription(Bool, f"plugs/sensor/{h}", lambda m, h=h: self.inv.sensed(h, m.data), 10)
        self.create_timer(0.2, lambda: self.pub.publish(String(data=json.dumps(
            self.inv.status(self.get_clock().now().nanoseconds * 1e-9)))))
        stock = ", ".join(f"{n} {sum(c['full'])}/{len(c['full'])}" for n, c in self.inv.carriers.items())
        self.get_logger().info(f"plugs: {stock}")


class TractionEstimator(Node):
    """Wheel loads from the static split of the machine's weight; slip from wheel speed against the
    GNSS ground speed; traction from drive torque over radius. The JSON matches fs::Terramechanics."""

    def __init__(self):
        super().__init__("traction_estimator")
        self.wheels = list(_param(self, "wheels", [""]))
        self.loads = list(_param(self, "loads_n", [0.0]))
        self.radius = _param(self, "radius", 0.38)
        self.soil = _param(self, "soil", "")
        self.js, self.ground_speed, self.prev = {}, None, None
        self.pub = self.create_publisher(String, "terramechanics", 10)
        self.create_subscription(JointState, HW_STATES, self.on_js, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, "gnss/fix", self.on_fix, qos_profile_sensor_data)
        self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.odom_speed = 0.0
        self.create_timer(0.1, self.publish)

    def on_js(self, m):
        for i, n in enumerate(m.name):
            self.js[n] = (m.velocity[i] if m.velocity else 0.0, m.effort[i] if m.effort else 0.0)

    def on_odom(self, m):
        self.odom_speed = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)

    def on_fix(self, m: NavSatFix):
        if not math.isfinite(m.latitude) or m.position_covariance[0] > 0.25:
            self.ground_speed = None                       # only an RTK-grade fix measures slip
            return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.prev:
            lat0, lon0, t0 = self.prev
            dy = (m.latitude - lat0) * 111320.0
            dx = (m.longitude - lon0) * 111320.0 * math.cos(math.radians(m.latitude))
            if t > t0:
                self.ground_speed = math.hypot(dx, dy) / (t - t0)
        self.prev = (m.latitude, m.longitude, t)

    def publish(self):
        wheels, slips, res = [], [], 0.0
        v = self.ground_speed if self.ground_speed is not None else self.odom_speed
        for k, n in enumerate(self.wheels):
            if not n:
                continue
            w, tau = self.js.get(n, (0.0, 0.0))
            surf = w * self.radius
            den = max(abs(v), abs(surf), 0.05)
            slip = max(-1.0, min(1.0, (surf - v) / den)) if self.ground_speed is not None else 0.0
            fx = tau / self.radius
            res += fx
            slips.append(slip)
            wheels.append({"name": n, "touch": True, "N": self.loads[k] if k < len(self.loads) else 0.0, "z": 0.0,
                           "slip": slip, "fx": fx, "fy": 0.0, "rc": 0.0, "fmax": 0.0, "rut": 0.0})
        d = {"t": self.get_clock().now().nanoseconds * 1e-9, "mean_slip": sum(slips) / max(len(slips), 1),
             "lateral_utilisation": 0.0, "motion_resistance": res if abs(v) > 0.05 else 0.0, "wheels": wheels,
             "soil": self.soil, "measured": True}
        self.pub.publish(String(data=json.dumps(d)))


class SafetyBridge(Node):
    def __init__(self):
        super().__init__("safety_bridge")
        from .safety_bridge import BITS, CanSocket, MODES, decode
        self.decode, self.bits, self.modes = decode, BITS, MODES
        self.bus = _param(self, "bus", "can0")
        self.node_id = int(_param(self, "plc_node", 0x20))
        self.can = CanSocket(self.bus) if self.bus != "mock" else None
        self.mode, self.count, self.last, self.state = "travel", 0, None, None
        self.pub = self.create_publisher(HwSafety, "safety/hw", 10)
        self.create_subscription(String, "auger/status", self.on_auger, 10)
        self.create_subscription(Bool, "/fleet/estop", lambda m: setattr(self, "mock_estop", m.data), 10)
        self.mock_estop = False
        self.create_timer(0.01, self.poll)
        self.create_timer(0.05, self.heartbeat)
        self.get_logger().info(f"safety PLC node {self.node_id:#x} on {self.bus}"
                               + (" (mock: always clear except /fleet/estop)" if self.can is None else ""))

    def on_auger(self, m):
        try:
            self.mode = "planting" if json.loads(m.data).get("phase") not in ("idle", "stopped") else "travel"
        except ValueError:
            pass

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def poll(self):
        if self.can is None:
            self.state = {k: False for k in self.bits} | {"heartbeat_ok": True, "field_set": 0, "estop": self.mock_estop,
                                                         "sto_drive": self.mock_estop, "sto_tool": self.mock_estop}
            self.last = self.now()
        else:
            while (f := self.can.recv()) is not None:
                if f[0] == 0x180 + self.node_id:
                    self.state, self.last = self.decode(f[1]), self.now()
        if self.state is None or self.now() - self.last > 0.3:
            return                                   # PLC silent: publish nothing, the monitor stops on plc_link
        m = HwSafety()
        m.header.stamp = self.get_clock().now().to_msg()
        for k in self.bits:
            setattr(m, k, bool(self.state[k]))
        m.field_set = int(self.state["field_set"])
        m.field_name = str(m.field_set)
        m.simulated = self.can is None
        self.pub.publish(m)

    def heartbeat(self):
        self.count = (self.count + 1) & 0xFF
        if self.can is not None:
            self.can.send(0x200 + self.node_id, bytes([self.modes[self.mode], self.count]))


def hw_bridge():
    _spin(HwBridge)


def auger_driver():
    _spin(AugerDriver)


def plug_inventory():
    _spin(PlugInventory)


def traction_estimator():
    _spin(TractionEstimator)


def safety_bridge():
    _spin(SafetyBridge)
