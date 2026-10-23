"""Stability monitor: force-angle tip-over margin from the real mass distribution.

Centre of gravity: every link's mass and CoM from robot_description, placed with live TF (so the
arm's reach and the articulation angle move the CG), minus payload already used (plugs and water,
from head/status), plus the auger's thrust reaction at the bit (it lifts that side of the machine).
Gravity direction: IMU roll and pitch. Support polygon: the convex hull of the ground contacts
(each road wheel or tyre centre minus its radius along its module's z axis).

Publishes <ns>/stability (fs_interfaces/StabilityStatus) at 10 Hz; level WARN below
<warn_deg>, CRITICAL below <critical_deg>. Navigators slow down on WARN and stop on CRITICAL.
"""
import math

import numpy as np
import rclpy
from fs_interfaces.msg import HeadStatus, StabilityStatus
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from ..geometry import quat_to_matrix, transform_to_matrix
from ..ros_common import LATCHED, SENSOR, lookup, param, pt
from ..stability import edge_name, force_angle_margins, parse_urdf_masses

G = 9.81


class StabilityMonitor(Node):
    def __init__(self):
        super().__init__("stability_monitor")
        self.ns = param(self, "robot", "pahadi")
        self.contacts = list(param(self, "contact_links", []))
        self.contact_parent = list(param(self, "contact_parents", []))
        self.radius = float(param(self, "wheel_radius", 0.21))
        self.warn = param(self, "warn_deg", 12.0)
        self.critical = param(self, "critical_deg", 6.0)
        # payload items: [link, x, y, z, kg_per_unit, units_full, kind]; kind plugs|water
        self.payload = param(self, "payload", ["rear_link,-0.69,0,0.85,0.25,200,plugs",
                                                "rear_link,-0.85,0,0.20,1.0,150,water"])
        self.payload = [p.split(",") for p in self.payload]
        self.masses = {}
        self.imu = None
        self.head = None
        self.thrust = 0.0
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.pub = self.create_publisher(StabilityStatus, "stability", 10)
        self.create_subscription(String, "robot_description", self.on_desc, LATCHED)
        self.create_subscription(Imu, "imu", lambda m: setattr(self, "imu", m), SENSOR)
        self.create_subscription(HeadStatus, "head/status", self.on_head, 10)
        self.last_level = 0
        self.create_timer(0.1, self.tick)

    def on_desc(self, m: String):
        self.masses = parse_urdf_masses(m.data)
        total = sum(v[0] for v in self.masses.values())
        self.get_logger().info(f"{len(self.masses)} links, {total:.0f} kg; {len(self.contacts)} ground contacts")

    def on_head(self, h: HeadStatus):
        self.head = h
        self.thrust = h.auger_thrust_n

    def tick(self):
        if not self.masses or self.imu is None or not self.contacts:
            return
        base = f"{self.ns}/base_link"
        q = self.imu.orientation
        R_wb = quat_to_matrix(q.x, q.y, q.z, q.w)
        yaw = math.atan2(R_wb[1, 0], R_wb[0, 0])
        c, s = math.cos(-yaw), math.sin(-yaw)
        level = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ R_wb   # base -> gravity-aligned, heading kept

        def in_level(frame, local):
            if frame == "base_link":
                p = np.asarray(local, float)
            else:
                t = lookup(self.tf, base, f"{self.ns}/{frame}")
                if t is None:
                    return None
                T = transform_to_matrix(t.transform)
                p = (T @ np.array([*local, 1.0]))[:3]
            return level @ p

        items = []
        for link, (m, com) in self.masses.items():
            p = in_level(link, com)
            if p is not None:
                items.append((m, p))
        # payload already used comes off the CG
        if self.head is not None:
            for link, x, y, z, kg, full, kind in self.payload:
                left = self.head.plugs_left if kind == "plugs" else self.head.water_left_l
                used = max(0.0, float(full) - float(left)) * float(kg)
                p = in_level(link, (float(x), float(y), float(z)))
                if p is not None and used > 0:
                    items.append((-used, p))
        # auger reaction: an upward force at the bit, modelled as negative weight there
        if self.thrust > 1.0:
            p = in_level("auger_link", (0.0, 0.0, 0.0))
            if p is not None:
                items.append((-self.thrust / G, p))
        M = sum(m for m, _ in items)
        cg = sum(m * p for m, p in items) / M

        pts = []
        for link, parent in zip(self.contacts, self.contact_parent):
            centre = in_level(link, (0.0, 0.0, 0.0))
            if centre is None:
                continue
            if parent == "base_link":
                down = level @ np.array([0.0, 0.0, -1.0])
            else:
                t = lookup(self.tf, base, f"{self.ns}/{parent}")
                if t is None:
                    continue
                down = level @ (transform_to_matrix(t.transform)[:3, :3] @ np.array([0.0, 0.0, -1.0]))
            pts.append(centre + self.radius * down)
        if len(pts) < 3:
            return
        pts = np.array(pts)
        idx, margins = force_angle_margins(pts, cg, [0.0, 0.0, -M * G])
        if not margins:
            return
        k = int(np.argmin(margins))
        md = math.degrees(margins[k])
        st = StabilityStatus()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = "map"
        st.margin_deg = md
        st.level = StabilityStatus.CRITICAL if md < self.critical else (StabilityStatus.WARN if md < self.warn else StabilityStatus.OK)
        h0, h1 = pts[idx[k]], pts[idx[(k + 1) % len(idx)]]
        st.critical_edge = edge_name(h0, h1, cg, 0.0)
        st.roll_deg = math.degrees(math.atan2(R_wb[2, 1], R_wb[2, 2]))
        st.pitch_deg = math.degrees(-math.asin(max(-1.0, min(1.0, R_wb[2, 0]))))
        st.mass_kg = float(M)
        st.edge_margins_deg = [math.degrees(m) for m in margins]
        # express CG and polygon in the map frame for RViz
        t = lookup(self.tf, "map", base)
        if t is not None:
            T = transform_to_matrix(t.transform)
            back = T[:3, :3] @ level.T
            cgm = back @ cg + T[:3, 3]
            st.cg = pt(*cgm)
            st.support_polygon = [pt(*(back @ pts[i] + T[:3, 3])) for i in idx]
        if st.level != self.last_level:
            log = [self.get_logger().info, self.get_logger().warning, self.get_logger().error][st.level]
            log(f"stability {['OK', 'WARN', 'CRITICAL'][st.level]}: {md:.1f} deg margin over the {st.critical_edge} "
                f"edge (roll {st.roll_deg:.1f}, pitch {st.pitch_deg:.1f})")
            self.last_level = st.level
        self.pub.publish(st)


def main():
    rclpy.init()
    n = StabilityMonitor()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
