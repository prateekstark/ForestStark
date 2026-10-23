"""Traction monitor: wheel/track-soil state -> fs_interfaces/TractionStatus, with immobilisation
and side-slip detection the navigators act on.

In simulation the per-contact numbers come from the fs::Terramechanics system (terramechanics
topic). On hardware the same message would be filled from motor currents and odometry-vs-GNSS
slip; the navigators only read TractionStatus.
"""
import rclpy
from fs_interfaces.msg import TractionStatus
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from ..ros_common import param, parse_json, sim_now


class TractionMonitor(Node):
    def __init__(self):
        super().__init__("traction_monitor")
        self.declare_parameter("site_file", "")
        self.slip_limit = param(self, "immobilised_slip", 0.6)
        self.slip_time = param(self, "immobilised_time_s", 3.0)
        self.lat_limit = param(self, "side_slip_utilisation", 0.85)
        self.cmd_v = 0.0
        self.high_slip_since = None
        self.high_lat_since = None
        self.was = (False, False)
        self.pub = self.create_publisher(TractionStatus, "traction", 10)
        self.create_subscription(String, "terramechanics", self.on_tm, 10)
        self.create_subscription(Twist, "cmd_vel", lambda m: setattr(self, "cmd_v", abs(m.linear.x) + abs(m.angular.z)), 10)

    def on_tm(self, m: String):
        d = parse_json(m.data)
        if not d:
            return
        t = sim_now(self)
        s = TractionStatus()
        s.header.stamp = self.get_clock().now().to_msg()
        s.soil = d.get("soil", "")
        for w in d.get("wheels", []):
            s.wheels.append(w["name"])
            s.normal_load_n.append(float(w["N"]))
            s.sinkage_m.append(float(w["z"]))
            s.slip.append(float(w["slip"]))
            s.traction_n.append(float(w["fx"]))
            s.lateral_n.append(float(w["fy"]))
        touching = [abs(w["slip"]) for w in d.get("wheels", []) if w.get("touch")]
        s.mean_slip = sum(touching) / len(touching) if touching else 0.0
        s.lateral_utilisation = float(d.get("lateral_utilisation", 0.0))
        s.motion_resistance_n = float(d.get("motion_resistance", 0.0))
        moving_cmd = self.cmd_v > 0.05
        if moving_cmd and s.mean_slip > self.slip_limit:
            self.high_slip_since = self.high_slip_since or t
        else:
            self.high_slip_since = None
        if s.lateral_utilisation > self.lat_limit:
            self.high_lat_since = self.high_lat_since or t
        else:
            self.high_lat_since = None
        s.immobilised = self.high_slip_since is not None and t - self.high_slip_since > self.slip_time
        s.side_slipping = self.high_lat_since is not None and t - self.high_lat_since > 1.0
        if (s.immobilised, s.side_slipping) != self.was:
            if s.immobilised:
                self.get_logger().error(f"immobilised: slip {s.mean_slip:.0%} on {s.soil}")
            if s.side_slipping:
                self.get_logger().error(f"side-slipping: lateral shear {s.lateral_utilisation:.0%} of capacity")
            self.was = (s.immobilised, s.side_slipping)
        self.pub.publish(s)


def main():
    rclpy.init()
    n = TractionMonitor()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
