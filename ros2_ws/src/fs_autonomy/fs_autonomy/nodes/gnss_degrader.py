"""GNSS quality model between Gazebo's NavSat sensor and the robot, and the same contract on hardware.

Gazebo gives an RTK-grade fix everywhere. This node makes it behave like a field receiver:
  mode rtk       RTK fix (sigma 2 cm) except inside the site's degraded zones (tree lines),
                 where it falls back to float (correlated wander, sigma ~0.35 m), plus random
                 dropouts (no fix) at <dropouts_per_hour>.
  mode standard  plain GNSS: correlated wander with sigma 1.5 m, 1 Hz.
  mode none      no fix at all (odometry only).
  mode receiver  on the robot: a real receiver's driver publishes gnss/fix_raw; the fix passes through
                 unchanged and its status is read from NavSatStatus and the covariance
                 (GBAS = RTK fix, sigma < 0.5 m = float, any other fix = standard).
Publishes gnss/fix and gnss/status (rtk_fix | float | standard | none).
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String

from ..geometry import point_in_polygon
from ..ros_common import SENSOR, load_site, param, sim_now


class GnssDegrader(Node):
    def __init__(self):
        super().__init__("gnss_degrader")
        self.site = load_site(self)
        self.mode = param(self, "mode", "rtk")
        self.dropouts_per_hour = param(self, "dropouts_per_hour", 0.0)
        self.rng = np.random.default_rng(param(self, "seed", 1))
        self.zones = self.site.get("rtk_degraded_zones", []) or []
        self.err = np.zeros(2)
        self.last_t = None
        self.dropout_until = -1.0
        self.last_standard = -1.0
        self.status = ""
        self.pub = self.create_publisher(NavSatFix, "gnss/fix", 10)
        self.spub = self.create_publisher(String, "gnss/status", 10)
        self.create_subscription(NavSatFix, "gnss/fix_raw", self.on_fix, SENSOR)
        self.get_logger().info(f"GNSS model: {self.mode}, {len(self.zones)} degraded zone(s), "
                               f"{self.dropouts_per_hour} dropouts/h")

    def _wander(self, sigma, tau, dt):
        a = math.exp(-dt / tau)
        self.err = a * self.err + sigma * math.sqrt(1 - a * a) * self.rng.normal(size=2)

    def on_receiver(self, m: NavSatFix):
        sigma = math.sqrt(max(m.position_covariance[0], 0.0)) if m.position_covariance_type else 9.0
        if m.status.status < NavSatStatus.STATUS_FIX or not math.isfinite(m.latitude):
            status = "none"
        elif m.status.status == NavSatStatus.STATUS_GBAS_FIX and sigma < 0.1:
            status = "rtk_fix"
        else:
            status = "float" if sigma < 0.5 else "standard"
        if status != self.status:
            self.get_logger().info(f"GNSS {self.status or 'start'} -> {status}")
            self.status = status
        self.spub.publish(String(data=status))
        self.pub.publish(m)

    def on_fix(self, m: NavSatFix):
        if self.mode == "receiver":
            return self.on_receiver(m)
        t = sim_now(self)
        dt = 0.1 if self.last_t is None else max(1e-3, t - self.last_t)
        self.last_t = t
        x, y = self.site.geo_to_enu(m.latitude, m.longitude)
        out = NavSatFix()
        out.header = m.header
        out.altitude = m.altitude
        out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        status, sigma = "rtk_fix", 0.02

        if self.mode == "none":
            status = "none"
        elif self.mode == "standard":
            self._wander(1.5, 30.0, dt)
            status, sigma = "standard", 1.5
            if t - self.last_standard < 1.0:
                return
            self.last_standard = t
        else:
            if self.dropouts_per_hour > 0 and t > self.dropout_until and \
                    self.rng.random() < self.dropouts_per_hour * dt / 3600:
                self.dropout_until = t + self.rng.uniform(20, 90)
                self.get_logger().warning(f"RTK dropout for {self.dropout_until - t:.0f} s")
            zone = next((z for z in self.zones if point_in_polygon(x, y, z["polygon"])), None)
            if t < self.dropout_until:
                status = "none"
            elif zone is not None:
                self._wander(zone.get("sigma", 0.35), 20.0, dt)
                status, sigma = "float", zone.get("sigma", 0.35)
            else:
                self.err *= 0.5      # fix re-acquired: wander collapses
        if status != self.status:
            self.get_logger().info(f"GNSS {self.status or 'start'} -> {status}")
            self.status = status
        self.spub.publish(String(data=status))
        if status == "none":
            out.status.status = NavSatStatus.STATUS_NO_FIX
            out.latitude = out.longitude = float("nan")
            self.pub.publish(out)
            return
        ex, ey = (x, y) if status == "rtk_fix" else (x + self.err[0], y + self.err[1])
        out.latitude, out.longitude = self.site.enu_to_geo(ex, ey)
        out.status.status = NavSatStatus.STATUS_GBAS_FIX if status == "rtk_fix" else NavSatStatus.STATUS_FIX
        out.status.service = NavSatStatus.SERVICE_GPS
        out.position_covariance[0] = out.position_covariance[4] = sigma ** 2
        out.position_covariance[8] = (2 * sigma) ** 2
        self.pub.publish(out)


def main():
    rclpy.init()
    n = GnssDegrader()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
