"""Localization: EKF over wheel odometry, IMU and GNSS -> map-frame pose.

Subscribes (in the robot namespace): odom (wheel odometry), imu, gnss/fix, gnss/status.
Publishes: localization/odom (map frame), TF map -> <ns>/odom and <ns>/odom -> <ns>/base_link.

The map frame is the site's ENU frame; GNSS fixes are converted with the site origin.
Initial pose: the first GNSS fix plus the parked heading from the site file (operators park
the machine on a known bearing; on hardware a dual-antenna heading would replace this).

The published pose is 3-D: height from the site terrain model under the machine plus
<base_height>, roll and pitch from the IMU. The arm IK and the stability monitor need it on slopes.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from ..ekf import PoseEKF
from ..geometry import matrix_to_quat, quat_from_yaw, rpy_from_quat, yaw_from_quat
from ..ros_common import SENSOR, load_site, param, sim_now


def _pose_matrix(x, y, z, roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    T = np.eye(4)
    T[:3, :3] = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) \
        @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    T[:3, 3] = (x, y, z)
    return T


class Localization(Node):
    def __init__(self):
        super().__init__("localization")
        self.site = load_site(self)
        self.ns = param(self, "robot", "robot")
        lever = list(param(self, "gnss_lever", [0.0, 0.0, 0.0])) + [0.0]
        self.lever3 = np.array(lever[:3], float)          # x, y, z of the antenna in base_link
        self.use_imu_yaw = param(self, "use_imu_yaw", True)
        self.sigma_v = param(self, "odom_sigma_v", 0.05)
        self.sigma_w = param(self, "odom_sigma_w", 0.03)
        self.rate = param(self, "rate", 50.0)
        self.base_height = param(self, "base_height", 0.0)
        self.terrain = param(self, "terrain_height", True)
        self.roll = self.pitch = 0.0
        spawn = self.site.spawn(self.ns)
        self.ekf = PoseEKF(spawn["x"], spawn["y"], spawn["yaw"], lever=lever[:2])
        self.initialised = False
        self.last_t = None
        self.odom_pose = (0.0, 0.0, 0.0)
        self.imu_yaw_offset = None
        self.gnss_status = "rtk_fix"
        self.tfb = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, "localization/odom", 10)
        self.create_subscription(Odometry, "odom", self.on_odom, SENSOR)
        self.create_subscription(Imu, "imu", self.on_imu, SENSOR)
        self.create_subscription(NavSatFix, "gnss/fix", self.on_fix, SENSOR)
        self.create_subscription(String, "gnss/status", lambda m: setattr(self, "gnss_status", m.data), 10)
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(f"localization for {self.ns}: lever {lever}, start ({spawn['x']:.1f}, {spawn['y']:.1f})")

    def _advance(self):
        t = sim_now(self)
        if self.last_t is not None:
            self.ekf.predict(min(t - self.last_t, 0.5))
        self.last_t = t

    def on_odom(self, m: Odometry):
        self._advance()
        tw = m.twist.twist
        self.ekf.update_odom(tw.linear.x, tw.angular.z, tw.linear.y, self.sigma_v, self.sigma_w)
        q = m.pose.pose.orientation
        self.odom_pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_from_quat(q.x, q.y, q.z, q.w))

    def on_imu(self, m: Imu):
        self._advance()
        self.ekf.update_gyro(m.angular_velocity.z, 0.01)
        q = m.orientation
        self.roll, self.pitch, _ = rpy_from_quat(q.x, q.y, q.z, q.w)
        if self.use_imu_yaw and self.initialised:
            yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
            if self.imu_yaw_offset is None:   # align the AHRS heading once with the estimate
                self.imu_yaw_offset = self.ekf.x[2] - yaw
            self.ekf.update_heading(yaw + self.imu_yaw_offset, math.radians(3.0))

    def on_fix(self, m: NavSatFix):
        if m.status.status == NavSatStatus.STATUS_NO_FIX or math.isnan(m.latitude):
            return
        x, y = self.site.geo_to_enu(m.latitude, m.longitude)
        # Tilt compensation: on a slope a tall mast puts the antenna metres-scale lever arm * sin(roll)
        # off to the side. The EKF models the planar lever arm; remove the part due to roll/pitch here.
        th = self.ekf.x[2]
        full = _pose_matrix(0, 0, 0, self.roll, self.pitch, th)[:3, :3] @ self.lever3
        planar = _pose_matrix(0, 0, 0, 0.0, 0.0, th)[:3, :3] @ self.lever3
        x -= full[0] - planar[0]
        y -= full[1] - planar[1]
        sigma = math.sqrt(max(m.position_covariance[0], 1e-6)) if m.position_covariance_type else 0.05
        self._advance()
        if not self.initialised:
            lx, ly = self.ekf.lever
            th = self.ekf.x[2]
            self.ekf.x[0] = x - (math.cos(th) * lx - math.sin(th) * ly)
            self.ekf.x[1] = y - (math.sin(th) * lx + math.cos(th) * ly)
            self.initialised = True
            return
        self.ekf.update_gnss(x, y, sigma)

    def tick(self):
        if not self.initialised:
            return
        self._advance()
        stamp = self.get_clock().now().to_msg()
        x, y, th = self.ekf.pose
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = "map"
        o.child_frame_id = f"{self.ns}/base_link"
        o.pose.pose.position.x, o.pose.pose.position.y = x, y
        P = self.ekf.P
        o.pose.covariance[0], o.pose.covariance[7], o.pose.covariance[35] = P[0, 0], P[1, 1], P[2, 2]
        o.twist.twist.linear.x, o.twist.twist.linear.y = float(self.ekf.x[3]), float(self.ekf.x[4])
        o.twist.twist.angular.z = float(self.ekf.x[5])
        z = self.base_height + (float(self.site.soil_map.elevation(x, y)) if self.terrain else 0.0)
        o.pose.pose.position.z = z
        T_est = _pose_matrix(x, y, z, self.roll, self.pitch, th)
        q = o.pose.pose.orientation
        q.x, q.y, q.z, q.w = matrix_to_quat(T_est[:3, :3])
        self.pub.publish(o)
        # map -> odom so that map -> odom -> base_link equals the 3-D estimate.
        ox, oy, oth = self.odom_pose
        T_odom = _pose_matrix(ox, oy, 0.0, 0.0, 0.0, oth)
        self._send_matrix("map", f"{self.ns}/odom", T_est @ np.linalg.inv(T_odom), stamp)
        self._send(f"{self.ns}/odom", f"{self.ns}/base_link", ox, oy, oth, stamp)

    def _send_matrix(self, parent, child, T, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, T[:3, 3])
        r = t.transform.rotation
        r.x, r.y, r.z, r.w = matrix_to_quat(T[:3, :3])
        self.tfb.sendTransform(t)

    def _send(self, parent, child, x, y, yaw, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x, t.transform.translation.y = float(x), float(y)
        _, _, qz, qw = quat_from_yaw(yaw)
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tfb.sendTransform(t)


def main():
    rclpy.init()
    n = Localization()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
