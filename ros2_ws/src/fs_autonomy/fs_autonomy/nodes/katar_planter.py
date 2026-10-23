"""Katar planter: spacing trigger, cab STOP cue / speed governor, boom correction.

Plan: a spot every <spacing> m along the planting line (shoulder centreline offset 2.5 m into
the verge), minus no-dig zones (all of them with surveyed:=true; only visible ones without).

stop_and_go  Tells the driver to stop early enough for their reaction time and braking, then
             lets the boom sled absorb the remaining along-track error (+-rail) and the carriage
             the lateral error, so spacing does not depend on how precisely the driver stops.
on_the_move  Never stops the tractor. The sled starts at its front end and runs back at ground
             speed while the bit is in the soil. Speed is governed so the sled never runs out:
             v <= rail_length / time_in_ground, and v <= spacing / cycle_time.

Station estimates come from localization (GNSS + odometry), never from ground truth.
"""
import math

import rclpy
from action_msgs.msg import GoalStatus
from fs_interfaces.action import PlantSpot
from fs_interfaces.msg import Mission, PlantRecord, Spot
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float64, String
from tf2_ros import Buffer, TransformListener

from .. import head_model
from ..geometry import transform_to_matrix
from ..planners import planting_line, roadside_spots
from ..ros_common import LATCHED, load_site, lookup, param, pt, sim_now


class KatarPlanter(Node):
    def __init__(self):
        super().__init__("katar_planter")
        self.site = load_site(self)
        self.ns = param(self, "robot", "katar")
        self.spacing = param(self, "spacing", 3.0)
        self.mode = param(self, "mode", "stop_and_go")
        self.rail_min, self.rail_max = param(self, "rail_min", -1.8), param(self, "rail_max", 0.6)
        self.surveyed = param(self, "surveyed", True)
        self.v_set = param(self, "speed_kmh", 2.5) / 3.6
        self.reaction = param(self, "driver_reaction_s", 0.8)
        self.decel = param(self, "driver_decel", 0.9)
        self.depth = param(self, "hole_depth", 0.3)
        self.dose = param(self, "water_dose_l", 1.0)
        self.line = planting_line(self.site)
        self.spots = roadside_spots(self.site, self.spacing, surveyed=self.surveyed)
        self.k = 0
        self.state = "drive"
        self.state_t = 0.0
        self.v = 0.0
        self.hint = None
        self.goal_future = None
        self.result_future = None
        self.planted = self.skipped = 0
        self.t_start = None

        soil = self.site.soil_map.soil_at(self.spots[0].x, self.spots[0].y)
        cyc = head_model.predict("katar", soil, 0.15, self.depth, self.dose)
        self.t_ground = cyc.total_s - 1.0
        self.v_gov = min(self.v_set, 0.85 * (self.rail_max - self.rail_min) / self.t_ground,
                         self.spacing / (cyc.total_s + 3.0))
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.limit_pub = self.create_publisher(Float64, "speed_limit", 10)
        self.state_pub = self.create_publisher(String, "planter/state", 10)
        self.record_pub = self.create_publisher(PlantRecord, "/plants/records", 50)
        self.mission_pub = self.create_publisher(Mission, "mission", LATCHED)
        self.client = ActionClient(self, PlantSpot, "plant_spot")
        self.create_subscription(Odometry, "localization/odom", self.on_odom, 10)
        self.create_timer(0.1, self.tick)
        self.publish_mission()
        n_skip = sum(s.skip for s in self.spots)
        self.get_logger().info(
            f"{len(self.spots)} spots at {self.spacing} m ({n_skip} no-dig), mode {self.mode}, "
            f"survey {'yes' if self.surveyed else 'NO'}; predicted cycle {cyc.total_s:.1f} s"
            + (f", governed speed {self.v_gov * 3.6:.2f} km/h" if self.mode == "on_the_move" else ""))

    def publish_mission(self):
        m = Mission()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.robot, m.pattern, m.spacing_m = self.ns, "roadside_row", float(self.spacing)
        for sp in self.spots:
            s = Spot()
            s.id, s.position, s.station = sp.id, pt(sp.x, sp.y), sp.station
            s.tool, s.note = "skip" if sp.skip else sp.tool, sp.note
            m.spots.append(s)
        self.mission_pub.publish(m)

    def on_odom(self, m: Odometry):
        self.v = m.twist.twist.linear.x

    def limit(self, v):
        self.limit_pub.publish(Float64(data=float(v)))

    def set_state(self, s):
        self.state = s
        self.state_t = sim_now(self)
        self.state_pub.publish(String(data=s))

    def tip_station(self, sled_q: float):
        """Station on the planting line of the bit with the sled at sled_q."""
        t = lookup(self.tf, "map", f"{self.ns}/trailer_link")
        if t is None:
            return None
        T = transform_to_matrix(t.transform)
        p = T @ [-0.85 + sled_q, 2.5, 0.0, 1.0]
        s, _, self.hint = self.line.project(p[0], p[1], self.hint)
        return s

    def record_skip(self, sp, reason):
        r = PlantRecord()
        r.header.stamp = self.get_clock().now().to_msg()
        r.header.frame_id = "map"
        r.robot, r.spot_id, r.tool, r.outcome, r.reason = self.ns, sp.id, "none", "skipped", reason
        r.intended = r.actual = pt(sp.x, sp.y)
        r.latitude, r.longitude = self.site.enu_to_geo(sp.x, sp.y)
        self.record_pub.publish(r)
        self.skipped += 1

    def send(self, sp, track):
        g = PlantSpot.Goal()
        g.spot_id, g.target, g.tool, g.track_ground = sp.id, pt(sp.x, sp.y), "auger", track
        self.goal_future = self.client.send_goal_async(g)
        self.set_state("planting")

    def tick(self):
        if self.t_start is None:
            self.t_start = sim_now(self)
        if self.k >= len(self.spots):
            if self.state != "done":
                self.limit(-1.0)
                self.set_state("done")
                dt = sim_now(self) - self.t_start
                self.get_logger().info(f"row complete: {self.planted} planted, {self.skipped} skipped in "
                                       f"{dt / 60:.1f} min ({self.planted / max(dt, 1) * 3600:.0f} plants/h)")
            return
        sp = self.spots[self.k]
        home = self.rail_max if self.mode == "on_the_move" else 0.0
        s_tip = self.tip_station(home)
        if s_tip is None:
            return
        d = sp.station - s_tip

        if self.state == "planting":
            if self.goal_future is not None and self.goal_future.done():
                gh = self.goal_future.result()
                self.goal_future = None
                if not gh.accepted:
                    self.get_logger().warning("head busy, retrying")
                    self.set_state("drive")
                    return
                self.result_future = gh.get_result_async()
            if self.result_future is not None and self.result_future.done():
                res = self.result_future.result()
                ok = res.status == GoalStatus.STATUS_SUCCEEDED and res.result.success
                self.planted += int(ok)
                self.skipped += int(not ok)
                self.result_future = None
                self.k += 1
                self.set_state("drive")
                if self.mode == "stop_and_go":
                    self.limit(-1.0)
            elif self.mode == "on_the_move" and self.k + 1 < len(self.spots):
                nxt = self.spots[self.k + 1]
                if nxt.station - s_tip < 0.3:      # next spot arrives before the head is free
                    self.limit(0.0)
            return

        if sp.skip:
            if d < 0:
                self.record_skip(sp, sp.note)
                self.k += 1
            return

        if self.mode == "on_the_move":
            self.limit(self.v_gov)
            if d <= 0.0:
                if d < self.rail_min - self.rail_max:
                    self.record_skip(sp, "passed before the head was ready")
                    self.k += 1
                else:
                    self.send(sp, track=True)
            return

        # stop and go
        if self.state == "drive":
            v = max(self.v, 0.0)
            stop_dist = v * self.reaction + v * v / (2 * self.decel) + 0.15
            if d < stop_dist:
                self.limit(0.0)
                self.set_state("stopping")
        elif self.state == "stopping":
            if abs(self.v) < 0.03 and sim_now(self) - self.state_t > 1.0:
                if self.rail_min <= d <= self.rail_max:
                    self.send(sp, track=False)
                elif d > self.rail_max:          # stopped short: creep on
                    self.limit(-1.0)
                    self.set_state("drive")
                else:
                    self.record_skip(sp, f"overshot by {-d:.2f} m")
                    self.k += 1
                    self.limit(-1.0)
                    self.set_state("drive")


def main():
    rclpy.init()
    n = KatarPlanter()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
