"""Jaal fleet manager: coverage plan for the block, one mission per robot, and the tender truck.

Plan   grid (square or triangular) over the field, spots on surveyed obstacles shifted along the
       row or skipped (stones are not in the drone survey, so the robots find those by auger
       refusal), rows split into balanced contiguous blocks, boustrophedon order in each block.
Tender one or more service bays; robots reserve a bay (/fleet/reserve_bay) and wait at their
       own slot if it is taken, then run /fleet/tender_service: drum swap, water refill and
       charging in parallel. Spare drums run out; then the tender makes a restock trip to the
       nursery and robots wait. This is the logistics the concept sheet says sets the real
       throughput, so it is modelled explicitly and logged as a per-robot state timeline.
"""
import json
import time
from pathlib import Path

import rclpy
from fs_interfaces.action import TenderService
from fs_interfaces.msg import Mission, RobotState, Spot
from fs_interfaces.srv import ReserveBay
from geometry_msgs.msg import Pose
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from ..geometry import quat_from_yaw
from ..planners import grid_rows, resolve_obstacles, serpentine, split_rows
from ..ros_common import LATCHED, load_site, param, pt, sim_now, wait_sim


def pose(x, y, yaw=0.0) -> Pose:
    p = Pose()
    p.position = pt(x, y)
    _, _, p.orientation.z, p.orientation.w = quat_from_yaw(yaw)
    return p


class FleetManager(Node):
    def __init__(self):
        super().__init__("fleet_manager")
        self.site = load_site(self)
        self.robots = list(param(self, "robots", ["jaal_1", "jaal_2", "jaal_3"]))
        self.field = param(self, "field", "demo")
        self.spacing = param(self, "spacing", 3.0)
        self.pattern = param(self, "pattern", "square")
        self.bays = [None] * int(param(self, "bays", 1))
        self.swap_s = param(self, "drum_swap_s", 120.0)
        self.fill_lps = param(self, "water_fill_lps", 2.0)
        self.charge_kw = param(self, "charge_kw", 11.0)
        self.battery_kwh = param(self, "battery_kwh", 30.0)
        self.capacity = int(param(self, "plugs_per_robot", 360))
        out = param(self, "out_dir", "")
        self.out = Path(out) if out else Path.home() / ".ros" / "fs_logs" / f"fleet-{time.strftime('%Y%m%d-%H%M%S')}"
        self.out.mkdir(parents=True, exist_ok=True)
        t = self.site["tender"]
        stock = int(param(self, "stock_plugs", -1))
        self.stock = stock if stock >= 0 else int(t["spare_drums"]) * int(t["plugs_per_drum"])
        self.stock_water = float(t["water_l"])
        self.restock_s = float(param(self, "restock_s", t["restock_s"]))
        self.restock_plugs = int(t["spare_drums"]) * int(t["plugs_per_drum"])
        self.restock_eta = None
        self.bay_xy = t["bay"]
        self.wait_xy = t["wait"]
        self.queue = []
        self.slots = {}
        self.states = {}
        self.timeline = []          # (robot, state, t0, t1)
        self.cb = ReentrantCallbackGroup()

        self.missions = self.plan()
        self.mission_pubs = {}
        for r, m in self.missions.items():
            self.mission_pubs[r] = self.create_publisher(Mission, f"/{r}/mission", LATCHED)
            self.mission_pubs[r].publish(m)
            self.create_subscription(RobotState, f"/{r}/state", self.on_state, 10, callback_group=self.cb)
        self.status_pub = self.create_publisher(String, "/fleet/status", 10)
        self.create_service(ReserveBay, "/fleet/reserve_bay", self.on_reserve, callback_group=self.cb)
        self.server = ActionServer(self, TenderService, "/fleet/tender_service", self.on_service, callback_group=self.cb)
        self.create_timer(1.0, self.tick, callback_group=self.cb)

    # ------------------------------------------------------------------ plan
    def plan(self):
        f = self.site["field"]
        poly = f["demo_polygon"] if self.field == "demo" else f["polygon"]
        rows, row_dy = grid_rows(poly, self.spacing, self.pattern)
        surveyed = [o for o in self.site.get("obstacles", []) if o["kind"] != "stone"]
        spots = resolve_obstacles(rows, surveyed)
        k = 0
        for row in spots:
            for sp in row:
                sp.id = k
                k += 1
        blocks = split_rows(spots, len(self.robots))
        out = {}
        for r, block in zip(self.robots, blocks):
            order = serpentine(spots, block, start_dir=1)
            m = Mission()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.robot, m.pattern = r, f"{self.pattern}_grid"
            m.spacing_m, m.row_spacing_m = float(self.spacing), float(row_dy)
            for sp in order:
                s = Spot()
                s.id, s.position, s.row = sp.id, pt(sp.x, sp.y), sp.row
                s.tool, s.note = ("skip" if sp.skip else "auger"), sp.note
                m.spots.append(s)
            out[r] = m
            plantable = sum(1 for sp in order if not sp.skip)
            self.get_logger().info(f"{r}: rows {block[0]}-{block[-1]}, {plantable} spots" if block else f"{r}: idle")
        total = sum(1 for row in spots for sp in row if not sp.skip)
        area = (poly[1][0] - poly[0][0]) * (poly[2][1] - poly[1][1]) / 1e4
        self.get_logger().info(f"{self.field} block {area:.2f} ha, {self.pattern} {self.spacing} m: {total} spots "
                               f"({total / area:.0f}/ha), {len(self.robots)} robots, tender stock {self.stock} plugs")
        return out

    # ------------------------------------------------------------------ state + timeline
    def on_state(self, s: RobotState):
        t = sim_now(self)
        prev = self.states.get(s.robot)
        if prev is None or prev.state != s.state:
            if self.timeline and any(e[0] == s.robot and e[3] is None for e in self.timeline):
                for e in self.timeline:
                    if e[0] == s.robot and e[3] is None:
                        e[3] = t
            self.timeline.append([s.robot, s.state, t, None])
        self.states[s.robot] = s

    def tick(self):
        t = sim_now(self)
        if self.restock_eta is not None and t >= self.restock_eta:
            self.stock += self.restock_plugs
            self.restock_eta = None
            self.get_logger().info(f"tender back from the nursery: {self.stock} plugs in stock")
        planted = sum(s.planted for s in self.states.values())
        st = {"t": round(t, 1), "stock_plugs": self.stock, "stock_water_l": round(self.stock_water),
              "restock_eta_s": None if self.restock_eta is None else round(self.restock_eta - t),
              "bays": self.bays, "queue": self.queue, "planted": planted,
              "robots": {r: {"state": s.state, "planted": s.planted, "plugs": s.plugs_left, "soc": round(s.soc, 3),
                             "gnss": s.gnss, "x": round(s.x, 1), "y": round(s.y, 1)} for r, s in self.states.items()}}
        self.status_pub.publish(String(data=json.dumps(st)))
        rows = [f"{r},{s},{a:.1f},{(b if b is not None else t):.1f}" for r, s, a, b in self.timeline]
        (self.out / "fleet_timeline.csv").write_text("robot,state,t0,t1\n" + "\n".join(rows) + "\n")

    # ------------------------------------------------------------------ tender
    def wait_slot(self, robot):
        """Slot by arrival order, kept until the robot is served; slot 0 is nearest the bay."""
        if robot not in self.slots:
            taken = set(self.slots.values())
            self.slots[robot] = next(k for k in range(len(self.robots) + 1) if k not in taken)
        k = self.slots[robot]
        return pose(self.wait_xy[0], self.wait_xy[1] - 4.5 * k)

    def on_reserve(self, req, res):
        r = req.robot
        if req.release:
            self.bays = [None if b == r else b for b in self.bays]
            res.granted = False
            return res
        res.wait_pose = self.wait_slot(r)
        if r in self.bays:
            i = self.bays.index(r)
        elif None in self.bays and (not self.queue or self.queue[0] == r):
            i = self.bays.index(None)
            self.bays[i] = r
            if r in self.queue:
                self.queue.remove(r)
            self.slots.pop(r, None)
        else:
            if r not in self.queue:
                self.queue.append(r)
            res.granted, res.queue_position = False, self.queue.index(r) + 1
            return res
        res.granted, res.queue_position = True, 0
        res.bay_pose = pose(self.bay_xy[0], self.bay_xy[1] + 4.0 * i, self.bay_xy[2])
        return res

    def on_service(self, gh):
        g = gh.request
        fb = TenderService.Feedback()
        t0 = sim_now(self)
        need = min(g.plugs_needed, self.capacity)
        if need > 0 and self.stock < need:
            if self.restock_eta is None:
                self.restock_eta = sim_now(self) + self.restock_s
                self.get_logger().warning(f"tender out of drums ({self.stock} plugs); restock trip "
                                       f"{self.restock_s / 60:.0f} min")
            while self.stock < need:
                fb.stage, fb.remaining_s = "waiting_stock", float((self.restock_eta or sim_now(self)) - sim_now(self))
                gh.publish_feedback(fb)
                wait_sim(self, 2.0)
        swap = self.swap_s if need > 0 else 0.0
        water = min(g.water_needed_l, self.stock_water)
        fill = water / self.fill_lps
        charge = 0.0
        soc_after = g.soc
        if g.soc < 0.6:
            soc_after = 0.95
            charge = (soc_after - g.soc) * self.battery_kwh / self.charge_kw * 3600
        dur = max(swap, fill, charge) + 20.0
        stage = "drum_swap" if swap >= max(fill, charge) else ("charge" if charge > fill else "refill")
        t_end = sim_now(self) + dur
        while sim_now(self) < t_end:
            fb.stage, fb.remaining_s = stage, float(t_end - sim_now(self))
            gh.publish_feedback(fb)
            wait_sim(self, 2.0)
        self.stock -= need
        self.stock_water -= water
        gh.succeed()
        res = TenderService.Result()
        res.plugs_loaded, res.water_loaded_l, res.soc_after = need, water, soc_after
        res.service_time_s = sim_now(self) - t0
        res.note = f"{need} plugs, {water:.0f} L, SoC {g.soc:.0%} -> {soc_after:.0%}"
        self.get_logger().info(f"serviced {g.robot}: {res.note} in {res.service_time_s:.0f} s; stock {self.stock}")
        return res


def main():
    rclpy.init()
    n = FleetManager()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
