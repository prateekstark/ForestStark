"""Mission logger: every PlantRecord goes to CSV and GeoJSON (for survival audits and GIS), with a
running summary. In simulation it also scores what the robot could not know: holes bored inside
a hidden no-dig zone (e.g. an unsurveyed telecom cable) are reported as utility strikes.

Output: <out_dir>/plants.csv, plants.geojson, summary.json (default ~/.ros/fs_logs/<site>-<time>/).
Publishes /plants/summary (std_msgs/String, JSON) every 5 s.
"""
import csv
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from fs_interfaces.msg import PlantRecord
from rclpy.node import Node
from std_msgs.msg import String

from ..geometry import Polyline
from ..ros_common import load_site, param, sim_now

FIELDS = ["time_s", "robot", "id", "spot_id", "outcome", "tool", "reason", "species", "x", "y", "lat", "lon",
          "error_cm", "soil", "rock_depth_m", "hole_depth_m", "bore_energy_kj", "cycle_time_s", "water_l",
          "region", "microsite", "method", "deviations", "survival_1y", "survival_risk"]


class MissionLogger(Node):
    def __init__(self):
        super().__init__("mission_logger")
        self.site = load_site(self)
        out = param(self, "out_dir", "")
        stamp = time.strftime("%Y%m%d-%H%M%S")
        self.out = Path(out) if out else Path.home() / ".ros" / "fs_logs" / f"{self.site.name}-{stamp}"
        self.out.mkdir(parents=True, exist_ok=True)
        self.csv_f = open(self.out / "plants.csv", "w", newline="")
        self.csv = csv.DictWriter(self.csv_f, fieldnames=FIELDS)
        self.csv.writeheader()
        self.records = []
        self.strikes = []
        self.t0 = None
        self.hidden = []
        rs = self.site.get("roadside")
        if rs:
            cl = Polyline(rs["shoulder_centerline"])
            self.hidden = [(cl, z) for z in rs["no_dig"] if not z.get("visible", True)]
        self.pub = self.create_publisher(String, "/plants/summary", 10)
        self.create_subscription(PlantRecord, "/plants/records", self.on_record, 100)
        self.create_timer(5.0, self.publish_summary)
        self.create_timer(60.0, self.log_summary)
        self.get_logger().info(f"logging plant records to {self.out}")

    def on_record(self, r: PlantRecord):
        t = sim_now(self)
        if self.t0 is None:
            self.t0 = t
        row = {"time_s": round(t, 1), "robot": r.robot, "id": r.id, "spot_id": r.spot_id, "outcome": r.outcome,
               "tool": r.tool, "reason": r.reason, "species": r.species, "x": round(r.actual.x, 3),
               "y": round(r.actual.y, 3), "lat": f"{r.latitude:.8f}", "lon": f"{r.longitude:.8f}",
               "error_cm": round(r.error_m * 100, 1), "soil": r.soil, "rock_depth_m": round(r.rock_depth_m, 3),
               "hole_depth_m": round(r.hole_depth_m, 3), "bore_energy_kj": round(r.bore_energy_kj, 2),
               "cycle_time_s": round(r.cycle_time_s, 1), "water_l": round(r.water_l, 2),
               "region": r.region, "microsite": r.microsite, "method": r.method, "deviations": r.deviations,
               "survival_1y": round(r.survival_1y, 3), "survival_risk": r.survival_risk}
        self.records.append(row)
        self.csv.writerow(row)
        self.csv_f.flush()
        if r.outcome in ("planted", "seeded") and r.tool in ("auger", "pick"):
            for cl, z in self.hidden:
                s, lat, _ = cl.project(r.actual.x, r.actual.y)
                if z["s0"] <= s <= z["s1"] and z["lat0"] <= lat <= z["lat1"]:
                    self.strikes.append(row)
                    self.get_logger().error(f"UTILITY STRIKE: {r.robot} bored into the {z['name']} at "
                                            f"({r.actual.x:.1f}, {r.actual.y:.1f}); a survey would have prevented it")

    def summary(self) -> dict:
        rec = self.records
        ok = [r for r in rec if r["outcome"] in ("planted", "seeded")]
        planted = [r for r in rec if r["outcome"] == "planted"]
        err = np.array([r["error_cm"] for r in ok]) if ok else np.array([0.0])
        span = (sim_now(self) - self.t0) if self.t0 is not None else 0.0
        by_robot = {}
        for r in rec:
            b = by_robot.setdefault(r["robot"], {"planted": 0, "seeded": 0, "skipped": 0})
            b[r["outcome"]] = b.get(r["outcome"], 0) + 1
        return {
            "site": self.site.name, "records": len(rec), "planted": len(planted),
            "seeded": sum(r["outcome"] == "seeded" for r in rec),
            "skipped": sum(r["outcome"] == "skipped" for r in rec),
            "rate_per_h": round(len(ok) / span * 3600, 1) if span > 60 else 0.0,
            "error_cm_mean": round(float(err.mean()), 1), "error_cm_p95": round(float(np.percentile(err, 95)), 1),
            "energy_kj_per_plant": round(float(np.mean([r["bore_energy_kj"] for r in ok])) if ok else 0.0, 1),
            "cycle_s_mean": round(float(np.mean([r["cycle_time_s"] for r in ok])) if ok else 0.0, 1),
            "water_l": round(sum(r["water_l"] for r in rec), 1),
            # expected trees alive after a year (fs_autonomy.survival), and the main risks
            "survival_mean": round(float(np.mean([r["survival_1y"] for r in planted])) if planted else 0.0, 3),
            "trees_expected_1y": round(sum(r["survival_1y"] for r in planted), 1),
            "survival_risks": _count(r["survival_risk"] for r in planted if r["survival_risk"]),
            "utility_strikes": len(self.strikes), "by_robot": by_robot,
            "skip_reasons": _count(r["reason"] for r in rec if r["outcome"] == "skipped"),
            "tools": _count(r["tool"] for r in ok), "sim_minutes": round(span / 60, 1),
        }

    def publish_summary(self):
        s = self.summary()
        self.pub.publish(String(data=json.dumps(s)))
        (self.out / "summary.json").write_text(json.dumps(s, indent=2))
        feats = [{"type": "Feature", "geometry": {"type": "Point", "coordinates": [float(r["lon"]), float(r["lat"])]},
                  "properties": {k: r[k] for k in FIELDS if k not in ("lat", "lon")}} for r in self.records]
        (self.out / "plants.geojson").write_text(json.dumps({"type": "FeatureCollection", "features": feats}))

    def log_summary(self):
        s = self.summary()
        if s["records"]:
            self.get_logger().info(
                f"{s['planted']} planted, {s['seeded']} seeded, {s['skipped']} skipped | {s['rate_per_h']}/h | "
                f"error {s['error_cm_mean']} cm (p95 {s['error_cm_p95']}) | {s['energy_kj_per_plant']} kJ/plant | "
                f"cycle {s['cycle_s_mean']} s | strikes {s['utility_strikes']}")


def _count(it):
    d = {}
    for k in it:
        d[k or "-"] = d.get(k or "-", 0) + 1
    return d


def main():
    rclpy.init()
    n = MissionLogger()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.publish_summary()
        n.destroy_node()
        rclpy.try_shutdown()
