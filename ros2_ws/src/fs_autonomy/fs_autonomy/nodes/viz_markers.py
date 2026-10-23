"""RViz markers for the site survey, missions, planting results and stability.

/viz/site     (latched) planting line, no-dig zones, trees, field, obstacles, tender, work area
/viz/mission  planned spots per robot, coloured by tool or planned skip
/viz/plants   one marker per planting record: sapling, seed balls or skipped
/viz/stability support polygon and CG of each robot with a StabilityStatus
"""
import rclpy
from fs_interfaces.msg import Mission, PlantRecord, StabilityStatus
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from ..geometry import Polyline
from ..ros_common import LATCHED, load_site, param, pt

COL = {"auger": (0.89, 0.65, 0.08), "pick": (0.85, 0.45, 0.15), "seedball": (0.55, 0.36, 0.2),
       "skip": (0.75, 0.2, 0.15), "auto": (0.89, 0.65, 0.08), "planted": (0.2, 0.62, 0.25),
       "seeded": (0.55, 0.36, 0.2), "skipped": (0.85, 0.25, 0.15)}


def rgba(c, a=1.0):
    return ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=float(a))


class VizMarkers(Node):
    def __init__(self):
        super().__init__("viz_markers")
        self.site = load_site(self)
        robots = param(self, "robots", ["katar", "jaal_1", "jaal_2", "jaal_3", "jaal_4", "jaal_5", "jaal_6", "pahadi"])
        self.site_pub = self.create_publisher(MarkerArray, "/viz/site", LATCHED)
        self.mission_pub = self.create_publisher(MarkerArray, "/viz/mission", LATCHED)
        self.plant_pub = self.create_publisher(MarkerArray, "/viz/plants", 10)
        self.stab_pub = self.create_publisher(MarkerArray, "/viz/stability", 10)
        self.plants = MarkerArray()
        for r in robots:
            self.create_subscription(Mission, f"/{r}/mission", self.on_mission, LATCHED)
            self.create_subscription(StabilityStatus, f"/{r}/stability", lambda m, r=r: self.on_stab(r, m), 10)
        self.create_subscription(PlantRecord, "/plants/records", self.on_record, 100)
        self.create_timer(2.0, lambda: self.plant_pub.publish(self.plants))
        self.publish_site()

    def marker(self, ns, mid, mtype, color, scale=(0.1, 0.1, 0.1)):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)
        m.color = color
        m.pose.orientation.w = 1.0
        return m

    def z(self, x, y):
        return float(self.site.soil_map.elevation(x, y)) + 0.05

    def publish_site(self):
        arr = MarkerArray()
        k = 0
        rs = self.site.get("roadside")
        if rs:
            cl = Polyline(rs["shoulder_centerline"])
            line = cl.offset(rs["planting_offset"])
            m = self.marker("planting_line", k, Marker.LINE_STRIP, rgba((0.89, 0.65, 0.08), 0.8), (0.06, 0, 0))
            m.points = [pt(x, y, 0.05) for x, y in line.p]
            arr.markers.append(m)
            for z in rs["no_dig"]:
                k += 1
                c = (0.9, 0.2, 0.15) if z.get("visible", True) else (0.6, 0.2, 0.7)
                m = self.marker("no_dig", k, Marker.LINE_STRIP, rgba(c, 0.9), (0.08, 0, 0))
                edge0 = cl.slice(z["s0"], z["s1"]).offset(z["lat0"])
                edge1 = cl.slice(z["s0"], z["s1"]).offset(z["lat1"])
                ring = list(edge0.p) + list(edge1.p[::-1]) + [edge0.p[0]]
                m.points = [pt(x, y, 0.06) for x, y in ring]
                arr.markers.append(m)
        f = self.site.get("field")
        if f:
            k += 1
            m = self.marker("field", k, Marker.LINE_STRIP, rgba((0.89, 0.65, 0.08)), (0.1, 0, 0))
            m.points = [pt(x, y, 0.05) for x, y in f["polygon"] + [f["polygon"][0]]]
            arr.markers.append(m)
            for o in self.site.get("obstacles", []):
                k += 1
                m = self.marker("obstacles", k, Marker.CYLINDER, rgba((0.6, 0.45, 0.3), 0.6),
                                (2 * o["r"], 2 * o["r"], o["h"]))
                m.pose.position = pt(o["x"], o["y"], o["h"] / 2)
                arr.markers.append(m)
            for zz in self.site.get("rtk_degraded_zones", []):
                k += 1
                m = self.marker("rtk_float", k, Marker.LINE_STRIP, rgba((0.6, 0.2, 0.7), 0.8), (0.1, 0, 0))
                m.points = [pt(x, y, 0.05) for x, y in zz["polygon"] + [zz["polygon"][0]]]
                arr.markers.append(m)
        h = self.site.get("hill")
        if h:
            x0, y0, x1, y1 = h["work_area"]
            k += 1
            m = self.marker("work_area", k, Marker.LINE_STRIP, rgba((0.89, 0.65, 0.08)), (0.1, 0, 0))
            m.points = [pt(x, y, self.z(x, y)) for x, y in [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]]
            arr.markers.append(m)
        self.site_pub.publish(arr)

    def on_mission(self, msg: Mission):
        arr = MarkerArray()
        for tool in ("auger", "pick", "seedball", "skip"):
            m = self.marker(f"mission/{msg.robot}/{tool}", 0, Marker.CUBE_LIST, rgba(COL[tool], 0.85),
                            (0.3, 0.3, 0.03))
            m.points = [pt(s.position.x, s.position.y, self.z(s.position.x, s.position.y))
                        for s in msg.spots if (s.tool if s.tool in COL else "auger") == tool]
            arr.markers.append(m)
        self.mission_pub.publish(arr)

    def on_record(self, r: PlantRecord):
        x, y = r.actual.x, r.actual.y
        mid = hash((r.robot, r.id)) & 0x7FFFFFFF
        if r.outcome == "planted":
            m = self.marker("plants/sapling", mid, Marker.CYLINDER, rgba(COL["planted"]), (0.25, 0.25, 0.6))
            m.pose.position = pt(x, y, self.z(x, y) + 0.3)
        elif r.outcome == "seeded":
            m = self.marker("plants/seed", mid, Marker.SPHERE, rgba(COL["seeded"]), (0.3, 0.3, 0.15))
            m.pose.position = pt(x, y, self.z(x, y))
        else:
            m = self.marker("plants/skipped", mid, Marker.CUBE, rgba(COL["skipped"]), (0.25, 0.25, 0.25))
            m.pose.position = pt(r.intended.x, r.intended.y, self.z(r.intended.x, r.intended.y) + 0.12)
        self.plants.markers.append(m)

    def on_stab(self, robot, s: StabilityStatus):
        arr = MarkerArray()
        lvl = {0: (0.2, 0.7, 0.3), 1: (0.95, 0.65, 0.1), 2: (0.9, 0.15, 0.1)}[min(s.level, 2)]
        m = self.marker(f"stability/{robot}/polygon", 0, Marker.LINE_STRIP, rgba(lvl), (0.05, 0, 0))
        m.points = list(s.support_polygon) + list(s.support_polygon[:1])
        arr.markers.append(m)
        c = self.marker(f"stability/{robot}/cg", 1, Marker.SPHERE, rgba(lvl), (0.15, 0.15, 0.15))
        c.pose.position = s.cg
        arr.markers.append(c)
        drop = self.marker(f"stability/{robot}/plumb", 2, Marker.LINE_LIST, rgba(lvl, 0.7), (0.02, 0, 0))
        zmin = min((p.z for p in s.support_polygon), default=s.cg.z - 0.8)
        drop.points = [s.cg, Point(x=s.cg.x, y=s.cg.y, z=zmin)]
        arr.markers.append(drop)
        self.stab_pub.publish(arr)


def main():
    rclpy.init()
    n = VizMarkers()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
