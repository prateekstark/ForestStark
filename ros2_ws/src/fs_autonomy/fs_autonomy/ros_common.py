"""Small helpers shared by the ForestStark nodes."""
from __future__ import annotations

import json
import math
import time

from geometry_msgs.msg import Point, TransformStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from .site import Site

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST)
SENSOR = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)


def load_site(node: Node) -> Site:
    node.declare_parameter("site_file", "")
    path = node.get_parameter("site_file").value
    if not path:
        raise RuntimeError(f"{node.get_name()}: parameter site_file is required")
    return Site(path)


def param(node: Node, name: str, default):
    """Declare and read a parameter. Dynamic typing, so a launch file may pass 600.0 where the
    default was 600 (or the other way round)."""
    if not node.has_parameter(name):
        node.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
    return node.get_parameter(name).value


def sim_now(node: Node) -> float:
    return node.get_clock().now().nanoseconds * 1e-9


def wait_sim(node: Node, seconds: float, until=None, poll: float = 0.02) -> bool:
    """Block (from a worker thread) for `seconds` of sim time, or until `until()` is true.
    Returns True if the condition was met (or no condition and the time elapsed)."""
    t_end = sim_now(node) + seconds
    while sim_now(node) < t_end:
        if until is not None and until():
            return True
        if not node.context.ok():         # shutting down: sim time may never advance again
            return False
        time.sleep(poll)
    return until is None or bool(until())


def pt(x: float, y: float, z: float = 0.0) -> Point:
    p = Point()
    p.x, p.y, p.z = float(x), float(y), float(z)
    return p


def tf_xyyaw(t: TransformStamped):
    q = t.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return t.transform.translation.x, t.transform.translation.y, yaw


def lookup(buffer, target: str, source: str, timeout: float = 0.0):
    """TF lookup that returns None instead of raising."""
    try:
        return buffer.lookup_transform(target, source, Time(), timeout=Duration(seconds=timeout))
    except Exception:  # noqa: BLE001 - tf2 raises several exception types
        return None


def parse_json(msg_data: str) -> dict:
    try:
        return json.loads(msg_data)
    except (ValueError, TypeError):
        return {}
