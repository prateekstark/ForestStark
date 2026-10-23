"""Inverse kinematics for Pahadi's 4-axis arm (yaw, shoulder, elbow, wrist pitch).

Joint convention (URDF): pitch joints rotate about +y, so a positive angle tips the link's +x
downward. The tool hangs from the wrist along the wrist link's +x; for planting it must point
straight down, i.e. shoulder + elbow + wrist = pi/2 (in the arm's vertical plane).
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class ArmGeometry:
    base_x: float = 0.25        # arm base in base_link, m
    base_y: float = 0.0
    shoulder_z: float = 0.85    # shoulder axis height above base_link
    l1: float = 0.85            # upper arm
    l2: float = 0.85            # forearm
    tool: float = 0.50          # wrist axis to bit tip with the feed retracted
    grip_x: float = 0.07        # grasp point (root-ball centre) in arm_wrist_link, gripper mode
    grip_z: float = -0.18
    stow: tuple = (0.0, -1.35, 2.6, 0.3)


class Unreachable(ValueError):
    pass


def solve(target, geo: ArmGeometry = ArmGeometry(), clearance: float = 0.08):
    """Joint angles (yaw, shoulder, elbow, wrist) placing the bit `clearance` m above `target`
    (x, y, z in base_link), pointing straight down. Raises Unreachable."""
    tx, ty, tz = target
    yaw = math.atan2(ty - geo.base_y, tx - geo.base_x)
    r = math.hypot(tx - geo.base_x, ty - geo.base_y)
    wz = tz + clearance + geo.tool - geo.shoulder_z      # wrist height relative to the shoulder
    d2 = r * r + wz * wz
    d = math.sqrt(d2)
    if d > geo.l1 + geo.l2 - 1e-3 or d < abs(geo.l1 - geo.l2) + 1e-3:
        raise Unreachable(f"target {r:.2f} m out, {-wz:.2f} m down is outside the arm's reach")
    c2 = (d2 - geo.l1 ** 2 - geo.l2 ** 2) / (2 * geo.l1 * geo.l2)
    q2 = math.acos(max(-1.0, min(1.0, c2)))               # elbow-up: forearm folds downward
    phi = math.atan2(-wz, r)                               # line to the wrist, below horizontal
    q1 = phi - math.atan2(geo.l2 * math.sin(q2), geo.l1 + geo.l2 * math.cos(q2))
    q3 = math.pi / 2 - q1 - q2
    return yaw, q1, q2, q3


def _two_link(r, wz, geo):
    d2 = r * r + wz * wz
    d = math.sqrt(d2)
    if d > geo.l1 + geo.l2 - 1e-3 or d < abs(geo.l1 - geo.l2) + 1e-3:
        raise Unreachable(f"wrist target {r:.2f} m out, {-wz:.2f} m down is outside the arm's reach")
    q2 = math.acos(max(-1.0, min(1.0, (d2 - geo.l1 ** 2 - geo.l2 ** 2) / (2 * geo.l1 * geo.l2))))
    q1 = math.atan2(-wz, r) - math.atan2(geo.l2 * math.sin(q2), geo.l1 + geo.l2 * math.cos(q2))
    return q1, q2


def solve_gripper(target, geo: ArmGeometry = ArmGeometry()):
    """Joint angles putting the gripper's grasp point at `target` (base_link) with the wrist level,
    so the gripper hangs straight down: shoulder + elbow + wrist = 0."""
    tx, ty, tz = target
    yaw = math.atan2(ty - geo.base_y, tx - geo.base_x)
    r = math.hypot(tx - geo.base_x, ty - geo.base_y) - geo.grip_x
    q1, q2 = _two_link(r, tz - geo.grip_z - geo.shoulder_z, geo)
    return yaw, q1, q2, -(q1 + q2)


def forward_gripper(q, geo: ArmGeometry = ArmGeometry()):
    yaw, q1, q2, q3 = q
    a = q1 + q2 + q3
    r = geo.l1 * math.cos(q1) + geo.l2 * math.cos(q1 + q2) + geo.grip_x * math.cos(a) + geo.grip_z * math.sin(a)
    z = geo.shoulder_z - geo.l1 * math.sin(q1) - geo.l2 * math.sin(q1 + q2) - geo.grip_x * math.sin(a) + geo.grip_z * math.cos(a)
    return geo.base_x + r * math.cos(yaw), geo.base_y + r * math.sin(yaw), z


def forward(q, geo: ArmGeometry = ArmGeometry()):
    """Bit tip position (feed retracted) in base_link for joints q."""
    yaw, q1, q2, q3 = q
    r = geo.l1 * math.cos(q1) + geo.l2 * math.cos(q1 + q2) + geo.tool * math.cos(q1 + q2 + q3)
    z = geo.shoulder_z - geo.l1 * math.sin(q1) - geo.l2 * math.sin(q1 + q2) - geo.tool * math.sin(q1 + q2 + q3)
    return geo.base_x + r * math.cos(yaw), geo.base_y + r * math.sin(yaw), z
