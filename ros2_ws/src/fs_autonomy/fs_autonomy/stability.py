"""Tip-over stability: centre of gravity from the URDF masses and the force-angle stability
measure (Papadopoulos & Rey, 1996) over the ground-contact support polygon.

All vectors are in one frame whose z axis is "up" against gravity (e.g. a gravity-aligned frame
built from the IMU). For each support edge the margin is the extra rotation about that edge the
machine could take before the net force line through the CG passes over it. Negative = tipping.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import numpy as np

from .geometry import convex_hull


def parse_urdf_masses(urdf_xml: str) -> dict:
    """{link: (mass_kg, com_xyz in the link frame)} from the robot_description."""
    root = ET.fromstring(urdf_xml)
    out = {}
    for link in root.iter("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        m = inertial.find("mass")
        if m is None:
            continue
        origin = inertial.find("origin")
        xyz = [0.0, 0.0, 0.0]
        if origin is not None and origin.get("xyz"):
            xyz = [float(v) for v in origin.get("xyz").split()]
        out[link.get("name")] = (float(m.get("value")), np.array(xyz))
    return out


def centre_of_gravity(items):
    """items: iterable of (mass, position 3-vector). Returns (total mass, cg)."""
    m = 0.0
    acc = np.zeros(3)
    for mass, pos in items:
        m += mass
        acc += mass * np.asarray(pos, float)
    return m, (acc / m if m > 0 else acc)


def force_angle_margins(contacts, cg, force):
    """contacts: (N, 3) ground contact points; cg: 3-vector; force: net force on the body at the CG
    (gravity plus any inertial or tool reaction terms), N.

    Returns (hull_indices, margins_rad) with one margin per hull edge i -> i+1.
    """
    P = np.asarray(contacts, float)
    c = np.asarray(cg, float)
    f = np.asarray(force, float)
    hull2 = convex_hull(P[:, :2].tolist())
    if len(hull2) < 3:
        return [], []
    idx = []
    for hx, hy in hull2:
        k = int(np.argmin((P[:, 0] - hx) ** 2 + (P[:, 1] - hy) ** 2))
        idx.append(k)
    H = P[idx]
    centroid = H.mean(axis=0)
    margins = []
    for i in range(len(H)):
        p1, p2 = H[i], H[(i + 1) % len(H)]
        a = p2 - p1
        a /= np.linalg.norm(a)
        Pa = np.eye(3) - np.outer(a, a)
        l = Pa @ (c - p1)                 # edge -> CG, perpendicular to the edge
        fp = Pa @ f
        if np.linalg.norm(fp) < 1e-9:
            margins.append(math.pi / 2)
            continue
        up = -fp / np.linalg.norm(fp)     # "up" as the net force sees it
        w = Pa @ (centroid - p1)
        w = w - (w @ up) * up             # inward, perpendicular to the edge and to up
        nw = np.linalg.norm(w)
        if nw < 1e-9:
            margins.append(0.0)
            continue
        w /= nw
        margins.append(math.atan2(l @ w, l @ up))
    return idx, margins


def edge_name(p1, p2, body_center, body_yaw) -> str:
    """Front/rear/left/right label of a support edge from its outward normal in the body frame."""
    mid = (np.asarray(p1[:2]) + np.asarray(p2[:2])) / 2 - np.asarray(body_center[:2])
    c, s = math.cos(-body_yaw), math.sin(-body_yaw)
    bx, by = c * mid[0] - s * mid[1], s * mid[0] + c * mid[1]
    ang = math.degrees(math.atan2(by, bx))
    names = [(0, "front"), (45, "front-left"), (90, "left"), (135, "rear-left"), (180, "rear"),
             (-135, "rear-right"), (-90, "right"), (-45, "front-right"), (-180, "rear")]
    return min(names, key=lambda t: abs(t[0] - ang))[1]


def static_tip_angle(half_track: float, cg_height: float, cg_lateral: float = 0.0) -> float:
    """Simple 2-D check: side slope (rad) at which a machine tips sideways."""
    return math.atan2(half_track - cg_lateral, cg_height)
