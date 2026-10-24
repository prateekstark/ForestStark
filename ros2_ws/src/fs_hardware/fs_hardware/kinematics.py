"""Drive kinematics on the machine, mirroring the simulator's drive systems.

Jaal: four swerve modules, as fs::SwerveDrive (module flipped instead of steered past 90 deg or
past the steering stop; wheel speed cut while the module is still turning). Pahadi: one velocity
per track side, as gz DiffDrive. Katar: the tractor is driven by its driver; odometry only, from
the rear wheels and the IMU's yaw rate.
"""
from __future__ import annotations

import math

JAAL_MODULES = {"fl": (0.95, 0.83), "fr": (0.95, -0.83), "rl": (-0.95, 0.83), "rr": (-0.95, -0.83)}
JAAL_R = 0.38
PAHADI_TRACK, PAHADI_R = 1.0, 0.21
KATAR_R, KATAR_TRACK = 0.72, 1.6


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def swerve_inverse(vx, vy, wz, steer_now: dict, steer_limit=1.9, modules=JAAL_MODULES, r=JAAL_R):
    """Steer angle (rad) and wheel speed (rad/s) per module for a body twist."""
    out = {}
    for m, (x, y) in modules.items():
        ux, uy = vx - wz * y, vy + wz * x
        speed = math.hypot(ux, uy)
        th = steer_now.get(m, 0.0)
        ang = math.atan2(uy, ux) if speed > 1e-3 else th
        if abs(wrap(ang - th)) > math.pi / 2:
            ang, speed = wrap(ang + math.pi), -speed
        if abs(ang) > steer_limit:
            ang, speed = wrap(ang + math.pi), -speed
        align = max(0.0, math.cos(wrap(ang - th)))
        out[m] = (ang, speed * align * align / r)
    return out


def swerve_forward(steer: dict, wheel: dict, modules=JAAL_MODULES, r=JAAL_R):
    """Body twist (vx, vy, wz) from measured module angles and wheel speeds (least squares)."""
    num = den = 0.0
    for m, (x, y) in modules.items():
        mvx, mvy = wheel[m] * r * math.cos(steer[m]), wheel[m] * r * math.sin(steer[m])
        num += x * mvy - y * mvx
        den += x * x + y * y
    wz = num / den if den else 0.0
    vx = sum(wheel[m] * r * math.cos(steer[m]) + wz * y for m, (x, y) in modules.items()) / len(modules)
    vy = sum(wheel[m] * r * math.sin(steer[m]) - wz * x for m, (x, y) in modules.items()) / len(modules)
    return vx, vy, wz


def diff_inverse(vx, wz, track=PAHADI_TRACK, r=PAHADI_R):
    """Left and right track speeds (rad/s of the drive wheel)."""
    return (vx - wz * track / 2) / r, (vx + wz * track / 2) / r


def diff_forward(wl, wr, track=PAHADI_TRACK, r=PAHADI_R):
    return r * (wl + wr) / 2, r * (wr - wl) / track


class Odometry:
    """Planar dead reckoning from body twists."""

    def __init__(self):
        self.x = self.y = self.yaw = 0.0

    def step(self, vx, vy, wz, dt):
        self.yaw = wrap(self.yaw + wz * dt)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (vx * c - vy * s) * dt
        self.y += (vx * s + vy * c) * dt
        return self.x, self.y, self.yaw
