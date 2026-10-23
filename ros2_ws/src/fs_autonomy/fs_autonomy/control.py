"""Path tracking and speed control shared by the navigators."""
from __future__ import annotations

import math

from .geometry import Polyline, wrap


class PurePursuit:
    """Pure pursuit on a Polyline. Tracks a reference point on the vehicle (rear axle for
    Ackermann, centre for skid/swerve). Returns curvature; positive turns left."""

    def __init__(self, path: Polyline, lookahead: float = 2.5, min_lookahead: float = 1.0, k_speed: float = 1.0):
        self.path = path
        self.ld = lookahead
        self.ld_min = min_lookahead
        self.k = k_speed
        self.hint = None
        self.s = 0.0
        self.lateral = 0.0
        self.goal = (0.0, 0.0)

    def step(self, x: float, y: float, yaw: float, v: float = 0.0) -> float:
        self.s, self.lateral, self.hint = self.path.project(x, y, self.hint)
        ld = max(self.ld_min, self.ld + self.k * abs(v))
        gx, gy, _ = self.path.at(self.s + ld)
        self.goal = (gx, gy)
        dx, dy = gx - x, gy - y
        alpha = wrap(math.atan2(dy, dx) - yaw)
        dist = max(math.hypot(dx, dy), 1e-3)
        return 2.0 * math.sin(alpha) / dist

    def heading_error(self, yaw: float) -> float:
        _, _, h = self.path.at(self.s)
        return wrap(h - yaw)

    def remaining(self) -> float:
        return self.path.length - self.s


def approach_speed(distance: float, v_max: float, decel: float, v_min: float = 0.05) -> float:
    """Highest speed from which the vehicle can still stop within `distance`."""
    if distance <= 0:
        return 0.0
    return max(min(v_max, math.sqrt(2 * decel * distance)), min(v_min, v_max))


class RateLimiter:
    def __init__(self, rate: float, value: float = 0.0):
        self.rate = rate
        self.value = value

    def __call__(self, target: float, dt: float) -> float:
        step = self.rate * dt
        self.value += max(-step, min(step, target - self.value))
        return self.value


class OrnsteinUhlenbeck:
    """Mean-reverting noise, used for the simulated human driver's weave."""

    def __init__(self, sigma: float, tau: float, rng):
        self.sigma, self.tau, self.rng, self.x = sigma, tau, rng, 0.0

    def __call__(self, dt: float) -> float:
        a = math.exp(-dt / self.tau)
        self.x = a * self.x + self.sigma * math.sqrt(1 - a * a) * self.rng.normal()
        return self.x
