"""Planar EKF fusing wheel odometry, IMU and GNSS.

State: [x, y, yaw, vx, vy, w], position and heading in the map frame (ENU), velocities in the
body frame. vy lets the filter follow Jaal's sideways (crab) moves; for Ackermann and skid-steer
machines the odometry reports vy = 0, which is their non-holonomic constraint.

Wheel odometry and the gyro are measurements of (vx, vy, w), so wheel slip shows up as a
disagreement GNSS corrects. GNSS measures the antenna, offset from base_link by a lever arm.
Outliers are gated with a chi-square test; if the gate keeps rejecting (the filter, not the
receiver, has drifted) it re-centres on the fix instead of diverging.
"""
from __future__ import annotations

import math

import numpy as np

from .geometry import wrap

N = 6


class PoseEKF:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, lever=(0.0, 0.0), q_acc=0.5, q_alpha=0.5, max_rejects=5):
        self.x = np.array([x, y, yaw, 0.0, 0.0, 0.0])
        self.P = np.diag([1.0, 1.0, 0.2, 0.1, 0.1, 0.1])
        self.lever = np.asarray(lever, float)
        self.q_acc, self.q_alpha = q_acc, q_alpha
        self.rejected = 0
        self.consecutive_rejects = 0
        self.max_rejects = max_rejects
        self.resets = 0
        self.last_nis = 0.0

    def predict(self, dt: float):
        if dt <= 0:
            return
        px, py, th, vx, vy, w = self.x
        c, s = math.cos(th), math.sin(th)
        self.x = np.array([px + (c * vx - s * vy) * dt, py + (s * vx + c * vy) * dt, wrap(th + w * dt), vx, vy, w])
        F = np.eye(N)
        F[0, 2], F[0, 3], F[0, 4] = (-s * vx - c * vy) * dt, c * dt, -s * dt
        F[1, 2], F[1, 3], F[1, 4] = (c * vx - s * vy) * dt, s * dt, c * dt
        F[2, 5] = dt
        a = (self.q_acc * dt) ** 2
        Q = np.diag([1e-4 * dt, 1e-4 * dt, 1e-5 * dt, a, a, (self.q_alpha * dt) ** 2])
        self.P = F @ self.P @ F.T + Q

    def _update(self, z, h, H, R, angle_rows=(), gate=None):
        y = np.asarray(z, float) - h
        for r in angle_rows:
            y[r] = wrap(y[r])
        S = H @ self.P @ H.T + R
        nis = float(y @ np.linalg.solve(S, y))
        self.last_nis = nis
        if gate is not None and nis > gate:
            self.rejected += 1
            return False
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.x[2] = wrap(self.x[2])
        self.P = (np.eye(N) - K @ H) @ self.P
        return True

    def update_odom(self, vx: float, w: float, vy: float = 0.0, sigma_v: float = 0.05, sigma_w: float = 0.03):
        H = np.zeros((3, N))
        H[0, 3] = H[1, 4] = H[2, 5] = 1
        return self._update([vx, vy, w], self.x[3:6], H, np.diag([sigma_v ** 2, sigma_v ** 2, sigma_w ** 2]))

    def update_gyro(self, w: float, sigma: float = 0.01):
        H = np.zeros((1, N))
        H[0, 5] = 1
        return self._update([w], self.x[5:6], H, np.array([[sigma ** 2]]))

    def update_heading(self, yaw: float, sigma: float = math.radians(2.0)):
        H = np.zeros((1, N))
        H[0, 2] = 1
        return self._update([yaw], self.x[2:3], H, np.array([[sigma ** 2]]), angle_rows=(0,))

    def antenna(self):
        th = self.x[2]
        c, s = math.cos(th), math.sin(th)
        lx, ly = self.lever
        return np.array([self.x[0] + c * lx - s * ly, self.x[1] + s * lx + c * ly])

    def update_gnss(self, ex: float, ey: float, sigma: float, gate: float = 13.8):
        th = self.x[2]
        c, s = math.cos(th), math.sin(th)
        lx, ly = self.lever
        H = np.zeros((2, N))
        H[0, 0] = H[1, 1] = 1
        H[0, 2] = -s * lx - c * ly
        H[1, 2] = c * lx - s * ly
        ok = self._update([ex, ey], self.antenna(), H, np.eye(2) * sigma ** 2, gate=gate)
        if ok:
            self.consecutive_rejects = 0
            return True
        self.consecutive_rejects += 1
        if self.consecutive_rejects >= self.max_rejects:
            ant = self.antenna()
            self.x[0] += ex - ant[0]
            self.x[1] += ey - ant[1]
            self.P[0, 0] = self.P[1, 1] = max(sigma ** 2, 0.05)
            self.consecutive_rejects = 0
            self.resets += 1
        return False

    @property
    def pose(self):
        return float(self.x[0]), float(self.x[1]), float(self.x[2])

    @property
    def sigma_xy(self) -> float:
        return float(math.sqrt(max(self.P[0, 0], 0) + max(self.P[1, 1], 0)))
