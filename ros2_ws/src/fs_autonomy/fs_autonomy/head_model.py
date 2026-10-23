"""Planting-cycle time and energy prediction, from the same auger physics the Gazebo system uses.
Used by planners and speed governors before the robot commits to a spot."""
from __future__ import annotations

import math
from dataclasses import dataclass

HEADS = {
    #           D     P_W    eta   thrust  feed  lower_m  place  close  pump_Lps  free  retract (m/s)
    "katar":  (0.15, 8000, 0.75, 8000, 0.15, 0.75, 1.2, 1.5, 0.5, 0.5, 0.6),
    "jaal":   (0.12, 4000, 0.85, 3000, 0.15, 0.58, 1.0, 1.2, 0.4, 0.25, 0.3),
    "pahadi": (0.12, 2000, 0.80, 2500, 0.12, 0.10, 2.0, 1.5, 0.3, 0.25, 0.3),
}


@dataclass
class CyclePrediction:
    bore_s: float
    total_s: float
    energy_kj: float
    thrust_n: float
    thrust_limited: bool


def moisture_factor(m: float) -> float:
    return min(max(1.6 - 2.4 * m, 0.7), 1.6)


def predict(robot: str, soil: dict, moisture: float, depth: float = 0.3, water_l: float = 1.0,
            arm_s: float = 0.0) -> CyclePrediction:
    D, P, eta, thrust_max, feed_max, lower, place, close, pump, free_speed, retract_speed = HEADS[robot]
    A = math.pi * D * D / 4
    e = soil["eb"] * 1e6 * moisture_factor(moisture)
    epm = e * A
    feed = min(feed_max, eta * P / epm)
    thrust = 0.012 * epm
    limited = thrust > thrust_max
    if limited:
        feed *= thrust_max / thrust
        thrust = thrust_max
    bore = depth / feed
    total = (lower / free_speed + bore + (lower + depth) / retract_speed + place + close + water_l / pump + arm_s)
    return CyclePrediction(bore, total, e * A * depth / eta / 1000, thrust, limited)
