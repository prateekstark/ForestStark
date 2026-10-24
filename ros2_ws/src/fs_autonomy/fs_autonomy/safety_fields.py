"""Safety-scanner fields and the safety PLC's logic, as the certified layer will implement them.

The software safety monitor (fs_autonomy.safety) is the comfort layer: it slows early and stops
for people well before anything certified has to act. Under it sits the certified layer (see
docs/safety_concept.md): outdoor safety laser scanners (IEC 61496-3 type 3), a safety PLC and the
drives' safe torque off (STO) / safe stop 1 (SS1), PL d to ISO 13849-1. This module sizes that
layer's fields and models its logic, so that

  * the field sets handed to the scanner configuration tool are computed, not drawn by hand
    (`ros2 run fs_bringup safety_fields` writes them to docs/safety_fields.md and .yaml), and
  * the simulator can run the same logic as a backstop (PlcModel), showing that the comfort layer
    always stops first in normal operation and that the certified layer catches what it misses.

Protective field length ahead of the footprint, per speed band (ISO 13855 approach, as applied to
mobile machines in ISO 3691-4):

    S = v_max T + 1.1 v_max^2 / (2 a) + Z

with T = scanner response + PLC cycle + drive STO/brake reaction, a the SS1 deceleration the
drives are configured for (+10 % for brake wear), and Z = scanner measurement tolerance plus the
supplement for leg detection at 70 mm resolution.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .safety import FOOTPRINTS, Footprint


@dataclass(frozen=True)
class StopChain:
    scanner_s: float = 0.10        # outdoor safety scanner response (2 scans + filtering)
    plc_s: float = 0.02            # safety PLC cycle
    drive_s: float = 0.15          # STO / SS1 reaction and brake engagement
    decel: float = 0.8             # m/s^2, SS1 ramp the drives are commissioned for
    z: float = 0.20                # m: 0.1 measurement tolerance + 0.1 leg-detection supplement
    side: float = 0.20             # m beside the footprint

    @property
    def T(self) -> float:
        return self.scanner_s + self.plc_s + self.drive_s


CHAINS = {
    "jaal": StopChain(decel=0.8),
    "pahadi": StopChain(decel=0.6, drive_s=0.25),          # hydraulic track drives brake later
    "katar": StopChain(decel=2.5, drive_s=0.3),            # the tool only: boom feed and auger
}
BANDS = {"jaal": (0.3, 0.8, 1.5), "pahadi": (0.3, 0.7), "katar": ()}     # m/s, upper edge of each band
DIRECTIONS = {"jaal": ("+x", "-x", "+y", "-y"), "pahadi": ("+x",), "katar": ()}  # what its scanners can guard
TOOL_R = 2.0                       # tool zone radius around the tool: spin and feed STO
WARN_S = 1.5                       # warning field: this many seconds of travel beyond the protective field


def stopping_distance(v: float, chain: StopChain) -> float:
    return v * chain.T + 1.1 * v * v / (2 * chain.decel)


def protective_length(v: float, chain: StopChain) -> float:
    return stopping_distance(v, chain) + chain.z


def rect_field(fp: Footprint, direction: str, ahead: float, side: float) -> np.ndarray:
    """Field polygon (base_link, counter-clockwise) covering the footprint's sides and `ahead` m
    beyond it in the direction of travel."""
    x0, x1, y0, y1 = fp.xmin - side, fp.xmax + side, fp.ymin - side, fp.ymax + side
    if direction == "+x":
        x1 = fp.xmax + ahead
    elif direction == "-x":
        x0 = fp.xmin - ahead
    elif direction == "+y":
        y1 = fp.ymax + ahead
    elif direction == "-y":
        y0 = fp.ymin - ahead
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])


def circle_field(cx: float, cy: float, r: float, n: int = 16) -> np.ndarray:
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    return np.c_[cx + r * np.cos(a), cy + r * np.sin(a)]


def inside(poly: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Points inside a convex polygon (counter-clockwise)."""
    pts = np.asarray(pts, float).reshape(-1, 2)
    ok = np.ones(len(pts), bool)
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        ok &= (b[0] - a[0]) * (pts[:, 1] - a[1]) - (b[1] - a[1]) * (pts[:, 0] - a[0]) >= 0
    return ok


@dataclass
class FieldSet:
    name: str
    band: float
    direction: str
    protective: np.ndarray
    warning: np.ndarray


def field_sets(kind: str) -> list:
    """Every (speed band x direction) field set the robot's scanners switch between."""
    fp, ch = FOOTPRINTS[kind], CHAINS[kind]
    out = []
    for v in BANDS[kind]:
        for d in DIRECTIONS[kind]:
            p = protective_length(v, ch)
            out.append(FieldSet(f"{d}_{v:.1f}", v, d, rect_field(fp, d, p, ch.side),
                                rect_field(fp, d, p + WARN_S * v + 0.5, ch.side + 0.3)))
    return out


def standstill(kind: str) -> np.ndarray:
    """The field held while stopped or crawling below the first band: the footprint plus the stop chain's margin."""
    fp, ch = FOOTPRINTS[kind], CHAINS[kind]
    return rect_field(fp, "", 0.0, ch.side + ch.z)


@dataclass
class PlcState:
    estop: bool = False
    protective_stop: bool = False
    warning: bool = False
    tool_zone: bool = False
    tilt: bool = False
    sto_drive: bool = False
    sto_tool: bool = False
    field_set: int = 0
    field_name: str = "standstill"


@dataclass
class PlcModel:
    """The safety PLC's logic. Inputs are what its own safe inputs would see: scanner points (base_link),
    the encoders' speed, the tool's position, the tool drives' state, E-stop circuits, inclinometer."""
    kind: str
    reset_s: float = 1.0           # automatic restart once the protective field has been clear this long
    tilt_limit_deg: float = 30.0
    sets: list = field(default_factory=list)
    state: PlcState = field(default_factory=PlcState)
    _clear_since: float | None = None

    def __post_init__(self):
        self.sets = field_sets(self.kind)
        self.still = standstill(self.kind)

    def select(self, vx: float, vy: float) -> tuple:
        """Active field set from the measured velocity: the lowest band above the speed, in the travel direction."""
        speed = math.hypot(vx, vy)
        if not self.sets or speed < 0.05:
            return 0, "standstill", self.still, None
        d = ("+x" if vx >= 0 else "-x") if abs(vx) >= abs(vy) else ("+y" if vy >= 0 else "-y")
        cands = [s for s in self.sets if s.direction == d] or [s for s in self.sets if s.direction == "+x"]
        for k, s in enumerate(self.sets):
            if s in cands and speed <= s.band + 1e-6:
                return k + 1, s.name, s.protective, s.warning
        s = max(cands, key=lambda s: s.band)           # above the top band: the drives' SLS should never let this happen
        return self.sets.index(s) + 1, s.name, s.protective, s.warning

    def update(self, t: float, points, vx: float, vy: float, tool_xy=None, tool_active=False,
               estop=False, tilt_deg=0.0) -> PlcState:
        pts = np.asarray(points, float).reshape(-1, 2)
        k, name, prot, warn = self.select(vx, vy)
        s = self.state
        s.field_set, s.field_name = k, name
        s.estop = bool(estop)
        s.tilt = abs(tilt_deg) > self.tilt_limit_deg
        hit = bool(len(pts) and inside(prot, pts).any()) if self.sets else False
        s.warning = bool(len(pts) and warn is not None and inside(warn, pts).any())
        if hit or s.estop or s.tilt:
            s.protective_stop, self._clear_since = True, None
        elif s.protective_stop:
            self._clear_since = t if self._clear_since is None else self._clear_since
            if t - self._clear_since >= self.reset_s:
                s.protective_stop = False
        s.tool_zone = bool(tool_active and tool_xy is not None and len(pts)
                           and (np.hypot(pts[:, 0] - tool_xy[0], pts[:, 1] - tool_xy[1]) < TOOL_R).any())
        s.sto_drive = s.protective_stop or s.estop
        s.sto_tool = s.tool_zone or s.estop or s.tilt
        return s
