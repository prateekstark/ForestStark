"""The auger on the machine: the same commands and status as the simulator's fs::AugerSoil, from the
real feed and spin drives.

Phases: idle, lowering (free speed until the feed force says the bit has met the ground), boring
(feed rate cut back to keep the spin torque under its limit, the machine's version of the power
limit), bored (target depth or end of stroke), refusal (feed stalled at full thrust: rock or a
stone), no_ground (end of stroke without contact), retracting, stopped, held (safety: spin and feed
stop where they are; resume carries on).

Depth is the feed travel since contact. A machine cannot see its hole's walls cave in, so "collapse"
is 0 and "open_depth" the bored depth; the planting head's dry-sand pre-wetting still applies by
the soil map's moisture. "wet <litres>" is accepted and recorded.
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class AugerParams:
    max_stroke: float = 1.3          # m of feed travel
    free_speed: float = 0.25         # m/s lowering
    max_feed: float = 0.15           # m/s boring
    retract_speed: float = 0.3
    rpm: float = 120.0
    contact_force: float = 600.0     # N on the feed above the no-load force: the bit is on the ground
    max_thrust: float = 8000.0       # N the feed can push
    spin_torque_limit: float = 800.0  # N m: feed backs off above this
    stall_speed: float = 0.004       # m/s: below this at full thrust for stall_time -> refusal
    stall_time: float = 3.0


@dataclass
class Auger:
    p: AugerParams = field(default_factory=AugerParams)
    phase: str = "idle"
    held_from: str = "idle"
    target: float = 0.3
    pick: bool = False
    contact_q: float = 0.0
    depth: float = 0.0
    energy: float = 0.0
    phase_t: float = 0.0
    stall_t: float = 0.0
    feed_cmd: float = 0.0
    spin_cmd: float = 0.0
    thrust_limited: bool = False
    hole: tuple = (0.0, 0.0)
    wet_l: float = 0.0
    t: float = 0.0

    def command(self, text: str):
        parts = text.split()
        if not parts:
            return
        verb = parts[0]
        arg = float(parts[1]) if len(parts) > 1 else 0.3
        if verb in ("bore", "pick"):
            self.pick, self.target = verb == "pick", min(max(arg, 0.02), self.p.max_stroke)
            self.depth = self.energy = 0.0
            self.wet_l = 0.0
            self._set("lowering")
        elif verb == "retract":
            self._set("retracting")
        elif verb == "stop":
            self._set("stopped")
        elif verb == "hold" and self.phase != "held":
            self.held_from = self.phase
            self._set("held")
        elif verb == "resume" and self.phase == "held":
            self._set(self.held_from)
        elif verb == "wet":
            self.wet_l += arg

    def _set(self, phase):
        self.phase, self.phase_t, self.stall_t = phase, 0.0, 0.0

    def step(self, dt, q, qd, feed_force, spin_torque, spin_speed, tip_xy=None):
        """q, qd: feed position and speed; feed_force: N pushing down (minus the no-load force);
        spin_torque N m; spin_speed rad/s. Returns (feed velocity, spin velocity) commands."""
        p = self.p
        self.t += dt
        self.phase_t += dt
        feed, spin = 0.0, 0.0
        self.thrust_limited = False
        ph = self.phase
        if ph == "lowering":
            spin = p.rpm * 2 * 3.141592653589793 / 60
            if feed_force > p.contact_force:
                self.contact_q = q
                if tip_xy is not None:
                    self.hole = tuple(tip_xy)
                self._set("boring")
            elif q >= p.max_stroke - 1e-3:
                self._set("no_ground")
            else:
                feed = p.free_speed
        elif ph == "boring":
            spin = p.rpm * 2 * 3.141592653589793 / 60
            self.depth = max(0.0, q - self.contact_q)
            if self.depth >= self.target or q >= p.max_stroke - 1e-3:
                self._set("bored")
            else:
                feed = p.max_feed * min(1.0, max(0.1, (p.spin_torque_limit - spin_torque) / (0.3 * p.spin_torque_limit)))
                self.thrust_limited = feed_force >= 0.95 * p.max_thrust
                self.stall_t = self.stall_t + dt if (qd < p.stall_speed and self.thrust_limited) else 0.0
                if self.stall_t > p.stall_time:
                    self._set("refusal")
                    feed = 0.0
            self.energy += (abs(spin_torque * spin_speed) + abs(feed_force * qd)) * dt
        elif ph == "retracting":
            spin = p.rpm * 2 * 3.141592653589793 / 60
            if q <= 0.003:
                self._set("idle")
            else:
                feed = -min(p.retract_speed, max(0.01, 2.0 * q))     # slow into the top stop
        self.feed_cmd, self.spin_cmd = feed, spin
        return feed, spin

    def status(self, q, feed_force, spin_torque, spin_speed) -> dict:
        return {"t": round(self.t, 3), "phase": self.phase, "phase_time": round(self.phase_t, 3),
                "depth": round(self.depth, 4), "collapse": 0.0, "open_depth": round(self.depth, 4),
                "target": self.target, "stroke": round(q, 4), "feed_cmd": self.feed_cmd, "feed": self.feed_cmd,
                "thrust": round(feed_force, 1), "torque": round(spin_torque, 1),
                "power": round(abs(spin_torque * spin_speed), 1), "energy_kj": round(self.energy / 1000, 3),
                "thrust_limited": self.thrust_limited, "in_rock": self.phase == "refusal", "in_pan": False,
                "pick": self.pick, "wet_l": self.wet_l, "hole": list(self.hole)}
