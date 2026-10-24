"""The plugs on the machine: which carrier cells are full and what each holder carries.

Same carriers, cells, commands and status as the simulator's fs::PlugInventory, read from the same
<plugin> block in the robot's URDF, so the planting head cannot tell the two apart. On the machine
the planting head's mechanism moves do the work; this keeps the books. Where a holder has a plug
sensor (plugs/sensor/<holder>, std_msgs/Bool), a pick that leaves the holder empty is flagged and
the cell is counted empty, so the feeder moves on rather than planting nothing.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET


class Inventory:
    def __init__(self, urdf_xml: str):
        root = ET.fromstring(urdf_xml)
        plug = next((p for p in root.iter("plugin") if p.get("name") == "fs::PlugInventory"), None)
        if plug is None:
            raise ValueError("no fs::PlugInventory block in the robot description")
        self.carriers, self.holders = {}, {}
        for c in plug.findall("carrier"):
            name = c.findtext("name")
            cells, angles = [], []
            ring, grid = c.find("ring"), c.find("grid")
            if ring is not None:
                rad, n = float(ring.findtext("radius", "0.4")), int(ring.findtext("count", "12"))
                tiers, z0, dz = int(ring.findtext("tiers", "1")), float(ring.findtext("z0", "0")), float(ring.findtext("dz", "0"))
                for t in range(tiers):
                    for i in range(n):
                        a = 2 * math.pi * i / n
                        cells.append((rad * math.cos(a), rad * math.sin(a), z0 + dz * t))
                        angles.append(a)
            elif grid is not None:
                x0, y0, z = (float(grid.findtext(k, "0")) for k in ("x0", "y0", "z"))
                pitch, nx, ny = float(grid.findtext("pitch", "0.052")), int(grid.findtext("nx", "5")), int(grid.findtext("ny", "10"))
                for i in range(nx):
                    for j in range(ny):
                        cells.append((x0 + i * pitch, y0 + j * pitch, z))
                        angles.append(0.0)
            fill = int(c.findtext("fill", str(len(cells))))
            self.carriers[name] = {"cells": cells, "angles": angles, "full": [k < fill for k in range(len(cells))]}
        for h in plug.findall("holder"):
            self.holders[h.findtext("name")] = False
        self.last_error = ""

    def command(self, text: str):
        w = text.split()
        if not w:
            return
        verb, args = w[0], w[1:]
        if verb == "pick" and len(args) == 3:
            car, cell, holder = self.carriers.get(args[0]), args[1], args[2]
            if car is None or holder not in self.holders:
                self.last_error = f"bad pick: {text}"
                return
            k = next((i for i, f in enumerate(car["full"]) if f), -1) if cell == "next" else int(cell)
            if not (0 <= k < len(car["full"])) or not car["full"][k]:
                self.last_error = f"{args[0]} cell {cell} is empty"
                return
            car["full"][k] = False
            self.holders[holder] = True
        elif verb == "move" and len(args) == 2 and self.holders.get(args[0]):
            self.holders[args[0]], self.holders[args[1]] = False, True
        elif verb == "drop" and args:
            self.holders[args[0]] = False
        elif verb in ("fill", "empty") and args and args[0] in self.carriers:
            n = int(args[1]) if len(args) > 1 else 1 << 30
            full = self.carriers[args[0]]["full"]
            idx = range(len(full)) if verb == "fill" else range(len(full) - 1, -1, -1)
            for i in idx:
                if n <= 0:
                    break
                if full[i] != (verb == "fill"):
                    full[i] = verb == "fill"
                    n -= 1

    def sensed(self, holder: str, present: bool):
        """A holder's plug sensor: a plug the books say is there but is not means a missed pick."""
        if holder in self.holders and self.holders[holder] and not present:
            self.holders[holder] = False
            self.last_error = f"no plug in the {holder}: pick missed"

    def status(self, t: float) -> dict:
        out = {"t": round(t, 3), "carriers": {}, "holders": dict(self.holders)}
        for n, c in self.carriers.items():
            nxt = next((i for i, f in enumerate(c["full"]) if f), -1)
            d = {"count": sum(c["full"]), "cells": len(c["full"]), "next": nxt, "occ": "".join("1" if f else "0" for f in c["full"])}
            if nxt >= 0:
                d["next_local"] = list(c["cells"][nxt])
                d["next_angle"] = c["angles"][nxt]
            out["carriers"][n] = d
        if self.last_error:
            out["error"] = self.last_error
        return out
