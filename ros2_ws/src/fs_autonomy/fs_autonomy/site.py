"""Site survey data: the YAML written by fs_gazebo.sites plus its soil map.

On a real deployment the same files come from the pre-planting survey (drone DEM, utility
survey, soil probes); the stack never reads anything from the simulator through here.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import yaml

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


class SoilMap:
    """Python reader for the fs_soilmap v1 format (same file the Gazebo systems read)."""

    LAYERS = ("elevation", "soil", "rock_depth", "moisture")

    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.soils: list[dict] = []
        self.layers: dict[str, np.ndarray] = {}
        lines = self.path.read_text().splitlines()
        i = 0
        while i < len(lines):
            ln = lines[i].strip()
            i += 1
            if not ln or ln.startswith("#"):
                continue
            key, *rest = ln.split()
            if key == "name":
                self.name = rest[0]
            elif key == "origin":
                self.x0, self.y0 = float(rest[0]), float(rest[1])
            elif key == "resolution":
                self.res = float(rest[0])
            elif key == "size":
                self.nx, self.ny = int(rest[0]), int(rest[1])
            elif key == "soil":
                idx, name, *v = rest
                n, kc, kphi, c, phi, K, eb = map(float, v)
                self.soils.append({"index": int(idx), "name": name, "n": n, "kc": kc, "kphi": kphi, "c": c,
                                   "phi": phi, "K": K, "eb": eb})
            elif key == "layer":
                block = " ".join(lines[i:i + self.ny])
                self.layers[rest[0]] = np.array(block.split(), dtype=float).reshape(self.ny, self.nx)
                i += self.ny
        self.soils.sort(key=lambda s: s["index"])

    def _frac(self, x, y):
        fx = np.clip((np.asarray(x, float) - self.x0) / self.res, 0, self.nx - 1)
        fy = np.clip((np.asarray(y, float) - self.y0) / self.res, 0, self.ny - 1)
        return fx, fy

    def bilinear(self, layer: str, x, y):
        a = self.layers[layer]
        fx, fy = self._frac(x, y)
        i = np.minimum(fx.astype(int), self.nx - 2)
        j = np.minimum(fy.astype(int), self.ny - 2)
        u, v = fx - i, fy - j
        return ((1 - u) * (1 - v) * a[j, i] + u * (1 - v) * a[j, i + 1] + (1 - u) * v * a[j + 1, i]
                + u * v * a[j + 1, i + 1])

    def elevation(self, x, y):
        return self.bilinear("elevation", x, y)

    def rock_depth(self, x, y):
        return self.bilinear("rock_depth", x, y)

    def soil_at(self, x: float, y: float) -> dict:
        fx, fy = self._frac(x, y)
        k = int(self.layers["soil"][int(round(float(fy))), int(round(float(fx)))])
        return self.soils[min(max(k, 0), len(self.soils) - 1)]

    def sample(self, x: float, y: float) -> dict:
        """Everything the soil map knows at a point. Optional layers default to 'absent':
        hardpan (depth to a kankar pan, m; 9 = none), esp (exchangeable sodium %), waterlog (0..1)."""
        opt = {k: (float(self.bilinear(k, x, y)) if k in self.layers else d)
               for k, d in (("hardpan", 9.0), ("esp", 0.0), ("waterlog", 0.0))}
        return {"soil": self.soil_at(x, y), "elevation": float(self.elevation(x, y)),
                "rock_depth": float(self.rock_depth(x, y)), "moisture": float(self.bilinear("moisture", x, y)),
                **opt}

    def normal(self, x: float, y: float):
        h = max(self.res, 0.05)
        dzdx = float(self.elevation(x + h, y) - self.elevation(x - h, y)) / (2 * h)
        dzdy = float(self.elevation(x, y + h) - self.elevation(x, y - h)) / (2 * h)
        n = np.array([-dzdx, -dzdy, 1.0])
        return n / np.linalg.norm(n)


class Site:
    def __init__(self, yaml_path: str | Path):
        self.path = Path(yaml_path)
        self.dir = self.path.parent
        self.data = yaml.safe_load(self.path.read_text())
        self.name = self.data["name"]
        o = self.data["origin"]
        self.lat0, self.lon0, self.alt0 = float(o["lat"]), float(o["lon"]), float(o["alt"])
        lat = math.radians(self.lat0)
        s = math.sin(lat)
        self._rn = WGS84_A / math.sqrt(1 - WGS84_E2 * s * s)            # prime vertical
        self._rm = WGS84_A * (1 - WGS84_E2) / (1 - WGS84_E2 * s * s) ** 1.5  # meridian
        self._soil = None

    @property
    def soil_map(self) -> SoilMap:
        if self._soil is None:
            self._soil = SoilMap(self.dir / self.data["soil_map"])
        return self._soil

    def __getitem__(self, k):
        return self.data[k]

    def get(self, k, default=None):
        return self.data.get(k, default)

    def geo_to_enu(self, lat: float, lon: float):
        x = math.radians(lon - self.lon0) * self._rn * math.cos(math.radians(self.lat0))
        y = math.radians(lat - self.lat0) * self._rm
        return x, y

    def enu_to_geo(self, x: float, y: float):
        lat = self.lat0 + math.degrees(y / self._rm)
        lon = self.lon0 + math.degrees(x / (self._rn * math.cos(math.radians(self.lat0))))
        return lat, lon

    def spawn(self, name: str) -> dict:
        for s in self.data.get("spawn", []):
            if s["name"] == name:
                return s
        raise KeyError(f"no spawn pose for {name} in {self.path}")
