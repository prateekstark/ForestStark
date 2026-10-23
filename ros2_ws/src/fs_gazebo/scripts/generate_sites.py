#!/usr/bin/env python3
"""Generate ForestStark site files (world, soil map, survey YAML) and the scenery models.

    generate_sites.py --site hill_slope --out /tmp/sites --slope 18
    generate_sites.py --models <fs_gazebo>/models          # (re)write scenery model directories
"""
import argparse
from pathlib import Path

from fs_gazebo.models import write_models
from fs_gazebo.sites import BUILDERS, generate


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--site", choices=sorted(BUILDERS), action="append")
    ap.add_argument("--out", type=Path, default=Path.cwd())
    ap.add_argument("--seed", type=int)
    ap.add_argument("--soil")
    ap.add_argument("--slope", type=float, help="hill_slope only, degrees")
    ap.add_argument("--models", type=Path, help="write scenery models into this directory")
    a = ap.parse_args()
    if a.models:
        write_models(a.models)
        print(f"models -> {a.models}")
    for site in a.site or []:
        kw = {k: v for k, v in (("seed", a.seed), ("soil", a.soil)) if v is not None}
        if site == "hill_slope" and a.slope is not None:
            kw["slope_deg"] = a.slope
        generate(site, a.out, **kw)
        print(f"{site} -> {a.out}")


if __name__ == "__main__":
    main()
