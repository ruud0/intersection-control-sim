#!/usr/bin/env python3
"""Run the full experiment: every controller, across a demand sweep, replicated.

Each (controller, demand level, seed) combination sees an identical network and
identically seeded demand, so the only thing that varies within a cell is the
control strategy. Results are written to results/results.csv.

    python run.py                # full sweep, ~2 minutes on 8 cores
    python run.py --quick        # smaller sweep for a fast check
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from dataclasses import fields
from multiprocessing import Pool

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "src"))

from controllers import DynamicScheduler, FixedTime, SemiActuated   # noqa: E402
from demand import generate_pedestrians, generate_vehicles          # noqa: E402
from metrics import Result, collect                                 # noqa: E402
from network import Network                                         # noqa: E402
from sim import Sim                                                 # noqa: E402

HORIZON = 3600.0          # s of demand generation
MAX_TIME = 14400.0        # s hard stop, so a gridlocked run still terminates
PED_PER_CROSSING_HR = 60  # pedestrians per crossing per hour at the peak
CONTROLLERS = [FixedTime, SemiActuated, DynamicScheduler]

DEMAND_LEVELS = [1000, 1500, 2000, 2500, 3000]
SEEDS = list(range(1, 9))
QUICK_LEVELS = [1500, 2500]
QUICK_SEEDS = [1, 2]


def one_run(job) -> dict:
    cls_name, level, seed = job
    cls = {c.__name__: c for c in CONTROLLERS}[cls_name]
    net = Network()
    ctrl = cls(net)
    veh = generate_vehicles(net, HORIZON, level, seed)
    peds = generate_pedestrians(net, HORIZON, PED_PER_CROSSING_HR, seed)
    sim = Sim(net, ctrl, veh, peds, dt=0.5, max_time=MAX_TIME, seed=seed).run()
    row = collect(sim, ctrl.name, seed).as_dict()
    row["demand"] = level
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="small sweep, fast")
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 4)
    args = ap.parse_args()

    levels = QUICK_LEVELS if args.quick else DEMAND_LEVELS
    seeds = QUICK_SEEDS if args.quick else SEEDS
    jobs = [(c.__name__, lv, s) for c in CONTROLLERS for lv in levels for s in seeds]

    print(f"{len(jobs)} runs: {len(CONTROLLERS)} controllers x "
          f"{len(levels)} demand levels x {len(seeds)} seeds")
    net = Network()
    print("network:", net.describe())

    with Pool(args.jobs) as pool:
        rows = []
        for i, row in enumerate(pool.imap_unordered(one_run, jobs), 1):
            rows.append(row)
            print(f"  [{i}/{len(jobs)}] {row['controller']:22} "
                  f"demand={row['demand']:5} seed={row['seed']} "
                  f"stops={row['stops_per_vehicle']:6.2f} "
                  f"delay={row['mean_delay']:7.1f}s", flush=True)

    os.makedirs("results", exist_ok=True)
    cols = ["controller", "demand", "seed"] + [
        f.name for f in fields(Result) if f.name not in ("controller", "seed")]
    rows.sort(key=lambda r: (r["controller"], r["demand"], r["seed"]))
    with open("results/results.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)
    print(f"\nwrote results/results.csv ({len(rows)} rows)")


if __name__ == "__main__":
    main()
