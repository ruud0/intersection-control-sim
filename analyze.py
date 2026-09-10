#!/usr/bin/env python3
"""Turn results/results.csv into the figures used in the README.

Produces docs/headline.png (the three-panel featured visual) and
docs/stops_vs_demand.png. Error bars are 95% confidence intervals across seeds.
"""

from __future__ import annotations

import csv
import math
import os
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt   # noqa: E402

ORDER = ["Fixed-time (Webster)", "Semi-actuated", "Dynamic scheduling"]
COLOR = {"Fixed-time (Webster)": "#C2453B",
         "Semi-actuated": "#E08A2E",
         "Dynamic scheduling": "#2E7DB8"}
MARKER = {"Fixed-time (Webster)": "o", "Semi-actuated": "s", "Dynamic scheduling": "^"}


def load(path="results/results.csv"):
    rows = []
    with open(path) as fh:
        for r in csv.DictReader(fh):
            for k, v in r.items():
                if k != "controller":
                    r[k] = float(v)
            rows.append(r)
    return rows


def summarise(rows, metric):
    """metric -> {controller: (demands, means, half-widths of 95% CI)}"""
    buckets = defaultdict(list)
    for r in rows:
        buckets[(r["controller"], r["demand"])].append(r[metric])
    out = {}
    for name in ORDER:
        ds = sorted({d for (c, d) in buckets if c == name})
        means, errs = [], []
        for d in ds:
            vals = buckets[(name, d)]
            m = sum(vals) / len(vals)
            if len(vals) > 1:
                var = sum((v - m) ** 2 for v in vals) / (len(vals) - 1)
                errs.append(1.96 * math.sqrt(var / len(vals)))
            else:
                errs.append(0.0)
            means.append(m)
        out[name] = (ds, means, errs)
    return out


def _panel(ax, rows, metric, title, ylabel):
    for name, (ds, ms, es) in summarise(rows, metric).items():
        ax.errorbar(ds, ms, yerr=es, label=name, color=COLOR[name],
                    marker=MARKER[name], markersize=5, capsize=3, linewidth=1.8)
    ax.set_title(title, fontsize=10.5, pad=8)
    ax.set_xlabel("Vehicles generated per hour", fontsize=9)
    ax.set_ylabel(ylabel, fontsize=9)
    ax.grid(alpha=0.25, linewidth=0.6)
    ax.tick_params(labelsize=8.5)
    ax.set_ylim(bottom=0)


def headline(rows, path="docs/headline.png"):
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.1))
    _panel(axes[0], rows, "stops_per_vehicle",
           "Stops per vehicle", "stops")
    _panel(axes[1], rows, "mean_delay",
           "Mean delay over free flow", "seconds")
    _panel(axes[2], rows, "ped_mean_wait",
           "Pedestrian wait at crossings", "seconds")
    axes[0].legend(fontsize=8.5, frameon=False, loc="upper left")
    fig.suptitle("Intersection control on a six-signal grid: "
                 "scheduling wins on stops, but pays for it at the crosswalk",
                 fontsize=12, y=1.0)
    fig.tight_layout()
    os.makedirs("docs", exist_ok=True)
    fig.savefig(path, dpi=160, bbox_inches="tight")
    print("wrote", path)


def stops_chart(rows, path="docs/stops_vs_demand.png"):
    fig, ax = plt.subplots(figsize=(6.4, 4.2))
    _panel(ax, rows, "stops_per_vehicle",
           "Stops per vehicle by control strategy", "stops per vehicle")
    ax.legend(fontsize=9, frameon=False)
    fig.tight_layout()
    fig.savefig(path, dpi=160, bbox_inches="tight")
    print("wrote", path)


def table(rows):
    print(f"\n{'controller':24}{'demand':>8}{'stops':>9}{'delay':>9}"
          f"{'p95':>9}{'clear':>9}{'pedwait':>9}")
    for metric in ("stops_per_vehicle",):
        pass
    agg = defaultdict(list)
    for r in rows:
        agg[(r["controller"], r["demand"])].append(r)
    for name in ORDER:
        for d in sorted({dd for (c, dd) in agg if c == name}):
            g = agg[(name, d)]
            n = len(g)
            f = lambda k: sum(x[k] for x in g) / n     # noqa: E731
            print(f"{name:24}{int(d):>8}{f('stops_per_vehicle'):>9.2f}"
                  f"{f('mean_delay'):>9.1f}{f('p95_delay'):>9.1f}"
                  f"{f('clearing_time'):>9.0f}{f('ped_mean_wait'):>9.1f}")


if __name__ == "__main__":
    rows = load()
    headline(rows)
    stops_chart(rows)
    table(rows)
