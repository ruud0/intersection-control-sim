#!/usr/bin/env python3
"""Render a short animation of the grid under one control strategy.

    python animate.py --controller scheduler --out docs/simulation.gif

Vehicles are drawn as dots on their links, coloured by whether they are moving
or stopped, so the difference in stop-and-go behaviour between strategies is
visible directly.
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "src"))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                      # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter   # noqa: E402

from controllers import DynamicScheduler, FixedTime, SemiActuated   # noqa: E402
from demand import generate_pedestrians, generate_vehicles          # noqa: E402
from network import Network                                         # noqa: E402
from sim import Sim                                                 # noqa: E402
from vehicle import STOPPED                                         # noqa: E402

CHOICES = {"fixed": FixedTime, "actuated": SemiActuated, "scheduler": DynamicScheduler}


def link_xy(net, link, frac):
    a, b = net.nodes[link.frm], net.nodes[link.to]
    # Offset to the right-hand side of the carriageway so both directions show.
    dx, dy = b.x - a.x, b.y - a.y
    n = (dx * dx + dy * dy) ** 0.5 or 1.0
    ox, oy = -dy / n * 7.0, dx / n * 7.0
    return a.x + dx * frac + ox, a.y + dy * frac + oy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--controller", choices=list(CHOICES), default="scheduler")
    ap.add_argument("--demand", type=int, default=2000)
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--start", type=float, default=600.0)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--fps", type=int, default=12)
    ap.add_argument("--out", default="docs/simulation.gif")
    args = ap.parse_args()

    net = Network()
    ctrl = CHOICES[args.controller](net)
    sim = Sim(net, ctrl,
              generate_vehicles(net, 3600.0, args.demand, args.seed),
              generate_pedestrians(net, 3600.0, 60, args.seed),
              dt=0.5, max_time=14400.0, seed=args.seed)

    # Warm the network up to the requested start time.
    while sim.t < args.start:
        sim.ctrl.step(sim.t, sim.dt, sim)
        sim._release_vehicles(); sim._release_pedestrians()
        sim._advance_vehicles(); sim._serve_pedestrians()
        sim.t += sim.dt

    frames = []
    steps_per_frame = max(1, int((1.0 / args.fps) / sim.dt * 2))
    n_frames = int(args.seconds * args.fps / 2)
    for _ in range(n_frames):
        for _ in range(steps_per_frame):
            sim.ctrl.step(sim.t, sim.dt, sim)
            sim._release_vehicles(); sim._release_pedestrians()
            sim._advance_vehicles(); sim._serve_pedestrians()
            sim.t += sim.dt
        pts = []
        for lid, queue in sim.on_road.items():
            link = net.links[lid]
            for v in queue:
                x, y = link_xy(net, link, min(1.0, v.pos / link.length))
                pts.append((x, y, v.speed < STOPPED))
        frames.append((sim.t, pts))

    fig, ax = plt.subplots(figsize=(5.6, 6.4))
    ax.set_aspect("equal"); ax.axis("off")
    for link in net.links.values():
        a, b = net.nodes[link.frm], net.nodes[link.to]
        ax.plot([a.x, b.x], [a.y, b.y], color="#D9D6D0", linewidth=7, zorder=1,
                solid_capstyle="round")
    for n in net.signals:
        nd = net.nodes[n]
        ax.plot(nd.x, nd.y, "s", color="#3A3A3A", markersize=6, zorder=3)
    moving = ax.scatter([], [], s=14, c="#2E7DB8", zorder=4, label="moving")
    halted = ax.scatter([], [], s=14, c="#C2453B", zorder=5, label="stopped")
    title = ax.set_title("", fontsize=10)
    ax.legend(fontsize=8, frameon=False, loc="lower center", ncol=2,
              bbox_to_anchor=(0.5, -0.06))

    def draw(i):
        t, pts = frames[i]
        mv = [(x, y) for x, y, s in pts if not s]
        st = [(x, y) for x, y, s in pts if s]
        moving.set_offsets(mv if mv else [(1e9, 1e9)])
        halted.set_offsets(st if st else [(1e9, 1e9)])
        title.set_text(f"{ctrl.name}   t = {t:.0f} s   "
                       f"{len(pts)} vehicles, {len(st)} stopped")
        return moving, halted, title

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    anim = FuncAnimation(fig, draw, frames=len(frames), blit=False)
    anim.save(args.out, writer=PillowWriter(fps=args.fps))
    print("wrote", args.out, f"({len(frames)} frames)")


if __name__ == "__main__":
    main()
