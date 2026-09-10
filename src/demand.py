"""Traffic and pedestrian demand generation.

Three things make this more realistic than the original study's demand, which
was thirteen identical flows of 100 vehicles each:

1. Uneven origin-destination weighting. The east-west streets act as an arterial
   carrying roughly 70% of vehicle demand; the north-south avenues carry cross
   traffic. The middle street is the busiest, and the peak direction is loaded
   more heavily than the counter-peak, as in a real commute.
2. A time-varying arrival rate. Demand follows a smooth peak rather than a flat
   rate, so the controllers are tested across under-saturated, saturated, and
   recovering conditions inside a single run.
3. Poisson arrivals. Headways are stochastic, so platoons form and dissolve on
   their own instead of vehicles arriving like clockwork.
"""

from __future__ import annotations

import math
import random

from network import N_COLS, N_ROWS
from vehicle import CAR, TRUCK, Pedestrian, Vehicle

TRUCK_SHARE = 0.08
PEAK_FRACTION = 0.35     # peak occurs 35% of the way through the horizon
PEAK_WIDTH = 0.18        # spread of the peak, as a fraction of the horizon
OFF_PEAK = 0.35          # multiplier away from the peak
PEAK_GAIN = 1.15         # additional multiplier at the peak


def rate_multiplier(t: float, horizon: float) -> float:
    """Smooth single-peak demand profile."""
    z = (t - PEAK_FRACTION * horizon) / (PEAK_WIDTH * horizon)
    return OFF_PEAK + PEAK_GAIN * math.exp(-0.5 * z * z)


def build_od(net) -> list[tuple[str, str, float]]:
    """Weighted origin-destination pairs across the ten boundary stubs."""
    ew_stubs = [n for n in net.entry_nodes if n[0] in "WE"]
    ns_stubs = [n for n in net.entry_nodes if n[0] in "NS"]
    od = []
    for o in net.entry_nodes:
        for d in net.entry_nodes:
            if o == d or net.route(o, d) is None:
                continue
            o_art, d_art = o in ew_stubs, d in ew_stubs
            if o_art and d_art:
                w = 1.0                      # arterial through traffic
            elif not o_art and not d_art:
                w = 0.22                     # cross-street through traffic
            else:
                w = 0.42                     # turning between the two
            # The middle east-west street is the busiest of the three.
            mid = str((N_ROWS - 1) // 2)
            if o.endswith(mid) and o_art:
                w *= 1.6
            if d.endswith(mid) and d_art:
                w *= 1.4
            # Peak direction: eastbound and northbound run heavier.
            if o[0] == "W":
                w *= 1.35
            if o[0] == "S":
                w *= 1.20
            od.append((o, d, w))
    return od


def generate_vehicles(net, horizon: float, total: int, seed: int) -> list[Vehicle]:
    """Poisson arrivals over the horizon, distributed across the OD matrix."""
    rng = random.Random(seed)
    od = build_od(net)
    w_sum = sum(w for _, _, w in od)

    # Normalise so the expected count over the horizon equals `total`.
    bin_s = 10.0
    n_bins = int(horizon / bin_s)
    profile = [rate_multiplier(i * bin_s, horizon) for i in range(n_bins)]
    prof_sum = sum(profile) * bin_s

    vehicles: list[Vehicle] = []
    for o, d, w in od:
        route = net.route(o, d)
        lam_total = total * (w / w_sum)          # expected vehicles on this pair
        for i, mult in enumerate(profile):
            lam = lam_total * (mult * bin_s) / prof_sum
            for _ in range(_poisson(rng, lam)):
                t = (i + rng.random()) * bin_s
                vt = TRUCK if rng.random() < TRUCK_SHARE else CAR
                vehicles.append(Vehicle(id=f"v{len(vehicles)}", vtype=vt,
                                        route=list(route), depart=t))
    vehicles.sort(key=lambda v: v.depart)
    for i, v in enumerate(vehicles):
        v.id = f"v{i}"
    return vehicles


def generate_pedestrians(net, horizon: float, per_crossing_hr: float,
                         seed: int) -> list[Pedestrian]:
    """Pedestrians arriving at each signalised crossing, same temporal profile."""
    rng = random.Random(seed + 9973)
    peds: list[Pedestrian] = []
    bin_s = 10.0
    n_bins = int(horizon / bin_s)
    profile = [rate_multiplier(i * bin_s, horizon) for i in range(n_bins)]
    prof_sum = sum(profile) * bin_s
    for node in net.signals:
        for axis in ("NS", "EW"):
            expected = per_crossing_hr * horizon / 3600.0
            for i, mult in enumerate(profile):
                lam = expected * (mult * bin_s) / prof_sum
                for _ in range(_poisson(rng, lam)):
                    t = (i + rng.random()) * bin_s
                    peds.append(Pedestrian(id=f"p{len(peds)}", node=node,
                                           axis=axis, arrive=t))
    peds.sort(key=lambda p: p.arrive)
    return peds


def _poisson(rng: random.Random, lam: float) -> int:
    """Knuth sampler; lam is small here so this is cheap."""
    if lam <= 0:
        return 0
    ell, k, p = math.exp(-lam), 0, 1.0
    while True:
        p *= rng.random()
        if p <= ell:
            return k
        k += 1
