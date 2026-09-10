"""Performance measures extracted from a completed run."""

from __future__ import annotations

from dataclasses import asdict, dataclass


def _pct(values, q):
    if not values:
        return 0.0
    s = sorted(values)
    k = max(0, min(len(s) - 1, int(round(q * (len(s) - 1)))))
    return s[k]


def _mean(values):
    return sum(values) / len(values) if values else 0.0


@dataclass
class Result:
    controller: str
    seed: int
    vehicles_loaded: int
    vehicles_arrived: int
    clearing_time: float
    mean_delay: float          # s per vehicle over free-flow
    p95_delay: float
    stops_per_vehicle: float
    mean_stopped_time: float   # s per vehicle at a standstill
    conflicts_blocked: int
    peds_served: int
    ped_mean_wait: float
    ped_p95_wait: float

    def as_dict(self):
        return asdict(self)


def collect(sim, controller_name: str, seed: int) -> Result:
    net = sim.net
    delays, stops, waits = [], [], []
    for v in sim.arrived:
        travel = v.arrived - v.entered
        delays.append(max(0.0, travel - v.free_flow_time(net)))
        stops.append(v.stops)
        waits.append(v.wait_time)

    ped_waits = [p.waited for p in sim.all_peds if p.waited is not None]

    return Result(
        controller=controller_name,
        seed=seed,
        vehicles_loaded=len(sim.all_vehicles),
        vehicles_arrived=len(sim.arrived),
        clearing_time=max((v.arrived for v in sim.arrived), default=0.0),
        mean_delay=_mean(delays),
        p95_delay=_pct(delays, 0.95),
        stops_per_vehicle=_mean(stops),
        mean_stopped_time=_mean(waits),
        conflicts_blocked=sim.conflicts_blocked,
        peds_served=len(ped_waits),
        ped_mean_wait=_mean(ped_waits),
        ped_p95_wait=_pct(ped_waits, 0.95),
    )
