"""Vehicles and the Krauss car-following model.

The Krauss model is what SUMO uses by default, so the driving behaviour here is
directly comparable to the original study: a driver picks the highest speed that
still lets them stop safely if the vehicle ahead brakes hard, then subtracts a
random dawdling term that produces realistic speed scatter and stop-and-go waves.
"""

from __future__ import annotations

from dataclasses import dataclass, field

TAU = 1.0          # s, driver reaction time
STOPPED = 0.1      # m/s, below this a vehicle counts as stopped


@dataclass(frozen=True)
class VehicleType:
    name: str
    length: float       # m
    accel: float        # m/s^2
    decel: float        # m/s^2, comfortable braking
    v_max: float        # m/s
    sigma: float        # dawdling, 0 = perfect driver
    min_gap: float      # m, standstill gap to the vehicle ahead


CAR = VehicleType("car", 5.0, 2.6, 4.5, 13.9, 0.5, 2.5)
TRUCK = VehicleType("truck", 12.0, 1.3, 4.0, 11.1, 0.4, 3.0)


def safe_speed(gap: float, v_lead: float, decel: float) -> float:
    """Krauss safe velocity: fastest speed from which we can still stop in `gap`."""
    if gap <= 0:
        return 0.0
    bt = decel * TAU
    return -bt + (bt * bt + v_lead * v_lead + 2.0 * decel * gap) ** 0.5


@dataclass
class Vehicle:
    id: str
    vtype: VehicleType
    route: list[str]                 # ordered link ids
    depart: float                    # s, when it wants to enter
    link_idx: int = 0
    pos: float = 0.0                 # m from the upstream end of the current link
    speed: float = 0.0
    advisory: float | None = None    # speed cap set by the scheduler controller

    # measurements
    entered: float | None = None
    arrived: float | None = None
    stops: int = 0
    wait_time: float = 0.0
    _was_stopped: bool = False

    @property
    def link(self) -> str:
        return self.route[self.link_idx]

    @property
    def on_last_link(self) -> bool:
        return self.link_idx == len(self.route) - 1

    def free_flow_time(self, net) -> float:
        """Travel time with no signals and no other traffic, for time-loss."""
        return sum(net.links[l].length / min(self.vtype.v_max, net.links[l].speed_limit)
                   for l in self.route)

    def step_speed(self, gap: float, v_lead: float, limit: float, dt: float, rng) -> None:
        """Advance speed one tick under Krauss."""
        vt = self.vtype
        cap = min(vt.v_max, limit)
        if self.advisory is not None:
            cap = min(cap, self.advisory)
        v_des = min(cap,
                    self.speed + vt.accel * dt,
                    safe_speed(gap, v_lead, vt.decel))
        # Dawdling: drivers do not hold the optimum exactly.
        v_des -= vt.sigma * vt.accel * dt * rng.random()
        # Never brake harder than physically reasonable.
        self.speed = max(0.0, min(v_des, self.speed + vt.accel * dt))

    def record(self, dt: float) -> None:
        stopped_now = self.speed < STOPPED
        if stopped_now:
            self.wait_time += dt
            if not self._was_stopped:
                self.stops += 1
        self._was_stopped = stopped_now


@dataclass
class Pedestrian:
    """A pedestrian waiting to cross one leg of an intersection."""
    id: str
    node: str
    axis: str            # axis of the roadway being crossed: 'NS' or 'EW'
    arrive: float
    served: float | None = None
    cross_until: float | None = None

    @property
    def waited(self) -> float | None:
        return None if self.served is None else self.served - self.arrive
