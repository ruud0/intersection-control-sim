"""Time-stepped microsimulation of the grid under a given control strategy.

Vehicles follow the Krauss model along their route. At each intersection the
active controller decides whether a vehicle may enter, and pedestrians cross
when their crossing is served. Every strategy sees an identical network and an
identical seeded demand, so differences in the results come from control alone.
"""

from __future__ import annotations

from controllers.base import PED_CLEARANCE, WALK_INTERVAL
from vehicle import STOPPED

BIG_GAP = 1e6
PED_CROSS_TIME = WALK_INTERVAL + PED_CLEARANCE
BOX_TIME = 2.0        # s a vehicle physically occupies the conflict area


class Sim:
    def __init__(self, net, controller, vehicles, pedestrians, dt=0.5,
                 max_time=7200.0, seed=0):
        import random
        self.net = net
        self.ctrl = controller
        self.dt = dt
        self.max_time = max_time
        self.rng = random.Random(seed + 4242)

        self.pending = sorted(vehicles, key=lambda v: v.depart)
        self.all_vehicles = list(vehicles)
        self.on_road: dict[str, list] = {lid: [] for lid in net.links}
        self.arrived: list = []

        self.peds_pending = sorted(pedestrians, key=lambda p: p.arrive)
        self.all_peds = list(pedestrians)
        self.ped_queue: dict[tuple[str, str], list] = {}
        self.ped_active: dict[tuple[str, str], float] = {}

        # Physical occupancy of each intersection: node -> (axis, busy until).
        # Enforced for every strategy, so no controller can book a movement the
        # geometry would not actually allow.
        self.box: dict[str, tuple[str, float]] = {}
        self.conflicts_blocked = 0

        self.t = 0.0
        self.history: list[tuple[float, int, int]] = []   # (t, on road, arrived)

    # ------------------------------------------------ queries for controllers
    def demand_within(self, node: str, axis: str, zone: float) -> bool:
        """Is a vehicle within `zone` metres of the stop line on `axis`?"""
        for lid in self.net.incoming[node]:
            link = self.net.links[lid]
            if link.axis != axis:
                continue
            for v in self.on_road[lid]:
                if link.length - v.pos <= zone:
                    return True
        return False

    def ped_waiting(self, node: str, axis: str) -> bool:
        return bool(self.ped_queue.get((node, axis)))

    def ped_crossing(self, node: str, axis: str) -> bool:
        return self.ped_active.get((node, axis), -1.0) > self.t

    def ped_wait_exceeds(self, node: str, axis: str, t: float, limit: float) -> bool:
        q = self.ped_queue.get((node, axis))
        return bool(q) and (t - q[0].arrive) > limit

    # ------------------------------------------------------------- main loop
    def run(self):
        n_total = len(self.all_vehicles)
        while self.t < self.max_time and len(self.arrived) < n_total:
            self.ctrl.step(self.t, self.dt, self)
            self._release_vehicles()
            self._release_pedestrians()
            self._advance_vehicles()
            self._serve_pedestrians()
            self.t += self.dt
            if int(self.t / self.dt) % int(10 / self.dt) == 0:
                on_road = sum(len(q) for q in self.on_road.values())
                self.history.append((self.t, on_road, len(self.arrived)))
        return self

    # ------------------------------------------------------------- vehicles
    def _release_vehicles(self):
        while self.pending and self.pending[0].depart <= self.t:
            v = self.pending[0]
            lid = v.route[0]
            queue = self.on_road[lid]
            # Only enter if there is room behind the last vehicle on the link.
            if queue and queue[-1].pos < v.vtype.length + v.vtype.min_gap:
                break
            self.pending.pop(0)
            v.pos, v.speed, v.entered = 0.0, 0.0, self.t
            queue.append(v)

    def _advance_vehicles(self):
        for lid, queue in self.on_road.items():
            link = self.net.links[lid]
            departed = []
            for i, v in enumerate(queue):
                if i > 0:
                    lead = queue[i - 1]
                    gap = lead.pos - lead.vtype.length - v.pos - v.vtype.min_gap
                    v_lead = lead.speed
                else:
                    gap, v_lead = self._head_gap(v, link)
                v.advisory = None
                if hasattr(self.ctrl, "advisory") and not v.on_last_link:
                    adv = self.ctrl.advisory(self.t, v, link.to,
                                             link.length - v.pos)
                    v.advisory = adv
                v.step_speed(max(gap, 0.0), v_lead, link.speed_limit, self.dt, self.rng)
                v.pos += v.speed * self.dt
                v.record(self.dt)

                if v.pos >= link.length:
                    if v.on_last_link:
                        v.arrived = self.t
                        self.arrived.append(v)
                        if hasattr(self.ctrl, "clear"):
                            self.ctrl.clear(v)
                        departed.append(v)
                    else:
                        overshoot = v.pos - link.length
                        if link.to in self.box or link.to in self.net.nodes:
                            if self.net.nodes[link.to].is_signal:
                                self.box[link.to] = (link.axis, self.t + BOX_TIME)
                        nxt = v.route[v.link_idx + 1]
                        v.link_idx += 1
                        v.pos = overshoot
                        self.on_road[nxt].append(v)
                        self.on_road[nxt].sort(key=lambda x: -x.pos)
                        if hasattr(self.ctrl, "clear"):
                            self.ctrl.clear(v)
                        departed.append(v)
            for v in departed:
                queue.remove(v)
            queue.sort(key=lambda x: -x.pos)

    def _head_gap(self, v, link):
        """Gap for the vehicle at the head of a link: stop line or next link."""
        dist = link.length - v.pos
        if v.on_last_link:
            return BIG_GAP, 0.0
        node = link.to
        axis = link.axis
        if hasattr(self.ctrl, "request"):
            self.ctrl.request(self.t, v, node, axis, dist, v.speed)
        if not self.ctrl.may_enter(self.t, v, node, axis):
            return dist - v.vtype.min_gap, 0.0
        held = self.box.get(node)
        if held and held[0] != axis and held[1] > self.t:
            # Conflicting movement still inside the box; hold at the stop line.
            self.conflicts_blocked += 1
            return dist - v.vtype.min_gap, 0.0
        nxt = self.net.links[v.route[v.link_idx + 1]]
        tail = self.on_road[nxt.id][-1] if self.on_road[nxt.id] else None
        if tail is None:
            return BIG_GAP, 0.0
        return dist + tail.pos - tail.vtype.length - v.vtype.min_gap, tail.speed

    # ---------------------------------------------------------- pedestrians
    def _release_pedestrians(self):
        while self.peds_pending and self.peds_pending[0].arrive <= self.t:
            p = self.peds_pending.pop(0)
            self.ped_queue.setdefault((p.node, p.axis), []).append(p)

    def _serve_pedestrians(self):
        for (node, axis), queue in self.ped_queue.items():
            if not queue:
                continue
            if self.ped_active.get((node, axis), -1.0) > self.t:
                continue
            if self.ctrl.ped_can_cross(self.t, node, axis):
                # Everyone waiting steps off together on the WALK indication.
                for p in queue:
                    p.served = self.t
                    p.cross_until = self.t + PED_CROSS_TIME
                self.ped_active[(node, axis)] = self.t + PED_CROSS_TIME
                queue.clear()
