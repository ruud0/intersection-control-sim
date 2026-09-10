"""Reservation-based dynamic scheduling.

There are no signal phases. Each intersection keeps a ledger of committed time
intervals. Every vehicle inside the request horizon asks for the earliest
interval it can reach without conflicting with what is already booked, and is
given a speed advisory that lands it there — so conflicts are resolved by
adjusting speed rather than by stopping.

This is a simplified Autonomous Intersection Management scheme. Three things
separate it from the version in the original study:

* Every approaching vehicle holds a reservation, not just the one at the stop
  line. Granting only to the lead vehicle is what turns AIM back into
  stop-and-go, because everyone behind it queues and then re-requests from rest.
* A vehicle's earliest feasible arrival is bounded by the vehicle ahead of it,
  so the scheduler never issues a slot the driver physically cannot make.
* Pedestrians are first-class. A waiting pedestrian books the crossing, which
  blocks the conflicting axis for the full walk plus clearance interval, and a
  pedestrian who has waited past the limit pre-empts vehicle bookings outright.
  A scheduler that ignores pedestrians is not deployable, and it flatters its
  own numbers.
"""

from __future__ import annotations

from .base import MAX_PED_WAIT, OTHER, PED_CLEARANCE, WALK_INTERVAL, Controller

REQUEST_HORIZON = 160.0   # m upstream at which a vehicle may request a slot
CLEARANCE = 2.0           # s of separation required between conflicting axes
OCCUPANCY = 2.0           # s a vehicle occupies the conflict area
HEADWAY = 1.9             # s between same-axis vehicles (~1900 veh/h saturation)
MIN_ADVISORY = 2.5        # m/s, below this let the vehicle stop instead
PED_BOOKING = WALK_INTERVAL + PED_CLEARANCE


class _Ledger:
    """Committed intervals for one intersection."""

    def __init__(self):
        self.slots: list[tuple[float, float, str]] = []   # (start, end, axis)

    def prune(self, t: float) -> None:
        if len(self.slots) > 24:
            self.slots = [s for s in self.slots if s[1] > t - 4.0]

    def earliest(self, t_min: float, duration: float, axis: str) -> float:
        """First start time at or after `t_min` that clears every booking."""
        t = t_min
        for _ in range(300):
            blocked = None
            for s_start, s_end, s_axis in self.slots:
                if s_axis == axis:
                    # Same axis may follow, but only at saturation headway.
                    if abs(t - s_start) < HEADWAY:
                        cand = s_start + HEADWAY
                        blocked = cand if blocked is None else max(blocked, cand)
                else:
                    if t < s_end + CLEARANCE and t + duration + CLEARANCE > s_start:
                        cand = s_end + CLEARANCE
                        blocked = cand if blocked is None else max(blocked, cand)
            if blocked is None:
                return t
            t = blocked + 1e-6
        return t

    def book(self, start: float, end: float, axis: str) -> None:
        self.slots.append((start, end, axis))

    def drop_future_on(self, axis: str, t: float) -> None:
        """Cancel not-yet-started bookings on one axis (pedestrian pre-emption)."""
        self.slots = [s for s in self.slots if s[2] != axis or s[0] <= t]


class DynamicScheduler(Controller):
    name = "Dynamic scheduling"

    def __init__(self, net):
        super().__init__(net)
        self.ledgers = {n: _Ledger() for n in net.signals}
        self.grants: dict[str, tuple[str, float]] = {}   # veh id -> (node, slot)
        self.ped_slots: dict[tuple[str, str], tuple[float, float]] = {}

    # ------------------------------------------------------------------ step
    def step(self, t, dt, sim):
        for node, ledger in self.ledgers.items():
            ledger.prune(t)
            self._book_pedestrians(t, node, ledger, sim)
            self._grant_vehicles(t, node, ledger, sim)

    def _book_pedestrians(self, t, node, ledger, sim):
        for axis in ("NS", "EW"):
            if not sim.ped_waiting(node, axis):
                continue
            slot = self.ped_slots.get((node, axis))
            if slot and slot[1] > t:
                continue
            blocked_axis = axis          # crossing this roadway conflicts with it
            if sim.ped_wait_exceeds(node, axis, t, MAX_PED_WAIT):
                ledger.drop_future_on(blocked_axis, t)
                start = t
            else:
                start = ledger.earliest(t, PED_BOOKING, OTHER[axis])
            ledger.book(start, start + PED_BOOKING, OTHER[axis])
            self.ped_slots[(node, axis)] = (start, start + PED_BOOKING)

    def _grant_vehicles(self, t, node, ledger, sim):
        """Grant slots to every approaching vehicle inside the horizon."""
        for lid in self.net.incoming[node]:
            link = self.net.links[lid]
            axis = link.axis
            prev_slot = None
            # sim keeps each link ordered head-first, so this is queue order.
            for v in sim.on_road[lid]:
                dist = link.length - v.pos
                if dist > REQUEST_HORIZON:
                    break
                held = self.grants.get(v.id)
                if held and held[0] == node:
                    prev_slot = held[1]
                    continue
                v_max = min(v.vtype.v_max, link.speed_limit)
                t_free = t + _time_to_cover(dist, v.speed, v_max, v.vtype.accel)
                if prev_slot is not None:
                    t_free = max(t_free, prev_slot + HEADWAY)
                slot = ledger.earliest(t_free, OCCUPANCY, axis)
                ledger.book(slot, slot + OCCUPANCY, axis)
                self.grants[v.id] = (node, slot)
                prev_slot = slot

    # ------------------------------------------------------------- interface
    def may_enter(self, t, veh, node, axis):
        held = self.grants.get(veh.id)
        return bool(held) and held[0] == node and t >= held[1] - 0.5

    def advisory(self, t, veh, node, dist):
        held = self.grants.get(veh.id)
        if not held or held[0] != node:
            return None
        remaining = held[1] - t
        if remaining <= 0.5 or dist <= 0:
            return None
        v = dist / remaining
        if v >= min(veh.vtype.v_max, self.net.links[veh.link].speed_limit):
            return None
        return max(v, MIN_ADVISORY)

    def clear(self, veh) -> None:
        self.grants.pop(veh.id, None)

    def ped_can_cross(self, t, node, axis):
        slot = self.ped_slots.get((node, axis))
        return bool(slot) and slot[0] <= t < slot[1]


def _time_to_cover(dist: float, v0: float, v_max: float, accel: float) -> float:
    """Time to travel `dist`, accelerating to `v_max` then cruising."""
    t_acc = max(0.0, (v_max - v0) / accel)
    d_acc = v0 * t_acc + 0.5 * accel * t_acc * t_acc
    if d_acc >= dist:
        return (-v0 + (v0 * v0 + 2 * accel * dist) ** 0.5) / accel
    return t_acc + (dist - d_acc) / v_max
