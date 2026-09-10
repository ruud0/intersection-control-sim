"""Semi-actuated control.

The arterial rests in green. The side street is served only when a detector or a
pedestrian push-button places a call, and its green extends vehicle by vehicle
until the arrivals gap out or the maximum green is reached. This is how most
real coordinated-arterial intersections actually run.
"""

from __future__ import annotations

from .base import (ALL_RED, MAX_PED_WAIT, OTHER, PED_MIN_GREEN, YELLOW,
                   Controller)

DETECTOR_ZONE = 45.0     # m upstream of the stop line
UNIT_EXTENSION = 2.5     # s of green added per actuation
GAP_OUT = 3.0            # s without an actuation ends the phase
MIN_GREEN = 7.0
MAX_GREEN = 35.0
MAIN_MIN_GREEN = 12.0    # arterial protection before it can be interrupted

MAIN, SIDE = "EW", "NS"


class SemiActuated(Controller):
    name = "Semi-actuated"

    def __init__(self, net):
        super().__init__(net)
        self.state = {n: {"axis": MAIN, "since": 0.0, "steady": True,
                          "last_act": 0.0, "budget": 0.0} for n in net.signals}

    def step(self, t, dt, sim):
        for node, st in self.state.items():
            elapsed = t - st["since"]
            serving = st["axis"]

            if not st["steady"]:
                # In yellow + all-red; switch when the interval expires.
                if elapsed >= YELLOW + ALL_RED:
                    st["axis"] = OTHER[serving]
                    st["since"] = t
                    st["steady"] = True
                    st["last_act"] = t
                    st["budget"] = MIN_GREEN if st["axis"] == SIDE else MAIN_MIN_GREEN
                continue

            if serving == SIDE:
                # Extend on each actuation, gap out or max out otherwise.
                if sim.demand_within(node, SIDE, DETECTOR_ZONE):
                    st["last_act"] = t
                    st["budget"] = min(max(st["budget"], elapsed + UNIT_EXTENSION),
                                       MAX_GREEN)
                ped_hold = sim.ped_crossing(node, MAIN) and elapsed < PED_MIN_GREEN
                gapped = (t - st["last_act"]) > GAP_OUT and elapsed >= MIN_GREEN
                maxed = elapsed >= MAX_GREEN
                if (gapped or maxed) and not ped_hold:
                    st["steady"] = False
                    st["since"] = t
            else:
                # Arterial rests in green until something calls the side street.
                call = (sim.demand_within(node, SIDE, DETECTOR_ZONE)
                        or sim.ped_waiting(node, MAIN))
                forced = sim.ped_wait_exceeds(node, MAIN, t, MAX_PED_WAIT)
                ped_hold = sim.ped_crossing(node, SIDE) and elapsed < PED_MIN_GREEN
                if (call and elapsed >= MAIN_MIN_GREEN and not ped_hold) or forced:
                    st["steady"] = False
                    st["since"] = t

    def may_enter(self, t, veh, node, axis):
        st = self.state[node]
        return st["steady"] and st["axis"] == axis

    def ped_can_cross(self, t, node, axis):
        st = self.state[node]
        return st["steady"] and st["axis"] == OTHER[axis]
