"""Fixed-time control with splits from Webster's formula.

Webster (1958) gives the cycle length that minimises average delay:

    C = (1.5 L + 5) / (1 - Y)

where L is total lost time per cycle and Y is the sum of critical flow ratios.
Green is then split between phases in proportion to those flow ratios. This is
the textbook operations-research method, so the fixed-time baseline is a fair
opponent rather than an arbitrary cycle picked by hand.
"""

from __future__ import annotations

from .base import (ALL_RED, LOST_TIME, OTHER, PED_MIN_GREEN, YELLOW,
                   Controller)

SATURATION_FLOW = 1900.0    # veh/h/lane, standard urban value


class FixedTime(Controller):
    name = "Fixed-time (Webster)"

    def __init__(self, net, crit_flow_ns: float = 420.0, crit_flow_ew: float = 900.0):
        super().__init__(net)
        y_ns = crit_flow_ns / SATURATION_FLOW
        y_ew = crit_flow_ew / SATURATION_FLOW
        Y = min(y_ns + y_ew, 0.85)                  # cap so the cycle stays finite
        L = 2 * (LOST_TIME + ALL_RED)
        cycle = (1.5 * L + 5.0) / (1.0 - Y)
        self.cycle = max(40.0, min(cycle, 150.0))   # practical bounds

        effective = self.cycle - L
        g_ns = effective * y_ns / (y_ns + y_ew)
        g_ew = effective - g_ns
        # Every phase must be long enough to walk a pedestrian across.
        self.green = {"NS": max(g_ns, PED_MIN_GREEN), "EW": max(g_ew, PED_MIN_GREEN)}
        self.cycle = sum(self.green.values()) + 2 * (YELLOW + ALL_RED)

    def _phase(self, t: float) -> tuple[str, bool]:
        """Return the axis currently served and whether it is in steady green."""
        x = t % self.cycle
        g_ns = self.green["NS"]
        if x < g_ns:
            return "NS", True
        x -= g_ns
        if x < YELLOW + ALL_RED:
            return "NS", False
        x -= YELLOW + ALL_RED
        if x < self.green["EW"]:
            return "EW", True
        return "EW", False

    def may_enter(self, t, veh, node, axis):
        served, steady = self._phase(t)
        return steady and served == axis

    def ped_can_cross(self, t, node, axis):
        served, steady = self._phase(t)
        return steady and served == OTHER[axis]
