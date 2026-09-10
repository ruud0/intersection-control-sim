"""Shared controller interface and pedestrian timing constants."""

from __future__ import annotations

# Pedestrian crossing timing, following MUTCD practice.
CROSSING_WIDTH = 12.0     # m of roadway to cross
WALK_SPEED = 1.07         # m/s, the 3.5 ft/s design walking speed
WALK_INTERVAL = 4.0       # s of steady WALK before the clearance interval
PED_CLEARANCE = CROSSING_WIDTH / WALK_SPEED          # ~11.2 s
PED_MIN_GREEN = WALK_INTERVAL + PED_CLEARANCE        # ~15.2 s
MAX_PED_WAIT = 60.0       # s before a pedestrian call is forced through

YELLOW = 3.0
ALL_RED = 1.0
LOST_TIME = 4.0           # s of startup lost time per phase

OTHER = {"NS": "EW", "EW": "NS"}


class Controller:
    """Interface every control strategy implements.

    The simulator asks three questions each tick: may this vehicle enter the
    intersection, what speed should it aim for, and may this pedestrian cross.
    """

    name = "base"

    def __init__(self, net):
        self.net = net

    def step(self, t: float, dt: float, sim) -> None:
        """Advance internal control state one tick."""

    def may_enter(self, t: float, veh, node: str, axis: str) -> bool:
        raise NotImplementedError

    def advisory(self, t: float, veh, node: str, dist: float):
        """Optional speed advisory in m/s; None means no guidance."""
        return None

    def ped_can_cross(self, t: float, node: str, axis: str) -> bool:
        """`axis` is the roadway being crossed, which conflicts with that axis."""
        raise NotImplementedError
