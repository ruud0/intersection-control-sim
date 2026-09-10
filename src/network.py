"""Grid network for the intersection-control study.

Geometry mirrors the IE 496 SUMO network: two north-south avenues crossing
three east-west streets, giving six signalised intersections, sixteen nodes
and seventeen undirected street segments.

    W --+----+-- E      row 2   (y = 400)
        |    |
    W --+----+-- E      row 1   (y = 200)
        |    |
    W --+----+-- E      row 0   (y = 0)
        |    |
        S    S
      col 0  col 1

Each undirected segment becomes two directed links, one per travel direction.
Vehicles enter and leave at the ten boundary stubs.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

BLOCK = 200.0   # m between adjacent intersections
STUB = 250.0    # m of approach road outside the grid, for queue storage
N_COLS = 2      # north-south avenues
N_ROWS = 3      # east-west streets

# Heading -> (dx, dy). Used to classify turns.
HEADING_VEC = {"N": (0, 1), "S": (0, -1), "E": (1, 0), "W": (-1, 0)}
AXIS_OF = {"N": "NS", "S": "NS", "E": "EW", "W": "EW"}
# Clockwise order, so a +1 step is a right turn and a -1 step is a left turn.
_CW = ["N", "E", "S", "W"]


def turn_type(from_heading: str, to_heading: str) -> str:
    """Classify a movement from its heading change."""
    step = (_CW.index(to_heading) - _CW.index(from_heading)) % 4
    return {0: "through", 1: "right", 2: "uturn", 3: "left"}[step]


@dataclass
class Node:
    id: str
    x: float
    y: float
    is_signal: bool


@dataclass
class Link:
    """A one-way carriageway between two nodes."""
    id: str
    frm: str
    to: str
    length: float
    heading: str
    speed_limit: float = 13.9        # m/s, ~50 km/h urban arterial
    vehicles: deque = field(default_factory=deque)   # head of queue first

    @property
    def axis(self) -> str:
        return AXIS_OF[self.heading]


class Network:
    def __init__(self, block: float = BLOCK, stub: float = STUB):
        self.nodes: dict[str, Node] = {}
        self.links: dict[str, Link] = {}
        self.outgoing: dict[str, list[str]] = {}
        self.incoming: dict[str, list[str]] = {}
        self._build(block, stub)

    # ---------------------------------------------------------------- build
    def _add_node(self, nid, x, y, is_signal):
        self.nodes[nid] = Node(nid, x, y, is_signal)
        self.outgoing[nid] = []
        self.incoming[nid] = []

    def _add_segment(self, a: str, b: str, heading_ab: str):
        """Add both directed links for one undirected street segment."""
        opposite = {"N": "S", "S": "N", "E": "W", "W": "E"}[heading_ab]
        for frm, to, head in ((a, b, heading_ab), (b, a, opposite)):
            na, nb = self.nodes[frm], self.nodes[to]
            length = abs(na.x - nb.x) + abs(na.y - nb.y)
            lid = f"{frm}->{to}"
            self.links[lid] = Link(lid, frm, to, length, head)
            self.outgoing[frm].append(lid)
            self.incoming[to].append(lid)

    def _build(self, block, stub):
        xs = [c * block for c in range(N_COLS)]
        ys = [r * block for r in range(N_ROWS)]

        for c in range(N_COLS):
            for r in range(N_ROWS):
                self._add_node(f"I{c}{r}", xs[c], ys[r], is_signal=True)

        # Ten boundary stubs: two per avenue (south/north), two per street (west/east).
        for c in range(N_COLS):
            self._add_node(f"S{c}", xs[c], ys[0] - stub, False)
            self._add_node(f"N{c}", xs[c], ys[-1] + stub, False)
        for r in range(N_ROWS):
            self._add_node(f"W{r}", xs[0] - stub, ys[r], False)
            self._add_node(f"E{r}", xs[-1] + stub, ys[r], False)

        # Vertical segments (four per avenue).
        for c in range(N_COLS):
            self._add_segment(f"S{c}", f"I{c}0", "N")
            for r in range(N_ROWS - 1):
                self._add_segment(f"I{c}{r}", f"I{c}{r+1}", "N")
            self._add_segment(f"I{c}{N_ROWS-1}", f"N{c}", "N")

        # Horizontal segments (three per street).
        for r in range(N_ROWS):
            self._add_segment(f"W{r}", f"I0{r}", "E")
            for c in range(N_COLS - 1):
                self._add_segment(f"I{c}{r}", f"I{c+1}{r}", "E")
            self._add_segment(f"I{N_COLS-1}{r}", f"E{r}", "E")

    # ------------------------------------------------------------- queries
    @property
    def signals(self) -> list[str]:
        return [n for n, nd in self.nodes.items() if nd.is_signal]

    @property
    def entry_nodes(self) -> list[str]:
        return [n for n, nd in self.nodes.items() if not nd.is_signal]

    def route(self, origin: str, dest: str) -> list[str] | None:
        """Shortest link path between two boundary nodes (BFS; grid is uniform).

        U-turns are disallowed, which keeps routes realistic on a grid.
        """
        if origin == dest:
            return None
        queue = deque([(origin, [])])
        seen = {origin}
        while queue:
            node, path = queue.popleft()
            for lid in self.outgoing[node]:
                link = self.links[lid]
                if path and turn_type(self.links[path[-1]].heading, link.heading) == "uturn":
                    continue
                if link.to == dest:
                    return path + [lid]
                if link.to not in seen:
                    seen.add(link.to)
                    queue.append((link.to, path + [lid]))
        return None

    def describe(self) -> str:
        undirected = len(self.links) // 2
        total_km = sum(l.length for l in self.links.values()) / 2 / 1000
        return (f"{len(self.nodes)} nodes, {undirected} street segments "
                f"({len(self.links)} directed links), {len(self.signals)} signals, "
                f"{total_km:.2f} km of road")


if __name__ == "__main__":
    net = Network()
    print(net.describe())
    r = net.route("W1", "E1")
    print("W1 -> E1:", r)
