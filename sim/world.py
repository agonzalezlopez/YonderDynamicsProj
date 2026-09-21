"""
The simulated world — ground truth that only the simulator knows.

Your planner never sees this. It only sees the GridSnapshot the sensor model
builds from it (see grid_feed_publisher.py). Reading this file is allowed and
useful for understanding what the scenarios do, but nothing here is something
you need to change.

Geometry
--------
A 30 m × 30 m arena, 0.5 m cells, 60 × 60 cells. The arena's lower-left corner
is at world (-15, -15) — NOT (0, 0) — so cell (0, 0) is not at the world origin.

Scenarios
---------
    open          empty arena
    boulders      static boulders scattered around, some on the direct route
    wall          a long wall between rover and goal with a gap at one end
    popup         obstacles that appear while the rover is driving
    goal-blocked  the goal itself gets blocked for a while, then clears
    random        a generated world, controlled by --seed

Everything is deterministic: the same scenario + seed always builds the same world.
"""

import math
import random
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

RESOLUTION = 0.5
WIDTH = 60
HEIGHT = 60
ORIGIN_X = -15.0
ORIGIN_Y = -15.0

START = (-13.2, -12.7)
GOAL = (12.8, 13.3)

# The rover's physical body. The simulator emergency-stops the rover if its
# centre gets closer than this to an obstacle. (GridSnapshot.robot_radius_m is
# bigger: that's the planning margin you are given.)
BODY_RADIUS_M = 0.3

SCENARIOS = ["open", "boulders", "wall", "popup", "goal-blocked", "random"]


def route(t: float) -> Tuple[float, float]:
    """A point a fraction t of the way along the straight line start → goal."""
    return (START[0] + t * (GOAL[0] - START[0]), START[1] + t * (GOAL[1] - START[1]))


@dataclass
class Event:
    """
    Something that changes the world while the rover is driving.

    mode "near":  fires the first time the rover comes within `radius` of `at`.
    mode "trail": fires once the rover has driven `radius` metres in total, and drops a
                  box `back_m` metres behind it along the route it actually drove
                  (so it lands on ground the rover has already covered).

    Fires at most once. If `remove_after_s` is set, the obstacle disappears again
    that many simulated seconds later.
    """
    name: str
    mode: str
    at: Tuple[float, float]
    radius: float
    cells: List[Tuple[int, int]]
    remove_after_s: Optional[float] = None
    fired: bool = False
    remove_at_tick: Optional[int] = None
    back_m: float = 0.0                       # "trail" mode only
    box: Tuple[float, float] = (1.5, 1.5)     # "trail" mode only: width, height in metres


class World:
    def __init__(self, name: str, description: str = ""):
        self.name = name
        self.description = description
        self.resolution = RESOLUTION
        self.width = WIDTH
        self.height = HEIGHT
        self.origin_x = ORIGIN_X
        self.origin_y = ORIGIN_Y
        self.start = START
        self.goal = GOAL
        self.occupied = bytearray(WIDTH * HEIGHT)
        self.events: List[Event] = []
        # (t_start_s, t_end_s) windows where the sensor is blind. Only used with --noise.
        self.dropouts: List[Tuple[float, float]] = []
        self.event_log: List[str] = []
        self._trail: List[Tuple[float, float]] = []   # where the rover has been, one point per tick
        self._driven = 0.0

    # ── coordinates ─────────────────────────────────────────────────────────
    def cell_of(self, x: float, y: float) -> Tuple[int, int]:
        """(row, col) of the cell containing world point (x, y)."""
        return (
            math.floor((y - self.origin_y) / self.resolution),
            math.floor((x - self.origin_x) / self.resolution),
        )

    def center_of(self, row: int, col: int) -> Tuple[float, float]:
        return (
            self.origin_x + (col + 0.5) * self.resolution,
            self.origin_y + (row + 0.5) * self.resolution,
        )

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.height and 0 <= col < self.width

    def is_occupied(self, row: int, col: int) -> bool:
        """Out-of-bounds counts as occupied: the rover may not leave the arena."""
        if not self.in_bounds(row, col):
            return True
        return bool(self.occupied[row * self.width + col])

    # ── building ────────────────────────────────────────────────────────────
    def cells_in_rect(self, x0, y0, x1, y1) -> List[Tuple[int, int]]:
        """Cells whose CENTRE lies inside the world rectangle [x0,x1] × [y0,y1]."""
        out = []
        for r in range(self.height):
            for c in range(self.width):
                cx, cy = self.center_of(r, c)
                if x0 <= cx <= x1 and y0 <= cy <= y1:
                    out.append((r, c))
        return out

    def add_rect(self, x0, y0, x1, y1) -> None:
        for r, c in self.cells_in_rect(x0, y0, x1, y1):
            self.occupied[r * self.width + c] = 1

    def add_box(self, cx, cy, w, h) -> None:
        """Boulder of size w × h metres centred on (cx, cy)."""
        self.add_rect(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)

    def add_event(self, ev: Event) -> None:
        self.events.append(ev)

    def event_box(self, name, mode, at, radius, box_center, w, h, remove_after_s=None) -> None:
        cx, cy = box_center
        cells = self.cells_in_rect(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)
        self.add_event(Event(name, mode, at, radius, cells, remove_after_s))

    def event_behind(self, name, driven_m, back_m, w=1.5, h=1.5) -> None:
        """An obstacle that drops onto the rover's own trail `back_m` behind it, once it has driven `driven_m`."""
        self.add_event(Event(name, "trail", START, driven_m, [], back_m=back_m, box=(w, h)))

    # ── dynamics ────────────────────────────────────────────────────────────
    def step_events(self, rover_xy: Tuple[float, float], tick: int, dt: float) -> None:
        """Fire due events. Called once per tick, before the sensor looks at the world."""
        if self._trail:
            self._driven += math.hypot(rover_xy[0] - self._trail[-1][0], rover_xy[1] - self._trail[-1][1])
        self._trail.append(rover_xy)
        for ev in self.events:
            if ev.fired:
                if ev.remove_at_tick is not None and tick >= ev.remove_at_tick:
                    for r, c in ev.cells:
                        self.occupied[r * self.width + c] = 0
                    ev.remove_at_tick = None
                    self.event_log.append(f"tick {tick}: '{ev.name}' cleared")
                continue
            if ev.mode == "trail":
                if self._driven < ev.radius:
                    continue
                cx, cy = self._trail_point_back(ev.back_m)
                ev.cells = self.cells_in_rect(cx - ev.box[0] / 2, cy - ev.box[1] / 2,
                                              cx + ev.box[0] / 2, cy + ev.box[1] / 2)
            else:
                d = math.hypot(rover_xy[0] - ev.at[0], rover_xy[1] - ev.at[1])
                if d > ev.radius:
                    continue
            # Never drop an obstacle on top of the rover.
            if self._min_dist_to_cells(rover_xy, ev.cells) < 2.5:
                continue
            for r, c in ev.cells:
                self.occupied[r * self.width + c] = 1
            ev.fired = True
            if ev.remove_after_s is not None:
                ev.remove_at_tick = tick + int(round(ev.remove_after_s / dt))
            self.event_log.append(f"tick {tick}: '{ev.name}' appeared")

    def _trail_point_back(self, back_m: float) -> Tuple[float, float]:
        """The point on the rover's trail `back_m` metres of driving ago."""
        remaining = back_m
        for i in range(len(self._trail) - 1, 0, -1):
            a, b = self._trail[i], self._trail[i - 1]
            seg = math.hypot(a[0] - b[0], a[1] - b[1])
            if seg >= remaining and seg > 0:
                f = remaining / seg
                return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]))
            remaining -= seg
        return self._trail[0]

    def _min_dist_to_cells(self, p, cells) -> float:
        best = float("inf")
        for r, c in cells:
            best = min(best, self._dist_to_cell(p, r, c))
        return best

    def _dist_to_cell(self, p, r, c) -> float:
        """Distance from point p to the nearest point of cell (r, c)'s square."""
        x0 = self.origin_x + c * self.resolution
        y0 = self.origin_y + r * self.resolution
        dx = max(x0 - p[0], 0.0, p[0] - (x0 + self.resolution))
        dy = max(y0 - p[1], 0.0, p[1] - (y0 + self.resolution))
        return math.hypot(dx, dy)

    # ── collision ───────────────────────────────────────────────────────────
    def nearest_obstacle_dist(self, p, search_cells: int = 2) -> float:
        """Distance from p to the nearest occupied cell (or arena wall), capped by the search window."""
        r0, c0 = self.cell_of(*p)
        best = float("inf")
        for r in range(r0 - search_cells, r0 + search_cells + 1):
            for c in range(c0 - search_cells, c0 + search_cells + 1):
                if self.is_occupied(r, c):
                    best = min(best, self._dist_to_cell(p, r, c))
        return best

    def point_clear(self, p, clearance: float = BODY_RADIUS_M) -> bool:
        return self.nearest_obstacle_dist(p) >= clearance - 1e-9

    def segment_clear(self, a, b, clearance: float = BODY_RADIUS_M, step: float = 0.05) -> bool:
        """
        True if a rover body of radius `clearance` can slide from a to b without touching anything.

        If the rover is ALREADY closer than `clearance` (it was dropped there by an earlier
        mishap), it may still drive away or along the obstacle: the move only has to not get
        closer than where it started.
        """
        floor = min(clearance, self.nearest_obstacle_dist(a))
        n = max(1, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / step)))
        for i in range(1, n + 1):
            t = i / n
            if not self.point_clear((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])), floor):
                return False
        return True

    def free_path_exists(self) -> bool:
        """
        Is the goal reachable in the final world (all events applied) by a rover
        that keeps a 1-cell margin from obstacles? Used to keep random worlds fair.
        """
        blocked = bytearray(self.occupied)
        for ev in self.events:
            for r, c in ev.cells:
                blocked[r * self.width + c] = 1
        W, H = self.width, self.height
        inflated = bytearray(W * H)
        for r in range(H):
            for c in range(W):
                if blocked[r * W + c]:
                    for dr in (-1, 0, 1):
                        for dc in (-1, 0, 1):
                            rr, cc = r + dr, c + dc
                            if 0 <= rr < H and 0 <= cc < W:
                                inflated[rr * W + cc] = 1
        s = self.cell_of(*self.start)
        g = self.cell_of(*self.goal)
        if inflated[s[0] * W + s[1]] or inflated[g[0] * W + g[1]]:
            return False
        seen = {s}
        q = deque([s])
        while q:
            r, c = q.popleft()
            if (r, c) == g:
                return True
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                rr, cc = r + dr, c + dc
                if not (0 <= rr < H and 0 <= cc < W) or (rr, cc) in seen or inflated[rr * W + cc]:
                    continue
                if dr and dc and (inflated[r * W + cc] or inflated[rr * W + c]):
                    continue
                seen.add((rr, cc))
                q.append((rr, cc))
        return False


# ─────────────────────────────────────────────────────────────────────────────
# Scenarios
# ─────────────────────────────────────────────────────────────────────────────

def build_world(name: str, seed: int = 0) -> World:
    if name == "open":
        return _open()
    if name == "boulders":
        return _boulders()
    if name == "wall":
        return _wall()
    if name == "popup":
        return _popup()
    if name == "goal-blocked":
        return _goal_blocked()
    if name == "random":
        return _random(seed)
    raise ValueError(f"unknown scenario '{name}'. Choose from: {', '.join(SCENARIOS)}")


def _open() -> World:
    w = World("open", "Empty arena. The straight line to the goal is clear.")
    w.dropouts = [(4.0, 7.0)]
    return w


def _boulders() -> World:
    w = World("boulders", "Static boulders. Several sit on the direct route, and none are known at the start.")
    for t, size in ((0.22, 1.5), (0.42, 2.0), (0.62, 1.5), (0.83, 2.0)):
        cx, cy = route(t)
        w.add_box(cx, cy, size, size)
    for cx, cy, sw, sh in (
        (-9.0, -2.0, 2.0, 1.5), (-3.0, -9.0, 1.5, 2.5), (3.0, 9.0, 2.5, 1.5),
        (9.0, 2.0, 1.5, 2.0), (-5.0, 5.0, 2.0, 2.0), (10.0, -7.0, 2.0, 2.0),
    ):
        w.add_box(cx, cy, sw, sh)
    w.dropouts = [(6.0, 9.0)]
    return w


def _wall() -> World:
    w = World("wall", "A long wall across the arena. The only way round is the gap at the top.")
    w.add_rect(-0.5, -15.0, 0.5, 8.0)
    w.add_box(-8.0, 3.0, 2.0, 2.0)
    w.add_box(7.0, -4.0, 2.0, 2.0)
    w.dropouts = [(3.0, 6.0)]
    return w


def _popup() -> World:
    w = World("popup", "Obstacles appear while the rover is driving: ahead of it, and once, behind it.")
    w.add_box(*route(0.50), 1.5, 1.5)
    w.add_box(-6.0, 4.0, 2.0, 2.0)
    w.add_box(8.0, -5.0, 2.0, 2.0)
    # Lands on the route ahead of the rover, well inside sensor range when it appears.
    w.event_box("rock ahead #1", "near", route(0.30), 6.0, route(0.30), 2.0, 1.5)
    # Lands BEHIND the rover, on the trail it has already driven. Nothing ahead changed.
    w.event_behind("rock behind", driven_m=24.0, back_m=4.0)
    w.event_box("rock ahead #2", "near", route(0.90), 5.5, route(0.90), 2.0, 2.0)
    return w


def _goal_blocked() -> World:
    w = World("goal-blocked", "Something blocks the goal for 8 seconds when the rover gets close, then clears.")
    w.add_box(*route(0.35), 1.5, 1.5)
    w.add_box(4.0, 10.0, 2.0, 2.0)
    w.event_box("goal blocked", "near", GOAL, 7.0, GOAL, 3.0, 3.0, remove_after_s=8.0)
    return w


def _random(seed: int) -> World:
    for attempt in range(200):
        rng = random.Random(seed * 1000 + attempt)
        w = World(f"random(seed={seed})", "Generated world: boulders, walls with gaps, and pop-up obstacles.")

        def clear_of_endpoints(cx, cy, margin):
            return (math.hypot(cx - START[0], cy - START[1]) > margin
                    and math.hypot(cx - GOAL[0], cy - GOAL[1]) > margin)

        for _ in range(rng.randint(16, 26)):
            cx, cy = rng.uniform(-13.5, 13.5), rng.uniform(-13.5, 13.5)
            sw, sh = rng.uniform(1.0, 2.5), rng.uniform(1.0, 2.5)
            if clear_of_endpoints(cx, cy, 4.0):
                w.add_box(cx, cy, sw, sh)

        for _ in range(rng.randint(1, 2)):
            length = rng.uniform(9.0, 16.0)
            gap = rng.uniform(3.0, 5.0)
            if rng.random() < 0.5:   # horizontal wall with a gap
                y = rng.uniform(-8.0, 8.0)
                x0 = rng.uniform(-14.0, 14.0 - length)
                gx = rng.uniform(x0 + 1.0, x0 + length - gap - 1.0)
                for xa, xb in ((x0, gx), (gx + gap, x0 + length)):
                    if clear_of_endpoints((xa + xb) / 2, y, 3.0):
                        w.add_rect(xa, y - 0.25, xb, y + 0.25)
            else:                    # vertical wall with a gap
                x = rng.uniform(-8.0, 8.0)
                y0 = rng.uniform(-14.0, 14.0 - length)
                gy = rng.uniform(y0 + 1.0, y0 + length - gap - 1.0)
                for ya, yb in ((y0, gy), (gy + gap, y0 + length)):
                    if clear_of_endpoints(x, (ya + yb) / 2, 3.0):
                        w.add_rect(x - 0.25, ya, x + 0.25, yb)

        for i in range(rng.randint(2, 3)):
            t = rng.uniform(0.25, 0.85)
            cx, cy = route(t)
            cx += rng.uniform(-1.0, 1.0)
            cy += rng.uniform(-1.0, 1.0)
            w.event_box(f"rock ahead #{i + 1}", "near", (cx, cy), rng.uniform(5.0, 6.5),
                        (cx, cy), rng.uniform(1.5, 2.5), rng.uniform(1.5, 2.5))
        if rng.random() < 0.6:
            w.event_behind("rock behind", driven_m=rng.uniform(10.0, 18.0), back_m=3.5)

        # Keep the start and goal areas clear of the initial obstacles.
        for pt in (START, GOAL):
            r0, c0 = w.cell_of(*pt)
            for r in range(r0 - 3, r0 + 4):
                for c in range(c0 - 3, c0 + 4):
                    if w.in_bounds(r, c):
                        w.occupied[r * w.width + c] = 0
        for ev in w.events:
            ev.cells = [(r, c) for r, c in ev.cells
                        if math.hypot(w.center_of(r, c)[0] - GOAL[0], w.center_of(r, c)[1] - GOAL[1]) > 2.0
                        and math.hypot(w.center_of(r, c)[0] - START[0], w.center_of(r, c)[1] - START[1]) > 2.0]
        if w.free_path_exists():
            return w
    raise RuntimeError(f"could not generate a solvable random world for seed {seed}")
