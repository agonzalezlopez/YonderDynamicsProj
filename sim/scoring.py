"""
Scoreboard — PROVIDED, do not modify.

Watches the run from the outside and reports what your planner actually did.
It does not look inside your code — only at the paths you publish, judged
against the grid you were given and against the true world.

What it counts
--------------
Every path message you publish is one "plan event", classified by what state the
rover was in when it arrived:

    initial     the first non-empty path you published
    replan      a new path replacing a non-empty one that the rover was following
    recovery    a non-empty path published after an empty one ("no route" → route)
    stop        an empty path replacing a non-empty one

A replan is NEEDLESS if the path being replaced was still fine according to the
grid you had just been given. The old path is NOT fine (so replacing it is not
needless) if, looking only at the waypoints the rover has yet to reach, EITHER:

  * a cell has become MORE occupied since that path was planned, within
    THREAT_CELLS cells of a waypoint. ("More occupied" means: free/unknown ->
    occupied, or occupied with low confidence -> occupied with confidence at or
    above the threshold.) Cells that were already occupied when you planned don't
    count by this rule (the path was built to avoid them), and neither do changes
    behind the rover or far off to the side; or
  * a waypoint is sitting on, or directly next to (up/down/left/right), a cell that
    is occupied with confidence at or above the threshold. Any path that has been
    properly planned around obstacles never does this.

The margins are deliberately generous so that any sensible safety margin is never
flagged.

With --noise there is a second kind of wasted replan, a PHANTOM replan: your grid
showed a new obstacle near the old path but, in the true world, nothing was there.

`blocked_ticks` is the outcome that matters most: it counts ticks where the rover
had to emergency-stop because your path led into a real obstacle.
"""

from dataclasses import dataclass
from typing import List

from messages import GridSnapshot, OCCUPIED

THREAT_CELLS = 2   # Chebyshev distance in cells (1.0 m at 0.5 m resolution: twice the planning radius)


@dataclass
class PlanEvent:
    tick: int
    t: float
    kind: str            # initial | replan | recovery | stop
    n_points: int
    length_m: float
    needless: bool = False
    phantom: bool = False


class Scoreboard:
    def __init__(self, world):
        self.world = world
        self.events: List[PlanEvent] = []
        self._had_plan = False
        self._prev_nonempty = False
        self._plan_occ = None       # per cell at the moment the current path was planned: 0 free/unknown, 1 occupied (unreliable), 2 occupied (reliable)
        self._snap = None
        # state captured by begin_tick
        self._threat_belief = False
        self._threat_truth = False
        self._tick = 0
        self._t = 0.0

    # ── called by the simulation ────────────────────────────────────────────
    def begin_tick(self, tick, t, snap: GridSnapshot, rover) -> None:
        """Judge the path the rover is currently following against this tick's grid."""
        self._tick, self._t = tick, t
        self._snap = snap
        self._threat_belief = self._threat_truth = False
        if not rover.path or self._plan_occ is None:
            return
        pts = rover.path[rover.remaining_from:]
        W = snap.width
        now, before, w = self._states(snap), self._plan_occ, self.world
        is_new = lambda r, c: now[r * W + c] > before[r * W + c]
        reliable = lambda r, c: now[r * W + c] == 2
        real = lambda r, c: bool(w.occupied[r * W + c])
        self._threat_belief = self._near(pts, is_new, snap) or self._on_or_beside(pts, reliable, snap)
        # Is at least one of those threatening cells actually there? If not, the threat was a phantom.
        self._threat_truth = (self._near(pts, lambda r, c: is_new(r, c) and real(r, c), snap)
                              or self._on_or_beside(pts, lambda r, c: reliable(r, c) and real(r, c), snap))

    def end_tick(self, new_paths: List[list]) -> None:
        for pts in new_paths:
            nonempty = len(pts) > 0
            length = sum(
                ((pts[i + 1][0] - pts[i][0]) ** 2 + (pts[i + 1][1] - pts[i][1]) ** 2) ** 0.5
                for i in range(len(pts) - 1)
            )
            ev = PlanEvent(self._tick, self._t, "", len(pts), length)
            if nonempty and not self._had_plan:
                ev.kind = "initial"
            elif nonempty and self._prev_nonempty:
                ev.kind = "replan"
                ev.needless = not self._threat_belief
                ev.phantom = self._threat_belief and not self._threat_truth
            elif nonempty:
                ev.kind = "recovery"
            elif self._prev_nonempty:
                ev.kind = "stop"
            else:
                continue   # empty path replacing an empty path: nothing changed
            if nonempty:
                self._had_plan = True
                self._plan_occ = self._states(self._snap)
            self._prev_nonempty = nonempty
            self.events.append(ev)

    # ── helpers ─────────────────────────────────────────────────────────────
    @staticmethod
    def _states(snap) -> bytearray:
        thr = snap.confidence_threshold
        return bytearray(
            0 if o != OCCUPIED else (2 if c >= thr else 1)
            for o, c in zip(snap.occupancy, snap.confidence)
        )

    @staticmethod
    def _near(points, occupied_at, snap) -> bool:
        k = THREAT_CELLS
        for x, y in points:
            r0 = int((y - snap.origin_y) // snap.resolution)
            c0 = int((x - snap.origin_x) // snap.resolution)
            for r in range(max(0, r0 - k), min(snap.height, r0 + k + 1)):
                for c in range(max(0, c0 - k), min(snap.width, c0 + k + 1)):
                    if occupied_at(r, c):
                        return True
        return False

    @staticmethod
    def _on_or_beside(points, occupied_at, snap) -> bool:
        """Is any waypoint's cell, or one of its 4 neighbours, a cell for which occupied_at is true?"""
        for x, y in points:
            r0 = int((y - snap.origin_y) // snap.resolution)
            c0 = int((x - snap.origin_x) // snap.resolution)
            for r, c in ((r0, c0), (r0 + 1, c0), (r0 - 1, c0), (r0, c0 + 1), (r0, c0 - 1)):
                if 0 <= r < snap.height and 0 <= c < snap.width and occupied_at(r, c):
                    return True
        return False

    # ── reporting ───────────────────────────────────────────────────────────
    @property
    def plans(self) -> int:
        return sum(1 for e in self.events if e.kind in ("initial", "replan", "recovery"))

    @property
    def replans(self) -> int:
        return sum(1 for e in self.events if e.kind == "replan")

    @property
    def needless(self) -> int:
        return sum(1 for e in self.events if e.needless)

    @property
    def phantom(self) -> int:
        return sum(1 for e in self.events if e.phantom)
