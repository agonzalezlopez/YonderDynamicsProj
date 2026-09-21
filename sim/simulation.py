"""
Simulation driver — PROVIDED, do not modify.

One call to step() is one tick (0.2 s of simulated time):

    1. scripted events fire (obstacles appear / disappear)
    2. the sensor updates the rover's belief grid
    3. the grid feed publishes a GridSnapshot        ← your on_grid callback runs HERE
    4. whatever path you published is handed to the rover
    5. the rover drives 0.3 m along it (or emergency-stops)

Everything happens in one thread in a fixed order, so a run is fully
deterministic: same scenario + seed + code = same result, every time. That is
also why the simulation is not tied to the wall clock — it runs as fast as your
planner lets it. Use the timestamps in the messages when you need "time".
"""

import math
import traceback

import rclpy_lite as rclpy
from rclpy_lite._bus import _Bus

from world import build_world, START, GOAL
from grid_feed_publisher import GridFeedPublisher, FEED_TOPIC
from rover import RoverFollower
from scoring import Scoreboard

DT = 0.2
MAX_TIME_S = 200.0
STALL_S = 30.0        # give up if the rover makes no progress for this long
GOAL_TOL_M = 0.5


class Simulation:
    def __init__(self, scenario: str, seed: int = 0, noise: bool = False):
        self.scenario = scenario
        self.seed = seed
        self.noise = noise
        self.world = build_world(scenario, seed)
        self.feed = GridFeedPublisher(self.world, DT, noise=noise, seed=seed)
        self.rover = RoverFollower(self.world, DT)
        self.score = Scoreboard(self.world)
        self.node = None
        self.tick = 0
        self.t = 0.0
        self.done = False
        self.outcome = ""          # REACHED | TIMEOUT | STALLED | NOT_IMPLEMENTED
        self.todo_error = None     # (message, "file:line") if a TODO was hit
        self.snapshot = None
        self._stalled_ticks = 0
        self.paths_seen = 0          # every /planned_path message the rover received (empty ones too)
        self.empty_paths_seen = 0
        self.edge_blocked_ticks = 0  # blocked ticks that happened in the outermost ring of cells

    def attach(self, node) -> None:
        self.node = node

    # ── main loop ───────────────────────────────────────────────────────────
    def step(self) -> None:
        if self.done:
            return
        self.t = self.tick * DT
        self.world.step_events(self.rover.pos, self.tick, DT)
        self.feed.observe(self.rover.pos, self.t)
        self.snapshot = self.feed.snapshot(self.rover.pos, self.t, self.tick)

        self.score.begin_tick(self.tick, self.t, self.snapshot, self.rover)
        try:
            self.feed.publish(self.snapshot)
        except NotImplementedError as exc:
            self._hit_todo(exc)
            return
        new_paths = self.rover.drain_new_paths()
        self.paths_seen += len(new_paths)
        self.empty_paths_seen += sum(1 for p in new_paths if not p)
        self.score.end_tick(new_paths)

        status = self.rover.step()
        if status == "blocked":
            r, c = self.world.cell_of(*self.rover.pos)
            if r in (0, self.world.height - 1) or c in (0, self.world.width - 1):
                self.edge_blocked_ticks += 1
        self.tick += 1

        if math.hypot(self.rover.pos[0] - GOAL[0], self.rover.pos[1] - GOAL[1]) <= GOAL_TOL_M:
            self._finish("REACHED")
        elif self.tick * DT >= MAX_TIME_S:
            self._finish("TIMEOUT")
        else:
            self._stalled_ticks = self._stalled_ticks + 1 if status != "moved" else 0
            if self._stalled_ticks * DT >= STALL_S:
                self._finish("STALLED")

    def run(self) -> None:
        while not self.done:
            self.step()

    def _finish(self, outcome: str) -> None:
        self.outcome = outcome
        self.done = True

    def _hit_todo(self, exc: NotImplementedError) -> None:
        tb = traceback.extract_tb(exc.__traceback__)
        # last frame that lives in the candidate's code (not the shim)
        where = "?"
        for fr in tb:
            if "rclpy_lite" not in fr.filename and "/sim/" not in fr.filename.replace("\\", "/"):
                where = f"{fr.filename.split('/')[-1]}:{fr.lineno}"
        self.todo_error = (str(exc) or "NotImplementedError", where)
        self._finish("NOT_IMPLEMENTED")

    # ── reporting ───────────────────────────────────────────────────────────
    def delivered_to_node(self) -> int:
        if self.node is None:
            return 0
        return _Bus.instance().delivered.get((FEED_TOPIC, self.node.get_name()), 0)

    def summary(self) -> str:
        r, s = self.rover, self.score
        straight = math.hypot(GOAL[0] - START[0], GOAL[1] - START[1])
        lines = []
        head = f"scenario={self.world.name}  noise={'ON' if self.noise else 'off'}"
        lines.append("=" * 72)
        lines.append(f" RUN SUMMARY   {head}")
        lines.append("=" * 72)

        if self.outcome == "NOT_IMPLEMENTED":
            msg, where = self.todo_error
            lines.append(f" Stopped: you haven't implemented this yet ({where}):")
            lines.append(f"          {msg}")
            return "\n".join(lines)

        result = {
            "REACHED": f"REACHED THE GOAL in {self.tick} ticks ({self.tick * DT:.1f} s simulated)",
            "TIMEOUT": f"TIMEOUT: goal not reached after {MAX_TIME_S:.0f} s",
            "STALLED": f"STALLED: rover made no progress for {STALL_S:.0f} s (gave up at t={self.tick * DT:.1f} s)",
        }[self.outcome]
        lines.append(f" Result             {result}")
        lines.append(f" Distance driven    {r.distance_driven:.1f} m   (straight line start to goal is {straight:.1f} m)")
        lines.append(f" Blocked ticks      {r.blocked_ticks}"
                     + ("   <- the rover had to emergency-stop; your path led into a real obstacle" if r.blocked_ticks else ""))
        if self.edge_blocked_ticks:
            lines.append(f"                    {self.edge_blocked_ticks} of those were in the outermost ring of cells: "
                         "the edge of the arena is a wall for the rover too (see the GridSnapshot docstring)")
        lines.append(f" Paths published    {len(s.events)}   ({s.plans} plans, {s.replans} of them replans)")
        lines.append(f" Needless replans   {s.needless}"
                     + ("   <- replaced a path that was still fine according to your grid" if s.needless else ""))
        if self.noise:
            lines.append(f" Phantom replans    {s.phantom}"
                         + ("   <- replanned because of something in your grid that wasn't really there" if s.phantom else ""))
        own = getattr(self.node, "replan_count", None) if self.node is not None else None
        if own is not None:
            lines.append(f" Your node reports  replan_count={own}   (the scoreboard counted {s.replans})")
        if s.events:
            lines.append("")
            lines.append(" Path events:")
            lines.append("   tick    t(s)  kind       waypoints  length(m)   note")
            for e in s.events:
                note = ""
                if e.needless:
                    note = "NEEDLESS"
                elif e.phantom:
                    note = "PHANTOM"
                lines.append(f"   {e.tick:>4}  {e.t:>6.1f}  {e.kind:<9}  {e.n_points:>9}  {e.length_m:>9.1f}   {note}")
        if self.world.event_log:
            lines.append("")
            lines.append(" World events: " + "; ".join(self.world.event_log))

        # Helpful diagnosis for the most common early problems
        if self.node is not None:
            subs = [x for x in getattr(self.node, "_subscriptions", []) if x.topic == FEED_TOPIC]
            got = self.delivered_to_node()
            crashed = _Bus.instance().callback_errors.get((FEED_TOPIC, self.node.get_name()), 0)
            problem = None
            if not subs:
                problem = ("Your node has no subscription to /grid_feed yet. "
                           "Create one in PlannerNode.__init__.")
            elif not any(x.connected for x in subs):
                problem = ("Your node subscribed to /grid_feed but was not connected: QoS mismatch. "
                           "The feed is BEST_EFFORT; see grid_feed_publisher.py.")
            elif got == 0:
                problem = "Your node is subscribed to /grid_feed but no messages arrived."
            elif crashed:
                problem = (f"Your on_grid callback raised an exception on {crashed} of {got} ticks. "
                           "Scroll up to the first traceback: that is the bug to fix first.")
            elif not s.events and self.empty_paths_seen:
                problem = (f"Your node published {self.empty_paths_seen} paths but every one was EMPTY, which means "
                           "'no route found, stop'. Print what your search returns on the first ticks to see why.")
            elif not s.events:
                problem = f"Your node received {got} grids but never published a path on /planned_path."
            if problem:
                lines.append("")
                lines.append(f" !! {problem}")
        else:
            lines.append("")
            lines.append(" !! No planner node was attached to this run (you used --no-node, or planner_node.py "
                         "failed to load: look for a message above), so nothing publishes a path and the rover never moves.")
        return "\n".join(lines)
