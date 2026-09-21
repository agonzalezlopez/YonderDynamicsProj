"""
Rover — PROVIDED, do not modify.

A minimal rover that drives along whatever path you last published on
/planned_path at a constant speed. There is no clever controller here on
purpose: the rover does exactly what your path says.

Rules of the road
-----------------
* When a new path arrives, the rover joins it at the nearest point (if that
  point is within SNAP_M of the rover; otherwise it heads straight for the
  first waypoint, which will probably end badly).
* It then visits the remaining waypoints in order, in straight lines.
* An EMPTY path means "stop".
* Before every move the simulator checks the TRUE world. If the move would bring
  the rover's body within BODY_RADIUS_M of a real obstacle, the rover
  emergency-stops instead and the tick is counted as "blocked". It will stay
  stopped until you publish a path that leads somewhere safe.

The rover never replans and never looks at the grid. If it drives into
trouble it is because the path you gave it led there.
"""

import math
from typing import List, Tuple

from rclpy_lite.node import Node
from rclpy_lite.qos import QoSProfile

from messages import path_msg_to_points
from nav_msgs.msg import Path
from world import World, BODY_RADIUS_M

PATH_TOPIC = "/planned_path"
SPEED_MPS = 1.5
SNAP_M = 0.75


class RoverFollower(Node):
    def __init__(self, world: World, dt: float):
        super().__init__("rover")
        self.world = world
        self.dt = dt
        self.pos: Tuple[float, float] = world.start
        self.path: List[Tuple[float, float]] = []
        self.target = 0                    # index of the waypoint we are driving towards
        self.new_paths: List[list] = []    # paths received since the scoreboard last looked
        self.distance_driven = 0.0
        self.blocked_ticks = 0
        self.moved_last_tick = False
        # RELIABLE (the default): compatible with any publisher QoS you choose.
        self.sub = self.create_subscription(Path, PATH_TOPIC, self._on_path, QoSProfile(depth=10))

    # ── receiving paths ─────────────────────────────────────────────────────
    def _on_path(self, msg: Path) -> None:
        pts = path_msg_to_points(msg)
        self.new_paths.append(pts)
        self.path = pts
        if not pts:
            self.target = 0
            return
        (px, py), seg_index, dist = _nearest_on_polyline(pts, self.pos)
        if dist <= SNAP_M:
            # Join the path, unless that would drop the rover closer to an obstacle than it is now.
            w = self.world
            if w.nearest_obstacle_dist((px, py)) >= min(BODY_RADIUS_M, w.nearest_obstacle_dist(self.pos)) - 1e-9:
                self.pos = (px, py)
            self.target = min(seg_index + 1, len(pts) - 1)
        else:
            self.target = 0

    def drain_new_paths(self) -> List[list]:
        out, self.new_paths = self.new_paths, []
        return out

    @property
    def remaining_from(self) -> int:
        """Index of the first waypoint that is still (roughly) ahead of the rover."""
        return max(self.target - 1, 0)

    # ── driving ─────────────────────────────────────────────────────────────
    def step(self) -> str:
        """Advance one tick. Returns 'moved', 'blocked' or 'idle'."""
        self.moved_last_tick = False
        if not self.path or self.target >= len(self.path):
            return "idle"
        budget = SPEED_MPS * self.dt
        status = "idle"
        while budget > 1e-9 and self.target < len(self.path):
            tx, ty = self.path[self.target]
            dx, dy = tx - self.pos[0], ty - self.pos[1]
            d = math.hypot(dx, dy)
            if d < 1e-9:
                self.target += 1
                continue
            step_len = min(d, budget)
            new = (self.pos[0] + dx / d * step_len, self.pos[1] + dy / d * step_len)
            if not self.world.segment_clear(self.pos, new):
                self.blocked_ticks += 1
                return "blocked"
            self.pos = new
            self.distance_driven += step_len
            budget -= step_len
            status = "moved"
            if step_len >= d - 1e-9:
                self.target += 1
        self.moved_last_tick = status == "moved"
        return status


def _nearest_on_polyline(pts, p):
    """Closest point to p on the polyline pts. Returns ((x, y), segment_index, distance)."""
    if len(pts) == 1:
        return pts[0], 0, math.hypot(p[0] - pts[0][0], p[1] - pts[0][1])
    best = (None, 0, float("inf"))
    for i in range(len(pts) - 1):
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        vx, vy = bx - ax, by - ay
        L2 = vx * vx + vy * vy
        t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((p[0] - ax) * vx + (p[1] - ay) * vy) / L2))
        qx, qy = ax + t * vx, ay + t * vy
        d = math.hypot(p[0] - qx, p[1] - qy)
        if d < best[2] - 1e-12:
            best = ((qx, qy), i, d)
    return best
