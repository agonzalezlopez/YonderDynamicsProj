"""
Live matplotlib visualizer — PROVIDED, do not modify.

Launched by:  python sim/launch.py --visualize
Picture only: python sim/launch.py --save-png out.png     (no window needed)

What you are looking at
-----------------------
  dark gray     UNKNOWN: the rover has never seen this cell
  white         FREE (in --noise mode, low-confidence cells fade toward gray)
  black         OCCUPIED (same fading in --noise mode)
  faint red     REAL obstacles the rover has not seen yet — the simulator's ground
                truth, which your planner never gets. Watch them get discovered.
  blue line     the path you last published, from the rover onward
  gray line     where the rover has actually driven
  ● red circle  the rover (drawn at its planning radius)
  dashed circle sensor range: the rover observes cells inside it
  ★ green star  the goal
  orange dots   places where you published a REPLAN     red ✕ = a needless one
"""

import math
import os

import numpy as np

from messages import UNKNOWN, OCCUPIED, FREE
from world import GOAL


class VisualizerUnavailable(Exception):
    """Raised when a live window can't be opened (e.g. tkinter isn't installed)."""


def _pick_backend(headless: bool) -> None:
    import matplotlib
    if headless:
        matplotlib.use("Agg", force=True)
        return
    if "MPLBACKEND" in os.environ:       # the user chose a backend; respect it
        return
    try:
        import tkinter  # noqa: F401   (matplotlib's TkAgg backend needs it)
    except ImportError:
        raise VisualizerUnavailable(
            "Can't open a live window: Python's tkinter is not installed.\n"
            "          On Ubuntu / WSL:  sudo apt install python3-tk\n"
            "          Or skip the window and use  --save-png out.png  to get a picture of the run."
        )
    matplotlib.use("TkAgg")


def _belief_image(sim) -> np.ndarray:
    snap = sim.snapshot
    W, H = snap.width, snap.height
    occ = np.array(snap.occupancy, dtype=int).reshape(H, W)
    conf = np.array(snap.confidence, dtype=float).reshape(H, W)
    img = np.zeros((H, W, 3))
    unknown = occ == UNKNOWN
    base = np.where(occ == OCCUPIED, 0.0, 1.0)
    fade = 0.6                                  # what low-confidence cells fade toward
    shade = base * conf + fade * (1.0 - conf) if sim.noise else base
    img[:] = shade[..., None]
    img[unknown] = 0.30
    return img


def draw(ax, sim) -> None:
    """Redraw the whole scene onto `ax`."""
    import matplotlib.patches as patches

    ax.clear()
    w = sim.world
    snap = sim.snapshot
    if snap is None:
        return
    x0, y0 = w.origin_x, w.origin_y
    x1, y1 = x0 + w.width * w.resolution, y0 + w.height * w.resolution
    ax.imshow(_belief_image(sim), origin="lower", extent=(x0, x1, y0, y1), interpolation="nearest", zorder=1)

    # Real obstacles the rover has not (yet) seen
    hidden = np.zeros((w.height, w.width, 4))
    truth = np.frombuffer(bytes(w.occupied), dtype=np.uint8).reshape(w.height, w.width)
    belief = np.array(snap.occupancy).reshape(w.height, w.width)
    hidden[(truth == 1) & (belief != OCCUPIED)] = (0.9, 0.1, 0.1, 0.35)
    ax.imshow(hidden, origin="lower", extent=(x0, x1, y0, y1), interpolation="nearest", zorder=2)

    trail = w._trail
    if len(trail) > 1:
        ax.plot([p[0] for p in trail], [p[1] for p in trail], color="#888888", lw=1.2, zorder=3)
    r = sim.rover
    if r.path:
        pts = r.path[r.remaining_from:]
        ax.plot([r.pos[0]] + [p[0] for p in pts], [r.pos[1]] + [p[1] for p in pts],
                color="#1f77b4", lw=2.0, zorder=4)
    for e in sim.score.events:
        if e.kind == "replan" and e.tick < len(trail):
            px, py = trail[e.tick]
            if e.needless or e.phantom:
                ax.plot(px, py, "x", color="red", ms=8, mew=2, zorder=6)
            else:
                ax.plot(px, py, "o", color="orange", ms=6, zorder=6)

    ax.add_patch(patches.Circle(r.pos, snap.sensor_range_m, fill=False, ls="--", lw=0.8, ec="#cc4444", zorder=5))
    ax.add_patch(patches.Circle(r.pos, snap.robot_radius_m, fc="#e63946", ec="white", lw=1.0, zorder=7))
    ax.plot(GOAL[0], GOAL[1], marker="*", color="#2a9d3f", ms=16, mec="black", zorder=7)
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")

    s = sim.score
    status = "" if not sim.done else f"   [{sim.outcome}]"
    ax.set_title(
        f"{w.name}   noise={'ON' if sim.noise else 'off'}   t={sim.tick * 0.2:.1f}s   tick {sim.tick}{status}\n"
        f"plans {s.plans}   replans {s.replans}   needless {s.needless}   blocked ticks {r.blocked_ticks}",
        fontsize=10,
    )


class Visualizer:
    def __init__(self, sim, speed: float = 1.0):
        _pick_backend(headless=False)
        import matplotlib.pyplot as plt
        self.plt = plt
        self.sim = sim
        self.speed = max(0.1, speed)
        self.fig, self.ax = plt.subplots(figsize=(8, 8.5))
        self.fig.canvas.manager.set_window_title("Yonder autonomous take-home")
        self.sim.step()
        self._draw()

    def _draw(self) -> None:
        draw(self.ax, self.sim)
        self.fig.canvas.draw_idle()

    def _update(self, _frame) -> None:
        if self.sim.done:
            self.anim.event_source.stop()
            return
        self.sim.step()
        self._draw()

    def run(self) -> None:
        import itertools
        import matplotlib.animation as animation
        self.anim = animation.FuncAnimation(
            self.fig, self._update, frames=itertools.count(), interval=200.0 / self.speed,
            cache_frame_data=False, repeat=False,
        )
        self.plt.show()
        # Window closed early: finish the run headlessly so the summary is still complete.
        self.sim.run()


def save_png(sim, path: str) -> None:
    """Draw the final state of a finished (or in-progress) run to an image file."""
    _pick_backend(headless=True)
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8, 8.5))
    draw(ax, sim)
    fig.savefig(path, dpi=110, bbox_inches="tight")
    plt.close(fig)
