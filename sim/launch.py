"""
Launch script — entry point for the entire simulation.

    python sim/launch.py                            # boulders scenario, headless, prints a summary
    python sim/launch.py --visualize                # same, with a live matplotlib window
    python sim/launch.py --scenario popup           # pick a scenario (see below)
    python sim/launch.py --scenario random --seed 7 # a generated world
    python sim/launch.py --noise                    # Part 2: noisy sensor + confidence
    python sim/launch.py --all                      # every scenario, one summary table
    python sim/launch.py --no-node                  # simulator only, no planner_node.py

Scenarios: open, boulders, wall, popup, goal-blocked, random

The script injects sim/ into sys.path so that:
  - `import rclpy_lite` resolves to sim/rclpy_lite/ (our ROS shim)
  - `from nav_msgs.msg import Path` resolves to sim/nav_msgs/
  - etc.
No ROS installation required.
"""

import argparse
import os
import sys

# ── Path injection ──────────────────────────────────────────────────────────
_SIM_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _SIM_DIR)
_PROJECT_ROOT = os.path.dirname(_SIM_DIR)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)
# ────────────────────────────────────────────────────────────────────────────

# Some terminals and redirects (older Windows consoles, `> file.txt`) can't print every character.
# A stray symbol in a print() must never crash a run, so replace what can't be encoded.
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(errors="replace")
    except (AttributeError, ValueError):
        pass

import rclpy_lite as rclpy
from world import SCENARIOS
from simulation import Simulation, DT

FIXED_SCENARIOS = [s for s in SCENARIOS if s != "random"]


def parse_args():
    p = argparse.ArgumentParser(description="Yonder autonomous take-home simulator")
    p.add_argument("--scenario", default="boulders", choices=SCENARIOS,
                   help="which world to drive in (default: boulders)")
    p.add_argument("--seed", type=int, default=0,
                   help="seed for the 'random' scenario and for --noise")
    p.add_argument("--noise", action="store_true",
                   help="Part 2: noisy sensor. Confidence now matters (see README)")
    p.add_argument("--visualize", action="store_true",
                   help="open a live matplotlib window")
    p.add_argument("--speed", type=float, default=1.0,
                   help="with --visualize: playback speed multiplier (default 1.0 = real time)")
    p.add_argument("--save-png", metavar="FILE",
                   help="run headless, then save a picture of the final state (handy if you have no display)")
    p.add_argument("--all", action="store_true",
                   help="run every scenario (plus random seeds 1-3) and print one table")
    p.add_argument("--no-node", action="store_true",
                   help="run the simulator only, skip loading planner_node.py")
    return p.parse_args()


def load_candidate_node():
    """Import and instantiate the candidate's PlannerNode."""
    try:
        from planner_node import PlannerNode  # type: ignore
        return PlannerNode()
    except NotImplementedError as exc:
        print(f"[launch] planner_node.py: not implemented yet: {exc}")
        return None
    except ImportError as exc:
        print(f"[launch] Warning: could not import planner_node.py: {exc}")
        return None
    except Exception as exc:
        print(f"[launch] Error initialising PlannerNode: {exc}")
        import traceback
        traceback.print_exc()
        return None


def make_sim(scenario, seed, noise, with_node=True):
    rclpy.init()                       # fresh message bus for every run
    sim = Simulation(scenario, seed=seed, noise=noise)
    if with_node:
        node = load_candidate_node()   # created AFTER the feed so QoS is checked against it
        sim.attach(node)
    return sim


def run_all(args) -> None:
    runs = [(s, 0) for s in FIXED_SCENARIOS] + [("random", s) for s in (1, 2, 3)]
    rows = []
    for scenario, seed in runs:
        sim = make_sim(scenario, seed, args.noise, with_node=not args.no_node)
        sim.run()
        s = sim.score
        label = scenario if scenario != "random" else f"random {seed}"
        rows.append((label, sim.outcome, sim.tick, sim.rover.blocked_ticks, s.plans, s.replans, s.needless, s.phantom))
        rclpy.shutdown()
    print()
    print(f"{'scenario':<14}{'result':<16}{'ticks':>6}{'blocked':>9}{'plans':>7}{'replans':>9}{'needless':>10}" + (f"{'phantom':>9}" if args.noise else ""))
    print("-" * (71 + (9 if args.noise else 0)))
    for label, outcome, ticks, blocked, plans, replans, needless, phantom in rows:
        print(f"{label:<14}{outcome:<16}{ticks:>6}{blocked:>9}{plans:>7}{replans:>9}{needless:>10}" + (f"{phantom:>9}" if args.noise else ""))
    print()


def main() -> None:
    args = parse_args()

    if args.all:
        run_all(args)
        return

    sim = make_sim(args.scenario, args.seed, args.noise, with_node=not args.no_node)
    print(f"[launch] Scenario '{sim.world.name}': {sim.world.description}")
    print(f"[launch] Sensor: {'NOISY (--noise)' if args.noise else 'clean'}   tick = {DT} s of simulated time")
    if sim.node is None and not args.no_node:
        print("[launch] Continuing without a planner node. Implement PlannerNode in planner_node.py.")
    print()

    if args.visualize:
        try:
            from visualizer import Visualizer, VisualizerUnavailable
            Visualizer(sim, speed=args.speed).run()
        except ImportError as exc:
            print(f"[launch] Can't draw: {exc}\n          Install numpy and matplotlib: see 'Running it' in the README (Option A: apt packages, Option B: virtual environment).")
            sim.run()
        except VisualizerUnavailable as exc:
            print(f"[launch] {exc}")
            sim.run()
    else:
        sim.run()
        if args.save_png:
            try:
                from visualizer import save_png
                save_png(sim, args.save_png)
                print(f"[launch] Saved final-state picture to {args.save_png}")
            except ImportError as exc:
                print(f"[launch] Can't draw: {exc}\n          Install numpy and matplotlib: see 'Running it' in the README (Option A: apt packages, Option B: virtual environment).")

    print(sim.summary())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
