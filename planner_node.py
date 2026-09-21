"""
Planner node — your implementation goes here.

Run via:
    python sim/launch.py                       # default scenario
    python sim/launch.py --scenario open       # easiest world — start here
    python sim/launch.py --visualize           # with a live plot
    python sim/launch.py --all                 # every scenario, one summary table

This file uses rclpy_lite, a lightweight simulator shim that mirrors the real
rclpy (ROS 2 Python) API. The class names, method signatures, and message types
are identical to real ROS 2 — only the import paths differ:

    Real ROS 2:     import rclpy / from rclpy.node import Node
    This shim:      import rclpy_lite as rclpy / from rclpy_lite.node import Node

To run on a real ROS 2 system, swap the imports back to plain rclpy.

Division of labour
------------------
    grid_utils.py   grid maths (world <-> cell, inflating obstacles)     no ROS
    search.py       A*                                                    no ROS
    replan.py       "is my plan still valid? where am I on it?"           no ROS
    planner_node.py THIS FILE: receives grids, decides when to replan,
                    publishes paths, prints monitoring output              ROS glue

Get the three pure files passing their tests first. Then this file is mostly wiring.
"""

import rclpy_lite as rclpy
from rclpy_lite.node import Node
from rclpy_lite.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Message types provided by the simulator
from sim.messages import GridSnapshot, make_path_msg

# Standard ROS message type (mirrored by the shim)
from nav_msgs.msg import Path

from grid_utils import Grid, inflate, path_length_m
from search import astar
from replan import next_waypoint_index, path_is_valid


class PlannerNode(Node):
    """
    Plans a path from the rover to the goal and keeps it up to date as the map changes.

    Subscribes:
        /grid_feed     (sim.messages.GridSnapshot)   5 Hz, BEST_EFFORT
                       occupancy grid + rover position + goal, all in one message

    Publishes:
        /planned_path  (nav_msgs/Path)               only when the plan changes
                       waypoints in WORLD coordinates; an EMPTY path means "stop"
    """

    def __init__(self):
        super().__init__("planner_node")

        # ------------------------------------------------------------------
        # Subscriber
        # ------------------------------------------------------------------
        # TODO: Create a subscription to /grid_feed that calls self.on_grid.
        #
        # IMPORTANT: check the QoS the feed publishes with (see
        # sim/grid_feed_publisher.py). If your subscription asks for more than
        # the publisher offers, ROS connects nothing and you receive nothing.
        # The sim prints a warning when this happens — read the terminal output.
        #
        # self.grid_sub = self.create_subscription(
        #     GridSnapshot, "/grid_feed", self.on_grid, ???
        # )

        # ------------------------------------------------------------------
        # Publisher
        # ------------------------------------------------------------------
        # TODO: Create a publisher for /planned_path (message type: Path).
        #
        # self.path_pub = self.create_publisher(Path, "/planned_path", 10)

        # ------------------------------------------------------------------
        # State — add whatever you need
        # ------------------------------------------------------------------
        self.path_xy = None          # current plan: list of (x, y) WORLD points. None = never planned.
        self.plan_count = 0          # how many plans you have published (initial one included)
        self.replan_count = 0        # how many of those replaced an existing plan
        self.last_replan_stamp = None  # sim time of the most recent replan, or None

    # -----------------------------------------------------------------------
    # The callback: runs once per tick
    # -----------------------------------------------------------------------

    def on_grid(self, msg: GridSnapshot) -> None:
        """
        Called every tick (5 Hz of SIMULATED time) with the rover's latest view.

        Your job on each call:
          1. Turn the message into a Grid (Grid.from_msg) and make it safe for a
             rover with a body (inflate by msg.robot_radius_m).
          2. Work out where the rover and the goal are in the grid.
          3. Decide: is the current plan still good?  (self.needs_replan)
          4. Only if it isn't: plan again (self.plan), publish it (self.publish_path),
             and update your counters.
          5. Print monitoring output (self.print_status).

        Step 3 is the heart of the task: replan when the plan is invalidated,
        and not otherwise. The scoreboard tells you how you did.
        """
        # TODO: implement
        raise NotImplementedError("PlannerNode.on_grid")

    def needs_replan(self, grid: Grid, rover_xy) -> tuple:
        """
        Should we compute a new plan right now?  Returns (bool, reason_string).

        `grid` is the current (already inflated) grid. `rover_xy` is where the rover is now.

        Think about every situation in which the answer should be True:
          * you have never planned
          * the last plan found no route (was it worth trying again? when?)
          * the plan is no longer safe
        and the situations in which it should be False, even though the map changed:
          * something changed, but not anywhere near the part of the path still ahead

        Return a short reason like "path blocked ahead" — it goes in your monitoring
        output and makes debugging a hundred times easier.
        """
        # TODO: implement
        raise NotImplementedError("PlannerNode.needs_replan")

    def plan(self, grid: Grid, rover_xy, goal_xy):
        """
        Run the search and return the result as a list of (x, y) WORLD points
        (first = the rover's cell, last = the goal's cell), or [] if there is no route.

        Convert world -> cell, call astar, convert the cells back to world points.
        """
        # TODO: implement
        raise NotImplementedError("PlannerNode.plan")

    def publish_path(self, points_xy, stamp: float) -> None:
        """Publish a list of (x, y) world points as nav_msgs/Path. (provided)"""
        self.path_pub.publish(make_path_msg(points_xy, stamp))

    # -----------------------------------------------------------------------
    # Monitoring
    # -----------------------------------------------------------------------

    def print_status(self, stamp: float) -> None:
        """
        Print a monitoring line to the terminal. Not every tick — about once per
        second of simulated time is plenty, plus a line whenever you (re)plan.

        Must show:
          * current path length (metres)
          * number of replans triggered so far
          * time since the last replan

        `stamp` is SIMULATION time in seconds (msg.header.stamp). The sim can run
        much faster than real time, so time.time() would give nonsense here.
        """
        # TODO: implement
        raise NotImplementedError("PlannerNode.print_status")
