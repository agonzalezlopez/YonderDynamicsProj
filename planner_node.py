"""
.
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
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=1,)

        self.grid_sub = self.create_subscription(GridSnapshot, "/grid_feed",self.on_grid, qos)
        # ------------------------------------------------------------------
        # Publisher
        # ------------------------------------------------------------------
        # TODO: Create a publisher for /planned_path (message type: Path).
        #
        # self.path_pub = self.create_publisher(Path, "/planned_path", 10)
        self.path_pub = self.create_publisher(Path, "/planned_path",10)

        # ------------------------------------------------------------------
        # State — add whatever you need
        # ------------------------------------------------------------------
        self.path_xy = None          # current plan: list of (x, y) WORLD points. None = never planned.
        self.plan_count = 0          # how many plans you have published (initial one included)
        self.replan_count = 0        # how many of those replaced an existing plan
        self.last_replan_stamp = None  # sim time of the most recent replan, or None
        
        self.last_status_stamp = None # How often it prints
        self.no_route_stamp = None # time stamp for retrying for new plan

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
        count = 1
        if(self.path_xy == None or not self.path_xy):
            count = 0
        converted_grid = Grid.from_msg(msg)
        inflated_grid = inflate(converted_grid, msg.robot_radius_m)
        #Current Positions (World Pos)
        starting_pos = (msg.rover_x,msg.rover_y)
        goal = (msg.goal_x,msg.goal_y)

        replan, reason = self.needs_replan(inflated_grid, starting_pos)
        if(replan):
            self.path_xy = self.plan(inflated_grid,starting_pos, goal)
            self.publish_path(self.path_xy,msg.header.stamp)
            self.replan_count += count
            self.plan_count += 1
            self.last_replan_stamp = msg.header.stamp

        self.print_status(msg.header.stamp)



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
        if(self.path_xy is None):
            return (True, "Path was never planned")
            
        if(not self.path_xy):
            if(self.no_route_stamp is None):
                self.no_route_stamp = stamp
                return (False, "No route")
            if(stamp - self.no_route_stamp >= 2): # After 2 sim seconds have passed
                return (True,"Locating different Route")
            
        index = next_waypoint_index(self.path_xy,rover_xy)
        valid = path_is_valid(grid, self.path_xy,index)
        if(not valid):
            return (True, "Path is no longer valid")

        return (False, "Path is still valid")
        

    def plan(self, grid: Grid, rover_xy, goal_xy):
        """
        Run the search and return the result as a list of (x, y) WORLD points
        (first = the rover's cell, last = the goal's cell), or [] if there is no route.

        Convert world -> cell, call astar, convert the cells back to world points.
        """
        # TODO: implement
        first = grid.world_to_cell(rover_xy[0],rover_xy[1])
        last = grid.world_to_cell(goal_xy[0],goal_xy[1])

        #originally True boolean was empty, causing unknown blocks to be seen as obstacles
        result_cells = astar(grid, first, last,True)

        if(result_cells is None):
            return []
        world_points = []
        for cells in result_cells:
            curr_point = grid.cell_to_world(cells[0],cells[1])

            curr_world_point = (curr_point[0],curr_point[1])
            world_points.append(curr_world_point)
        return world_points

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
        if(self.last_status_stamp is None):
            self.last_status_stamp = stamp
        if(stamp - self.last_status_stamp >= 1):
            self.last_status_stamp = stamp
            print("Length: ", path_length_m(self.path_xy), " Replans: ", self.replan_count, " Last Replan: ", self.last_replan_stamp)
