"""
Custom message types used in this simulation.

These are not real ROS message types — they are purpose-built for this
take-home. In a real system the grid would be a nav_msgs/OccupancyGrid built from
stereo depth, and the rover pose would come from localization (a TF lookup).
Here both arrive together in one message so you can focus on planning.

The planned path you publish IS a real message type: nav_msgs/Path.
"""

from dataclasses import dataclass, field

from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# Values used in GridSnapshot.occupancy (same convention as nav_msgs/OccupancyGrid):
#   UNKNOWN = -1   never observed      FREE = 0   observed, empty      OCCUPIED = 100   observed, obstacle
from grid_values import UNKNOWN, FREE, OCCUPIED  # noqa: F401  (re-exported)


@dataclass
class GridSnapshot:
    """
    Published by the grid feed at 5 Hz on /grid_feed (BEST_EFFORT).

    One message = everything the rover knows at this instant: the occupancy grid
    it has built up so far, where the rover is, and where it is trying to go.

    ── Grid layout ────────────────────────────────────────────────────────────
    The grid is width × height cells, each `resolution` metres square.
    `occupancy` and `confidence` are FLAT lists of length width*height in
    ROW-MAJOR order:

        index = row * width + col

    Cell (row, col) covers the world-space square

        x ∈ [origin_x + col * resolution,  origin_x + (col + 1) * resolution)
        y ∈ [origin_y + row * resolution,  origin_y + (row + 1) * resolution)

    i.e. (origin_x, origin_y) is the world position of the LOWER-LEFT CORNER of
    cell (0, 0) — not its centre — and row 0 is the BOTTOM row (y grows with
    row, like a normal graph, unlike an image). World coordinates are metres
    in a fixed "map" frame; +x is right/east, +y is up/north.

    ── Cell values ────────────────────────────────────────────────────────────
    occupancy[i]   -1  UNKNOWN   never observed (outside sensor range so far)
                    0  FREE      observed, empty
                  100  OCCUPIED  observed, obstacle
    confidence[i]  0.0 – 1.0     how much to trust occupancy[i]. Unknown cells
                                 have confidence 0.0.

    Fields:
        header.stamp        SIMULATION time in seconds (starts at 0.0). This is
                            not wall-clock time — the sim can run faster than
                            real time, so never use time.time() for anything
                            that should track the rover's world.
        seq                 tick counter: 0, 1, 2, ...
        rover_x, rover_y    where the rover is right now (world metres)
        goal_x, goal_y      where the rover wants to go (world metres)
        robot_radius_m      the rover is a disc this big. Its centre must stay
                            at least this far from any obstacle. The edge of the
                            grid counts as an obstacle too: it is a wall, and the
                            rover cannot drive over it or graze along it.
        sensor_range_m      the rover observes cells within this distance of
                            itself; everything farther keeps its last value
        confidence_threshold  a cell with confidence below this is
                            "unreliable". Only matters for Part 2 (--noise);
                            you can ignore it in Part 1.
    """
    header: Header = field(default_factory=Header)
    seq: int = 0
    resolution: float = 0.5
    width: int = 0
    height: int = 0
    origin_x: float = 0.0
    origin_y: float = 0.0
    occupancy: list = field(default_factory=list)
    confidence: list = field(default_factory=list)
    rover_x: float = 0.0
    rover_y: float = 0.0
    goal_x: float = 0.0
    goal_y: float = 0.0
    robot_radius_m: float = 0.5
    sensor_range_m: float = 8.0
    confidence_threshold: float = 0.6


def make_path_msg(points_xy, stamp: float) -> Path:
    """
    Build a nav_msgs/Path from a list of (x, y) WORLD coordinates (metres).
    An empty list produces an empty path, which tells the rover to stop.
    """
    msg = Path()
    msg.header.stamp = stamp
    msg.header.frame_id = "map"
    for x, y in points_xy:
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = "map"
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        msg.poses.append(ps)
    return msg


def path_msg_to_points(msg: Path) -> list:
    """Inverse of make_path_msg: a nav_msgs/Path -> list of (x, y) tuples."""
    return [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
