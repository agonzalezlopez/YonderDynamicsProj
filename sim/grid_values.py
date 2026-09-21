"""
Cell values used in GridSnapshot.occupancy (same convention as nav_msgs/OccupancyGrid).

Kept in its own tiny file, with no imports, so the pure-Python grid/search code can
use them without needing the simulator's ROS shim on sys.path.
"""

UNKNOWN = -1     # the rover has never observed this cell
FREE = 0         # observed, nothing there
OCCUPIED = 100   # observed, obstacle
