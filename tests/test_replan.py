"""
Tests for replan.py.   Run:  python -m unittest tests.test_replan -v
"""
import unittest

from grid_utils import Grid
from replan import next_waypoint_index, path_is_valid
from sim.grid_values import FREE, OCCUPIED, UNKNOWN


def free_grid() -> Grid:
    """20 x 20 cells, 0.5 m each, lower-left corner at world (-5, -5). Cell (r, c) is centred at (-4.75 + 0.5c, -4.75 + 0.5r)."""
    return Grid(20, 20, 0.5, -5.0, -5.0, [FREE] * 400, [1.0] * 400)


# A straight path along y = -4.25 (row 1) going east, one waypoint per cell: columns 0..9
PATH = [(-4.75 + 0.5 * c, -4.25) for c in range(10)]


class TestPathIsValid(unittest.TestCase):
    def test_free_path_is_valid(self):
        self.assertTrue(path_is_valid(free_grid(), PATH))

    def test_obstacle_on_the_path_invalidates_it(self):
        g = free_grid()
        g.occupancy[1 * 20 + 6] = OCCUPIED           # row 1, col 6: on the path
        self.assertFalse(path_is_valid(g, PATH))

    def test_obstacle_off_the_path_does_not(self):
        g = free_grid()
        g.occupancy[3 * 20 + 6] = OCCUPIED           # row 3: two rows above the path
        self.assertTrue(path_is_valid(g, PATH))

    def test_obstacle_behind_start_index_is_ignored(self):
        g = free_grid()
        g.occupancy[1 * 20 + 2] = OCCUPIED           # col 2 - the rover has already driven past it
        self.assertFalse(path_is_valid(g, PATH, start_index=0))
        self.assertTrue(path_is_valid(g, PATH, start_index=4),
                        "an obstacle on the part of the path the rover has already passed must not matter")

    def test_obstacle_ahead_of_start_index_still_counts(self):
        g = free_grid()
        g.occupancy[1 * 20 + 8] = OCCUPIED
        self.assertFalse(path_is_valid(g, PATH, start_index=4))

    def test_unknown_cells_do_not_invalidate(self):
        g = free_grid()
        g.occupancy[1 * 20 + 6] = UNKNOWN            # we simply haven't seen it yet
        self.assertTrue(path_is_valid(g, PATH))

    def test_waypoint_outside_the_grid_is_invalid(self):
        self.assertFalse(path_is_valid(free_grid(), PATH + [(50.0, 50.0)]))

    def test_empty_path_is_invalid(self):
        self.assertFalse(path_is_valid(free_grid(), []))

    def test_uses_world_coordinates_not_cell_indices(self):
        # Same obstacle cell, but the grid has been shifted 1 m (2 cells) east: the world
        # position of that cell changes, so the path is no longer blocked by it.
        g = free_grid()
        g.occupancy[1 * 20 + 6] = OCCUPIED
        shifted = Grid(20, 20, 0.5, -4.0, -5.0, list(g.occupancy), list(g.confidence))
        segment = PATH[2:7]              # x from -3.75 to -1.75: inside both grids
        self.assertFalse(path_is_valid(g, segment))
        self.assertTrue(path_is_valid(shifted, segment))


class TestNextWaypointIndex(unittest.TestCase):
    def test_rover_on_a_waypoint(self):
        self.assertEqual(next_waypoint_index(PATH, PATH[3]), 3)

    def test_rover_between_waypoints_picks_the_nearer(self):
        self.assertEqual(next_waypoint_index(PATH, (PATH[3][0] + 0.4, PATH[3][1])), 4)
        self.assertEqual(next_waypoint_index(PATH, (PATH[3][0] + 0.1, PATH[3][1])), 3)

    def test_rover_slightly_off_the_path(self):
        self.assertEqual(next_waypoint_index(PATH, (PATH[6][0], PATH[6][1] + 0.2)), 6)

    def test_tie_returns_lower_index(self):
        mid = ((PATH[2][0] + PATH[3][0]) / 2, PATH[2][1])
        self.assertEqual(next_waypoint_index(PATH, mid), 2)

    def test_single_waypoint(self):
        self.assertEqual(next_waypoint_index([(0.0, 0.0)], (5.0, 5.0)), 0)


if __name__ == "__main__":
    unittest.main()
