"""
Tests for grid_utils.py.   Run:  python -m unittest tests.test_grid_utils -v
"""
import unittest

from grid_utils import Grid, inflate
from sim.grid_values import FREE, OCCUPIED, UNKNOWN


def arena() -> Grid:
    """The same geometry as the simulator: 60 x 60 cells of 0.5 m, lower-left corner at (-15, -15)."""
    return Grid(60, 60, 0.5, -15.0, -15.0, [FREE] * 3600, [1.0] * 3600)


def empty(width, height, resolution=0.5) -> Grid:
    return Grid(width, height, resolution, 0.0, 0.0, [FREE] * (width * height), [1.0] * (width * height))


class TestWorldToCell(unittest.TestCase):
    def test_returns_row_then_col(self):
        # x = -10 is column 10, y = -14 is row 2. Answer is (row, col) = (2, 10).
        self.assertEqual(arena().world_to_cell(-10.0, -14.0), (2, 10))

    def test_typical_point_with_nonzero_origin(self):
        self.assertEqual(arena().world_to_cell(-13.2, -12.7), (4, 3))

    def test_cell_edges_belong_to_the_cell_above_them(self):
        g = arena()
        self.assertEqual(g.world_to_cell(-15.0, -15.0), (0, 0))
        self.assertEqual(g.world_to_cell(-14.51, -14.51), (0, 0))
        self.assertEqual(g.world_to_cell(-14.5, -14.5), (1, 1))

    def test_points_just_outside_the_grid_are_outside(self):
        g = arena()
        self.assertEqual(g.world_to_cell(-15.01, -15.01), (-1, -1),
                         "a point left of / below the grid must give a negative cell. int() truncates toward zero, floor doesn't")
        self.assertEqual(g.world_to_cell(15.0, 15.0), (60, 60))

    def test_non_square_grid_and_resolution(self):
        g = Grid(8, 3, 2.0, 10.0, 20.0, [FREE] * 24)        # 8 columns, 3 rows, 2 m cells
        self.assertEqual(g.world_to_cell(11.9, 20.1), (0, 0))
        self.assertEqual(g.world_to_cell(25.9, 25.9), (2, 7))


class TestCellToWorld(unittest.TestCase):
    def test_centre_of_first_cell(self):
        x, y = arena().cell_to_world(0, 0)
        self.assertAlmostEqual(x, -14.75)
        self.assertAlmostEqual(y, -14.75)

    def test_returns_x_then_y(self):
        x, y = arena().cell_to_world(2, 10)     # row 2, col 10
        self.assertAlmostEqual(x, -9.75)
        self.assertAlmostEqual(y, -13.75)

    def test_non_square_grid_and_resolution(self):
        g = Grid(8, 3, 2.0, 10.0, 20.0, [FREE] * 24)
        x, y = g.cell_to_world(2, 7)
        self.assertAlmostEqual(x, 25.0)
        self.assertAlmostEqual(y, 25.0)

    def test_round_trip(self):
        g = arena()
        for r in (0, 1, 17, 59):
            for c in (0, 3, 42, 59):
                self.assertEqual(g.world_to_cell(*g.cell_to_world(r, c)), (r, c))


class TestFromAscii(unittest.TestCase):
    """Documents the orientation of the provided helper: the first line is the TOP row."""
    def test_first_line_is_top(self):
        g = Grid.from_ascii(["#.",
                             ".."])
        self.assertTrue(g.is_occupied(1, 0))     # top-left is row 1, col 0
        self.assertFalse(g.is_occupied(0, 0))


class TestInflate(unittest.TestCase):
    def one_obstacle(self, resolution=0.5):
        g = empty(9, 9, resolution)
        g.occupancy[g.index(4, 4)] = OCCUPIED
        return g

    def test_cells_next_to_an_obstacle_become_occupied(self):
        out = inflate(self.one_obstacle(), 0.5)         # radius = exactly one cell
        for cell in [(4, 4), (3, 4), (5, 4), (4, 3), (4, 5)]:
            self.assertTrue(out.is_occupied(*cell), f"{cell} should be inflated")

    def test_far_cells_stay_free(self):
        out = inflate(self.one_obstacle(), 0.5)
        for cell in [(4, 6), (6, 4), (2, 4), (4, 2), (0, 0), (8, 8)]:
            self.assertFalse(out.is_occupied(*cell), f"{cell} is too far to be inflated")

    def test_radius_is_in_metres_not_cells(self):
        # Same 0.5 m radius, but cells are only 0.25 m wide -> reaches TWO cells out.
        out = inflate(self.one_obstacle(resolution=0.25), 0.5)
        self.assertTrue(out.is_occupied(4, 6))
        self.assertTrue(out.is_occupied(2, 4))
        self.assertFalse(out.is_occupied(4, 7))

    def test_zero_radius_changes_nothing(self):
        g = self.one_obstacle()
        self.assertEqual(inflate(g, 0.0).occupancy, g.occupancy)

    def test_does_not_modify_the_input_grid(self):
        g = self.one_obstacle()
        before = list(g.occupancy)
        inflate(g, 1.0)
        self.assertEqual(g.occupancy, before)

    def test_obstacle_in_a_corner_does_not_crash(self):
        g = empty(5, 5)
        g.occupancy[g.index(0, 0)] = OCCUPIED
        g.occupancy[g.index(4, 4)] = OCCUPIED
        out = inflate(g, 1.0)
        self.assertTrue(out.is_occupied(0, 1))
        self.assertTrue(out.is_occupied(4, 3))

    def test_obstacles_never_disappear(self):
        g = self.one_obstacle()
        g.occupancy[g.index(0, 8)] = UNKNOWN
        out = inflate(g, 0.5)
        self.assertTrue(out.is_occupied(4, 4))
        self.assertEqual(out.get(0, 8), UNKNOWN)         # unknown far from any obstacle is left alone


if __name__ == "__main__":
    unittest.main()
