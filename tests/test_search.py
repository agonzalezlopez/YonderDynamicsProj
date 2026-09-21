"""
Tests for search.py.   Run:  python -m unittest tests.test_search -v
"""
import math
import time
import unittest

from grid_utils import Grid
from search import astar, heuristic

SQ2 = math.sqrt(2.0)


def cost(path):
    """Cost of a path under the movement rules: 1 per straight move, sqrt(2) per diagonal."""
    return sum(SQ2 if (a[0] != b[0] and a[1] != b[1]) else 1.0 for a, b in zip(path, path[1:]))


def is_valid_path(test, grid, path, start, goal):
    test.assertEqual(path[0], start, "path must begin at start")
    test.assertEqual(path[-1], goal, "path must end at goal")
    for a, b in zip(path, path[1:]):
        test.assertLessEqual(max(abs(a[0] - b[0]), abs(a[1] - b[1])), 1, f"{a} -> {b} is not a single 8-connected step")
        test.assertNotEqual(a, b)
    for cell in path[1:]:
        test.assertFalse(grid.is_occupied(*cell), f"path goes through occupied cell {cell}")


class TestAstarBasics(unittest.TestCase):
    def test_start_equals_goal(self):
        g = Grid.from_ascii(["...", "...", "..."])
        self.assertEqual(astar(g, (1, 1), (1, 1)), [(1, 1)])

    def test_straight_line(self):
        g = Grid.from_ascii(["....."])
        path = astar(g, (0, 0), (0, 4))
        self.assertEqual(path, [(0, 0), (0, 1), (0, 2), (0, 3), (0, 4)])

    def test_diagonal_is_used_when_it_is_shorter(self):
        g = Grid.from_ascii(["....."] * 5)
        path = astar(g, (0, 0), (4, 4))
        is_valid_path(self, g, path, (0, 0), (4, 4))
        self.assertAlmostEqual(cost(path), 4 * SQ2, places=6)

    def test_mixed_straight_and_diagonal(self):
        g = Grid.from_ascii(["......"] * 4)
        path = astar(g, (0, 0), (3, 5))              # 3 diagonals + 2 straights is optimal
        self.assertAlmostEqual(cost(path), 3 * SQ2 + 2.0, places=6)

    def test_goes_around_a_wall(self):
        g = Grid.from_ascii([".....",
                             ".....",
                             "####.",       # wall with a gap at the right
                             ".....",
                             "....."])
        path = astar(g, (0, 0), (4, 0))
        is_valid_path(self, g, path, (0, 0), (4, 0))
        self.assertAlmostEqual(cost(path), 8 + 2 * SQ2, places=6)

    def test_no_path_returns_none(self):
        g = Grid.from_ascii([".....",
                             "#####",
                             "....."])
        self.assertIsNone(astar(g, (0, 0), (2, 4)))

    def test_goal_occupied_returns_none(self):
        g = Grid.from_ascii(["..#"])
        self.assertIsNone(astar(g, (0, 0), (0, 2)))

    def test_start_or_goal_outside_the_grid_returns_none(self):
        g = Grid.from_ascii(["..."])
        self.assertIsNone(astar(g, (0, 0), (0, 7)))
        self.assertIsNone(astar(g, (5, 5), (0, 1)))


class TestAstarRules(unittest.TestCase):
    def test_no_squeezing_between_diagonal_obstacles(self):
        g = Grid.from_ascii([".#",
                             "#."])
        # Start bottom-left (0,0), goal top-right (1,1). The only route is the diagonal
        # between two obstacles that touch at a corner.
        self.assertIsNone(astar(g, (0, 0), (1, 1)), "a diagonal move may not cut the corner of an occupied cell")

    def test_no_corner_cutting_next_to_a_single_obstacle(self):
        g = Grid.from_ascii(["..",
                             ".#"])
        # The obstacle is at row 0, col 1. Going (0,0) -> (1,1) diagonally would clip its corner.
        self.assertEqual(astar(g, (0, 0), (1, 1)), [(0, 0), (1, 0), (1, 1)])

    def test_unknown_cells_respect_the_flag(self):
        g = Grid.from_ascii(["..?..",
                             "#####"])
        self.assertIsNotNone(astar(g, (1, 0), (1, 4), unknown_is_free=True))
        self.assertIsNone(astar(g, (1, 0), (1, 4), unknown_is_free=False))

    def test_start_on_a_blocked_cell_can_still_leave(self):
        g = Grid.from_ascii(["...",
                             ".#.",
                             "..."])
        path = astar(g, (1, 1), (2, 2))       # rover is standing on a cell that is marked occupied
        self.assertIsNotNone(path, "the rover may already be inside an inflated zone; it must be able to get out")
        self.assertEqual(path[-1], (2, 2))


class TestOptimality(unittest.TestCase):
    """These grids were checked by exhaustive search. A* with a bad heuristic returns a longer path."""

    def test_tricky_grid_seed_223(self):
        g = Grid.from_ascii(["......",
                             "......",
                             "..#...",
                             "#.....",
                             "..##.#",
                             "..#..."])
        path = astar(g, (0, 0), (5, 5))
        is_valid_path(self, g, path, (0, 0), (5, 5))
        self.assertAlmostEqual(cost(path), 8.2426, places=3,
                               msg="not the shortest path. Is your heuristic admissible on an 8-connected grid?")

    def test_tricky_grid_seed_199(self):
        g = Grid.from_ascii(["..#.#.",
                             ".#....",
                             "...#..",
                             "......",
                             ".#....",
                             "......"])
        path = astar(g, (0, 0), (5, 5))
        is_valid_path(self, g, path, (0, 0), (5, 5))
        self.assertAlmostEqual(cost(path), 8.2426, places=3,
                               msg="not the shortest path. Is your heuristic admissible on an 8-connected grid?")


class TestHeuristic(unittest.TestCase):
    def test_zero_at_goal(self):
        self.assertEqual(heuristic((3, 4), (3, 4)), 0.0)

    def test_never_overestimates(self):
        # On an empty grid the true cost is: diagonals for the shorter side, straights for the rest.
        for dr in range(0, 8):
            for dc in range(0, 8):
                true_cost = SQ2 * min(dr, dc) + abs(dr - dc)
                h = heuristic((0, 0), (dr, dc))
                self.assertLessEqual(h, true_cost + 1e-9,
                                     f"h((0,0),({dr},{dc})) = {h:.3f} overestimates the true cost {true_cost:.3f}. "
                                     "A* needs an admissible heuristic to guarantee the shortest path")

    def test_symmetric(self):
        self.assertAlmostEqual(heuristic((1, 2), (6, 9)), heuristic((6, 9), (1, 2)))


class TestSpeed(unittest.TestCase):
    def test_large_grid_with_a_detour_is_fast(self):
        n = 150
        rows = [["."] * n for _ in range(n)]
        for line in range(6, n):
            rows[line][n // 2] = "#"           # a wall down the middle with a gap at the top
        g = Grid.from_ascii(["".join(r) for r in rows])
        t0 = time.time()
        path = astar(g, (0, 0), (0, n - 1))
        elapsed = time.time() - t0
        self.assertIsNotNone(path)
        self.assertLess(elapsed, 2.0, f"took {elapsed:.2f}s on a 150x150 grid: is the open set a priority queue (heapq)?")


if __name__ == "__main__":
    unittest.main()
