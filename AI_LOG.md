# AI Usage Log

Replace this template with your own entries. Add one entry per significant use of an AI tool.

## 1. Coordinate Conversion for functions in grid_utils.py

**What I asked:**
I asked for help implementing the two coordinate-conversion functions in grid_utils, and specifically how to handle points that fall outside the grid bounds and points that land exactly on a cell edge.
**What I kept vs. rewrote, and why:**
I kept the approach of using math.floor on (coordinate - origin) / resolution rather than ceil(). I initially had ceil() in program, which would cause issues for negative numbers, so any out-of-bounds point would be off by 1.  
**What the AI got wrong that I had to catch:**
Nothing incorrect in the explanation itself, but I had to change some of the values given because it mostly gave me the corner of the cell rather than the center

**How I verified it ran correctly (not just that it compiled):**
*(Which test, scenario, or picture showed you the problem? Paste the failing line or the before/after numbers.)*
Traceback (most recent call last):
  File "\Yonder\autonomous-take-home\tests\test_grid_utils.py", line 25, in test_typical_point_with_nonzero_origin
    self.assertEqual(arena().world_to_cell(-13.2, -12.7), (4, 3))
    ~~~~~~~~~~~~~~~~^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
AssertionError: Tuples differ: (5, 4) != (4, 3)

First differing element 0:
5
4

- (5, 4)
+ (4, 3)

and

  File "\Yonder\autonomous-take-home\tests\test_grid_utils.py", line 53, in test_returns_x_then_y
    self.assertAlmostEqual(x, -9.75)
    ~~~~~~~~~~~~~~~~~~~~~~^^^^^^^^^^
AssertionError: -10.0 != -9.75 within 7 places (0.25 difference)

## 2. Obstacle inflation

**What I asked:**
I asked how to get rows and columns from occupancy list into coordinates and back, and how to bound a search window around each obstacle.
**What I kept vs. rewrote, and why:**
I kept the general structure (iterate obstacles, inflate within a bounded window, write into the copy) but wrote the loop logic myself, including the final distance check. I had to add the distance formula in order to not have farther boxes as OCCUPIED 
**What the AI got wrong that I had to catch:**
The AI failed to recognize that the some of the boxes should not be filled despite it following a square field around the object. Adding the distance equation to check that it was still within raidus regardless of the boxes being in "range"
**How I verified it ran correctly (not just that it compiled):**
I graphed some of the points to get a general idea

Traceback (most recent call last):
  File 
  "\Yonder\autonomous-take-home\tests\test_grid_utils.py", line 103, in test_radius_is_in_metres_not_cells
    self.assertFalse(out.is_occupied(5, 2))
    ~~~~~~~~~~~~~~~~^^^^^^^^^^^^^^^^^^^^^^^
AssertionError: True is not false

## 3. <next use>

**What I asked:**
I asked for help implementing the unknown_is_free requirement in the A* search. I needed to determine where the check should be placed so that unknown cells would only be traversable when the flag was enabled.
**What I kept vs. rewrote, and why:**
I kept the existing neighbor-validation structure and added the unknown-cell check after checking whether the cell was occupied. I used grid.get() to determine the cell's occupancy value and the unknown_is_free flag to decide whether the cell could be traversed.
**What the AI got wrong that I had to catch:**
The AI used 0 as the UNKNOWN value. I changed the value to -1 as shown in the document
**How I verified it ran correctly (not just that it compiled):**
Traceback (most recent call last):
  File "/Yonder/autonomous-take-home/tests/test_search.py", line 161, in test_large_grid_with_a_detour_is_fast
    self.assertIsNotNone(path)
    ~~~~~~~~~~~~~~~~~~~~^^^^^^
AssertionError: unexpectedly None


## 4. <next use>

**What I asked:**

I asked for help understanding how to implement the neighbor exploration portion of A*. I specifically needed help with 8-directional movement, calculating straight versus diagonal movement costs, and determining how to calculate the accumulated cost for each neighbor.
**What I kept vs. rewrote, and why:**
I kept the approach of using direction tuples to represent the eight possible movements and using a priority queue containing (f, cell). I also kept the idea of storing`g costs and previous cells for reconstructing the final path. I wrote and organized the actual implementation myself.
**What the AI got wrong that I had to catch:**
The initial implementation guidance did not account for the fact that my Grid.in_bounds() function required separate row and column arguments. I had to adjust the calls to provide both coordinates.
**How I verified it ran correctly (not just that it compiled):**
Traceback (most recent call last):
  File "/Yonder/autonomous-take-home/tests/test_search.py", line 159, in test_large_grid_with_a_detour_is_fast
    path = astar(g, (0, 0), (0, n - 1))
  File "/Yonder/autonomous-take-home/search.py", line 101, in astar
    if(grid.in_bounds(cell_holder) and not grid.is_occupied(cell_holder)):
       ~~~~~~~~~~~~~~^^^^^^^^^^^^^
TypeError: Grid.in_bounds() missing 1 required positional argument: 'col'

