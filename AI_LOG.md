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

## 2. <next use>

**What I asked:**

**What I kept vs. rewrote, and why:**

**What the AI got wrong that I had to catch:**

**How I verified it ran correctly (not just that it compiled):**
