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

## 3. Handling Unknown sections

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


## 4. Diagonal Corners

**What I asked:**

I asked for help understanding how to implement the neighbor exploration portion of A*. I specifically needed help with 8-directional movement, calculating straight versus diagonal movement costs, and determining how to calculate the accumulated cost for each neighbor.
**What I kept vs. rewrote, and why:**
I kept the approach of using direction tuples to represent the eight possible movements and using a priority queue containing (f, cell). I also kept the idea of storing g costs and previous cells for reconstructing the final path. I wrote and organized the actual implementation myself.
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


## 5. Debugging Planner_node file

**What I asked:**
I asked to verify any issues with the program and if there was incorrect logic, causing the plan to work. 
**What I kept vs. rewrote, and why:**
I kept the overall structure the same, I changed the conditions or scenarios in which a replan would need to be called. I changed them since I only had taken account for when the plan is None and also made the unknown_is_free variable True when there was an empty plan
**What the AI got wrong that I had to catch:**
The AI wanted to create a periodical caller to try replanning whenever the plan was empty, but that wasn't what was asked for in description and would create more issues
**How I verified it ran correctly (not just that it compiled):**
========================================================================
 RUN SUMMARY   scenario=open  noise=off
========================================================================
 Result             STALLED: rover made no progress for 30 s (gave up at t=30.0 s)
 Distance driven    0.0 m   (straight line start to goal is 36.8 m)
 Blocked ticks      0
 Paths published    0   (0 plans, 0 of them replans)
 Needless replans   0
 Your node reports  replan_count=0   (the scoreboard counted 0)

 !! Your node published 1 paths but every one was EMPTY, which means 'no route found, stop'. Print what your search returns on the first ticks to see why.


## 6. Debugging Planner_node file

**What I asked:**
I asked to verify if my placement of retrying plans was correct or if I was missing something
**What I kept vs. rewrote, and why:**
I kept the logic of using a timestamp so that it doesn't try replanning the route every call. I rewrote the retry plan logic so I only need to use one of the timestamps, since it was trying to use all three global timestamps, so that it wouldn't replan twice, but logic takes care of it 
**What the AI got wrong that I had to catch:**
The AI tried modifying the function header to add a timestamp to the needs_replan(), which would've caused more issues and would not be properly tested
**How I verified it ran correctly (not just that it compiled):**

BEFORE:
========================================================================
 RUN SUMMARY   scenario=goal-blocked  noise=off
========================================================================
 Result             STALLED: rover made no progress for 30 s (gave up at t=50.4 s)
 Distance driven    30.6 m   (straight line start to goal is 36.8 m)
 Blocked ticks      0
 Paths published    4   (3 plans, 2 of them replans)
 Needless replans   0
 Your node reports  replan_count=3   (the scoreboard counted 2)

 Path events:
   tick    t(s)  kind       waypoints  length(m)   note
      0     0.0  initial           53       36.8
     14     2.8  replan            50       33.4
     15     3.0  replan            51       33.7
    102    20.4  stop               0        0.0

AFTER:
========================================================================
 RUN SUMMARY   scenario=goal-blocked  noise=off
========================================================================
 Result             REACHED THE GOAL in 164 ticks (32.8 s simulated)
 Distance driven    37.2 m   (straight line start to goal is 36.8 m)
 Blocked ticks      0
 Paths published    5   (4 plans, 2 of them replans)
 Needless replans   0
 Your node reports  replan_count=3   (the scoreboard counted 2)

 Path events:
   tick    t(s)  kind       waypoints  length(m)   note
      0     0.0  initial           53       36.8
     14     2.8  replan            50       33.4
     15     3.0  replan            51       33.7
    102    20.4  stop               0        0.0
    142    28.4  recovery          12        6.9

 World events: tick 102: 'goal blocked' appeared; tick 142: 'goal blocked' cleared
