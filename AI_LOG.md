# AI Usage Log

Replace this template with your own entries. Add one entry per significant use of an AI tool.

## 1. Coordinate Conversion for functions in grid_utils.py

**What I asked:**
I asked for help implementing the two coordinate-conversion functions in grid_utils, and specifically how to handle points that fall outside the grid bounds and points that land exactly on a cell edge.
**What I kept vs. rewrote, and why:**
I kept the approach of using math.floor on (coordinate - origin) / resolution rather than ceil(). I initially had ceil() in the program, which would cause issues for any negative coordinates, so any out-of-bounds point would return the wrong cell.
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
I asked how to get rows and columns from the occupancy list into coordinates and back, and how to bound a search window around each obstacle.
**What I kept vs. rewrote, and why:**
I kept the general structure (iterate obstacles, inflate within a bounded window, write into the copy) but wrote the loop logic myself, including the final distance check. I had to add the distance formula in order to not have farther boxes as OCCUPIED 
**What the AI got wrong that I had to catch:**
The AI failed to take into account the fact that not every cell inside the search window should be filled. I added the distance equation to check that cells around the object were still within the desired radius.
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


## 6. Debugging Replanning

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

## 7. Noisy Sensor Assistance

**What I asked:**
I asked the AI for help implementing the Noisy-sensor filter for the stretch goal. Specifically, to prevent replanning constantly when there was no need.
**What I kept vs. rewrote, and why:**
I kept the idea of using sensor confidence above a certain threshold to determine if the cell was considered reliable. I rewrote where the check was implemented so that it affected the inflation and path validation instead of only using it in the plan/replan functions.
**What the AI got wrong that I had to catch:**
The AI initially suggested using a timer to modify the needs_replan function. This would not have helped determine whether the reading was reliable or not, so I kept the confidence threshold approach instead.
**How I verified it ran correctly (not just that it compiled):**

BEFORE:
========================================================================
 RUN SUMMARY   scenario=boulders  noise=ON
========================================================================
 Result             REACHED THE GOAL in 146 ticks (29.2 s simulated)
 Distance driven    34.8 m   (straight line start to goal is 36.8 m)
 Blocked ticks      0
 Paths published    67   (64 plans, 60 of them replans)
 Needless replans   0
 Phantom replans    36   <- replanned because of something in your grid that wasn't really there
 Your node reports  replan_count=63   (the scoreboard counted 60)

 Path events:
   tick    t(s)  kind       waypoints  length(m)   note
      0     0.0  initial           63       39.7
      1     0.2  replan            56       37.2   PHANTOM
      2     0.4  replan            57       37.5
      3     0.6  replan            60       38.0
      4     0.8  replan            56       36.8
      5     1.0  replan            57       36.7   PHANTOM
      6     1.2  replan            56       36.4
      7     1.4  replan            56       36.0
      8     1.6  replan            55       35.7   PHANTOM
      9     1.8  replan            56       35.6   PHANTOM
     10     2.0  replan            56       36.0   PHANTOM
     11     2.2  replan            58       36.6   PHANTOM
     12     2.4  replan            52       34.0
     14     2.8  replan            51       33.3   PHANTOM
     15     3.0  replan            50       32.6   PHANTOM
     16     3.2  replan            51       32.9   PHANTOM
     17     3.4  replan            51       32.5
     18     3.6  replan            51       32.2
     19     3.8  replan            49       31.5   PHANTOM
     20     4.0  replan            53       32.4   PHANTOM
     21     4.2  replan            51       31.6
     22     4.4  replan            50       31.1   PHANTOM
     23     4.6  replan            44       29.4
     24     4.8  replan            47       29.8
     25     5.0  replan            43       28.7   PHANTOM
     26     5.2  replan            47       29.4
     27     5.4  replan            44       28.5   PHANTOM
     29     5.8  replan            45       28.4
     45     9.0  replan            37       23.4
     47     9.4  replan            40       23.8
     48     9.6  replan            40       23.6   PHANTOM
     49     9.8  replan            35       22.2   PHANTOM
     50    10.0  replan            37       22.3   PHANTOM
     51    10.2  replan            40       23.2   PHANTOM
     52    10.4  replan            36       21.6   PHANTOM
     53    10.6  replan            36       21.4
     54    10.8  replan            31       20.0   PHANTOM
     55    11.0  replan            30       19.3   PHANTOM
     56    11.2  replan            30       19.3
     57    11.4  replan            33       19.7   PHANTOM
     58    11.6  replan            29       18.6   PHANTOM
     59    11.8  replan            37       20.9   PHANTOM
     60    12.0  replan            41       22.3
     61    12.2  replan            29       17.7
     62    12.4  replan            30       18.0   PHANTOM
     64    12.8  replan            33       18.5   PHANTOM
     65    13.0  replan            32       18.0   PHANTOM
     66    13.2  replan            28       16.6   PHANTOM
     67    13.4  replan            27       16.1
     68    13.6  replan            24       15.2   PHANTOM
     69    13.8  replan            25       15.1
     71    14.2  replan            26       15.0
     74    14.8  replan            24       14.0   PHANTOM
     75    15.0  replan            24       13.6
     77    15.4  replan            20       12.0   PHANTOM
     78    15.6  replan            25       13.4   PHANTOM
     79    15.8  replan            20       11.8   PHANTOM
     81    16.2  replan            23       12.2
     82    16.4  replan            19       10.9   PHANTOM
     84    16.8  replan            19       10.4   PHANTOM
     86    17.2  stop               0        0.0
     96    19.2  recovery          17        9.4
     98    19.6  stop               0        0.0
    108    21.6  recovery          17        9.0
    109    21.8  replan            17        9.2   PHANTOM
    110    22.0  stop               0        0.0
    120    24.0  recovery          15        8.0


AFTER:
========================================================================
 RUN SUMMARY   scenario=boulders  noise=ON
========================================================================
 Result             REACHED THE GOAL in 128 ticks (25.6 s simulated)
 Distance driven    38.4 m   (straight line start to goal is 36.8 m)
 Blocked ticks      0
 Paths published    23   (23 plans, 22 of them replans)
 Needless replans   0
 Phantom replans    9   <- replanned because of something in your grid that wasn't really there
 Your node reports  replan_count=22   (the scoreboard counted 22)

 Path events:
   tick    t(s)  kind       waypoints  length(m)   note
      0     0.0  initial           55       37.4
      2     0.4  replan            54       36.6
      6     1.2  replan            54       35.8
      7     1.4  replan            57       36.7
     12     2.4  replan            52       34.0
     14     2.8  replan            52       33.6   PHANTOM
     17     3.4  replan            50       32.6   PHANTOM
     45     9.0  replan            38       24.5
     51    10.2  replan            37       23.4
     53    10.6  replan            34       22.1   PHANTOM
     58    11.6  replan            33       21.0
     59    11.8  replan            31       20.0   PHANTOM
     60    12.0  replan            33       20.6   PHANTOM
     63    12.6  replan            29       18.6   PHANTOM
     81    16.2  replan            23       13.9
     83    16.6  replan            23       13.5
     84    16.8  replan            24       13.8
     85    17.0  replan            23       13.3
     86    17.2  replan            24       13.6
     91    18.2  replan            21       11.7
     97    19.4  replan            18        9.7   PHANTOM
     99    19.8  replan            20       10.1   PHANTOM
    100    20.0  replan            16        8.7   PHANTOM
