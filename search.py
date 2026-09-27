"""
A* search on a Grid — pure Python, NO ROS in this file.

    python -m unittest tests.test_search -v

Movement rules (fixed, so the tests can check them)
---------------------------------------------------
* 8-connected: a cell has up to 8 neighbours (up, down, left, right and the 4 diagonals).
* Cost of a move: 1.0 for up/down/left/right, sqrt(2) for a diagonal. (Cost is in
  cell-widths, not metres — multiply by grid.resolution if you need metres.)
* No corner cutting: a diagonal move is only allowed if BOTH of the two cells it
  squeezes between are passable. (A real rover cannot slip between two obstacles
  that touch at a corner.)
* Cells outside the grid don't exist.
"""

from typing import List, Optional

from grid_utils import Grid, Cell

import math
import heapq

def heuristic(a: Cell, b: Cell) -> float:
    """
    Estimate of the cost to travel from cell `a` to cell `b` on an EMPTY grid
    under the movement rules above.

    Choose and justify your choice in the README. The one hard requirement:
    it must NEVER OVERESTIMATE the true cost (that's what "admissible" means, and
    A* is only guaranteed to return the shortest path if it holds). The test
    checks this, and it is worth understanding why some very common heuristics
    fail it on an 8-connected grid.
    """
    # TODO: implement
    x_route = abs(a[1] - b[1])
    y_route = abs(a[0] - b[0])
    diagonal_steps = min(y_route,x_route) 
    horiz_steps = abs(x_route - y_route)


    total_cost = horiz_steps*1 + diagonal_steps * math.sqrt(2)
    return total_cost
    # raise NotImplementedError("heuristic")


def astar(grid: Grid, start: Cell, goal: Cell, unknown_is_free: bool = False) -> Optional[List[Cell]]:
    """
    Find the cheapest path from `start` to `goal`.

    Args:
        grid             the grid to search. OCCUPIED cells can't be entered.
        start, goal      (row, col) cells.
        unknown_is_free  how to treat UNKNOWN cells: True = passable, False = blocked.
                         Which is right for a rover that has only seen a small
                         circle of the world so far? Look at the very first tick
                         of a run before you decide.

    Returns:
        The path as a list of (row, col) cells, INCLUDING both start and goal, or
        None if there is no path. If start == goal, returns [start].

    Things to think about (the tests poke at each):
      * What if the goal is occupied?
      * The rover might already be standing on a cell that your obstacle
        inflation marked as blocked. Should search refuse to leave it?
      * Speed: a 100 x 100 grid must finish in well under a second. What data
        structure makes "give me the cheapest open node" fast?
    """
    # TODO: implement
    directions = [(1,0),(1,1),(-1,0),(-1,-1),(-1,1),(0,1),(0,-1),(1,-1)]
    g = {}
    g[start] = 0 #no cost

    prev_cells = {}
    discovered_cells = []

    beginning_f = g[start] + heuristic(start,goal)

    heapq.heappush(discovered_cells, (beginning_f, start))

    while(discovered_cells):
      new_pos = heapq.heappop(discovered_cells)
    
      if(new_pos[1] == goal):
        trace_back_path = []
        curr = goal
        while curr in prev_cells:
          trace_back_path.append(curr)
          curr = prev_cells[curr]
        trace_back_path.append(start)
        return trace_back_path[::-1] #inverse list
      
      cost = 1
      for i in directions:
        cost = 1 # reset val if was crossed before
        curr_cell_row = new_pos[1][0] + i[0]
        curr_cell_col = new_pos[1][1] + i[1]
        cell_holder = (curr_cell_row,curr_cell_col)

        if(not grid.in_bounds(cell_holder[0],cell_holder[1])):
          continue
        
        if(grid.is_occupied(cell_holder[0],cell_holder[1])):
          continue
        #if block is unknown
        if(grid.get(curr_cell_row,curr_cell_col) == -1 and not unknown_is_free):
          continue

        #diagonal
        if(i[0] != 0 and i[1] != 0):
          if(grid.is_occupied(curr_cell_row,new_pos[1][1]) or grid.is_occupied(new_pos[1][0],curr_cell_col)):
            continue
          cost = math.sqrt(2)
        #cost from start to neighbor
        g_path = g[new_pos[1]] + cost 
        
        if((cell_holder not in g) or (g_path < g[cell_holder])):
          if grid.is_occupied(cell_holder[0], cell_holder[1]):
            print("ADDING OCCUPIED CELL:", cell_holder)
          g[cell_holder] = g_path
          prev_cells[cell_holder] = new_pos[1]

          #getting estimated cost path
          f = g_path + heuristic(cell_holder, goal)
          heapq.heappush(discovered_cells,(f,cell_holder))




