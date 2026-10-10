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
import heapq
import math
from sim.grid_values import UNKNOWN

from grid_utils import Grid, Cell


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
    dr = abs(a[0] - b[0])
    dc = abs(a[1] - b[1])
    return max(dr, dc) + (2**0.5 - 1) * min(dr, dc)
    # minimum difference for number of diagonals
    return 


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
    if start[0] < 0 or start[0] >= grid.height or start[1] < 0 or start[1] >= grid.width:
        return None
    if goal[0] < 0 or goal[0] >= grid.height or goal[1] < 0 or goal[1] >= grid.width:
        return None
    if start == goal:
        return [start]

    if grid.is_occupied(goal[0], goal[1]):
        return None

    if not unknown_is_free and grid.get(*goal) == UNKNOWN:
        return None
    g_score = {start: 0.0}
    came_from = {}

    open_set = [(heuristic(start, goal), 0.0, start)]

    directions = [
        (-1, -1), (-1, 0), (-1, 1),
        (0, -1),           (0, 1),
        (1, -1),  (1, 0),  (1, 1)
    ]
    

    while open_set:
        current_f, current_g, current = heapq.heappop(open_set)

        if current_g > g_score[current]:
            continue

        if current == goal:
            path = []

            while current in came_from:
                path.append(current)
                current = came_from[current]

            path.append(start)
            return path[::-1]

        for dr, dc in directions:
            nr = current[0] + dr
            nc = current[1] + dc
            neighbor = (nr, nc)

            if not (0 <= nr < grid.height and 0 <= nc < grid.width):
                continue

            if grid.is_occupied(neighbor[0], neighbor[1]):
                continue

            if not unknown_is_free and grid.get(*neighbor) == UNKNOWN:
                continue

            if dr != 0 and dc != 0:
              side_a = (current[0] + dr, current[1])
              side_b = (current[0], current[1] + dc)

              if grid.is_occupied(*side_a) or grid.is_occupied(*side_b):
                  continue

              if not unknown_is_free:
                  if grid.get(*side_a) == UNKNOWN or grid.get(*side_b) == UNKNOWN:
                      continue

            step_cost = math.sqrt(2) if dr != 0 and dc != 0 else 1.0

            tentative_g_score = g_score[current] + step_cost

            if tentative_g_score < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score

                f_score = tentative_g_score + heuristic(neighbor, goal)

                heapq.heappush(
                    open_set,
                    (f_score, tentative_g_score, neighbor)
                )

    return None
