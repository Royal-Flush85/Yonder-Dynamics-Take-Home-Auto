# AI Usage Log

Replace this template with your own entries. Add one entry per significant use of an AI tool.

## 1. Grid helpers

**What I asked:**
Help understanding coordinate conversion and how to give obstacles a safety 
margin, as well as verifying altogether whether my implementation would not
cause potential future errors.

**What I kept vs. rewrote, and why:**
Used the explanations and example
code for floor-based indexing, cell centres, and square inflation.


**What the AI got wrong that I had to catch:**



**How I verified it ran correctly (not just that it compiled):**
*(Which test, scenario, or picture showed you the problem? Paste the failing line or the before/after numbers.)*
test_centre_of_first_cell (tests.test_grid_utils.TestCellToWorld.test_centre_of_first_cell) ... ok
test_non_square_grid_and_resolution (tests.test_grid_utils.TestCellToWorld.test_non_square_grid_and_resolution) ... ok
test_returns_x_then_y (tests.test_grid_utils.TestCellToWorld.test_returns_x_then_y) ... ok
test_round_trip (tests.test_grid_utils.TestCellToWorld.test_round_trip) ... ok
test_first_line_is_top (tests.test_grid_utils.TestFromAscii.test_first_line_is_top) ... ok
test_cells_next_to_an_obstacle_become_occupied (tests.test_grid_utils.TestInflate.test_cells_next_to_an_obstacle_become_occupied) ... ok
test_does_not_modify_the_input_grid (tests.test_grid_utils.TestInflate.test_does_not_modify_the_input_grid) ... ok
test_far_cells_stay_free (tests.test_grid_utils.TestInflate.test_far_cells_stay_free) ... ok
test_obstacle_in_a_corner_does_not_crash (tests.test_grid_utils.TestInflate.test_obstacle_in_a_corner_does_not_crash) ... ok
test_obstacles_never_disappear (tests.test_grid_utils.TestInflate.test_obstacles_never_disappear) ... ok
test_radius_is_in_metres_not_cells (tests.test_grid_utils.TestInflate.test_radius_is_in_metres_not_cells) ... ok
test_zero_radius_changes_nothing (tests.test_grid_utils.TestInflate.test_zero_radius_changes_nothing) ... ok
test_cell_edges_belong_to_the_cell_above_them (tests.test_grid_utils.TestWorldToCell.test_cell_edges_belong_to_the_cell_above_them) ... ok
test_non_square_grid_and_resolution (tests.test_grid_utils.TestWorldToCell.test_non_square_grid_and_resolution) ... ok
test_points_just_outside_the_grid_are_outside (tests.test_grid_utils.TestWorldToCell.test_points_just_outside_the_grid_are_outside) ... ok
test_returns_row_then_col (tests.test_grid_utils.TestWorldToCell.test_returns_row_then_col) ... ok
test_typical_point_with_nonzero_origin (tests.test_grid_utils.TestWorldToCell.test_typical_point_with_nonzero_origin) ... ok

----------------------------------------------------------------------
Ran 17 tests in 0.007s

## A* heuristic and search

**What I asked:**
I asked Codex to explain the heuristic and A* algorithm, give
implementation guidance, and review my search code.

**What I kept vs. rewrote, and why:**
I used the explanation of octile distance: combine diagonal moves
costing sqrt(2) with the remaining straight moves costing 1.
I wrote an A* implementation and revised it using Codex's feedback.
The implementation uses a heap, best-known costs, predecessor
tracking, and path reconstruction.

**What needed correction:**
My initial code assumed Grid had rows, cols, neighbors, and
move_cost attributes or methods. Codex pointed out that the
provided Grid uses height and width, and that I needed to
implement neighbour generation and movement costs myself.

My initial occupancy checks also incorrectly allowed occupied
cells when unknown_is_free was True. I separated occupied-cell
blocking from unknown-cell handling.

A later version called a nonexistent is_unknown method and lacked
diagonal corner checks. I replaced that call with an occupancy
value comparison and checked both side cells before diagonal moves.

**How I verified it worked:**
I ran `python -m unittest tests.test_search.TestHeuristic -v`:
all three heuristic tests passed.

After completing A*, I ran
`python -m unittest tests.test_search -v`:
all 18 search tests passed, including optimality, unknown-cell
handling, corner cutting, and the large-grid performance test.

## Replan helpers

**What I asked:**
I asked Codex what to implement after A*, how enumerate works,
and how to identify and validate the remaining path.

**What I kept vs. rewrote, and why:**
I followed the guidance to find the closest waypoint using squared
distance. Updating only when the distance is strictly smaller
preserves the lower index when distances tie.

For path validity, I checked waypoints from start_index onward,
converted their world coordinates into cells, and rejected
out-of-bounds or occupied cells. Unknown cells remain traversable,
matching the search policy.

**What needed correction:**
No implementation error was identified during this step.
Codex noted that waypoint-only validation does not check every
point along the segments between waypoints. That remains a
limitation of this version.

**How I verified it worked:**
The final `python -m unittest -v` run included all 14 provided
replan-helper tests, and they passed. These cover nearest-waypoint
selection, ties, obstacles ahead and behind, unknown cells,
empty paths, and world-coordinate conversion.


## Replan helpers

**What I asked:**
I asked Codex what to implement after A*, how enumerate works,
and how to identify and validate the remaining path.

**What I kept vs. rewrote, and why:**
I followed the guidance to find the closest waypoint using squared
distance. Updating only when the distance is strictly smaller
preserves the lower index when distances tie.

For path validity, I checked waypoints from start_index onward,
converted their world coordinates into cells, and rejected
out-of-bounds or occupied cells. Unknown cells remain traversable,
matching the search policy.

**What needed correction:**
No implementation error was identified during this step.
Codex noted that waypoint-only validation does not check every
point along the segments between waypoints. That remains a
limitation of this version.

**How I verified it worked:**
The final `python -m unittest -v` run included all 14 provided
replan-helper tests, and they passed. These cover nearest-waypoint
selection, ties, obstacles ahead and behind, unknown cells,
empty paths, and world-coordinate conversion.

## Monitoring and arena-edge clearance

**What I asked:**
I asked Codex how to add the required monitoring output and what
safety handling remained after the basic scenarios passed.

**What I kept vs. rewrote, and why:**
I used simulated timestamps from msg.header.stamp to report path
length, replan count, and time since the last replan approximately
once per simulated second.

I followed the guidance to calculate each cell centre's distance
from the nearest arena edge and mark unsafe cells occupied in the
planning grid. This adds wall clearance beyond obstacle inflation.

**What needed correction:**
Obstacle inflation alone did not account for the arena boundary,
because the outer wall is not marked occupied in the sensor grid.
Codex identified this missing safety case and provided the
cell-centre clearance calculation.

**How I verified it worked:**
I inspected the goal-blocked monitoring output. It showed a
zero-length path while stopped and a nonempty path after recovery.
The output used simulated seconds, and the replan count matched
the scoreboard.

After adding arena-edge clearance, I ran the full unit-test suite
and `python sim/launch.py --all`. All 49 tests passed, and all
eight scenario runs reached the goal with zero blocked ticks and
zero needless replans. These results verify the tested runs;
they do not independently prove every boundary case is safe.


## Documentation and interpreting results

**What I asked:**
I asked Codex which terminal results belonged in README.md versus
AI_LOG.md, and requested copy-ready wording based on my outputs.

**What I kept vs. rewrote, and why:**
I used the guidance to place the full --all results table in the
README and summarize verification in the AI log. Codex helped
format the results and draft explanations of the implementation
and assistance received.

**What needed correction:**
My pasted commands accidentally joined --all and the seed-7
command together. Codex identified that seed 7 had not run,
so I reran it separately before recording its result.

**How I verified it worked:**
I compared the documented numbers with the terminal summaries
I supplied. The additional random runs reached in 141 ticks
for seed 7, 134 ticks for seed 19, and 130 ticks for seed 42.
All three reported zero blocked ticks and zero needless replans.





## Integration and scenario verification

**What I asked:**
I asked Codex for guidance on connecting the planner, triggering
replanning, printing monitoring output, and handling arena edges.

**What I kept vs. rewrote, and why:**
I used the subscription and plan-once examples, then followed
guidance to connect my replan helpers and add monitoring and
arena-edge clearance.

**What needed correction:**
My first plan method blocked unknown cells. The discussion
explained why the unseen goal requires unknown_is_free=True.
A simulator run also caught a leftover NotImplementedError
in on_grid, which I removed.

**How I verified it worked:**
python -m unittest -v: 49 tests passed.
python sim/launch.py --all: all eight runs reached the goal,
with zero blocked ticks and zero needless replans.
Boulders: 127 ticks and 8 replans.
Wall: 137 ticks and 14 replans.
Goal-blocked: stopped at tick 104 and recovered at tick 144.

I also tested additional random worlds:
- Seed 7: reached in 141 ticks (28.2 simulated seconds);
  0 blocked ticks, 8 replans, 0 needless replans.
- Seed 19: reached in 134 ticks (26.8 simulated seconds);
  0 blocked ticks, 5 replans, 0 needless replans.
- Seed 42: reached in 130 ticks (26.0 simulated seconds);
  0 blocked ticks, 7 replans, 0 needless replans.

In goal-blocked, the planner published a stop at tick 104
and recovered at tick 144 when the obstacle cleared.
For seed 7, the node reported 8 replans, matching the scoreboard.