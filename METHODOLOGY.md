# Methodology

## How to run

Tested on Windows using Python [REPLACE WITH python --version OUTPUT].
The project requires Python 3.10 or newer. The headless simulator
and unit tests use only the standard library.

From a fresh clone, enter the repository root:

```powershell
git clone https://gitlab.com/Yonder-Dynamics/take-home-projects/autonomous-take-home.git
cd .\autonomous-take-home\
python -m venv .venv
```

Run tests and all supplied scenarios without needing to activate
the environment:

```powershell
.venv\Scripts\python.exe -m unittest -v
.venv\Scripts\python.exe sim/launch.py --all
```

Every scenario should report REACHED with zero blocked ticks and
zero needless replans.

Run additional random worlds:

```powershell
.venv\Scripts\python.exe sim/launch.py --scenario random --seed 7
.venv\Scripts\python.exe sim/launch.py --scenario random --seed 19
.venv\Scripts\python.exe sim/launch.py --scenario random --seed 42
```

For pictures, install the optional dependencies:

```powershell
.venv\Scripts\python.exe -m pip install -r requirements.txt
.venv\Scripts\python.exe sim/launch.py --scenario boulders --save-png boulders.png
```

Alternatively, use `--visualize` for a live view. No ROS installation
is needed because the project provides a ROS-style simulator shim.

## Thought process

- Followed the GridSnapshot layout: origin is a lower-left corner,
  rows grow with y, and cells are stored in row-major order.
- Used floor for world-to-cell conversion so points just outside
  the lower boundaries remain outside.
- Converted cells to their centres for world-coordinate waypoints.
- Kept grid mathematics, search, and validity helpers separate
  from the node's pub/sub wiring.
- Used octile distance because it matches the cheapest empty-grid
  route for straight moves costing 1 and diagonals costing sqrt(2).
- Used a heap to explore promising cells efficiently and tracked
  predecessors to reconstruct the route.
- Prohibited diagonal corner cutting, including unknown side cells
  when unknown space is configured as blocked.
- Treated unknown cells as traversable in the node so a route can
  reach beyond the initial sensor footprint.
- Inflated obstacles using a square neighbourhood and separately
  blocked cell centres too close to arena edges.
- Retained the current path while its checked remaining waypoints
  were valid, rather than replanning on every map update.
- Published an empty path when no route existed so the rover
  stopped instead of following an obsolete route.
- Used simulated timestamps for monitoring rather than wall time.

## Verification

All 49 provided tests passed.

The --all run reached the goal in all eight runs with zero blocked
ticks and zero needless replans. Additional random seeds 7, 19,
and 42 also reached with zero blocked ticks and zero needless
replans. The full measurements are in README.md.

Goal-blocked demonstrated a stop at tick 104 and recovery at
tick 144, followed by reaching the goal at tick 167.

An additional test in tests/test_extra.py checks diagonal movement
beside an unknown cell under both traversal policies.
Ran 1 test in 0.001s

OK
test_unknown_side_cell_respects_traversal_policy (tests.test_extra.TestUnknownDiagonalCorner.test_unknown_side_cell_respects_traversal_policy) ... ok
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
test_rover_between_waypoints_picks_the_nearer (tests.test_replan.TestNextWaypointIndex.test_rover_between_waypoints_picks_the_nearer) ... ok
test_rover_on_a_waypoint (tests.test_replan.TestNextWaypointIndex.test_rover_on_a_waypoint) ... ok
test_rover_slightly_off_the_path (tests.test_replan.TestNextWaypointIndex.test_rover_slightly_off_the_path) ... ok
test_single_waypoint (tests.test_replan.TestNextWaypointIndex.test_single_waypoint) ... ok
test_tie_returns_lower_index (tests.test_replan.TestNextWaypointIndex.test_tie_returns_lower_index) ... ok
test_empty_path_is_invalid (tests.test_replan.TestPathIsValid.test_empty_path_is_invalid) ... ok
test_free_path_is_valid (tests.test_replan.TestPathIsValid.test_free_path_is_valid) ... ok
test_obstacle_ahead_of_start_index_still_counts (tests.test_replan.TestPathIsValid.test_obstacle_ahead_of_start_index_still_counts) ... ok
test_obstacle_behind_start_index_is_ignored (tests.test_replan.TestPathIsValid.test_obstacle_behind_start_index_is_ignored) ... ok
test_obstacle_off_the_path_does_not (tests.test_replan.TestPathIsValid.test_obstacle_off_the_path_does_not) ... ok
test_obstacle_on_the_path_invalidates_it (tests.test_replan.TestPathIsValid.test_obstacle_on_the_path_invalidates_it) ... ok
test_unknown_cells_do_not_invalidate (tests.test_replan.TestPathIsValid.test_unknown_cells_do_not_invalidate) ... ok
test_uses_world_coordinates_not_cell_indices (tests.test_replan.TestPathIsValid.test_uses_world_coordinates_not_cell_indices) ... ok
test_waypoint_outside_the_grid_is_invalid (tests.test_replan.TestPathIsValid.test_waypoint_outside_the_grid_is_invalid) ... ok
test_diagonal_is_used_when_it_is_shorter (tests.test_search.TestAstarBasics.test_diagonal_is_used_when_it_is_shorter) ... ok
test_goal_occupied_returns_none (tests.test_search.TestAstarBasics.test_goal_occupied_returns_none) ... ok
test_goes_around_a_wall (tests.test_search.TestAstarBasics.test_goes_around_a_wall) ... ok
test_mixed_straight_and_diagonal (tests.test_search.TestAstarBasics.test_mixed_straight_and_diagonal) ... ok
test_no_path_returns_none (tests.test_search.TestAstarBasics.test_no_path_returns_none) ... ok
test_start_equals_goal (tests.test_search.TestAstarBasics.test_start_equals_goal) ... ok
test_start_or_goal_outside_the_grid_returns_none (tests.test_search.TestAstarBasics.test_start_or_goal_outside_the_grid_returns_none) ... ok
test_straight_line (tests.test_search.TestAstarBasics.test_straight_line) ... ok
test_no_corner_cutting_next_to_a_single_obstacle (tests.test_search.TestAstarRules.test_no_corner_cutting_next_to_a_single_obstacle) ... ok
test_no_squeezing_between_diagonal_obstacles (tests.test_search.TestAstarRules.test_no_squeezing_between_diagonal_obstacles) ... ok
test_start_on_a_blocked_cell_can_still_leave (tests.test_search.TestAstarRules.test_start_on_a_blocked_cell_can_still_leave) ... ok
test_unknown_cells_respect_the_flag (tests.test_search.TestAstarRules.test_unknown_cells_respect_the_flag) ... ok
test_never_overestimates (tests.test_search.TestHeuristic.test_never_overestimates) ... ok
test_symmetric (tests.test_search.TestHeuristic.test_symmetric) ... ok
test_zero_at_goal (tests.test_search.TestHeuristic.test_zero_at_goal) ... ok
test_tricky_grid_seed_199 (tests.test_search.TestOptimality.test_tricky_grid_seed_199) ... ok
test_tricky_grid_seed_223 (tests.test_search.TestOptimality.test_tricky_grid_seed_223) ... ok
test_large_grid_with_a_detour_is_fast (tests.test_search.TestSpeed.test_large_grid_with_a_detour_is_fast) ... ok

----------------------------------------------------------------------
Ran 50 tests in 0.547s

OK

AI assistance included explanations, code examples, review of
implementation attempts, an additional test, and documentation
drafting. AI_LOG.md records that assistance and its verification.

## Known limitations and next steps

- Confidence is ignored. Noisy-sensor planning and phantom-replan
  reduction have not been implemented.
- Path validity checks waypoints rather than every cell touched
  by each segment. A newly blocked diagonal side cell could be
  missed by the validity helper even though A* checks corners
  when initially planning.
- The closest waypoint approximates progress along the path.
  A segment-based progress check would more accurately identify
  the portion still ahead of the rover.
- A failed search is retried each tick. Caching the effective
  search inputs would avoid repeating an unchanged no-route search.
- Square inflation is conservative and may reject narrow passages.
- Paths use cell-centre waypoints rather than an any-angle planner.
- The clean scenarios tested successfully, but this does not prove
  correctness for every possible world or random seed.
- Real ROS 2 integration and Webots were not tested.