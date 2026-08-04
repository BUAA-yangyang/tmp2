# A1 Single-Floor Exploration

`a1_exploration` consumes the current floor's standard
`nav_msgs/OccupancyGrid`, detects and ranks free/unknown frontiers, sends every
motion goal through `move_base_msgs/MoveBaseAction`, and returns to the pose
recorded at action start.

The package never publishes `/cmd_vel_nav` or `/cmd_vel`. Its complete motion
chain is:

```text
a1_exploration
  -> /move_base (MoveBaseAction)
  -> /cmd_vel_nav
  -> a1_cmd_mux
  -> /cmd_vel
```

## Scope and competition compliance

The algorithm reads only:

- `/a1/floor_mapping/map` (`nav_msgs/OccupancyGrid`);
- `/a1/floor_mapping/status` (`diagnostic_msgs/DiagnosticStatus`);
- the map-frame TF to `base`;
- `/move_base`, `/move_base/make_plan`, the standard DWA dynamic-reconfigure
  service, `/cmd_vel`, and the cmd_mux safety lock;
- `/a1/controller_ready` and the caller-supplied entry-relative ROI.

It does not read Gazebo model/link states, generated building layouts, danger
ground truth, referee truth, or scripted scene coordinates. The dev launch in
`a1_navigation_tests` may adapt `/Odometry_gazebo` into the public localization
contract, but that adapter is explicitly test-only and is not part of this
business package.

## State machine

```text
IDLE
  -> RECORD_START
  -> REQUEST_ENTRY_DOOR_OPEN
  -> TRANSIT_TO_ENTRY
  -> ENTERED_FLOOR
  -> SELECT_FRONTIER
  -> NAVIGATING
  -> UPDATE_COVERAGE
  -> ... repeat ...
  -> EXPLORATION_DONE
  -> RETURNING
  -> RETURNED
```

Any active state can terminate as `FAILED` or `CANCELLED`.

- `RECORD_START` binds the action to
  `(map frame, localization generation, floor session)` and records `map->base`
  for return; the ROI has its own floor-entry anchor.
- `SELECT_FRONTIER` clusters free cells adjacent to unknown cells, enforces
  obstacle clearance and standoff, and calls `/move_base/make_plan`.
- `NAVIGATING` sends exactly one standard MoveBaseAction goal.
- `UPDATE_COVERAGE` waits for a newer floor map and records visited/failed
  targets.
- A transient `/move_base/make_plan` transport error is retried and reported as
  degraded planning; it does not count as an unreachable frontier. Only a
  continuous outage longer than
  `planning/make_plan_unavailable_timeout_wall` fails the action.
- `EXPLORATION_DONE` is reached only after the configured number of distinct
  map updates have no eligible frontier. Frontiers already visited or reaching
  the configured failure limit count as exhausted.
- `RETURNING` sends the recorded start pose through MoveBaseAction.
- `RETURNED` additionally requires position and yaw tolerances, a fresh final
  `/cmd_vel`, and at least 0.5 ROS/simulation seconds since the last non-zero
  command. Stale messages or a clock rollback fail closed.

Fixed elapsed time is not an exploration-complete criterion. Overall and
per-goal timeouts are fault bounds only.

The default return bound is 480 ROS seconds: at the conservative unattended-dev
speed, the 40 x 40 m grid diagonal alone needs about 283 seconds before turns
and detours. After MoveBaseAction succeeds, exploration still waits up to eight
wall seconds for the ROS-time final-zero conditions. A cancelled or
timed-out goal must settle into a terminal MoveBaseAction state before retry.
The mapping-health grace is 3.5 wall seconds, aligned just above
floor_mapping's 3 s input-lost threshold; shorter degraded blips do not change
the bound map identity or abort a return.

## Frontier selection

1. A frontier cell is known free and 4-neighbor adjacent to unknown.
2. Frontier cells outside the active single-floor work region are removed.
3. Remaining 8-connected frontier cells are clustered.
4. Clusters shorter than `frontier/min_length` are rejected.
5. Occupied cells are dilated by `frontier/obstacle_clearance`.
6. The goal is placed inside both known free space and the work region by
   `frontier/goal_standoff`, facing the unknown side.
7. Candidates are ranked by information length minus travel distance.
8. `/move_base/make_plan` is the final reachability gate.
9. Failed goals are spatially de-duplicated, cooled down, and permanently
   exhausted after `frontier/maximum_failures`.

### Single-floor ROI

`ExploreFloor.floor_entry_pose` is independent of the return pose and uses the
active OccupancyGrid frame. Its +X direction points into the floor. A supplied
entry orientation must be a finite unit planar quaternion
(`abs(x), abs(y) <= 1e-6`, norm error `<= 1e-3`). Invalid input is never
silently projected: the action aborts with `ERROR_INVALID_ENTRY_POSE` before
door behavior or MoveBaseAction starts.
`ExploreFloor.roi_local` is a polygon in metres relative to that entry
(x forward, y left), so it can include a narrow entrance and a wider indoor
body without any fixed world coordinate. Production configuration requires an
explicit entry pose. The server retains a temporary RECORD_START-relative
compatibility fallback only when `entry/require_explicit_pose` is deliberately
set to `false`.

This is an explicit mission contract, not an inference from Gazebo, generated
building data, model states, or referee truth.

Opening the public door is followed by two independent pre-motion checks.
OccupancyGrid connectivity must remain inside an entry-local corridor, so a
known-free exterior side route cannot masquerade as an open doorway. The
`/move_base/make_plan` response must then use the entry frame, contain finite
poses, begin/end near the request, remain inside the same corridor, and stay
within the configured direct-path length bound. A rejected plan never reaches
`MoveBaseAction`; the action fails closed with `ERROR_ENTRY_TRANSIT` if no safe
local plan appears before `timeouts/entry_plan`.

Immediately before that one entry MoveBaseAction, the server queries the live
`DWAPlannerROS` configuration, applies the validated paired minima/maxima under
`entry/speed_limit`, verifies the service response, and saves the exact prior
values. It restores and verifies that snapshot before `ENTERED_FLOOR` or any
frontier selection. A missing service, incomplete response, attempted speed
increase, `min > max` relation, apply failure, or restore mismatch aborts
without sending another goal. This changes planner limits only: exploration
still never publishes a velocity and cmd_mux remains the sole `/cmd_vel`
output.

ROI clipping happens before frontier clustering so a large exterior frontier
cannot contribute cells, centroid, information gain, or a goal through a small
in-scope segment. The nearest-free goal search, optional `seed_target`, and
coverage metric use the same mask. Invalid/non-finite geometry, a frame
mismatch, an invalid entry orientation, or an ROI with no map overlap fails
the action before any MoveBaseAction goal is sent.

`coverage_ratio` is the known-cell fraction whose cell centres lie inside the
effective active ROI; that cell count is its denominator. It is diagnostic only
and never terminates exploration. The latched standard
`/a1/exploration/roi` (`geometry_msgs/PolygonStamped`) and compatibility blue
`/a1/exploration/scope` marker show the boundary for RViz and bag review.

## Interfaces

| Direction | Name | Type | Frame/lifecycle |
|---|---|---|---|
| input | `/a1/floor_mapping/map` | `nav_msgs/OccupancyGrid` | current session frame, currently `odom` |
| input | `/a1/floor_mapping/status` | `diagnostic_msgs/DiagnosticStatus` | must remain `MAPPING`, map/cloud valid, same generation/session |
| input/output | `/move_base` | `move_base_msgs/MoveBaseAction` | all frontier and return goals |
| input | `/move_base/make_plan` | `nav_msgs/GetPlan` | candidate reachability |
| input/output | `/move_base/DWAPlannerROS/set_parameters` | `dynamic_reconfigure/Reconfigure` | temporary verified entry-only speed maxima; exact live snapshot restored before indoor exploration |
| input | `/a1_cmd_mux/safety_lock` | `std_msgs/Bool` | true cancels motion and fails safely |
| input | `/a1/controller_ready` | `std_msgs/Bool` | fresh true heartbeat required before every MoveBaseAction goal |
| input | `/cmd_vel` | `geometry_msgs/Twist` | observed only for final-zero verification |
| output | `/a1/exploration/status` | `ExplorationStatus` | latched and periodic |
| server | `/a1/exploration/explore_floor` | `ExploreFloorAction` | one floor identity per goal |
| output | `/a1/exploration/frontiers` | `visualization_msgs/MarkerArray` | green frontier cells |
| output | `/a1/exploration/selected_target` | `visualization_msgs/Marker` | selected frontier/return arrow |
| output | `/a1/exploration/failed_targets` | `visualization_msgs/Marker` | red de-duplicated failures |
| output | `/a1/exploration/trajectory` | `nav_msgs/Path` | actual TF-sampled path |
| output | `/a1/exploration/roi` | `geometry_msgs/PolygonStamped` | authoritative map-frame ROI; replaced at each action |
| output | `/a1/exploration/scope` | `visualization_msgs/Marker` | backward-compatible blue ROI marker |

See `a1_navigation_interfaces/说明.md` for action error codes, timestamps,
success semantics, lifecycle, and compatibility.

## Start and test

Business package only:

```bash
roslaunch a1_exploration exploration.launch
```

Repeatable pure algorithm test:

```bash
catkin_make --pkg a1_exploration -DCATKIN_ENABLE_TESTING=ON
catkin_make run_tests_a1_exploration -DCATKIN_ENABLE_TESTING=ON
catkin_test_results build/test_results/a1_exploration
```

Use the dev launch and action client from `a1_navigation_tests` for the full
Gazebo/RViz/headless rosbag workflow.

## Multi-floor boundary for the next milestone

This package intentionally owns only a single floor identity and returns to the
outdoor start pose recorded for that action. It requests only the public main
entrance behavior needed to enter that floor; it does not orchestrate
elevators, stairs, room-by-room doors, or a multi-floor return stack.

Tomorrow's multi-floor orchestration belongs to:

- `a1_mission_manager`: floor sequence, building-level return stack, retry and
  mission completion;
- `a1_building_behavior`: door/elevator/stairs traversal;
- `a1_floor_mapping`: per-floor map cache, stable floor IDs, activation and
  history restoration;
- `a1_navigation`: rebind the active map while keeping MoveBaseAction stable.

The selector is already isolated behind `GridSpec` and the action binds an
explicit map identity. Tomorrow the mission manager can provide per-floor entry
poses and local polygons through `ExploreFloor` while reusing the same frontier
and return logic. The map-frame observable remains the standard
`geometry_msgs/PolygonStamped`.
