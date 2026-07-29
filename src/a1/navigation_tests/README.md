# A1 Navigation and Exploration Tests

## Single-floor exploration, DEV-ONLY

This workflow uses the real simulated Livox scan and the production
floor_mapping/exploration algorithms. Gazebo truth is used only as a clearly
marked localization substitute through `/Odometry_gazebo`; FAST-LIO2 is not
claimed as integrated.

Build every runtime package first; `a1_floor_mapping` contains a C++ node and
must not be omitted even when the Python exploration tests already pass:

```bash
catkin_make --pkg a1_navigation_interfaces a1_floor_mapping a1_exploration \
  a1_navigation a1_navigation_tests
```

Start the simulation:

```bash
GUI=false \
ENABLE_SENSOR_DATA=0 \
ENABLE_LIVOX=1 \
ENABLE_LIVOX_IMU=0 \
ENABLE_REALSENSE=0 \
ENABLE_POINTCLOUD_CONVERTER=0 \
ENABLE_REFEREE_ODOM=1 \
PUBLISH_REFEREE_TF=1 \
./auto.sh
```

Switch the controller from state `2` to state `5`, then start the complete
headless stack:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch a1_navigation_tests single_floor_exploration_dev.launch \
  use_cmd_mux:=true use_rviz:=false
```

This unattended acceptance launch deliberately limits DWA to
`0.20 m/s` longitudinal, zero lateral velocity, and `0.30 rad/s` yaw. The
production navigation defaults remain `0.40/0.25/0.60`; this non-holonomic,
conservative envelope is intended to avoid combined lateral/turn commands
that can topple the classic Gazebo gait during long autonomous runs.
Lowering the classic gait below this envelope was tested and made foot
placement less stable; `0.20/0.30` is the long-run validated dev setting.

Confirm prerequisites before sending the action:

```bash
rostopic echo -n 1 /a1/floor_mapping/status
rostopic echo -n 1 /a1/cmd_mux/status
rosnode info /cmd_vel_guard
```

The mapping status must be `MAPPING` with `map_valid=true`,
`obstacle_cloud_valid=true`, and a stable generation/session. `/cmd_vel` must
have exactly one publisher: `/cmd_vel_guard`.

Record a repeatable bag on the remote host:

```bash
rosbag record -O /tmp/a1_single_floor_exploration.bag \
  /clock /tf /tf_static \
  /scan /Odometry_gazebo \
  /a1/localization/odom \
  /a1/localization/status \
  /a1/localization/supervisor_status \
  /a1/floor_mapping/map \
  /a1/floor_mapping/status \
  /a1/floor_mapping/marking_cloud \
  /a1/floor_mapping/obstacle_cloud \
  /a1/exploration/status \
  /a1/exploration/frontiers \
  /a1/exploration/selected_target \
  /a1/exploration/failed_targets \
  /a1/exploration/trajectory \
  /a1/exploration/scope \
  /move_base/status \
  /move_base/goal \
  /move_base/result \
  /move_base/GlobalPlanner/plan \
  /move_base/DWAPlannerROS/local_plan \
  /cmd_vel_nav /cmd_vel
```

Send the long-running action without RViz or a remote desktop:

```bash
rosrun a1_navigation_tests single_floor_exploration_client.py \
  _floor_id:=-1 \
  _timeout_s:=900 \
  _wall_timeout:=7200 \
  _output:=/tmp/a1_single_floor_exploration_result.json
```

`success=true` means all of the following were observed by the action server:

- no eligible frontier on the configured number of distinct map updates, or
  all remaining frontiers were visited/unreachable;
- return MoveBaseAction succeeded;
- final pose is within configured position and yaw tolerances;
- `/cmd_vel` remained fresh and zero for the configured settling interval.

The result JSON and rosbag are runtime artifacts and must not be committed.

## Automated Gazebo single-floor acceptance, DEV-ONLY

`single_floor_gazebo_acceptance.launch` runs the production door, mapping,
move_base, exploration, return, final-zero, and guarded fixed-stand chain
without RViz or remote-desktop input. It never reads the generated world,
layout metadata, model/link states, or referee truth for door/entry decisions;
the closed/open passage evidence comes from the Livox-derived OccupancyGrid.
The existing `/Odometry_gazebo` bridge remains an explicitly DEV-ONLY
localization substitute and is not a competition-localization claim.

Start a fresh deterministic simulation with real Livox, foot contacts, and
the controller in the background:

```bash
GUI=false SEED=20260729 \
FLOOR_COUNT=1 ROOMS_PER_FLOOR=4 \
AUTO_UNPAUSE=0 ACCEPTANCE_MANAGED_PHYSICS=1 \
START_CONTROLLER=1 CONTROLLER_FOREGROUND=0 START_VIRTUAL_JOY=0 \
ENABLE_SENSOR_DATA=0 ENABLE_LIVOX=1 ENABLE_LIVOX_IMU=0 \
ENABLE_REALSENSE=0 ENABLE_POINTCLOUD_CONVERTER=0 \
ENABLE_REFEREE_ODOM=1 PUBLISH_REFEREE_TF=1 \
ENABLE_GROUND_TRUTH=1 ENABLE_FOOT_CONTACT_SENSOR=1 \
ENABLE_FOOT_FORCE_VISUAL=0 WRITE_GENERATED_TRUTH_COPY=false \
./auto.sh
```

With `ACCEPTANCE_MANAGED_PHYSICS=1`, Gazebo remains paused through robot
spawn. The acceptance node first proves that the named rosbag recorder is
subscribed to `/clock`, the ExploreFloor goal topic, and the MoveBase goal
topic; only then does it unpause physics and immediately command fixed stand.
Use a new output directory for every run.

The bounded gate uses a floor-entry-local 12 m by 12 m ROI. It must open the
public main entrance, show the occupied entry corridor becoming a known-free
OccupancyGrid path, reach the entry, gain at least 20 new indoor cells,
successfully execute at least two real frontier goals, naturally finish with
no reachable frontier in that ROI, return to RECORD_START, settle `/cmd_vel`,
and complete the guarded all-foot fixed-stand transition:

```bash
roslaunch a1_navigation_tests single_floor_gazebo_acceptance.launch \
  run_id:=bounded roi_depth:=12.0 roi_half_width:=6.0 \
  minimum_frontier_successes:=2 action_timeout_sim:=600 \
  output:=/workspace/SimEnv/results/single_floor/bounded.json \
  bag_path:=/workspace/SimEnv/results/single_floor/bounded.bag
```

Only after that gate passes, restart the simulation from a clean process graph
and run the one permitted complete floor acceptance:

```bash
roslaunch a1_navigation_tests single_floor_gazebo_acceptance.launch \
  run_id:=full roi_depth:=40.0 roi_half_width:=8.65 \
  minimum_frontier_successes:=2 action_timeout_sim:=1200 \
  action_wall_timeout:=14400 \
  output:=/workspace/SimEnv/results/single_floor/full.json \
  bag_path:=/workspace/SimEnv/results/single_floor/full.bag
```

The JSON records simulation duration, public door result, map corridor counts,
entry map gain, frontier targets/outcomes, completion reason, trajectory,
coverage denominator, return error, maximum attitude, final zero, foot forces,
gyro, `safe_stand_ready`, and the inferred fixed-stand transition. Any failed
gate cancels the action, commands zero, requests guarded all-foot support,
latches the safety lock if that transition cannot be verified, writes JSON,
and exits nonzero so rosbag closes before external process cleanup.

## Constant-velocity fall diagnostic, DEV-ONLY

Do not use the full exploration launch for gait/fall diagnosis.  The dedicated
launch starts only `a1_cmd_mux`, a constant-command monitor, and `rosbag`.
`move_base` and `a1_exploration` are intentionally absent.

Start each case from a freshly generated simulation with foot contact sensors
enabled.  Keep the same seed, physics settings, controller dt, command, and
recording set across rows; change only the matrix variable:

| Case | Surface/start | Yaw | Livox |
|---|---|---:|---:|
| `ground_south_livox` | open ground_plane | -100 deg | on |
| `foundation_south_livox` | foundation control lane | -100 deg | on |
| `ground_south_no_livox` | same open ground_plane | -100 deg | off |
| `ground_north_livox` | open ground_plane | +90 deg | on |

The generated competition world has an important caveat: the standard
`ground_plane` remains under the whole building, while the foundation top is
also exactly `z=0`.  Consequently an in-building run observes two coplanar
collisions (`ground_plane + foundation`), not a clean second material.  Record
this row as such, or use an isolated dev-only copy of the foundation collision
as a long unobstructed control lane.  Do not interpret it as a pure friction
comparison.

Example simulation settings:

```bash
GUI=false \
SEED=20260728 \
ROBOT_X=0.0 \
ROBOT_Y=-5.0 \
ROBOT_YAW=-1.745329252 \
ENABLE_SENSOR_DATA=0 \
ENABLE_LIVOX=1 \
ENABLE_LIVOX_IMU=0 \
ENABLE_REALSENSE=0 \
ENABLE_POINTCLOUD_CONVERTER=0 \
ENABLE_REFEREE_ODOM=1 \
PUBLISH_REFEREE_TF=0 \
ENABLE_GROUND_TRUTH=1 \
ENABLE_FOOT_CONTACT_SENSOR=1 \
ENABLE_FOOT_FORCE_VISUAL=0 \
WRITE_GENERATED_TRUTH_COPY=false \
./auto.sh
```

Mode 5 is still the controller-owned interface.  Switch `2` (fixed stand), let
the stand ramp finish, then switch `5`.  In another terminal create the output
directory and launch the case:

```bash
roslaunch a1_navigation_tests constant_velocity_stability_diagnostic.launch \
  case_id:=ground_south_livox \
  surface:=ground_plane \
  livox_enabled:=true \
  expected_x:=0.0 \
  expected_y:=-5.0 \
  expected_yaw_deg:=-100.0 \
  output:=/workspace/SimEnv/results/stability_matrix/ground_south_livox.json \
  bag_path:=/workspace/SimEnv/results/stability_matrix/ground_south_livox.bag
```

Before releasing motion, the monitor verifies the start pose, mode-5 ready
heartbeat, IMU/odometry, all 12 motor command/state publishers, and all four
foot-force publishers.  It publishes exactly `0.15/0/0` through
`/cmd_vel_nav`, records at least 200 simulation seconds, and publishes
roll/pitch/yaw plus sliding RTF on
`/a1/navigation_tests/stability_diagnostic`.  At 35 degrees tilt, clock stall,
missing prerequisites, or timeout it latches `/a1_cmd_mux/safety_lock=True`
and commands zero.  The JSON contains an explicit outcome, validity flag,
distance, tilt/rate extrema, RTF distribution, and final-zero result.

After each case, derive the same pre-fall metrics from the bag:

```bash
rosrun a1_navigation_tests stability_bag_analyzer.py \
  /workspace/SimEnv/results/stability_matrix/ground_south_livox.bag \
  --output /workspace/SimEnv/results/stability_matrix/ground_south_livox_analysis.json
```

The analyzer reports the 5/10/20/35/45-degree crossing times, per-foot contact
duty and force extrema, the final 0.2-second force balance before 35 degrees,
and every joint command/state extremum up to that crossing.

### 2026-07-28 time-base diagnosis

All rows used `vx=0.15 m/s`, `vy=0`, `wz=0`; no row launched move_base or
exploration. `foundation_overlap` is not a pure surface comparison because the
generated foundation and standard ground plane are coplanar.

| Case | Livox | RTF | Sim duration | Max roll/pitch | Outcome |
|---|---:|---:|---:|---:|---|
| ground, yaw -100 deg | on | 0.226–0.275 | 0.832 s | 117.65/18.06 deg | tilt stop |
| ground + coplanar foundation, yaw -100 deg | on | 0.253–0.283 | 0.612 s | 35.41/25.95 deg | tilt stop |
| ground, yaw +90 deg | on | 0.276–0.321 | 0.852 s | 35.39/9.64 deg | tilt stop |
| ground, yaw -100 deg | off | 0.998–1.000 | 200.002 s | 1.51/0.38 deg | pass |
| ground, Livox off, forced RTF 0.25 | off | 0.250 | 6.858 s | 35.14/4.90 deg | tilt stop |
| same forced RTF after phase-clock fix | off | 0.250 | 30.002 s | 1.36/0.35 deg | pass |
| ground after phase-clock fix, yaw -100 deg | on | 0.237–0.354 | 200.000 s | 1.57/0.35 deg | pass |

The discriminating control is the forced-low-RTF row: low RTF alone reproduced
the fall with Livox disabled. Ground surface, yaw sign, and a combined
linear/angular move_base command are therefore not necessary causes.

`WaveGenerator` previously advanced trot phase from wall time while Gazebo
state, the estimator, and trajectory generators advanced on the controller
period in simulation time. At RTF 0.25, gait phase ran approximately four times
faster than robot dynamics. It now advances by the validated controller `dt`,
keeping phase, estimator, and Gazebo state on one discrete control clock. The
post-fix 200-second bag contains all 12 motor command/state streams, four foot
forces, IMU, odometry, command chain, `/clock`, RTF diagnostics, and a final
zero command.

Artifacts are intentionally outside git:

```text
/workspace/SimEnv/results/stability_matrix/
  ground_south_livox_phasefix_200s.{bag,json}
  ground_south_livox_phasefix_200s_analysis.json
  ground_south_no_livox_forced_rtf025_phasefix.{bag,json}
  ground_south_no_livox_forced_rtf025_phasefix_analysis.json
```

## DEV-ONLY indoor-start milestone

`single_floor_indoor_start_acceptance.launch` isolates the capability after the
main entrance. Its test adapter starts with `RECORD_START` on indoor flat
ground and suppresses only door request, entry transit, and post-entry map
gates. Production frontier selection, MoveBaseAction navigation, return,
final-zero, cmd mux/guard, and guarded all-foot safe-stop remain active.

This is not an official-start competition acceptance. The test-only spawn
pose, fixed scene seed, and expected 12 m by 12 m ROI are owned by
`a1_navigation_tests`; production launch defaults remain unchanged. Run it
only in a dedicated container and supply a new absolute result directory:

```bash
rosrun a1_navigation_tests run_indoor_start_once.sh \
  /workspace/SimEnv/results/indoor_start_YYYYMMDD_HHMMSS
```

The runner refuses to overwrite an existing path. Its bag recorder must be
subscribed to `/clock`, the ExploreFloor goal, and the MoveBase goal before
physics is unpaused. Passing requires three spatially distinct indoor frontier
targets, two successful frontier goals with real motion, strict exhausted-ROI
completion, return within 0.40 m and 0.65 rad, fresh continuous zero on both
`/cmd_vel_nav` and `/cmd_vel`, and `safe_stand_ready` before fixed stand.
Before launching that recorder/acceptance pair, the runner also requires the
simulation startup sentinel, a live controller process, and three consecutive
`pause: True` observations after explicit pause calls. Its final verdict parses
the top-level JSON boolean; a nested safe-stop success cannot mask a failed
acceptance.

## Fast repeatable tests

The business selector has ROS-independent tests, and the runtime rostest
provides a fake OccupancyGrid, mapping health, TF, `make_plan`, and
MoveBaseAction. It verifies at least two autonomous frontier goals followed by
the recorded start goal:

```bash
catkin_make --pkg a1_navigation_interfaces a1_exploration a1_navigation_tests \
  -DCATKIN_ENABLE_TESTING=ON
source devel/setup.bash
catkin_make run_tests_a1_exploration run_tests_a1_navigation_tests \
  -DCATKIN_ENABLE_TESTING=ON
catkin_test_results build/test_results/a1_exploration \
  build/test_results/a1_navigation_tests
```
