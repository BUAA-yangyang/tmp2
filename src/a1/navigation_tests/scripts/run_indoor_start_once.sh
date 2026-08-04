#!/usr/bin/env bash
set -euo pipefail

# DEV-ONLY post-entry local-capability acceptance. This is intentionally kept
# in a1_navigation_tests: its spawn pose and ROI are not production inputs.

if [ "$#" -ne 1 ]; then
  echo "usage: $0 ABSOLUTE_NEW_RESULT_DIRECTORY" >&2
  exit 64
fi

RESULT_DIR="$1"
case "$RESULT_DIR" in
  /*) ;;
  *)
    echo "result directory must be absolute: $RESULT_DIR" >&2
    exit 64
    ;;
esac
if [ -e "$RESULT_DIR" ]; then
  echo "refusing to overwrite existing result path: $RESULT_DIR" >&2
  exit 73
fi

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
RUNNER_HELPER="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/indoor_start_runner.py"
if [ ! -f "$WORKSPACE_DIR/auto.sh" ]; then
  echo "could not locate workspace auto.sh under $WORKSPACE_DIR" >&2
  exit 66
fi
if [ ! -x "$RUNNER_HELPER" ]; then
  echo "missing executable indoor-start runner helper: $RUNNER_HELPER" >&2
  exit 66
fi

mkdir -p "$RESULT_DIR"
RESULT_JSON="$RESULT_DIR/indoor_start_acceptance.json"
BAG_PATH="$RESULT_DIR/indoor_start_acceptance.bag"
SIM_LOG="$RESULT_DIR/simulation.log"
ACCEPTANCE_LOG="$RESULT_DIR/acceptance.log"
STARTUP_READINESS="$RESULT_DIR/startup_readiness.json"
RUN_ID="$(basename "$RESULT_DIR")"

SIM_PID=""
ACCEPTANCE_PID=""

process_is_running() {
  local pid="$1"
  local state
  if ! kill -0 "$pid" 2>/dev/null; then
    return 1
  fi
  state="$(ps -o stat= -p "$pid" 2>/dev/null || true)"
  [ -n "$state" ] && [ "${state#Z}" = "$state" ]
}

wait_for_process_exit() {
  local pid="$1"
  local attempts="$2"
  for _ in $(seq 1 "$attempts"); do
    if ! process_is_running "$pid"; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

shutdown_simulation() {
  if [ -n "$ACCEPTANCE_PID" ] && kill -0 "$ACCEPTANCE_PID" 2>/dev/null; then
    kill -INT "$ACCEPTANCE_PID" 2>/dev/null || true
    wait "$ACCEPTANCE_PID" 2>/dev/null || true
  fi
  if [ -n "$SIM_PID" ] && kill -0 "$SIM_PID" 2>/dev/null; then
    # auto.sh owns an isolated process group inside the dedicated test
    # container. SIGINT gives roslaunch, Gazebo, and junior_ctrl a normal exit.
    kill -INT -- "-$SIM_PID" 2>/dev/null || true
    if ! wait_for_process_exit "$SIM_PID" 300; then
      echo "simulation group ignored SIGINT; escalating to SIGTERM" >&2
      kill -TERM -- "-$SIM_PID" 2>/dev/null || true
      if ! wait_for_process_exit "$SIM_PID" 100; then
        echo "simulation group ignored SIGTERM; escalating to SIGKILL" >&2
        kill -KILL -- "-$SIM_PID" 2>/dev/null || true
        wait_for_process_exit "$SIM_PID" 50 || true
      fi
    fi
    wait "$SIM_PID" 2>/dev/null || true
  fi
}

on_signal() {
  shutdown_simulation
  exit 130
}
trap on_signal INT TERM
trap shutdown_simulation EXIT

set +u
source /opt/ros/noetic/setup.bash
source "$WORKSPACE_DIR/devel/setup.bash"
set -u

(
  cd "$WORKSPACE_DIR"
  exec setsid env \
    SEED=20260729 \
    FLOOR_COUNT=3 \
    ROOMS_PER_FLOOR=4 \
    GUI=false \
    PAUSED=true \
    AUTO_UNPAUSE=0 \
    ACCEPTANCE_MANAGED_PHYSICS=1 \
    START_CONTROLLER=1 \
    CONTROLLER_FOREGROUND=0 \
    START_BUILDING_CONTROL=0 \
    ENABLE_SENSOR_DATA=0 \
    ENABLE_LIVOX=1 \
    ENABLE_LIVOX_IMU=0 \
    ENABLE_REALSENSE=0 \
    ENABLE_FRONT_CAMERA=0 \
    ENABLE_REFEREE_ODOM=1 \
    PUBLISH_REFEREE_TF=1 \
    ENABLE_GROUND_TRUTH=1 \
    ENABLE_FOOT_CONTACT_SENSOR=1 \
    ENABLE_FOOT_FORCE_VISUAL=0 \
    ENABLE_POINTCLOUD_CONVERTER=0 \
    POINTCLOUD_USE_GROUND_TRUTH_ODOM=1 \
    WRITE_GENERATED_TRUTH_COPY=false \
    ROBOT_X=0.0 \
    ROBOT_Y=2.5 \
    ROBOT_Z=0.6 \
    ROBOT_YAW=1.5708 \
    UNITREE_CTRL_DT=0.004 \
    "$WORKSPACE_DIR/auto.sh"
) >"$SIM_LOG" 2>&1 &
SIM_PID=$!

# auto.sh intentionally clears stale controller processes before starting a
# fresh graph.  Do not put the literal junior_ctrl pid-file argument on a live
# helper command line until that cleanup is complete: pkill -f would otherwise
# match the readiness helper itself and abort before motion.
AUTO_CLEANUP_COMPLETE=false
for _ in $(seq 1 200); do
  if grep -Fq "Sourcing ROS environment..." "$SIM_LOG" 2>/dev/null; then
    AUTO_CLEANUP_COMPLETE=true
    break
  fi
  if ! process_is_running "$SIM_PID"; then
    echo "simulation process exited during auto.sh startup cleanup" >&2
    exit 74
  fi
  sleep 0.1
done
if [ "$AUTO_CLEANUP_COMPLETE" != true ]; then
  echo "timed out waiting for auto.sh startup cleanup to complete" >&2
  exit 74
fi

"$RUNNER_HELPER" wait-ready \
  --simulation-log "$SIM_LOG" \
  --simulation-pid "$SIM_PID" \
  --controller-pid-file "$WORKSPACE_DIR/logs/junior_ctrl.pid" \
  --timeout 300.0 \
  --poll-interval 0.10 \
  --stable-observations 3 \
  >"$STARTUP_READINESS"

if [ -e "$RESULT_JSON" ] || [ -e "$BAG_PATH" ]; then
  echo "result artifacts appeared before acceptance; refusing overwrite" >&2
  exit 73
fi

set +e
roslaunch a1_navigation_tests single_floor_indoor_start_acceptance.launch \
  run_id:="$RUN_ID" \
  output:="$RESULT_JSON" \
  bag_path:="$BAG_PATH" \
  >"$ACCEPTANCE_LOG" 2>&1 &
ACCEPTANCE_PID=$!
wait "$ACCEPTANCE_PID"
LAUNCH_STATUS=$?
ACCEPTANCE_PID=""
set -e

# The acceptance node completes its guarded safe-stop before roslaunch exits.
shutdown_simulation
SIM_PID=""

if [ ! -s "$BAG_PATH" ] || [ -e "$BAG_PATH.active" ]; then
  echo "sealed non-empty acceptance bag was not produced" >&2
  exit 74
fi
sha256sum "$BAG_PATH" >"$RESULT_DIR/bag.sha256"
if ! rosbag info --yaml "$BAG_PATH" >"$RESULT_DIR/bag_info.yaml"; then
  echo "sealed acceptance bag is not readable/indexed" >&2
  exit 74
fi
if [ -d "$WORKSPACE_DIR/logs" ]; then
  cp -a "$WORKSPACE_DIR/logs" "$RESULT_DIR/workspace_logs"
fi

if [ "$LAUNCH_STATUS" -ne 0 ]; then
  echo "indoor-start roslaunch failed with status $LAUNCH_STATUS" >&2
  exit "$LAUNCH_STATUS"
fi
if [ ! -s "$RESULT_JSON" ]; then
  echo "acceptance JSON was not produced" >&2
  exit 74
fi
"$RUNNER_HELPER" check-result "$RESULT_JSON"

echo "DEV-ONLY indoor-start milestone passed: $RESULT_DIR"
