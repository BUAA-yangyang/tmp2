#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE_DIR"

as_ros_bool() {
  case "$1" in
    1|true|TRUE|True|yes|YES|on|ON) printf "true" ;;
    0|false|FALSE|False|no|NO|off|OFF) printf "false" ;;
    *) printf "%s" "$1" ;;
  esac
}

SEED="${SEED:-}"
FLOOR_COUNT="${FLOOR_COUNT:-3}"
ROOMS_PER_FLOOR="${ROOMS_PER_FLOOR:-4}"
BUILDING_WIDTH="${BUILDING_WIDTH:-20.0}"
BUILDING_LENGTH="${BUILDING_LENGTH:-36.0}"
DANGER_COUNT="${DANGER_COUNT:-3:6}"
DISTRACTOR_COUNT="${DISTRACTOR_COUNT:-4:8}"
GUI="${GUI:-true}"
PAUSED="${PAUSED:-true}"
START_CONTROLLER="${START_CONTROLLER:-1}"
START_VIRTUAL_JOY="${START_VIRTUAL_JOY:-0}"
CONTROLLER_FOREGROUND="${CONTROLLER_FOREGROUND:-1}"
START_BUILDING_CONTROL="${START_BUILDING_CONTROL:-1}"
ENABLE_SENSOR_DATA_DEFAULT="${ENABLE_SENSORS:-1}"
ENABLE_SENSOR_DATA="$(as_ros_bool "${ENABLE_SENSOR_DATA:-$ENABLE_SENSOR_DATA_DEFAULT}")"
ENABLE_REFEREE_ODOM="$(as_ros_bool "${ENABLE_REFEREE_ODOM:-1}")"
ENABLE_GROUND_TRUTH="$(as_ros_bool "${ENABLE_GROUND_TRUTH:-1}")"
ENABLE_FOOT_FORCE_VISUAL="$(as_ros_bool "${ENABLE_FOOT_FORCE_VISUAL:-0}")"
ENABLE_POINTCLOUD_CONVERTER="$(as_ros_bool "${ENABLE_POINTCLOUD_CONVERTER:-1}")"
POINTCLOUD_USE_GROUND_TRUTH_ODOM="$(as_ros_bool "${POINTCLOUD_USE_GROUND_TRUTH_ODOM:-1}")"
WRITE_GENERATED_TRUTH_COPY="$(as_ros_bool "${WRITE_GENERATED_TRUTH_COPY:-1}")"
UNITREE_CTRL_DT="${UNITREE_CTRL_DT:-0.002}"
UNITREE_STAND_DURATION="${UNITREE_STAND_DURATION:-3.0}"
UNITREE_STAND_SETTLE_DURATION="${UNITREE_STAND_SETTLE_DURATION:-0.5}"
UNITREE_SIM_PASSIVE_HOLD="$(as_ros_bool "${UNITREE_SIM_PASSIVE_HOLD:-1}")"
UNITREE_ENABLE_REALTIME="${UNITREE_ENABLE_REALTIME:-auto}"
UNITREE_LOG_WAIT_WARNINGS="$(as_ros_bool "${UNITREE_LOG_WAIT_WARNINGS:-0}")"
UNITREE_ENABLE_AMP_LOG="$(as_ros_bool "${UNITREE_ENABLE_AMP_LOG:-0}")"
GAZEBO_PHYSICS_MAX_STEP_SIZE="${GAZEBO_PHYSICS_MAX_STEP_SIZE:-0.002}"
GAZEBO_PHYSICS_REAL_TIME_UPDATE_RATE="${GAZEBO_PHYSICS_REAL_TIME_UPDATE_RATE:-500}"
GAZEBO_PHYSICS_ODE_ITERS="${GAZEBO_PHYSICS_ODE_ITERS:-40}"
GAZEBO_PHYSICS_CONTACT_MAX_CORRECTING_VEL="${GAZEBO_PHYSICS_CONTACT_MAX_CORRECTING_VEL:-5.0}"
ROBOT_X="${ROBOT_X:-0.0}"
ROBOT_Y="${ROBOT_Y:--3.2}"
ROBOT_Z="${ROBOT_Z:-0.09}"
ROBOT_YAW="${ROBOT_YAW:-1.5708}"

echo "Terminating previous Gazebo, launch, controller, and optional joystick processes..."
pkill -f "roslaunch unitree_guide multi_floor_gazeboSim.launch" 2>/dev/null || true
pkill -f "roslaunch .*multi_floor_gazeboSim.launch" 2>/dev/null || true
pkill -f "building_generator_classic_control" 2>/dev/null || true
pkill -f "gzserver|gzclient|gazebo" 2>/dev/null || true
pkill -f "junior_ctrl" 2>/dev/null || true
pkill -f "virtual_joy.py" 2>/dev/null || true

echo "Sourcing ROS environment..."
source /opt/ros/noetic/setup.bash
source "$WORKSPACE_DIR/devel/setup.bash"
export ROS_PACKAGE_PATH="$WORKSPACE_DIR/src:${ROS_PACKAGE_PATH:-}"
export CMAKE_PREFIX_PATH="$WORKSPACE_DIR/devel:${CMAKE_PREFIX_PATH:-}"

GENERATOR_SCRIPT="$WORKSPACE_DIR/src/building_obstacles/scripts/generate_competition_scene.py"
if [ ! -f "$GENERATOR_SCRIPT" ]; then
  GENERATOR_SCRIPT="$(rospack find building_obstacles)/scripts/generate_competition_scene.py"
fi
UNITREE_GAZEBO_MODELS="$WORKSPACE_DIR/src/unitree_guide/unitree_ros/unitree_gazebo/models"
if [ ! -d "$UNITREE_GAZEBO_MODELS" ]; then
  UNITREE_GAZEBO_MODELS="$(rospack find unitree_gazebo)/models"
fi
LAUNCH_FILE="$WORKSPACE_DIR/src/unitree_guide/unitree_guide/unitree_guide/launch/multi_floor_gazeboSim.launch"
if [ ! -f "$LAUNCH_FILE" ]; then
  LAUNCH_FILE="$(rospack find unitree_guide)/launch/multi_floor_gazeboSim.launch"
fi
SCENE_OUTPUT_DIR="$WORKSPACE_DIR/generated_building"
RESULTS_DIR="$WORKSPACE_DIR/results"
REFEREE_RESULTS_DIR="${REFEREE_RESULTS_DIR:-$RESULTS_DIR}"
TEAM_INFO_DIR="${TEAM_INFO_DIR:-$SCENE_OUTPUT_DIR}"
mkdir -p "$SCENE_OUTPUT_DIR" "$RESULTS_DIR" "$REFEREE_RESULTS_DIR" "$TEAM_INFO_DIR" "$WORKSPACE_DIR/logs"

echo "Generating competition scene..."
GENERATOR_ARGS=(
  --output-dir "$SCENE_OUTPUT_DIR"
  --results-dir "$RESULTS_DIR"
  --floor-count "$FLOOR_COUNT"
  --rooms-per-floor "$ROOMS_PER_FLOOR"
  --width "$BUILDING_WIDTH"
  --length "$BUILDING_LENGTH"
  --danger-count "$DANGER_COUNT"
  --distractor-count "$DISTRACTOR_COUNT"
  --robot-x "$ROBOT_X"
  --robot-y "$ROBOT_Y"
  --robot-z "$ROBOT_Z"
  --robot-yaw "$ROBOT_YAW"
)
if [ -n "$SEED" ]; then
  GENERATOR_ARGS+=(--seed "$SEED")
fi
GENERATOR_HELP="$(python3 "$GENERATOR_SCRIPT" --help 2>&1 || true)"
if [[ "$GENERATOR_HELP" == *"--referee-results-dir"* ]]; then
  GENERATOR_ARGS+=(--referee-results-dir "$REFEREE_RESULTS_DIR")
fi
if [[ "$GENERATOR_HELP" == *"--team-info-dir"* ]]; then
  GENERATOR_ARGS+=(--team-info-dir "$TEAM_INFO_DIR")
fi
if [[ "$GENERATOR_HELP" == *"--physics-max-step-size"* ]]; then
  GENERATOR_ARGS+=(--physics-max-step-size "$GAZEBO_PHYSICS_MAX_STEP_SIZE")
fi
if [[ "$GENERATOR_HELP" == *"--physics-real-time-update-rate"* ]]; then
  GENERATOR_ARGS+=(--physics-real-time-update-rate "$GAZEBO_PHYSICS_REAL_TIME_UPDATE_RATE")
fi
if [[ "$GENERATOR_HELP" == *"--physics-ode-iters"* ]]; then
  GENERATOR_ARGS+=(--physics-ode-iters "$GAZEBO_PHYSICS_ODE_ITERS")
fi
if [[ "$GENERATOR_HELP" == *"--physics-contact-max-correcting-vel"* ]]; then
  GENERATOR_ARGS+=(--physics-contact-max-correcting-vel "$GAZEBO_PHYSICS_CONTACT_MAX_CORRECTING_VEL")
fi
if [ "$WRITE_GENERATED_TRUTH_COPY" = "false" ] && [[ "$GENERATOR_HELP" == *"--no-generated-truth-copy"* ]]; then
  GENERATOR_ARGS+=(--no-generated-truth-copy)
fi
python3 "$GENERATOR_SCRIPT" "${GENERATOR_ARGS[@]}" \
  > "$SCENE_OUTPUT_DIR/scene_manifest.stdout.json"

export BUILDING_WORLD_FILE="$SCENE_OUTPUT_DIR/competition_scene.world"
export COMPETITION_ROBOT_X="$ROBOT_X"
export COMPETITION_ROBOT_Y="$ROBOT_Y"
export COMPETITION_ROBOT_Z="$ROBOT_Z"
export COMPETITION_ROBOT_YAW="$ROBOT_YAW"
export UNITREE_CTRL_DT
export UNITREE_STAND_DURATION
export UNITREE_STAND_SETTLE_DURATION
export UNITREE_SIM_PASSIVE_HOLD
export UNITREE_ENABLE_REALTIME
export UNITREE_LOG_WAIT_WARNINGS
export UNITREE_ENABLE_AMP_LOG
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}:$SCENE_OUTPUT_DIR:$UNITREE_GAZEBO_MODELS"

echo "=========================================="
echo "Competition scene is ready"
echo "  World:   $BUILDING_WORLD_FILE"
echo "  Truth:   $REFEREE_RESULTS_DIR/danger_truth.json"
echo "  TeamInfo:$TEAM_INFO_DIR/team_scene_info.json"
echo "  Manifest:$SCENE_OUTPUT_DIR/scene_manifest.json"
echo "  Result:  $RESULTS_DIR/detected_danger.json"
echo "  Sensor model: visible"
echo "  Sensor data:  $ENABLE_SENSOR_DATA"
echo "  Foot force visual: $ENABLE_FOOT_FORCE_VISUAL"
echo "  Controller dt: $UNITREE_CTRL_DT s"
echo "  Stand duration: $UNITREE_STAND_DURATION s"
echo "  Stand settle: $UNITREE_STAND_SETTLE_DURATION s"
echo "  Passive hold: $UNITREE_SIM_PASSIVE_HOLD"
echo "  AMP log: $UNITREE_ENABLE_AMP_LOG"
echo "  Gazebo physics: max_step=$GAZEBO_PHYSICS_MAX_STEP_SIZE update_rate=$GAZEBO_PHYSICS_REAL_TIME_UPDATE_RATE ode_iters=$GAZEBO_PHYSICS_ODE_ITERS"
echo "  Wait warning log: $UNITREE_LOG_WAIT_WARNINGS"
echo "=========================================="

if [ "$START_VIRTUAL_JOY" = "1" ]; then
  echo "Starting virtual joystick. This may require uinput permissions."
  rosrun unitree_guide virtual_joy.py > "$WORKSPACE_DIR/logs/virtual_joy.log" 2>&1 &
  echo $! > "$WORKSPACE_DIR/logs/virtual_joy.pid"
fi

echo "Launching Gazebo, Unitree A1 model, sensors, and ROS interfaces..."
roslaunch "$LAUNCH_FILE" \
  gui:="$GUI" \
  paused:="$PAUSED" \
  user_debug:=False \
  rname:=a1 \
  robot_x:="$ROBOT_X" \
  robot_y:="$ROBOT_Y" \
  robot_z:="$ROBOT_Z" \
  robot_yaw:="$ROBOT_YAW" \
  enable_sensor_data:="$ENABLE_SENSOR_DATA" \
  enable_referee_odom:="$ENABLE_REFEREE_ODOM" \
  enable_ground_truth:="$ENABLE_GROUND_TRUTH" \
  enable_foot_force_visual:="$ENABLE_FOOT_FORCE_VISUAL" \
  enable_pointcloud_converter:="$ENABLE_POINTCLOUD_CONVERTER" \
  pointcloud_use_ground_truth_odom:="$POINTCLOUD_USE_GROUND_TRUTH_ODOM" \
  > "$WORKSPACE_DIR/logs/competition_gazebo.log" 2>&1 &
LAUNCH_PID=$!
echo "$LAUNCH_PID" > "$WORKSPACE_DIR/logs/competition_gazebo.pid"
sleep 6

if [ "$START_BUILDING_CONTROL" = "1" ]; then
  echo "Starting building door/elevator control service..."
  rosrun building_generator_classic building_generator_classic_control \
    --door-config "$SCENE_OUTPUT_DIR/door_config.yaml" \
    --elevator-config "$SCENE_OUTPUT_DIR/elevator_config.yaml" \
    > "$WORKSPACE_DIR/logs/building_control.log" 2>&1 &
  echo $! > "$WORKSPACE_DIR/logs/building_control.pid"
fi

if [ "$START_CONTROLLER" = "1" ]; then
  if [ "$CONTROLLER_FOREGROUND" = "1" ]; then
    echo "Starting junior_ctrl controller in the foreground."
    echo "UNITREE_CTRL_DT=$UNITREE_CTRL_DT seconds."
    echo "Use keyboard input in this terminal: 1 = passive/down, 2 = stand, 6 = RL mode."
    "$WORKSPACE_DIR/devel/lib/unitree_guide/junior_ctrl"
  else
    echo "Starting junior_ctrl controller in the background. Keyboard state switching may not be available."
    echo "UNITREE_CTRL_DT=$UNITREE_CTRL_DT seconds."
    "$WORKSPACE_DIR/devel/lib/unitree_guide/junior_ctrl" \
      > "$WORKSPACE_DIR/logs/junior_ctrl.log" 2>&1 &
    echo $! > "$WORKSPACE_DIR/logs/junior_ctrl.pid"
  fi
fi

echo "Simulation startup command completed."
echo "Controller mode remains governed by unitree_guide keyboard/joy input; publish geometry_msgs/Twist to /cmd_vel after RL mode is enabled."
