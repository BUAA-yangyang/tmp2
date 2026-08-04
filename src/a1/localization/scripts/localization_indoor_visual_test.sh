#!/usr/bin/env bash
set -Eeuo pipefail

# Interactive localization acceptance test for an already running SimEnv.
# Start auto.sh, press 2, and then run this script in a second terminal.

WORKSPACE="${SIMENV_WORKSPACE:-/workspace/SimEnv}"
DURATION="${1:-300}"
OUTPUT_ROOT="${2:-$WORKSPACE/artifacts/localization_indoor_$(date +%Y%m%d_%H%M%S)}"
RVIZ_CONFIG="$WORKSPACE/src/third_party/FAST_LIO/rviz_cfg/loam_livox.rviz"
PIDS=()

source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"
mkdir -p "$OUTPUT_ROOT/logs"

cleanup_runtime() {
  set +e
  for pid in "${PIDS[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
  rosnode kill /a1_localization/localization_supervisor \
    /a1_localization/localization_map_manager \
    /a1_localization/pointcloud_adapter \
    /localization_viz_cloud /localization_viz_map /localization_viz_odom \
    >/dev/null 2>&1 || true
}
trap cleanup_runtime INT TERM EXIT

wait_for_topic() {
  local topic="$1"
  local timeout_s="$2"
  timeout "$timeout_s" rostopic echo -n 1 "$topic" >/dev/null
}

echo "[1/8] Checking simulation and sensor inputs..."
if ! ROSNODES="$(rosnode list 2>/dev/null)"; then
  cat >&2 <<'EOF'
No ROS master is available. This acceptance script does not start auto.sh,
because junior_ctrl requires an interactive terminal for keys 2/4/W/Space.

In terminal A, run:
  cd /workspace/SimEnv
  SEED=1 GUI=true ENABLE_REALSENSE=0 ENABLE_FRONT_CAMERA=0 \
  ENABLE_LIVOX=1 ENABLE_LIVOX_IMU=1 ENABLE_POINTCLOUD_CONVERTER=0 \
  ENABLE_REFEREE_ODOM=0 ENABLE_GROUND_TRUTH=1 START_BUILDING_CONTROL=1 \
  ./auto.sh

Wait for the junior_ctrl prompt, press 2, and wait until A1 is standing.
Then run this localization test again in terminal B.
EOF
  exit 4
fi
if ! grep -qx /gazebo <<<"$ROSNODES"; then
  echo "ROS master is running, but /gazebo is absent. Start auto.sh first." >&2
  exit 4
fi
wait_for_topic /scan 15
wait_for_topic /trunk_imu 15
wait_for_topic /clock 15

python3 - <<'PY'
import rospy
from gazebo_msgs.srv import GetModelState

rospy.init_node("localization_visual_preflight", anonymous=True, disable_signals=True)
rospy.wait_for_service("/gazebo/get_model_state", timeout=10.0)
state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)("a1_gazebo", "world")
if not state.success:
    raise SystemExit("Could not read A1 model state")
z = state.pose.position.z
speed = (state.twist.linear.x ** 2 + state.twist.linear.y ** 2 + state.twist.linear.z ** 2) ** 0.5
print(f"A1 preflight: z={z:.3f} m, linear_speed={speed:.4f} m/s")
if z < 0.20:
    raise SystemExit("A1 is not standing. Restart auto.sh and press 2 before retrying.")
if speed > 0.05:
    raise SystemExit("A1 is still moving. Stop it and retry after it settles.")
PY

if rosnode list | grep -q '^/a1_localization/'; then
  echo "Existing localization nodes detected; refusing to start a duplicate instance." >&2
  exit 2
fi

echo "[2/8] Starting localization..."
roslaunch a1_localization localization.launch \
  >"$OUTPUT_ROOT/logs/localization.log" 2>&1 &
PIDS+=("$!")

python3 - <<'PY'
import rospy
from diagnostic_msgs.msg import DiagnosticStatus

rospy.init_node("localization_visual_health_wait", anonymous=True, disable_signals=True)
deadline = rospy.Time.now() + rospy.Duration(45.0)
while not rospy.is_shutdown():
    try:
        msg = rospy.wait_for_message("/a1/localization/status", DiagnosticStatus, timeout=5.0)
    except rospy.ROSException:
        if rospy.Time.now() > deadline:
            raise SystemExit("Timed out waiting for localization status")
        continue
    values = {item.key: item.value for item in msg.values}
    if values.get("state") == "TRACKING" and values.get("results_valid") == "true":
        print("Localization is TRACKING and results_valid=true")
        break
    if values.get("state") == "LOST":
        raise SystemExit(f"Localization entered LOST: {values.get('reason')}")
PY

echo "[3/8] Starting visualization relays and RViz..."
rosrun topic_tools relay /a1/localization/cloud_registered /cloud_registered \
  __name:=localization_viz_cloud >"$OUTPUT_ROOT/logs/relay_cloud.log" 2>&1 & PIDS+=("$!")
rosrun topic_tools relay /a1/localization/map /Laser_map \
  __name:=localization_viz_map >"$OUTPUT_ROOT/logs/relay_map.log" 2>&1 & PIDS+=("$!")
rosrun topic_tools relay /a1/localization/odom /Odometry \
  __name:=localization_viz_odom >"$OUTPUT_ROOT/logs/relay_odom.log" 2>&1 & PIDS+=("$!")

if ! pgrep -x gzclient >/dev/null; then
  gzclient >"$OUTPUT_ROOT/logs/gzclient.log" 2>&1 & PIDS+=("$!")
fi
rviz -f odom -d "$RVIZ_CONFIG" >"$OUTPUT_ROOT/logs/rviz.log" 2>&1 & PIDS+=("$!")

echo "[4/8] Opening the main entrance..."
if rosservice list | grep -qx /set_door_state; then
  rosservice call /set_door_state main_entrance true
else
  echo "WARNING: /set_door_state is unavailable; start auto.sh with START_BUILDING_CONTROL=1." >&2
fi

echo
echo "Gazebo shows ground truth; RViz shows localization output."
echo "In the auto.sh terminal: press 4, walk through the entrance, scan indoors, then Space and 2."
echo "Recording for $DURATION wall-clock seconds."
read -r -p "Press Enter to begin recording... "

echo "[5/8] Recording trajectory and health..."
rosrun a1_localization localization_validation_recorder.py \
  --duration "$DURATION" --output "$OUTPUT_ROOT/validation"

echo "[6/8] Waiting for a fresh online map and saving PCD..."
MAP_ID="map_product_indoor"
rosparam set /a1_localization/localization_map_manager/output_root "$OUTPUT_ROOT"
rosparam set /a1_localization/localization_map_manager/map_id "$MAP_ID"
saved=false
for _ in 1 2 3 4 5; do
  timeout 35 rostopic echo -n 1 /a1/localization/map/header >/dev/null || true
  response="$(rosservice call /a1/localization/save_map)"
  echo "$response"
  if grep -q 'success: True' <<<"$response"; then
    saved=true
    break
  fi
done
if [[ "$saved" != true ]]; then
  echo "Map save failed after five fresh-map attempts." >&2
  exit 3
fi

echo "[7/8] Verifying product..."
sha256sum "$OUTPUT_ROOT/$MAP_ID/map.pcd"
sed -n '1,220p' "$OUTPUT_ROOT/$MAP_ID/metadata.yaml"
sed -n '1,220p' "$OUTPUT_ROOT/validation/metrics.yaml"

echo "[8/8] Opening final PCD..."
pcl_viewer "$OUTPUT_ROOT/$MAP_ID/map.pcd" >"$OUTPUT_ROOT/logs/pcl_viewer.log" 2>&1 &
PIDS+=("$!")

echo
echo "Test complete. Artifacts: $OUTPUT_ROOT"
read -r -p "Inspect the windows, then press Enter to stop test-owned runtime processes... "
