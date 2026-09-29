#!/usr/bin/env bash
#
# Bring up the simulation stack (Gazebo + ArduPilot SITL + MAVROS), record the
# onboard camera, and run one of the autonomous_drone integration tests.
#
# Usage:
#   bash sim/run_sim_test.sh [gps|avoidance|follow|gps_with_avoidance]
#
# 'gps' needs only MAVROS. The other tests additionally start the perception
# pipeline (camera -> depth, detector, avoidance/following).

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="${PROJECT_ROOT:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
readonly ROS_WS="${PROJECT_ROOT}/ros_ws"
readonly SIM_DIR="${PROJECT_ROOT}/sim"
readonly ARDUPILOT_DIR="${ARDUPILOT_DIR:-${SIM_DIR}/ardupilot}"
readonly REC_DIR="${SIM_DIR}/recordings"

# Onboard gimbal camera (gz transport). Bridged to ROS for both the recorder
# and perception's sim_bridge; x11grab is unusable under WSLg's rootless X.
readonly CAMERA_TOPIC="/world/iris_warehouse/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image"
readonly WORLD="${WORLD:-/usr/local/share/ardupilot_gazebo/worlds/iris_warehouse.sdf}"
readonly PERCEPTION_NODES="sim_bridge|object_detector|object_avoidance|object_following|vslam_node"

TEST="${1:-gps}"
case "${TEST}" in
    gps)                                    need_perception=0 ;;
    avoidance|follow|gps_with_avoidance)    need_perception=1 ;;
    *)
        echo "Unknown test: ${TEST}" >&2
        echo "Usage: $0 [gps|avoidance|follow|gps_with_avoidance]" >&2
        exit 1 ;;
esac

# Run Gazebo headless (server only) by default. Under WSLg's software GL the 3D
# GUI starves the sim clock, which destabilizes MAVProxy on the perception tests.
# The camera sensor still renders offscreen, so recording is unaffected.
# Set HEADLESS=0 to show the 3D window.
headless="${HEADLESS:-1}"

export DISPLAY="${DISPLAY:-:0}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/usr/local/lib/ardupilot_gazebo:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="/usr/local/share/ardupilot_gazebo/models:/usr/local/share/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}"
# Flush ROS log lines immediately so wait_for_log sees events without buffering lag.
export RCUTILS_LOGGING_BUFFERED_STREAM=0

mkdir -p "${REC_DIR}"
VIDEO_OUT="${REC_DIR}/${TEST}_$(date +%Y%m%d_%H%M%S).mp4"

# ROS setup scripts reference unbound vars, so relax nounset while sourcing.
set +u
# shellcheck disable=SC1091
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
# shellcheck disable=SC1091
source "${ROS_WS}/install/setup.bash"
# Make the isolated ML venv (torch/ultralytics/timm/transformers) importable by
# the perception nodes. Appended (not prepended) so ROS keeps priority for its
# own packages (rclpy, cv_bridge); the venv only supplies the ML modules.
VENV_SITE="${DRONE_VENV:-$HOME/drone_venv}/lib/python3.12/site-packages"
[ -d "${VENV_SITE}" ] && export PYTHONPATH="${PYTHONPATH:-}:${VENV_SITE}"
set -u

gazebo_pid="" sitl_pid="" mavproxy_pid="" mavros_pid="" bridge_pid="" perception_pid="" recorder_pid=""

cleanup() {
    echo
    echo "=== Cleaning up ==="
    # Recorder first: signal it, give it up to 5s to flush/finalize the mp4,
    # then force-kill. Never block indefinitely on it.
    if [ -n "${recorder_pid}" ]; then
        kill "${recorder_pid}" 2>/dev/null || true
        for _ in 1 2 3 4 5; do
            kill -0 "${recorder_pid}" 2>/dev/null || break
            sleep 1
        done
        kill -9 "${recorder_pid}" 2>/dev/null || true
    fi
    # OpenCV writes mpeg4/mp4v, which some players (e.g. Windows Photos) reject.
    # Transcode to H.264 (yuv420p) for universal playback.
    if [ -f "${VIDEO_OUT}" ] && command -v ffmpeg >/dev/null 2>&1; then
        tmp="${VIDEO_OUT%.mp4}.h264.mp4"
        if ffmpeg -y -loglevel error -i "${VIDEO_OUT}" \
                -c:v libx264 -pix_fmt yuv420p -movflags +faststart "${tmp}" 2>/dev/null; then
            mv -f "${tmp}" "${VIDEO_OUT}"
        else
            rm -f "${tmp}"
        fi
    fi
    [ -n "${perception_pid}" ] && kill "${perception_pid}" 2>/dev/null || true
    [ -n "${bridge_pid}" ]     && kill "${bridge_pid}"     2>/dev/null || true
    [ -n "${mavros_pid}" ]     && kill "${mavros_pid}"     2>/dev/null || true
    [ -n "${mavproxy_pid}" ]   && kill "${mavproxy_pid}"   2>/dev/null || true
    [ -n "${sitl_pid}" ]       && kill "${sitl_pid}"       2>/dev/null || true
    [ -n "${gazebo_pid}" ]     && kill "${gazebo_pid}"     2>/dev/null || true
    pkill -f "${PERCEPTION_NODES}" 2>/dev/null || true
    echo "Video saved to: ${VIDEO_OUT}"
}
trap cleanup EXIT

# Block until a TCP port is listening, or fail after a timeout.
wait_for_port() {
    local port="$1" tries="${2:-60}"
    for ((i = 0; i < tries; i++)); do
        if ss -tln | grep -q ":${port}\b"; then
            return 0
        fi
        sleep 2
    done
    return 1
}

# Block until a regex appears in a log file, or fail after a timeout.
wait_for_log() {
    local pattern="$1" file="$2" tries="${3:-150}"
    for ((i = 0; i < tries; i++)); do
        if grep -qE "${pattern}" "${file}" 2>/dev/null; then
            return 0
        fi
        echo "  ...waiting for '${pattern}' (~$((i * 2))s elapsed)"
        sleep 2
    done
    return 1
}

kill_stale() {
    pkill -f mavros_node 2>/dev/null || true
    pkill -f mavproxy    2>/dev/null || true
    pkill -f arducopter  2>/dev/null || true
    pkill -f "${PERCEPTION_NODES}" 2>/dev/null || true
}

if [ "${headless}" -eq 1 ]; then
    echo "=== Starting Gazebo (headless server) ==="
    gz sim -s -v4 -r "${WORLD}" &
else
    echo "=== Starting Gazebo (GUI) ==="
    gz sim -v4 -r "${WORLD}" &
fi
gazebo_pid=$!
sleep 15

kill_stale
sleep 2

echo "=== Bridging camera to ROS ==="
ros2 run ros_gz_image image_bridge "${CAMERA_TOPIC}" &
bridge_pid=$!
sleep 3

if [ "${need_perception}" -eq 1 ]; then
    # Start early so YOLO/MiDaS finish loading while SITL and MAVROS come up.
    echo "=== Starting perception pipeline ==="
    ros2 launch autonomous_drone autonomous_drone_sim.launch.py &
    perception_pid=$!
fi

# SITL runs without sim_vehicle's own MAVProxy/xterm (more robust headless), and
# skipping -w avoids a reboot that drops the Gazebo physics link. ARMING_CHECK=0
# lets us arm without waiting on every pre-arm check, which is fine for SITL.
echo "=== Starting ArduPilot SITL ==="
cd "${ARDUPILOT_DIR}"
python3 Tools/autotest/sim_vehicle.py \
    -v ArduCopter -f gazebo-iris --model JSON \
    --no-mavproxy \
    -P FRAME_CLASS=1 -P FRAME_TYPE=1 -P ARMING_CHECK=0 \
    -I0 &
sitl_pid=$!

echo "Waiting for SITL on TCP 5760..."
wait_for_port 5760 || { echo "SITL did not open port 5760" >&2; exit 1; }
echo "Waiting 30s for SITL boot and Gazebo sync..."
sleep 30

# MAVProxy bridges SITL (TCP 5760) to MAVROS (UDP 14550) and requests the MAVLink
# data streams. Without it ArduPilot never sends position/altitude and MAVROS
# reports altitude 0. Run with --daemon (no xterm); the terrain module is
# excluded because its SRTM download crashes.
echo "=== Starting MAVProxy (SITL TCP 5760 -> UDP 14550) ==="
mavproxy.py --master=tcp:127.0.0.1:5760 --out=udpout:127.0.0.1:14550 --daemon \
    --streamrate=10 \
    --default-modules=log,param,mode,arm,cmdlong,wp,rally,fence,rc,output \
    > /tmp/mavproxy_${TEST}.log 2>&1 &
mavproxy_pid=$!
sleep 5

# MAVROS output is redirected to a log so we can watch it for the GPS-ready
# event. Tail it in another terminal with: tail -f "${MAVROS_LOG}"
MAVROS_LOG="/tmp/mavros_${TEST}.log"
echo "=== Starting MAVROS (log: ${MAVROS_LOG}) ==="
ros2 run mavros mavros_node --ros-args \
    -p fcu_url:=udp://0.0.0.0:14550@ \
    -p tgt_system:=1 \
    -p tgt_component:=1 > "${MAVROS_LOG}" 2>&1 &
mavros_pid=$!

# Wait until the EKF is fusing GPS, not just "origin set". GUIDED takeoff needs a
# trusted horizontal position to climb, and the test issues the takeoff command
# only once; arming before GPS is trusted leaves the drone on the ground. This
# can take 1-2 min in sim.
echo "Waiting for EKF to start using GPS (can take ~1-2 min in sim)..."
wait_for_log "EKF3 IMU[0-9]+ is using GPS" "${MAVROS_LOG}" \
    || echo "WARNING: GPS-ready event not seen; the test may fail to take off." >&2
echo "EKF is using GPS; letting it settle..."
sleep 5

# Perception tests get the debug overlay (depth shading, clearance bar, steering,
# detection boxes); gps records the plain camera feed.
if [ "${need_perception}" -eq 1 ]; then
    echo "=== Recording onboard camera (perception overlay) ==="
    python3 "${SIM_DIR}/record_overlay.py" "${CAMERA_TOPIC}" "${VIDEO_OUT}" 10 &
else
    echo "=== Recording onboard camera ==="
    python3 "${SIM_DIR}/record_camera.py" "${CAMERA_TOPIC}" "${VIDEO_OUT}" 10 &
fi
recorder_pid=$!

echo "=== Running '${TEST}' test ==="
cd "${ROS_WS}"
sed 's/\r//' run_test.sh | bash -s -- "${TEST}"

echo "=== '${TEST}' test finished ==="
