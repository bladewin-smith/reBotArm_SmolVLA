#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# ==============================
# User settings
# ==============================
# Edit this block before recording. Command line arguments can still override
# these values temporarily, but the intended workflow is to keep your normal
# setup here and run this script directly.

# Local dataset folder to create/write. Example:
# DATASET_ROOT="/home/r/datasets/rebot_b601_banana_bottle_rgbd"
DATASET_ROOT="/home/r/ws/rebot_lerobot/datasets/rebot_b601_banana_bottle_rgbd_3"

# Dataset id stored in metadata. Keep this identical when training locally.
DATASET_REPO_ID="${HF_USER:-local}/rebot_b601_banana_bottle_rgbd_3"

# reBot B601 motorbridge ports verified by the teleoperation setup.
FOLLOWER_PORT="/dev/ttyACM0"
LEADER_PORT="/dev/ttyACM1"

# Follower safety defaults for dataset collection. Keep these conservative
# until the leader/follower pose mapping has been verified across the whole task.
FOLLOWER_MAX_RELATIVE_TARGET=12.0
FOLLOWER_GRIPPER_MAX_RELATIVE_TARGET=30.0
# Seeed-style follower gripper close control: bounded feedforward torque while
# closing, followed by a lower holding torque when contact/stall is detected.
FOLLOWER_GRIPPER_CONTROL_MODE="torque_limited_close"
FOLLOWER_GRIPPER_MAX_TORQUE=1.5
FOLLOWER_GRIPPER_CLOSE_TORQUE=1.0
FOLLOWER_GRIPPER_CLOSE_KD=0.5
FOLLOWER_GRIPPER_CONTACT_MIN_ERROR_DEG=8.0
FOLLOWER_GRIPPER_CONTACT_MAX_VELOCITY_DEG_S=3.0
FOLLOWER_GRIPPER_CONTACT_MIN_TORQUE=0.0
FOLLOWER_GRIPPER_CONTACT_DETECTION_DELAY_S=0.25
FOLLOWER_GRIPPER_CONTACT_DETECTION_SAMPLES=3
FOLLOWER_GRIPPER_CONTACT_HOLD_KP=5.0
FOLLOWER_GRIPPER_CONTACT_HOLD_KD=1.0
FOLLOWER_GRIPPER_CONTACT_HOLD_TORQUE=0.30
FOLLOWER_GRIPPER_CONTACT_RELEASE_HYSTERESIS_DEG=8.0
FOLLOWER_DISABLE_TORQUE_ON_DISCONNECT=false
FOLLOWER_COMMAND_STREAM_ENABLED=true
FOLLOWER_COMMAND_STREAM_HZ=500
FOLLOWER_COMMAND_STREAM_MAX_FAILURES=5
FOLLOWER_COMMAND_STREAM_MAX_GAP_S=0.05
FOLLOWER_COMMAND_STREAM_HARD_GAP_S=0.5
FOLLOWER_ABORT_ON_MOTOR_FAULT_STATUS=true
FOLLOWER_MOTOR_FEEDBACK_MAX_MISSES=3
FOLLOWER_RUNTIME_ERROR_HOLD_S=15
LEADER_COMMAND_STREAM_ENABLED=true
LEADER_COMMAND_STREAM_HZ=500
LEADER_COMMAND_STREAM_MAX_FAILURES=5
LEADER_COMMAND_STREAM_MAX_GAP_S=0.05
LEADER_COMMAND_STREAM_HARD_GAP_S=0.5
LEADER_ABORT_ON_MOTOR_FAULT_STATUS=true
LEADER_MOTOR_FEEDBACK_MAX_MISSES=3
FOLLOWER_SAFETY_HOLD_ON_RELATIVE_CLAMP=true
# Optional empirical joint_2/joint_3 envelope guard. A later reproduction
# disabled in a different, moderate pose, so this is off by default and must
# not be treated as the root-cause fix for the shared disable.
FOLLOWER_SAFETY_COUPLED_POSE_GUARD=false
FOLLOWER_SAFETY_COUPLED_JOINT_2_MIN_DEG=-110
FOLLOWER_SAFETY_COUPLED_JOINT_3_MAX_DEG=-15
FOLLOWER_SAFETY_ABORT_EPISODE_ON_HOLD=true
FOLLOWER_SAFETY_AUTO_RECOVER_TO_EPISODE_START=true
FOLLOWER_SAFETY_RECOVERY_STEP_DEG=1.5
FOLLOWER_SAFETY_RECOVERY_HZ=20
FOLLOWER_SAFETY_RECOVERY_TIMEOUT_S=25
FOLLOWER_SAFETY_RECOVERY_TOLERANCE_DEG=3.0
FOLLOWER_SAFETY_RECOVERY_POSITION_KP_SCALE=0.5
FOLLOWER_SAFETY_WAIT_FOR_LEADER_START=true
FOLLOWER_SAFETY_LEADER_START_TOLERANCE_DEG=8.0
FOLLOWER_SAFETY_LEADER_START_TIMEOUT_S=45

# Top Gemini 335L OrbbecSDK serial number. This is not a /dev/video* path.
# If the bridge reports available serials, choose the Gemini 335L serial from that list.
TOP_SERIAL="CP3L44P0001N"

# Wrist Gemini 305 OrbbecSDK serial number. OpenCV /dev/video* nodes may expose
# raw depth/IR/metadata streams that look like green speckle images.
WRIST_SERIAL="CV2TC5100075"

# Orbbec C++ bridge and LingBot EnhancedDepthFilter model.
ORBBEC_BRIDGE="${REPO_ROOT}/src/lerobot/cameras/orbbec/cpp/build/orbbec_rgbd_bridge"
LINGBOT_MODEL="/home/r/ws/model.sm4"


# Camera and recording timing.
WIDTH=640
HEIGHT=480
FPS=10
NUM_EPISODES=20
EPISODE_TIME_S=300
RESET_TIME_S=45

# Task prompt saved in every frame.
TASK="Arrange the banana model and the transparent plastic cola bottle back to their assigned places on the desktop"

# Local recording defaults.
PUSH_TO_HUB=false
DISPLAY_DATA=false
VCODEC="h264"
# Serialize the three camera video encoders so they do not compete with the
# 500 Hz leader/follower command streams on Jetson Orin NX.
PARALLEL_VIDEO_ENCODING=false
# libx264 otherwise selects seven worker threads on Orin NX and can briefly
# starve the motor command threads at every episode boundary.
VIDEO_ENCODING_THREADS=1
# Keep PIL/PNG work outside the control process. With zero processes, the
# previous default created eight Python writer threads for two cameras.
IMAGE_WRITER_PROCESSES=1
IMAGE_WRITER_THREADS_PER_CAMERA=1

# Top camera depth visualization range for observation.images.top_depth.
TOP_DEPTH_MIN_MM=250
TOP_DEPTH_MAX_MM=1800
TOP_DEPTH_ALIGN_MODE="sw"
TOP_WARMUP_S=25
TOP_TIMEOUT_MS=25000
WRIST_WARMUP_S=15
WRIST_TIMEOUT_MS=15000

# LingBot EnhancedDepthFilter settings. License is expected to be stored on the device.
ENHANCED_DEPTH_FILTER_NAME="EnhancedDepthFilter"
ENHANCED_DEPTH_CONFIDENCE_KEY="confidence_threshold"
ENHANCED_DEPTH_CONFIDENCE_THRESHOLD=51

# Append extra lerobot-record arguments here if you use them often, for example:
# EXTRA_ARGS=(--resume=true)
EXTRA_ARGS=()

usage() {
  cat <<'EOF'
Usage:
  bash examples/rebot_b601_smolvla_record/record_b601_smolvla_rgbd.sh

Recommended:
  Edit the "User settings" block at the top of this script, then run it without
  a long command line.

Optional command line overrides:
  --dataset-root PATH          Local LeRobot dataset root to create/write.
  --top-serial SERIAL         Orbbec Gemini 335L serial number.
  --wrist-serial SERIAL       Orbbec Gemini 305 serial number.
  --repo-id ID                Dataset repo id stored in metadata. Default: ${HF_USER:-local}/rebot_b601_banana_bottle_rgbd
  --follower-port PORT        B601 follower motorbridge port. Default: User settings block.
  --leader-port PORT          B601 leader motorbridge port. Default: User settings block.
  --follower-max-relative-target N
                              Max follower joint step per control frame. Default: 12.0
  --follower-gripper-max-relative-target N
                              Max follower gripper step per control frame. Default: 30.0
  --follower-gripper-control-mode position|torque_limited_close
                              Follower gripper close controller. Default: torque_limited_close.
  --follower-gripper-max-torque N
                              Hard software cap for close/hold torque. Default: 1.5 Nm.
  --follower-gripper-close-torque N
                              Bounded closing feedforward torque in Nm. Default: 1.0.
  --follower-gripper-hold-torque N
                              Contact holding feedforward torque in Nm. Default: 0.30.
  --follower-disable-torque-on-disconnect true|false
                              Whether to disable follower torque when the script exits. Default: false.
  --follower-command-stream-enabled true|false
                              Keep sending the latest MIT target independently of camera I/O. Default: true.
  --follower-command-stream-hz N
                              Follower MIT command refresh frequency. Default: 500 (Seeed DM SDK rate).
  --follower-command-stream-max-failures N
                              Consecutive stream failures before collection aborts. Default: 5.
  --follower-command-stream-max-gap-s N
                              Warn/check if no complete MIT refresh occurs for this many seconds. Default: 0.05.
  --follower-command-stream-hard-gap-s N
                              Latch one recovered command-stream gap at this duration. Default: 0.5.
  --follower-abort-on-motor-fault-status true|false
                              Abort on DM drive faults or unexpected disable during control. Default: true.
  --follower-motor-feedback-max-misses N
                              Consecutive missing feedback samples before aborting. Default: 3.
  --follower-runtime-error-hold-s N
                              Hold the last healthy pose this long after a recording exception. Default: 15.
  --leader-command-stream-enabled true|false
                              Keep leader gravity-comp MIT commands alive independently. Default: true.
  --leader-command-stream-hz N Leader MIT command refresh frequency. Default: 500.
  --leader-command-stream-max-failures N
                              Consecutive leader stream failures before aborting. Default: 5.
  --leader-command-stream-max-gap-s N
                              Leader MIT refresh warning/check threshold. Default: 0.05.
  --leader-command-stream-hard-gap-s N
                              Latch one recovered leader stream gap at this duration. Default: 0.5.
  --leader-abort-on-motor-fault-status true|false
                              Abort collection on a persistent leader DM fault. Default: true.
  --leader-motor-feedback-max-misses N
                              Consecutive missing leader feedback samples before aborting. Default: 3.
  --follower-safety-abort-episode true|false
                              Abort and discard the current episode after safety hold. Default: true.
  --follower-safety-coupled-pose-guard true|false
                              Guard the configured joint_2/joint_3 envelope. Default: false.
  --follower-safety-coupled-joint-2-min-deg N
                              Guard when joint_2 is at or below this value. Default: -110.
  --follower-safety-coupled-joint-3-max-deg N
                              Guard when joint_3 is at or above this value at the same time. Default: -15.
  --follower-safety-auto-recover true|false
                              Recover follower arm joints to the episode start pose after safety hold. Default: true.
  --follower-safety-recovery-step-deg N
                              Max degrees per recovery control step. Default: 1.5
  --follower-safety-recovery-hz N
                              Recovery command frequency. Default: 20.
  --follower-safety-recovery-timeout-s N
                              Stop collection if follower recovery times out. Default: 25.
  --follower-safety-recovery-tolerance-deg N
                              Follower start-pose tolerance. Default: 3.0.
  --follower-safety-recovery-kp-scale N
                              Position Kp multiplier during recovery. Default: 0.5.
  --follower-safety-wait-for-leader-start true|false
                              Wait for leader arm to return near the episode start pose. Default: true.
  --follower-safety-leader-tolerance-deg N
                              Leader start-pose tolerance before rerecording. Default: 8.0.
  --follower-safety-leader-timeout-s N
                              Stop collection if leader does not return in time. Default: 45.
  --orbbec-bridge PATH        Built orbbec_rgbd_bridge binary.
  --lingbot-model PATH        LingBot EnhancedDepthFilter model.sm4 path.
  --fps N                     Dataset/control FPS. Default: 10 with EnhancedDepthFilter.
  --width N                   Camera width. Default: 640
  --height N                  Camera height. Default: 480
  --num-episodes N            Number of episodes. Default: 30
  --episode-time-s N          Seconds per episode. Default: 210
  --reset-time-s N            Seconds between episodes. Default: 45
  --task TEXT                 Task prompt saved into every frame.
  --push-to-hub true|false    Upload dataset after recording. Default: false
  --display-data true|false   Show Rerun visualization. Default: false
  --parallel-video-encoding true|false
                              Encode camera videos concurrently. Default: false on Jetson.
  --video-encoding-threads N Limit each video encoder to this many CPU threads. Default: 1 on Jetson.
  --image-writer-processes N Move PNG writing into this many subprocesses. Default: 1.
  --image-writer-threads-per-camera N
                              PNG writer threads per camera in each process. Default: 1.
  --top-depth-align-mode sw|hw Depth-to-color alignment mode. Default: sw.
  --top-warmup-s N            Top Orbbec warmup seconds. Default: 25.
  --top-timeout-ms N          Top Orbbec first-frame timeout. Default: 25000.
  --wrist-warmup-s N          Wrist Orbbec warmup seconds. Default: 15.
  --wrist-timeout-ms N        Wrist Orbbec first-frame timeout. Default: 15000.
  --extra ARG                 Append one raw argument to lerobot-record; repeatable.

Notes:
  The LingBot license is expected to be stored on the Orbbec device. This script
  does not run an external LicenseTool precheck; OrbbecSDK validates the device
  license when the C++ bridge creates/runs EnhancedDepthFilter.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dataset-root) DATASET_ROOT="$2"; shift 2 ;;
    --repo-id) DATASET_REPO_ID="$2"; shift 2 ;;
    --follower-port) FOLLOWER_PORT="$2"; shift 2 ;;
    --leader-port) LEADER_PORT="$2"; shift 2 ;;
    --follower-max-relative-target) FOLLOWER_MAX_RELATIVE_TARGET="$2"; shift 2 ;;
    --follower-gripper-max-relative-target) FOLLOWER_GRIPPER_MAX_RELATIVE_TARGET="$2"; shift 2 ;;
    --follower-gripper-control-mode) FOLLOWER_GRIPPER_CONTROL_MODE="$2"; shift 2 ;;
    --follower-gripper-max-torque) FOLLOWER_GRIPPER_MAX_TORQUE="$2"; shift 2 ;;
    --follower-gripper-close-torque) FOLLOWER_GRIPPER_CLOSE_TORQUE="$2"; shift 2 ;;
    --follower-gripper-hold-torque) FOLLOWER_GRIPPER_CONTACT_HOLD_TORQUE="$2"; shift 2 ;;
    --follower-disable-torque-on-disconnect) FOLLOWER_DISABLE_TORQUE_ON_DISCONNECT="$2"; shift 2 ;;
    --follower-command-stream-enabled) FOLLOWER_COMMAND_STREAM_ENABLED="$2"; shift 2 ;;
    --follower-command-stream-hz) FOLLOWER_COMMAND_STREAM_HZ="$2"; shift 2 ;;
    --follower-command-stream-max-failures) FOLLOWER_COMMAND_STREAM_MAX_FAILURES="$2"; shift 2 ;;
    --follower-command-stream-max-gap-s) FOLLOWER_COMMAND_STREAM_MAX_GAP_S="$2"; shift 2 ;;
    --follower-command-stream-hard-gap-s) FOLLOWER_COMMAND_STREAM_HARD_GAP_S="$2"; shift 2 ;;
    --follower-abort-on-motor-fault-status) FOLLOWER_ABORT_ON_MOTOR_FAULT_STATUS="$2"; shift 2 ;;
    --follower-motor-feedback-max-misses) FOLLOWER_MOTOR_FEEDBACK_MAX_MISSES="$2"; shift 2 ;;
    --follower-runtime-error-hold-s) FOLLOWER_RUNTIME_ERROR_HOLD_S="$2"; shift 2 ;;
    --leader-command-stream-enabled) LEADER_COMMAND_STREAM_ENABLED="$2"; shift 2 ;;
    --leader-command-stream-hz) LEADER_COMMAND_STREAM_HZ="$2"; shift 2 ;;
    --leader-command-stream-max-failures) LEADER_COMMAND_STREAM_MAX_FAILURES="$2"; shift 2 ;;
    --leader-command-stream-max-gap-s) LEADER_COMMAND_STREAM_MAX_GAP_S="$2"; shift 2 ;;
    --leader-command-stream-hard-gap-s) LEADER_COMMAND_STREAM_HARD_GAP_S="$2"; shift 2 ;;
    --leader-abort-on-motor-fault-status) LEADER_ABORT_ON_MOTOR_FAULT_STATUS="$2"; shift 2 ;;
    --leader-motor-feedback-max-misses) LEADER_MOTOR_FEEDBACK_MAX_MISSES="$2"; shift 2 ;;
    --follower-safety-hold-on-relative-clamp) FOLLOWER_SAFETY_HOLD_ON_RELATIVE_CLAMP="$2"; shift 2 ;;
    --follower-safety-coupled-pose-guard) FOLLOWER_SAFETY_COUPLED_POSE_GUARD="$2"; shift 2 ;;
    --follower-safety-coupled-joint-2-min-deg) FOLLOWER_SAFETY_COUPLED_JOINT_2_MIN_DEG="$2"; shift 2 ;;
    --follower-safety-coupled-joint-3-max-deg) FOLLOWER_SAFETY_COUPLED_JOINT_3_MAX_DEG="$2"; shift 2 ;;
    --follower-safety-abort-episode) FOLLOWER_SAFETY_ABORT_EPISODE_ON_HOLD="$2"; shift 2 ;;
    --follower-safety-auto-recover) FOLLOWER_SAFETY_AUTO_RECOVER_TO_EPISODE_START="$2"; shift 2 ;;
    --follower-safety-recovery-step-deg) FOLLOWER_SAFETY_RECOVERY_STEP_DEG="$2"; shift 2 ;;
    --follower-safety-recovery-hz) FOLLOWER_SAFETY_RECOVERY_HZ="$2"; shift 2 ;;
    --follower-safety-recovery-timeout-s) FOLLOWER_SAFETY_RECOVERY_TIMEOUT_S="$2"; shift 2 ;;
    --follower-safety-recovery-tolerance-deg) FOLLOWER_SAFETY_RECOVERY_TOLERANCE_DEG="$2"; shift 2 ;;
    --follower-safety-recovery-kp-scale) FOLLOWER_SAFETY_RECOVERY_POSITION_KP_SCALE="$2"; shift 2 ;;
    --follower-safety-wait-for-leader-start) FOLLOWER_SAFETY_WAIT_FOR_LEADER_START="$2"; shift 2 ;;
    --follower-safety-leader-tolerance-deg) FOLLOWER_SAFETY_LEADER_START_TOLERANCE_DEG="$2"; shift 2 ;;
    --follower-safety-leader-timeout-s) FOLLOWER_SAFETY_LEADER_START_TIMEOUT_S="$2"; shift 2 ;;
    --top-serial) TOP_SERIAL="$2"; shift 2 ;;
    --wrist-serial) WRIST_SERIAL="$2"; shift 2 ;;
    --orbbec-bridge) ORBBEC_BRIDGE="$2"; shift 2 ;;
    --lingbot-model) LINGBOT_MODEL="$2"; shift 2 ;;
    --fps) FPS="$2"; shift 2 ;;
    --width) WIDTH="$2"; shift 2 ;;
    --height) HEIGHT="$2"; shift 2 ;;
    --num-episodes) NUM_EPISODES="$2"; shift 2 ;;
    --episode-time-s) EPISODE_TIME_S="$2"; shift 2 ;;
    --reset-time-s) RESET_TIME_S="$2"; shift 2 ;;
    --task) TASK="$2"; shift 2 ;;
    --push-to-hub) PUSH_TO_HUB="$2"; shift 2 ;;
    --display-data) DISPLAY_DATA="$2"; shift 2 ;;
    --parallel-video-encoding) PARALLEL_VIDEO_ENCODING="$2"; shift 2 ;;
    --video-encoding-threads) VIDEO_ENCODING_THREADS="$2"; shift 2 ;;
    --image-writer-processes) IMAGE_WRITER_PROCESSES="$2"; shift 2 ;;
    --image-writer-threads-per-camera) IMAGE_WRITER_THREADS_PER_CAMERA="$2"; shift 2 ;;
    --vcodec) VCODEC="$2"; shift 2 ;;
    --top-depth-min-mm) TOP_DEPTH_MIN_MM="$2"; shift 2 ;;
    --top-depth-max-mm) TOP_DEPTH_MAX_MM="$2"; shift 2 ;;
    --top-depth-align-mode) TOP_DEPTH_ALIGN_MODE="$2"; shift 2 ;;
    --top-warmup-s) TOP_WARMUP_S="$2"; shift 2 ;;
    --top-timeout-ms) TOP_TIMEOUT_MS="$2"; shift 2 ;;
    --wrist-warmup-s) WRIST_WARMUP_S="$2"; shift 2 ;;
    --wrist-timeout-ms) WRIST_TIMEOUT_MS="$2"; shift 2 ;;
    --enhanced-depth-filter-name) ENHANCED_DEPTH_FILTER_NAME="$2"; shift 2 ;;
    --enhanced-depth-confidence-key) ENHANCED_DEPTH_CONFIDENCE_KEY="$2"; shift 2 ;;
    --enhanced-depth-confidence-threshold) ENHANCED_DEPTH_CONFIDENCE_THRESHOLD="$2"; shift 2 ;;
    --extra) EXTRA_ARGS+=("$2"); shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ -z "${DATASET_ROOT}" || "${DATASET_ROOT}" == "/path/to/local_dataset_root" ]]; then
  echo "Error: set DATASET_ROOT in the script or pass --dataset-root." >&2
  usage
  exit 2
fi

if [[ -z "${TOP_SERIAL}" || "${TOP_SERIAL}" == "GEMINI335L_SERIAL" ]]; then
  echo "Error: set TOP_SERIAL in the script or pass --top-serial for the Gemini 335L top camera." >&2
  usage
  exit 2
fi

if [[ -z "${WRIST_SERIAL}" || "${WRIST_SERIAL}" == "GEMINI305_SERIAL" ]]; then
  echo "Error: set WRIST_SERIAL in the script or pass --wrist-serial for the Gemini 305 wrist camera." >&2
  usage
  exit 2
fi

if [[ "${TOP_SERIAL}" == /dev/video* ]]; then
  echo "Error: TOP_SERIAL must be an OrbbecSDK device serial, not an OpenCV path: ${TOP_SERIAL}" >&2
  echo "The Orbbec bridge error usually prints available serials, for example CV2TC... or CP3L..." >&2
  echo "Use OrbbecSDK serials for both TOP_SERIAL and WRIST_SERIAL." >&2
  exit 2
fi

if [[ "${WRIST_SERIAL}" == /dev/video* ]]; then
  echo "Error: WRIST_SERIAL must be an OrbbecSDK device serial, not an OpenCV path: ${WRIST_SERIAL}" >&2
  echo "OpenCV /dev/video* nodes on Orbbec RGB-D cameras may expose raw depth/IR data." >&2
  exit 2
fi

if [[ ! -x "${ORBBEC_BRIDGE}" ]]; then
  echo "Error: Orbbec bridge is not executable: ${ORBBEC_BRIDGE}" >&2
  echo "Build it first under src/lerobot/cameras/orbbec/cpp/build/." >&2
  exit 2
fi

if [[ ! -r "${LINGBOT_MODEL}" ]]; then
  echo "Error: LingBot model is not readable: ${LINGBOT_MODEL}" >&2
  exit 2
fi

if [[ ! "${VIDEO_ENCODING_THREADS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "Error: VIDEO_ENCODING_THREADS must be a positive integer: ${VIDEO_ENCODING_THREADS}" >&2
  exit 2
fi

if [[ ! "${IMAGE_WRITER_PROCESSES}" =~ ^[1-9][0-9]*$ ]]; then
  echo "Error: IMAGE_WRITER_PROCESSES must be a positive integer: ${IMAGE_WRITER_PROCESSES}" >&2
  exit 2
fi

if [[ ! "${IMAGE_WRITER_THREADS_PER_CAMERA}" =~ ^[1-9][0-9]*$ ]]; then
  echo "Error: IMAGE_WRITER_THREADS_PER_CAMERA must be a positive integer: ${IMAGE_WRITER_THREADS_PER_CAMERA}" >&2
  exit 2
fi

export LEROBOT_VIDEO_ENCODING_THREADS="${VIDEO_ENCODING_THREADS}"

CAMERAS_CONFIG=$(cat <<EOF
{
  wrist: {
    type: orbbec,
    serial_number: "${WRIST_SERIAL}",
    bridge_binary: "${ORBBEC_BRIDGE}",
    width: ${WIDTH},
    height: ${HEIGHT},
    fps: ${FPS},
    record_color: true,
    use_depth: false,
    warmup_s: ${WRIST_WARMUP_S},
    timeout_ms: ${WRIST_TIMEOUT_MS},
    align_depth_to_color: false,
    record_depth: false,
    record_depth_viz: false
  },
  top: {
    type: orbbec,
    serial_number: "${TOP_SERIAL}",
    bridge_binary: "${ORBBEC_BRIDGE}",
    width: ${WIDTH},
    height: ${HEIGHT},
    fps: ${FPS},
    record_color: true,
    use_depth: true,
    warmup_s: ${TOP_WARMUP_S},
    timeout_ms: ${TOP_TIMEOUT_MS},
    record_depth: true,
    depth_key: "depths.top",
    record_depth_viz: true,
    depth_viz_key: "top_depth",
    depth_viz_min_mm: ${TOP_DEPTH_MIN_MM},
    depth_viz_max_mm: ${TOP_DEPTH_MAX_MM},
    align_depth_to_color: true,
    align_depth_to_color_mode: "${TOP_DEPTH_ALIGN_MODE}",
    use_enhanced_depth_filter: true,
    enhanced_depth_filter_name: "${ENHANCED_DEPTH_FILTER_NAME}",
    enhanced_depth_model_path: "${LINGBOT_MODEL}",
    enhanced_depth_confidence_key: "${ENHANCED_DEPTH_CONFIDENCE_KEY}",
    enhanced_depth_confidence_threshold: ${ENHANCED_DEPTH_CONFIDENCE_THRESHOLD}
  }
}
EOF
)

echo "Recording B601 SmolVLA RGB-D dataset"
echo "  dataset root : ${DATASET_ROOT}"
echo "  repo id      : ${DATASET_REPO_ID}"
echo "  follower     : ${FOLLOWER_PORT}"
echo "  leader       : ${LEADER_PORT}"
echo "  top serial   : ${TOP_SERIAL}"
echo "  wrist serial : ${WRIST_SERIAL}"
echo "  FPS          : ${FPS}"
echo "  video threads: ${VIDEO_ENCODING_THREADS}"
echo "  image writer : ${IMAGE_WRITER_PROCESSES} process(es), ${IMAGE_WRITER_THREADS_PER_CAMERA} thread(s)/camera"
echo "  pose guard   : ${FOLLOWER_SAFETY_COUPLED_POSE_GUARD} (joint_2<=${FOLLOWER_SAFETY_COUPLED_JOINT_2_MIN_DEG}, joint_3>=${FOLLOWER_SAFETY_COUPLED_JOINT_3_MAX_DEG})"
echo

cd "${REPO_ROOT}"

lerobot-record \
  --robot.type=rebot_b601_follower \
  --robot.port="${FOLLOWER_PORT}" \
  --robot.transport=motorbridge \
  --robot.id=b601_follower \
  --robot.max_relative_target="${FOLLOWER_MAX_RELATIVE_TARGET}" \
  --robot.gripper_max_relative_target="${FOLLOWER_GRIPPER_MAX_RELATIVE_TARGET}" \
  --robot.gripper_control_mode="${FOLLOWER_GRIPPER_CONTROL_MODE}" \
  --robot.gripper_max_torque="${FOLLOWER_GRIPPER_MAX_TORQUE}" \
  --robot.gripper_close_torque="${FOLLOWER_GRIPPER_CLOSE_TORQUE}" \
  --robot.gripper_close_kd="${FOLLOWER_GRIPPER_CLOSE_KD}" \
  --robot.gripper_contact_min_closing_error_deg="${FOLLOWER_GRIPPER_CONTACT_MIN_ERROR_DEG}" \
  --robot.gripper_contact_max_velocity_deg_s="${FOLLOWER_GRIPPER_CONTACT_MAX_VELOCITY_DEG_S}" \
  --robot.gripper_contact_min_torque="${FOLLOWER_GRIPPER_CONTACT_MIN_TORQUE}" \
  --robot.gripper_contact_detection_delay_s="${FOLLOWER_GRIPPER_CONTACT_DETECTION_DELAY_S}" \
  --robot.gripper_contact_detection_samples="${FOLLOWER_GRIPPER_CONTACT_DETECTION_SAMPLES}" \
  --robot.gripper_contact_hold_kp="${FOLLOWER_GRIPPER_CONTACT_HOLD_KP}" \
  --robot.gripper_contact_hold_kd="${FOLLOWER_GRIPPER_CONTACT_HOLD_KD}" \
  --robot.gripper_contact_hold_torque="${FOLLOWER_GRIPPER_CONTACT_HOLD_TORQUE}" \
  --robot.gripper_contact_release_hysteresis_deg="${FOLLOWER_GRIPPER_CONTACT_RELEASE_HYSTERESIS_DEG}" \
  --robot.disable_torque_on_disconnect="${FOLLOWER_DISABLE_TORQUE_ON_DISCONNECT}" \
  --robot.command_stream_enabled="${FOLLOWER_COMMAND_STREAM_ENABLED}" \
  --robot.command_stream_hz="${FOLLOWER_COMMAND_STREAM_HZ}" \
  --robot.command_stream_max_consecutive_failures="${FOLLOWER_COMMAND_STREAM_MAX_FAILURES}" \
  --robot.command_stream_max_gap_s="${FOLLOWER_COMMAND_STREAM_MAX_GAP_S}" \
  --robot.command_stream_hard_gap_s="${FOLLOWER_COMMAND_STREAM_HARD_GAP_S}" \
  --robot.abort_on_motor_fault_status="${FOLLOWER_ABORT_ON_MOTOR_FAULT_STATUS}" \
  --robot.motor_feedback_max_consecutive_misses="${FOLLOWER_MOTOR_FEEDBACK_MAX_MISSES}" \
  --robot.runtime_error_hold_s="${FOLLOWER_RUNTIME_ERROR_HOLD_S}" \
  --robot.safety_hold_on_relative_clamp="${FOLLOWER_SAFETY_HOLD_ON_RELATIVE_CLAMP}" \
  --robot.safety_coupled_pose_guard_enabled="${FOLLOWER_SAFETY_COUPLED_POSE_GUARD}" \
  --robot.safety_coupled_joint_2_min_deg="${FOLLOWER_SAFETY_COUPLED_JOINT_2_MIN_DEG}" \
  --robot.safety_coupled_joint_3_max_deg="${FOLLOWER_SAFETY_COUPLED_JOINT_3_MAX_DEG}" \
  --robot.safety_abort_episode_on_hold="${FOLLOWER_SAFETY_ABORT_EPISODE_ON_HOLD}" \
  --robot.safety_auto_recover_to_episode_start="${FOLLOWER_SAFETY_AUTO_RECOVER_TO_EPISODE_START}" \
  --robot.safety_recovery_step_deg="${FOLLOWER_SAFETY_RECOVERY_STEP_DEG}" \
  --robot.safety_recovery_hz="${FOLLOWER_SAFETY_RECOVERY_HZ}" \
  --robot.safety_recovery_timeout_s="${FOLLOWER_SAFETY_RECOVERY_TIMEOUT_S}" \
  --robot.safety_recovery_tolerance_deg="${FOLLOWER_SAFETY_RECOVERY_TOLERANCE_DEG}" \
  --robot.safety_recovery_position_kp_scale="${FOLLOWER_SAFETY_RECOVERY_POSITION_KP_SCALE}" \
  --robot.safety_wait_for_leader_start="${FOLLOWER_SAFETY_WAIT_FOR_LEADER_START}" \
  --robot.safety_leader_start_tolerance_deg="${FOLLOWER_SAFETY_LEADER_START_TOLERANCE_DEG}" \
  --robot.safety_leader_start_timeout_s="${FOLLOWER_SAFETY_LEADER_START_TIMEOUT_S}" \
  --robot.cameras="${CAMERAS_CONFIG}" \
  --teleop.type=rebot_b601_leader \
  --teleop.port="${LEADER_PORT}" \
  --teleop.transport=motorbridge \
  --teleop.id=b601_leader \
  --teleop.manual_control_mode=gravity_comp \
  --teleop.command_stream_enabled="${LEADER_COMMAND_STREAM_ENABLED}" \
  --teleop.command_stream_hz="${LEADER_COMMAND_STREAM_HZ}" \
  --teleop.command_stream_max_consecutive_failures="${LEADER_COMMAND_STREAM_MAX_FAILURES}" \
  --teleop.command_stream_max_gap_s="${LEADER_COMMAND_STREAM_MAX_GAP_S}" \
  --teleop.command_stream_hard_gap_s="${LEADER_COMMAND_STREAM_HARD_GAP_S}" \
  --teleop.abort_on_motor_fault_status="${LEADER_ABORT_ON_MOTOR_FAULT_STATUS}" \
  --teleop.motor_feedback_max_consecutive_misses="${LEADER_MOTOR_FEEDBACK_MAX_MISSES}" \
  --dataset.repo_id="${DATASET_REPO_ID}" \
  --dataset.root="${DATASET_ROOT}" \
  --dataset.fps="${FPS}" \
  --dataset.num_episodes="${NUM_EPISODES}" \
  --dataset.episode_time_s="${EPISODE_TIME_S}" \
  --dataset.reset_time_s="${RESET_TIME_S}" \
  --dataset.single_task="${TASK}" \
  --dataset.vcodec="${VCODEC}" \
  --dataset.parallel_video_encoding="${PARALLEL_VIDEO_ENCODING}" \
  --dataset.num_image_writer_processes="${IMAGE_WRITER_PROCESSES}" \
  --dataset.num_image_writer_threads_per_camera="${IMAGE_WRITER_THREADS_PER_CAMERA}" \
  --dataset.push_to_hub="${PUSH_TO_HUB}" \
  --display_data="${DISPLAY_DATA}" \
  "${EXTRA_ARGS[@]}"

