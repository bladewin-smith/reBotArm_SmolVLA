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
DATASET_ROOT="/home/r/ws/rebot_lerobot/datasets/rebot_b601_banana_bottle_rgbd"

# Dataset id stored in metadata. Keep this identical when training locally.
DATASET_REPO_ID="${HF_USER:-local}/rebot_b601_banana_bottle_rgbd"

# reBot B601 motorbridge ports.
FOLLOWER_PORT="/dev/ttyACM0"
LEADER_PORT="/dev/ttyACM1"

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
NUM_EPISODES=75
EPISODE_TIME_S=60
RESET_TIME_S=30

# Task prompt saved in every frame.
TASK="Arrange the banana model and the transparent plastic cola bottle back to their assigned places on the desktop"

# Local recording defaults.
PUSH_TO_HUB=false
DISPLAY_DATA=false
VCODEC="h264"

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
  --follower-port PORT        B601 follower motorbridge port. Default: /dev/ttyACM1
  --leader-port PORT          B601 leader motorbridge port. Default: /dev/ttyACM0
  --orbbec-bridge PATH        Built orbbec_rgbd_bridge binary.
  --lingbot-model PATH        LingBot EnhancedDepthFilter model.sm4 path.
  --fps N                     Dataset/control FPS. Default: 10 with EnhancedDepthFilter.
  --width N                   Camera width. Default: 640
  --height N                  Camera height. Default: 480
  --num-episodes N            Number of episodes. Default: 50
  --episode-time-s N          Seconds per episode. Default: 45
  --reset-time-s N            Seconds between episodes. Default: 20
  --task TEXT                 Task prompt saved into every frame.
  --push-to-hub true|false    Upload dataset after recording. Default: false
  --display-data true|false   Show Rerun visualization. Default: false
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
echo

cd "${REPO_ROOT}"

lerobot-record \
  --robot.type=rebot_b601_follower \
  --robot.port="${FOLLOWER_PORT}" \
  --robot.transport=motorbridge \
  --robot.id=b601_follower \
  --robot.cameras="${CAMERAS_CONFIG}" \
  --teleop.type=rebot_b601_leader \
  --teleop.port="${LEADER_PORT}" \
  --teleop.transport=motorbridge \
  --teleop.id=b601_leader \
  --teleop.manual_control_mode=gravity_comp \
  --dataset.repo_id="${DATASET_REPO_ID}" \
  --dataset.root="${DATASET_ROOT}" \
  --dataset.fps="${FPS}" \
  --dataset.num_episodes="${NUM_EPISODES}" \
  --dataset.episode_time_s="${EPISODE_TIME_S}" \
  --dataset.reset_time_s="${RESET_TIME_S}" \
  --dataset.single_task="${TASK}" \
  --dataset.vcodec="${VCODEC}" \
  --dataset.push_to_hub="${PUSH_TO_HUB}" \
  --display_data="${DISPLAY_DATA}" \
  "${EXTRA_ARGS[@]}"
