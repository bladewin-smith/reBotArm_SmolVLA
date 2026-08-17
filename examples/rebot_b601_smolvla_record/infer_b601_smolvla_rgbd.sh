#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# The device-stored LingBot-Depth license is time validated by OrbbecSDK.
MIN_VALID_WALL_CLOCK_S=1704067200  # 2024-01-01T00:00:00Z
CURRENT_WALL_CLOCK_S="$(date +%s)"
if ! [[ "${CURRENT_WALL_CLOCK_S}" =~ ^[0-9]+$ ]] || (( CURRENT_WALL_CLOCK_S < MIN_VALID_WALL_CLOCK_S )); then
  echo "ERROR: System date/time is invalid: $(date -Is 2>/dev/null || date)" >&2
  echo "EnhancedDepthFilter license validation requires the correct current time." >&2
  exit 2
fi

# ==============================
# User settings
# ==============================
# Copy one cloud checkpoint's `pretrained_model` directory to the Jetson and
# point this variable at that directory, not at the parent `checkpoints` folder.
POLICY_PATH="/home/r/ws/rebot_lerobot/rebot_smolvla_b601_20260815_023709/checkpoints/014000/pretrained_model/"

# SmolVLA still needs the local SmolVLM2 config, tokenizer, and processor files
# while constructing the model. Keeping the full local directory is simplest.
VLM_PATH="/home/r/ws/rebot_lerobot/lerobot/models/SmolVLM2-500M-Video-Instruct/"

# Every run records one local evaluation rollout so that camera streams and
# actions can be inspected afterwards. Use a new path/repo id for every run.
RUN_TAG="$(date +%Y%m%d_%H%M%S)"
EVAL_ROOT="/home/r/eval_rollouts/rebot_b601_smolvla_${RUN_TAG}"
EVAL_REPO_ID="local/eval_rebot_b601_smolvla_${RUN_TAG}"

FOLLOWER_PORT="/dev/ttyACM0"
TOP_SERIAL="CP3L44P0001N"
WRIST_SERIAL="CV2TC5100075"
ORBBEC_BRIDGE="${REPO_ROOT}/src/lerobot/cameras/orbbec/cpp/build/orbbec_rgbd_bridge"
LINGBOT_MODEL="/home/r/ws/model.sm4"

WIDTH=640
HEIGHT=480
FPS=10
NUM_EPISODES=1
EPISODE_TIME_S=30
TASK="Arrange the banana model and the transparent plastic cola bottle back to their assigned places on the desktop"

# The first autonomous rollout deliberately uses tighter step limits than
# teleoperated collection. Start in a demonstrated neutral pose with the arm
# physically supported. Raise these only after inspecting a successful run.
FOLLOWER_MAX_RELATIVE_TARGET=5.0
FOLLOWER_GRIPPER_MAX_RELATIVE_TARGET=15.0
FOLLOWER_RUNTIME_ERROR_HOLD_S=15
FOLLOWER_SAFETY_HOLD_CLAMP_RATIO=1.2
# SmolVLA emits absolute joint targets in chunks. Keep the 5 deg/cycle slew
# limit above, but only treat larger model discontinuities as a safety fault.
FOLLOWER_SAFETY_HOLD_MULTI_JOINT_DELTA_DEG=20.0
FOLLOWER_SAFETY_HOLD_SINGLE_JOINT_DELTA_DEG=30.0

FOLLOWER_GRIPPER_CONTROL_MODE="torque_limited_close"
FOLLOWER_GRIPPER_MAX_TORQUE=1.5
FOLLOWER_GRIPPER_CLOSE_TORQUE=1.0
FOLLOWER_GRIPPER_CLOSE_KD=0.5
FOLLOWER_GRIPPER_CONTACT_MIN_ERROR_DEG=8.0
FOLLOWER_GRIPPER_CONTACT_MIN_TRAVEL_DEG=17.0
FOLLOWER_GRIPPER_CONTACT_MAX_VELOCITY_DEG_S=3.0
FOLLOWER_GRIPPER_CONTACT_MIN_TORQUE=0.0
FOLLOWER_GRIPPER_CONTACT_DETECTION_DELAY_S=0.25
FOLLOWER_GRIPPER_CONTACT_DETECTION_SAMPLES=3
FOLLOWER_GRIPPER_CONTACT_HOLD_KP=5.0
FOLLOWER_GRIPPER_CONTACT_HOLD_KD=1.0
FOLLOWER_GRIPPER_CONTACT_HOLD_TORQUE=0.30
FOLLOWER_GRIPPER_CONTACT_RELEASE_HYSTERESIS_DEG=8.0

COMMAND_STREAM_HZ=500
COMMAND_STREAM_MAX_FAILURES=5
COMMAND_STREAM_MAX_GAP_S=0.05
COMMAND_STREAM_HARD_GAP_S=0.5
MOTOR_FEEDBACK_MAX_MISSES=3

TOP_DEPTH_MIN_MM=250
TOP_DEPTH_MAX_MM=1800
TOP_DEPTH_ALIGN_MODE="sw"
TOP_WARMUP_S=25
TOP_TIMEOUT_MS=25000
WRIST_WARMUP_S=15
WRIST_TIMEOUT_MS=15000
ENHANCED_DEPTH_FILTER_NAME="EnhancedDepthFilter"
ENHANCED_DEPTH_CONFIDENCE_KEY="confidence_threshold"
ENHANCED_DEPTH_CONFIDENCE_THRESHOLD=51

DEVICE="cuda"
USE_AMP=true
N_ACTION_STEPS=10
VCODEC="h264"
VIDEO_ENCODING_THREADS=1
IMAGE_WRITER_PROCESSES=1
IMAGE_WRITER_THREADS_PER_CAMERA=1
DISPLAY_DATA=false

EXTRA_ARGS=()

usage() {
  cat <<'EOF'
Usage:
  bash examples/rebot_b601_smolvla_record/infer_b601_smolvla_rgbd.sh

Edit the "User settings" block for normal use. Optional overrides:
  --policy-path PATH
  --vlm-path PATH
  --output-root PATH
  --repo-id NAMESPACE/eval_NAME
  --follower-port PORT
  --top-serial SERIAL
  --wrist-serial SERIAL
  --episode-time-s SECONDS
  --n-action-steps N
  --max-relative-target DEG
  --gripper-max-relative-target DEG
  --safety-hold-clamp-ratio RATIO
  --safety-hold-multi-joint-delta DEG
  --safety-hold-single-joint-delta DEG
  --display-data true|false
  --extra ARG                    Append one raw lerobot-record argument.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --policy-path) POLICY_PATH="$2"; shift 2 ;;
    --vlm-path) VLM_PATH="$2"; shift 2 ;;
    --output-root) EVAL_ROOT="$2"; shift 2 ;;
    --repo-id) EVAL_REPO_ID="$2"; shift 2 ;;
    --follower-port) FOLLOWER_PORT="$2"; shift 2 ;;
    --top-serial) TOP_SERIAL="$2"; shift 2 ;;
    --wrist-serial) WRIST_SERIAL="$2"; shift 2 ;;
    --episode-time-s) EPISODE_TIME_S="$2"; shift 2 ;;
    --n-action-steps) N_ACTION_STEPS="$2"; shift 2 ;;
    --max-relative-target) FOLLOWER_MAX_RELATIVE_TARGET="$2"; shift 2 ;;
    --gripper-max-relative-target) FOLLOWER_GRIPPER_MAX_RELATIVE_TARGET="$2"; shift 2 ;;
    --safety-hold-clamp-ratio) FOLLOWER_SAFETY_HOLD_CLAMP_RATIO="$2"; shift 2 ;;
    --safety-hold-multi-joint-delta) FOLLOWER_SAFETY_HOLD_MULTI_JOINT_DELTA_DEG="$2"; shift 2 ;;
    --safety-hold-single-joint-delta) FOLLOWER_SAFETY_HOLD_SINGLE_JOINT_DELTA_DEG="$2"; shift 2 ;;
    --display-data) DISPLAY_DATA="$2"; shift 2 ;;
    --extra) EXTRA_ARGS+=("$2"); shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ "${EVAL_REPO_ID}" != */* ]]; then
  echo "Error: evaluation repo id must use NAMESPACE/eval_NAME, got: ${EVAL_REPO_ID}" >&2
  exit 2
fi
EVAL_DATASET_NAME="${EVAL_REPO_ID#*/}"
if [[ "${EVAL_DATASET_NAME}" != eval_* ]]; then
  echo "Error: policy evaluation dataset name must begin with 'eval_'." >&2
  echo "Use, for example: --repo-id local/eval_rebot_b601_smolvla_${RUN_TAG}" >&2
  exit 2
fi

for required_file in \
  "${POLICY_PATH}/config.json" \
  "${POLICY_PATH}/model.safetensors" \
  "${POLICY_PATH}/policy_preprocessor.json" \
  "${POLICY_PATH}/policy_postprocessor.json"; do
  if [[ ! -r "${required_file}" ]]; then
    echo "Error: required checkpoint file is not readable: ${required_file}" >&2
    exit 2
  fi
done

if [[ ! -r "${VLM_PATH}/config.json" ]]; then
  echo "Error: local SmolVLM2 directory is incomplete: ${VLM_PATH}" >&2
  exit 2
fi
if [[ ! -x "${ORBBEC_BRIDGE}" ]]; then
  echo "Error: Orbbec bridge is not executable: ${ORBBEC_BRIDGE}" >&2
  exit 2
fi
if [[ ! -r "${LINGBOT_MODEL}" ]]; then
  echo "Error: LingBot model is not readable: ${LINGBOT_MODEL}" >&2
  exit 2
fi
if [[ "${TOP_SERIAL}" == "${WRIST_SERIAL}" ]]; then
  echo "Error: top and wrist Orbbec serials must be different." >&2
  exit 2
fi

export PYTHONPATH="${REPO_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}"
export HF_HUB_OFFLINE=1
export TRANSFORMERS_OFFLINE=1
export LEROBOT_VIDEO_ENCODING_THREADS="${VIDEO_ENCODING_THREADS}"
PYTHON_BIN="${PYTHON_BIN:-python}"

if ! "${PYTHON_BIN}" - <<'PY'
from importlib.metadata import PackageNotFoundError, version

from packaging.version import Version


required = ("transformers", "huggingface-hub", "accelerate", "num2words", "safetensors")
missing = []
for package in required:
    try:
        version(package)
    except PackageNotFoundError:
        missing.append(package)

problems = []
if missing:
    problems.append(f"missing packages: {', '.join(missing)}")
else:
    transformers_version = Version(version("transformers"))
    hub_version = Version(version("huggingface-hub"))
    if not Version("4.57.1") <= transformers_version < Version("5.0.0"):
        problems.append(f"transformers={transformers_version}, expected >=4.57.1,<5.0.0")
    if not Version("0.34.2") <= hub_version < Version("0.36.0"):
        problems.append(f"huggingface-hub={hub_version}, expected >=0.34.2,<0.36.0")

if problems:
    for problem in problems:
        print(f"SmolVLA dependency error: {problem}")
    raise SystemExit(1)

from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy  # noqa: F401, E402

print(
    "SmolVLA dependencies OK: "
    f"transformers={version('transformers')}, huggingface-hub={version('huggingface-hub')}"
)
PY
then
  echo "Error: the current Python environment cannot import the pinned SmolVLA stack." >&2
  echo "Do not install an unconstrained transformers release. Repair it with:" >&2
  echo "  python -m pip install --upgrade 'transformers>=4.57.1,<5.0.0' 'huggingface-hub[cli,hf-transfer]>=0.34.2,<0.36.0' 'accelerate>=1.10.0,<2.0.0' 'num2words>=0.5.14,<0.6.0' 'safetensors>=0.4.3,<1.0.0'" >&2
  exit 2
fi

CAMERAS_CONFIG=$(cat <<EOF
{
  wrist: {
    type: orbbec,
    serial_number: "${WRIST_SERIAL}",
    bridge_binary: "${ORBBEC_BRIDGE}",
    width: ${WIDTH}, height: ${HEIGHT}, fps: ${FPS},
    record_color: true, use_depth: false,
    warmup_s: ${WRIST_WARMUP_S}, timeout_ms: ${WRIST_TIMEOUT_MS},
    align_depth_to_color: false, record_depth: false, record_depth_viz: false
  },
  top: {
    type: orbbec,
    serial_number: "${TOP_SERIAL}",
    bridge_binary: "${ORBBEC_BRIDGE}",
    width: ${WIDTH}, height: ${HEIGHT}, fps: ${FPS},
    record_color: true, use_depth: true,
    warmup_s: ${TOP_WARMUP_S}, timeout_ms: ${TOP_TIMEOUT_MS},
    record_depth: true, depth_key: "depths.top",
    record_depth_viz: true, depth_viz_key: "top_depth",
    depth_viz_min_mm: ${TOP_DEPTH_MIN_MM}, depth_viz_max_mm: ${TOP_DEPTH_MAX_MM},
    align_depth_to_color: true, align_depth_to_color_mode: "${TOP_DEPTH_ALIGN_MODE}",
    use_enhanced_depth_filter: true,
    enhanced_depth_filter_name: "${ENHANCED_DEPTH_FILTER_NAME}",
    enhanced_depth_model_path: "${LINGBOT_MODEL}",
    enhanced_depth_confidence_key: "${ENHANCED_DEPTH_CONFIDENCE_KEY}",
    enhanced_depth_confidence_threshold: ${ENHANCED_DEPTH_CONFIDENCE_THRESHOLD}
  }
}
EOF
)

echo "B601 SmolVLA autonomous evaluation"
echo "  policy       : ${POLICY_PATH}"
echo "  VLM config   : ${VLM_PATH}"
echo "  output       : ${EVAL_ROOT}"
echo "  follower     : ${FOLLOWER_PORT}"
echo "  cameras      : top=${TOP_SERIAL}, wrist=${WRIST_SERIAL}"
echo "  task         : ${TASK}"
echo "  action steps : ${N_ACTION_STEPS}"
echo "  arm step     : ${FOLLOWER_MAX_RELATIVE_TARGET} deg"
echo "  hold guard   : ${FOLLOWER_SAFETY_HOLD_MULTI_JOINT_DELTA_DEG} deg on multiple joints, ${FOLLOWER_SAFETY_HOLD_SINGLE_JOINT_DELTA_DEG} deg on one joint"
echo
echo "Support the follower and keep the E-stop ready. Start from a demonstrated neutral pose."
read -r -p "Press ENTER to connect cameras and enable the follower... "

cd "${REPO_ROOT}"
"${PYTHON_BIN}" -m lerobot.scripts.lerobot_record \
  --robot.type=rebot_b601_follower \
  --robot.port="${FOLLOWER_PORT}" \
  --robot.transport=motorbridge \
  --robot.id=b601_follower \
  --robot.gripper_map_leader_to_follower=false \
  --robot.max_relative_target="${FOLLOWER_MAX_RELATIVE_TARGET}" \
  --robot.gripper_max_relative_target="${FOLLOWER_GRIPPER_MAX_RELATIVE_TARGET}" \
  --robot.gripper_control_mode="${FOLLOWER_GRIPPER_CONTROL_MODE}" \
  --robot.gripper_max_torque="${FOLLOWER_GRIPPER_MAX_TORQUE}" \
  --robot.gripper_close_torque="${FOLLOWER_GRIPPER_CLOSE_TORQUE}" \
  --robot.gripper_close_kd="${FOLLOWER_GRIPPER_CLOSE_KD}" \
  --robot.gripper_contact_min_closing_error_deg="${FOLLOWER_GRIPPER_CONTACT_MIN_ERROR_DEG}" \
  --robot.gripper_contact_min_travel_deg="${FOLLOWER_GRIPPER_CONTACT_MIN_TRAVEL_DEG}" \
  --robot.gripper_contact_max_velocity_deg_s="${FOLLOWER_GRIPPER_CONTACT_MAX_VELOCITY_DEG_S}" \
  --robot.gripper_contact_min_torque="${FOLLOWER_GRIPPER_CONTACT_MIN_TORQUE}" \
  --robot.gripper_contact_detection_delay_s="${FOLLOWER_GRIPPER_CONTACT_DETECTION_DELAY_S}" \
  --robot.gripper_contact_detection_samples="${FOLLOWER_GRIPPER_CONTACT_DETECTION_SAMPLES}" \
  --robot.gripper_contact_hold_kp="${FOLLOWER_GRIPPER_CONTACT_HOLD_KP}" \
  --robot.gripper_contact_hold_kd="${FOLLOWER_GRIPPER_CONTACT_HOLD_KD}" \
  --robot.gripper_contact_hold_torque="${FOLLOWER_GRIPPER_CONTACT_HOLD_TORQUE}" \
  --robot.gripper_contact_release_hysteresis_deg="${FOLLOWER_GRIPPER_CONTACT_RELEASE_HYSTERESIS_DEG}" \
  --robot.disable_torque_on_disconnect=false \
  --robot.command_stream_enabled=true \
  --robot.command_stream_hz="${COMMAND_STREAM_HZ}" \
  --robot.command_stream_max_consecutive_failures="${COMMAND_STREAM_MAX_FAILURES}" \
  --robot.command_stream_max_gap_s="${COMMAND_STREAM_MAX_GAP_S}" \
  --robot.command_stream_hard_gap_s="${COMMAND_STREAM_HARD_GAP_S}" \
  --robot.abort_on_motor_fault_status=true \
  --robot.motor_feedback_max_consecutive_misses="${MOTOR_FEEDBACK_MAX_MISSES}" \
  --robot.runtime_error_hold_s="${FOLLOWER_RUNTIME_ERROR_HOLD_S}" \
  --robot.safety_hold_on_relative_clamp=true \
  --robot.safety_hold_clamp_ratio="${FOLLOWER_SAFETY_HOLD_CLAMP_RATIO}" \
  --robot.safety_hold_multi_joint_delta_deg="${FOLLOWER_SAFETY_HOLD_MULTI_JOINT_DELTA_DEG}" \
  --robot.safety_hold_single_joint_delta_deg="${FOLLOWER_SAFETY_HOLD_SINGLE_JOINT_DELTA_DEG}" \
  --robot.safety_abort_episode_on_hold=true \
  --robot.safety_auto_recover_to_episode_start=false \
  --robot.safety_wait_for_leader_start=false \
  --robot.cameras="${CAMERAS_CONFIG}" \
  --policy.path="${POLICY_PATH}" \
  --policy.vlm_model_name="${VLM_PATH}" \
  --policy.load_vlm_weights=false \
  --policy.device="${DEVICE}" \
  --policy.use_amp="${USE_AMP}" \
  --policy.n_action_steps="${N_ACTION_STEPS}" \
  --dataset.repo_id="${EVAL_REPO_ID}" \
  --dataset.root="${EVAL_ROOT}" \
  --dataset.fps="${FPS}" \
  --dataset.num_episodes="${NUM_EPISODES}" \
  --dataset.episode_time_s="${EPISODE_TIME_S}" \
  --dataset.reset_time_s=0 \
  --dataset.single_task="${TASK}" \
  --dataset.vcodec="${VCODEC}" \
  --dataset.parallel_video_encoding=false \
  --dataset.num_image_writer_processes="${IMAGE_WRITER_PROCESSES}" \
  --dataset.num_image_writer_threads_per_camera="${IMAGE_WRITER_THREADS_PER_CAMERA}" \
  --dataset.push_to_hub=false \
  --display_data="${DISPLAY_DATA}" \
  --play_sounds=false \
  "${EXTRA_ARGS[@]}"
