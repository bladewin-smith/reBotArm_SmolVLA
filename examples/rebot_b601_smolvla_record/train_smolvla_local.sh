#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# ==============================
# User settings
# ==============================
# Edit this block before training. Command line arguments can still override
# these values temporarily, but the intended workflow is to run this script
# directly after changing the values below.

# Local LeRobot dataset root copied from the robot/Jetson recording machine.
DATASET_ROOT="/root/gpufree-data/datasets/rebot_b601_banana_bottle_rgbd_merged"

# Must match the repo id used while recording the local dataset.
DATASET_REPO_ID="local/rebot_b601_banana_bottle_rgbd_merged"

# Base SmolVLA policy or a local checkpoint path.
POLICY_PATH="lerobot/smolvla_base"

# Optional local SmolVLM backbone directory for an offline cloud machine.
# Leave empty when Hugging Face Hub access is available.
VLM_PATH=""

# Training output folder. If empty, a timestamped folder is created under outputs/train.
OUTPUT_DIR="/root/gpufree-data/datasets/outputs/rebot_b601_smolvla"

# RTX 4090 24GB starting point for three 512x512 SmolVLA visual inputs.
# Lower BATCH_SIZE to 4 if CUDA memory is tight.
BATCH_SIZE=8
STEPS=20000
SAVE_FREQ=5000
LOG_FREQ=100
NUM_WORKERS=8
DEVICE="cuda"
USE_AMP=true

# Conservative first-stage fine-tuning. Raw uint16 depth remains archival;
# wrist, top, and top_depth are the three visual inputs consumed by SmolVLA.
FREEZE_VISION_ENCODER=true
TRAIN_EXPERT_ONLY=true
OPTIMIZER_LR=1e-4
SCHEDULER_WARMUP_STEPS=1000
SCHEDULER_DECAY_STEPS=20000

# The model still learns a 50-step action chunk, but executes/replans every
# 10 actions by default. At 10 FPS this is a one-second execution horizon.
N_ACTION_STEPS=10

# Offline/local defaults.
POLICY_PUSH_TO_HUB=false
WANDB_ENABLE=false
JOB_NAME="rebot_b601_smolvla"

# Append extra lerobot-train arguments here if you use them often, for example:
# EXTRA_ARGS=(--resume=true)
EXTRA_ARGS=()

usage() {
  cat <<'EOF'
Usage:
  bash examples/rebot_b601_smolvla_record/train_smolvla_local.sh

Recommended:
  Edit the "User settings" block at the top of this script, then run it without
  a long command line.

Optional command line overrides:
  --dataset-root PATH          Local LeRobot dataset root copied from recording.
  --repo-id ID                 Dataset repo id. Default: local/rebot_b601_banana_bottle_rgbd_merged
  --policy-path ID_OR_PATH     Base policy or local checkpoint. Default: lerobot/smolvla_base
  --vlm-path PATH              Optional local SmolVLM2 backbone for offline training.
  --output-dir PATH            Training output directory. Default: outputs/train/rebot_b601_smolvla_YYYYmmdd_HHMMSS
  --batch-size N               Per-GPU batch size. Default: 8 for RTX 4090 24GB.
  --steps N                    Training steps. Default: 20000
  --save-freq N                Checkpoint save frequency. Default: 5000
  --log-freq N                 Log frequency. Default: 100
  --num-workers N              Dataloader workers. Default: 8
  --device DEVICE              Training device. Default: cuda
  --use-amp true|false         Mixed precision. Default: true
  --freeze-vision-encoder true|false
  --train-expert-only true|false
  --optimizer-lr FLOAT
  --scheduler-warmup-steps N
  --scheduler-decay-steps N
  --n-action-steps N           Actions executed before replanning. Default: 10
  --wandb-enable true|false    Enable Weights & Biases. Default: false
  --policy-push-to-hub true|false
                              Push trained policy to Hub. Default: false
  --job-name NAME              Training job name. Default: rebot_b601_smolvla
  --extra ARG                  Append one raw argument to lerobot-train; repeatable.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dataset-root) DATASET_ROOT="$2"; shift 2 ;;
    --repo-id) DATASET_REPO_ID="$2"; shift 2 ;;
    --policy-path) POLICY_PATH="$2"; shift 2 ;;
    --vlm-path) VLM_PATH="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    --batch-size) BATCH_SIZE="$2"; shift 2 ;;
    --steps) STEPS="$2"; shift 2 ;;
    --save-freq) SAVE_FREQ="$2"; shift 2 ;;
    --log-freq) LOG_FREQ="$2"; shift 2 ;;
    --num-workers) NUM_WORKERS="$2"; shift 2 ;;
    --device) DEVICE="$2"; shift 2 ;;
    --use-amp) USE_AMP="$2"; shift 2 ;;
    --freeze-vision-encoder) FREEZE_VISION_ENCODER="$2"; shift 2 ;;
    --train-expert-only) TRAIN_EXPERT_ONLY="$2"; shift 2 ;;
    --optimizer-lr) OPTIMIZER_LR="$2"; shift 2 ;;
    --scheduler-warmup-steps) SCHEDULER_WARMUP_STEPS="$2"; shift 2 ;;
    --scheduler-decay-steps) SCHEDULER_DECAY_STEPS="$2"; shift 2 ;;
    --n-action-steps) N_ACTION_STEPS="$2"; shift 2 ;;
    --wandb-enable) WANDB_ENABLE="$2"; shift 2 ;;
    --policy-push-to-hub) POLICY_PUSH_TO_HUB="$2"; shift 2 ;;
    --job-name) JOB_NAME="$2"; shift 2 ;;
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

if [[ ! -d "${DATASET_ROOT}" ]]; then
  echo "Error: dataset root does not exist: ${DATASET_ROOT}" >&2
  exit 2
fi

if [[ -n "${VLM_PATH}" && ! -d "${VLM_PATH}" ]]; then
  echo "Error: local VLM path does not exist: ${VLM_PATH}" >&2
  exit 2
fi

if [[ -n "${VLM_PATH}" ]]; then
  EXTRA_ARGS+=("--policy.vlm_model_name=${VLM_PATH}")
fi

if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${REPO_ROOT}/outputs/train/${JOB_NAME}_$(date +%Y%m%d_%H%M%S)"
fi

# Always import LeRobot from this checkout. This keeps a copied cloud checkout
# from accidentally using a stale editable install that points to another path.
export PYTHONPATH="${REPO_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}"
PYTHON_BIN="${PYTHON_BIN:-python}"

if [[ ! -f "${REPO_ROOT}/src/lerobot/scripts/lerobot_train.py" ]]; then
  echo "Error: could not find the LeRobot training module under ${REPO_ROOT}/src." >&2
  exit 2
fi

echo "Training SmolVLA from local B601 dataset"
echo "  dataset root : ${DATASET_ROOT}"
echo "  repo id      : ${DATASET_REPO_ID}"
echo "  policy       : ${POLICY_PATH}"
echo "  VLM          : ${VLM_PATH:-Hugging Face config/default}"
echo "  output dir   : ${OUTPUT_DIR}"
echo "  batch size   : ${BATCH_SIZE}"
echo "  steps        : ${STEPS}"
echo "  vision frozen: ${FREEZE_VISION_ENCODER}"
echo "  expert only  : ${TRAIN_EXPERT_ONLY}"
echo "  action steps : ${N_ACTION_STEPS}"
echo "  python        : $(command -v "${PYTHON_BIN}" || echo "${PYTHON_BIN}")"
echo

cd "${REPO_ROOT}"

"${PYTHON_BIN}" -m lerobot.scripts.lerobot_train \
  --policy.path="${POLICY_PATH}" \
  --policy.device="${DEVICE}" \
  --policy.use_amp="${USE_AMP}" \
  --policy.freeze_vision_encoder="${FREEZE_VISION_ENCODER}" \
  --policy.train_expert_only="${TRAIN_EXPERT_ONLY}" \
  --policy.optimizer_lr="${OPTIMIZER_LR}" \
  --policy.scheduler_warmup_steps="${SCHEDULER_WARMUP_STEPS}" \
  --policy.scheduler_decay_steps="${SCHEDULER_DECAY_STEPS}" \
  --policy.n_action_steps="${N_ACTION_STEPS}" \
  --policy.push_to_hub="${POLICY_PUSH_TO_HUB}" \
  --dataset.repo_id="${DATASET_REPO_ID}" \
  --dataset.root="${DATASET_ROOT}" \
  --output_dir="${OUTPUT_DIR}" \
  --job_name="${JOB_NAME}" \
  --batch_size="${BATCH_SIZE}" \
  --steps="${STEPS}" \
  --save_freq="${SAVE_FREQ}" \
  --log_freq="${LOG_FREQ}" \
  --num_workers="${NUM_WORKERS}" \
  --wandb.enable="${WANDB_ENABLE}" \
  "${EXTRA_ARGS[@]}"
