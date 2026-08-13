#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# ==============================
# User settings
# ==============================
# Merge several separately recorded but schema-compatible B601 datasets into a
# new local LeRobot dataset directory.

# Output dataset directory and repo id for training.
OUTPUT_ROOT="/home/r/ws/rebot_lerobot/datasets/rebot_b601_banana_bottle_rgbd_merged"
OUTPUT_REPO_ID="${HF_USER:-local}/rebot_b601_banana_bottle_rgbd_merged"

# Source dataset directories. Each directory must contain data/, meta/, videos/.
SOURCE_ROOTS=(
  "/home/r/ws/rebot_lerobot/datasets/rebot_b601_banana_bottle_rgbd_batch01"
  "/home/r/ws/rebot_lerobot/datasets/rebot_b601_banana_bottle_rgbd_batch02"
)

# Source repo ids recorded into each dataset. These may be identical or unique,
# but the list length must match SOURCE_ROOTS.
SOURCE_REPO_IDS=(
  "local/rebot_b601_banana_bottle_rgbd_batch01"
  "local/rebot_b601_banana_bottle_rgbd_batch02"
)

# Set true only after you have verified the merged local dataset.
PUSH_TO_HUB=false

usage() {
  cat <<'EOF'
Usage:
  bash examples/rebot_b601_smolvla_record/merge_b601_datasets.sh

Recommended:
  Edit SOURCE_ROOTS, SOURCE_REPO_IDS, OUTPUT_ROOT, and OUTPUT_REPO_ID in the
  User settings block, then run this script.

Optional command line overrides:
  --output-root PATH       Merged local dataset directory.
  --output-repo-id ID      Merged dataset repo id stored in metadata.
  --source-root PATH       Add one source dataset directory; repeatable.
  --source-repo-id ID      Add one source repo id; repeatable.
  --push-to-hub true|false Push merged dataset to Hub. Default: false.

Notes:
  All source datasets must have the same fps, robot_type, feature keys, shapes,
  video/image layout, and depth settings. Use this for batches collected with
  the same record_b601_smolvla_rgbd.sh visual/robot configuration.
EOF
}

cli_source_roots=()
cli_source_repo_ids=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-root) OUTPUT_ROOT="$2"; shift 2 ;;
    --output-repo-id) OUTPUT_REPO_ID="$2"; shift 2 ;;
    --source-root) cli_source_roots+=("$2"); shift 2 ;;
    --source-repo-id) cli_source_repo_ids+=("$2"); shift 2 ;;
    --push-to-hub) PUSH_TO_HUB="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ ${#cli_source_roots[@]} -gt 0 ]]; then
  SOURCE_ROOTS=("${cli_source_roots[@]}")
fi
if [[ ${#cli_source_repo_ids[@]} -gt 0 ]]; then
  SOURCE_REPO_IDS=("${cli_source_repo_ids[@]}")
fi

if [[ ${#SOURCE_ROOTS[@]} -eq 0 ]]; then
  echo "Error: configure at least one source dataset root." >&2
  exit 2
fi
if [[ ${#SOURCE_ROOTS[@]} -ne ${#SOURCE_REPO_IDS[@]} ]]; then
  echo "Error: SOURCE_ROOTS and SOURCE_REPO_IDS must have the same length." >&2
  exit 2
fi
if [[ -e "${OUTPUT_ROOT}" ]]; then
  echo "Error: output root already exists: ${OUTPUT_ROOT}" >&2
  echo "Choose a new OUTPUT_ROOT or move/delete the existing directory manually." >&2
  exit 2
fi

for root in "${SOURCE_ROOTS[@]}"; do
  if [[ ! -d "${root}/data" || ! -d "${root}/meta" ]]; then
    echo "Error: source root does not look like a LeRobot dataset: ${root}" >&2
    exit 2
  fi
done

repo_ids_python=$(printf '%s\n' "${SOURCE_REPO_IDS[@]}" | python -c "import json,sys; print(json.dumps([line.strip() for line in sys.stdin if line.strip()]))")
roots_python=$(printf '%s\n' "${SOURCE_ROOTS[@]}" | python -c "import json,sys; print(json.dumps([line.strip() for line in sys.stdin if line.strip()]))")

echo "Merging B601 datasets"
echo "  output root    : ${OUTPUT_ROOT}"
echo "  output repo id : ${OUTPUT_REPO_ID}"
echo "  sources        : ${#SOURCE_ROOTS[@]}"
for idx in "${!SOURCE_ROOTS[@]}"; do
  echo "    [$idx] ${SOURCE_REPO_IDS[$idx]} -> ${SOURCE_ROOTS[$idx]}"
done
echo

cd "${REPO_ROOT}"

python - "$repo_ids_python" "$roots_python" "$OUTPUT_REPO_ID" "$OUTPUT_ROOT" "$PUSH_TO_HUB" <<'PY'
import json
import sys
from pathlib import Path

from lerobot.datasets.aggregate import aggregate_datasets
from lerobot.datasets.lerobot_dataset import LeRobotDataset

repo_ids = json.loads(sys.argv[1])
roots = [Path(p) for p in json.loads(sys.argv[2])]
output_repo_id = sys.argv[3]
output_root = Path(sys.argv[4])
push_to_hub = sys.argv[5].lower() == "true"

aggregate_datasets(
    repo_ids=repo_ids,
    roots=roots,
    aggr_repo_id=output_repo_id,
    aggr_root=output_root,
)

dataset = LeRobotDataset(output_repo_id, root=output_root)
print(
    f"Merged dataset ready: episodes={dataset.meta.total_episodes}, "
    f"frames={dataset.meta.total_frames}, fps={dataset.fps}"
)

if push_to_hub:
    dataset.push_to_hub()
PY
