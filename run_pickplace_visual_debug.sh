#!/usr/bin/env bash
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

CONDA_SH="${CONDA_SH:-/home/vilmos/miniconda3/etc/profile.d/conda.sh}"
CONDA_ENV="${CONDA_ENV:-unitree_sim_env}"
ISAACLAB_SH="${ISAACLAB_SH:-/home/vilmos/IsaacLab/isaaclab.sh}"

export CONDA_NO_PLUGINS="${CONDA_NO_PLUGINS:-true}"
export PROJECT_ROOT="${PROJECT_ROOT:-$REPO_ROOT}"
export ROOM_RANDOMIZER_ROOM_SHELL_USD="${ROOM_RANDOMIZER_ROOM_SHELL_USD:-$REPO_ROOT/isaac-projects/new_base_room.usda}"
export CYCLONEDDS_HOME="${CYCLONEDDS_HOME:-/home/vilmos/cyclonedds/install}"
export CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:-$CYCLONEDDS_HOME}"
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}"
if [[ -z "${TERM:-}" || "${TERM:-}" == "dumb" ]]; then
  export TERM=xterm
fi
export TELEIMAGER_REQUEST_PORT="${TELEIMAGER_REQUEST_PORT:-60010}"
export TELEIMAGER_PORT_OFFSET="${TELEIMAGER_PORT_OFFSET:-100}"
export TELEIMAGER_DISABLE_WEBRTC="${TELEIMAGER_DISABLE_WEBRTC:-1}"
export ISAAC_IMAGE_SHM_PREFIX="${ISAAC_IMAGE_SHM_PREFIX:-vilmos_isaac}"

DEVICE="${DEVICE:-cuda:0}"

if [[ ! -f "$ROOM_RANDOMIZER_ROOM_SHELL_USD" ]]; then
  echo "Room shell USD not found: $ROOM_RANDOMIZER_ROOM_SHELL_USD" >&2
  exit 2
fi

cd "$REPO_ROOT"
"$REPO_ROOT/stop_own_pickplace.sh"
source "$CONDA_SH"
conda activate "$CONDA_ENV"

echo "[VISUAL_DEBUG] room_shell=$ROOM_RANDOMIZER_ROOM_SHELL_USD" >&2

set +u
exec "$ISAACLAB_SH" -p run_pickplace_visual_debug.py \
  --device "$DEVICE" \
  --enable_cameras \
  "$@"
