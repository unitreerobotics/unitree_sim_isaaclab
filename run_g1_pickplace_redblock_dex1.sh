#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export TASK="${TASK:-Isaac-PickPlace-RedBlock-G129-Dex1-Joint}"
export HAND_DDS="${HAND_DDS:-dex1}"
export ROBOT_TYPE="${ROBOT_TYPE:-g129}"

exec "$SCRIPT_DIR/run_pickplace_isolated.sh" "$@"
