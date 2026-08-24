#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export TASK="${TASK:-Isaac-PickPlace-Cylinder-G129-Dex3-Joint}"
export HAND_DDS="${HAND_DDS:-dex3}"
export ROBOT_TYPE="${ROBOT_TYPE:-g129}"

exec "$SCRIPT_DIR/run_pickplace_isolated.sh" "$@"
