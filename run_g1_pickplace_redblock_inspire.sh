#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export TASK="${TASK:-Isaac-PickPlace-RedBlock-G129-Inspire-Joint}"
export HAND_DDS="${HAND_DDS:-inspire}"
export ROBOT_TYPE="${ROBOT_TYPE:-g129}"

exec "$SCRIPT_DIR/run_pickplace_isolated.sh" "$@"
