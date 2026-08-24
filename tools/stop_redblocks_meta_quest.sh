#!/usr/bin/env bash

set -Eeuo pipefail

readonly SCRIPT_NAME="$(basename "$0")"
readonly TASK_USER="$(id -un)"
readonly SELF_PID="$$"
readonly PARENT_PID="$PPID"

usage() {
    cat <<EOF
Stop this user's medicine-bottle simulator and XR teleoperation processes.

Usage:
  $SCRIPT_NAME [--dry-run]

Options:
  --dry-run   Print matching processes without stopping them.
  -h, --help  Show this help.
EOF
}

dry_run=false
while (($#)); do
    case "$1" in
        --dry-run)
            dry_run=true
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            printf 'Unknown option: %s\n\n' "$1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

declare -A SEEN_PIDS=()
TARGET_PIDS=()

add_pid() {
    local pid="$1"

    [[ "$pid" =~ ^[0-9]+$ ]] || return 0
    [[ "$pid" != "$SELF_PID" && "$pid" != "$PARENT_PID" ]] || return 0
    [[ -z "${SEEN_PIDS[$pid]:-}" ]] || return 0
    SEEN_PIDS["$pid"]=1
    TARGET_PIDS+=("$pid")
}

collect_descendants() {
    local parent_pid="$1"
    local child_pid

    while read -r child_pid; do
        [[ -n "$child_pid" ]] || continue
        add_pid "$child_pid"
        collect_descendants "$child_pid"
    done < <(pgrep -P "$parent_pid" 2>/dev/null || true)
}

mapfile -t ROOT_PIDS < <(
    ps -u "$TASK_USER" -o pid=,comm=,args= | awk '
        {
            pid = $1
            command = $2
            $1 = ""
            $2 = ""
            arguments = $0
        }
        command == "python" && arguments ~ /sim_main\.py/ && arguments ~ /--meta_quest/ {
            print pid
            next
        }
        command == "python" && arguments ~ /teleop_hand_and_arm\.py/ && arguments ~ /--sim/ {
            print pid
            next
        }
        command == "bash" && arguments ~ /start_redblocks_meta_quest\.sh/ {
            print pid
            next
        }
        command == "xfce4-terminal" && arguments ~ /start_redblocks_meta_quest\.sh/ {
            print pid
            next
        }
        command == "gnome-terminal" && arguments ~ /start_redblocks_meta_quest\.sh/ {
            print pid
            next
        }
        command == "xterm" && arguments ~ /start_redblocks_meta_quest\.sh/ {
            print pid
        }
    '
)

for root_pid in "${ROOT_PIDS[@]}"; do
    add_pid "$root_pid"
    collect_descendants "$root_pid"
done

is_live_process() {
    local pid="$1"
    local process_state

    process_state="$(ps -p "$pid" -o stat= 2>/dev/null | awk '{ print $1 }')"
    [[ -n "$process_state" && "$process_state" != Z* ]]
}

live_targets() {
    local pid

    for pid in "${TARGET_PIDS[@]}"; do
        if is_live_process "$pid"; then
            printf '%s\n' "$pid"
        fi
    done
}

mapfile -t LIVE_PIDS < <(live_targets)
if ((${#LIVE_PIDS[@]} == 0)); then
    printf 'No medicine-bottle simulator or XR teleoperation processes are running for %s.\n' "$TASK_USER"
    exit 0
fi

printf 'Matched medicine-bottle/teleoperation processes for %s:\n' "$TASK_USER"
ps -o pid=,ppid=,stat=,args= -p "$(IFS=,; printf '%s' "${LIVE_PIDS[*]}")"

if [[ "$dry_run" == true ]]; then
    printf 'Dry run: no processes were stopped.\n'
    exit 0
fi

printf 'Requesting graceful shutdown...\n'
kill -TERM "${LIVE_PIDS[@]}" 2>/dev/null || true

for _ in {1..20}; do
    mapfile -t LIVE_PIDS < <(live_targets)
    if ((${#LIVE_PIDS[@]} == 0)); then
        printf 'All medicine-bottle simulator and XR teleoperation processes stopped.\n'
        exit 0
    fi
    sleep 0.5
done

printf 'Force-stopping remaining processes: %s\n' "${LIVE_PIDS[*]}"
kill -KILL "${LIVE_PIDS[@]}" 2>/dev/null || true

for _ in {1..10}; do
    mapfile -t LIVE_PIDS < <(live_targets)
    if ((${#LIVE_PIDS[@]} == 0)); then
        printf 'All medicine-bottle simulator and XR teleoperation processes stopped.\n'
        exit 0
    fi
    sleep 0.2
done

printf 'Warning: processes still present after SIGKILL: %s\n' "${LIVE_PIDS[*]}" >&2
exit 1
