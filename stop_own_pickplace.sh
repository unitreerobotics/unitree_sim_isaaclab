#!/usr/bin/env bash
set -eo pipefail

USER_NAME="${USER:-$(id -un)}"
SELF_PID="$$"
PARENT_PID="$PPID"
CURRENT_PGID="$(ps -p "$SELF_PID" -o pgid= 2>/dev/null | awk '{ print $1 }')"

mapfile -t MATCHED_PIDS < <(
  ps -u "$USER_NAME" -o pid= -o args= | awk -v self="$SELF_PID" -v parent="$PARENT_PID" '
    $1 == self || $1 == parent { next }
    /sim_main\.py/ && /--task/ && /Isaac-PickPlace-/ { print $1; next }
    /run_pickplace_visual_debug\.py/ { print $1; next }
    /isaaclab\.sh/ && /-p sim_main\.py/ && /Isaac-PickPlace-/ { print $1; next }
    /isaaclab\.sh -p run_pickplace_visual_debug\.py/ { print $1; next }
  '
)

declare -A SEEN_PIDS=()
declare -A SEEN_PGIDS=()
PIDS=()
PGIDS=()

add_pid() {
  local pid="$1"
  [[ -n "$pid" ]] || return 0
  [[ "$pid" == "$SELF_PID" || "$pid" == "$PARENT_PID" ]] && return 0
  [[ -n "${SEEN_PIDS[$pid]:-}" ]] && return 0
  SEEN_PIDS["$pid"]=1
  PIDS+=("$pid")
}

add_pgid() {
  local pgid="$1"
  [[ -n "$pgid" ]] || return 0
  [[ "$pgid" == "$CURRENT_PGID" ]] && return 0
  [[ "$pgid" == "$SELF_PID" || "$pgid" == "$PARENT_PID" ]] && return 0
  [[ -n "${SEEN_PGIDS[$pgid]:-}" ]] && return 0
  SEEN_PGIDS["$pgid"]=1
  PGIDS+=("$pgid")
}

pid_pgid() {
  local pid="$1"
  ps -p "$pid" -o pgid= 2>/dev/null | awk '{ print $1 }'
}

collect_descendants() {
  local parent="$1"
  local child
  while read -r child; do
    [[ -n "$child" ]] || continue
    add_pid "$child"
    collect_descendants "$child"
  done < <(pgrep -P "$parent" 2>/dev/null || true)
}

collect_group_members() {
  local pgid="$1"
  local pid
  [[ -n "$pgid" ]] || return 0
  while read -r pid; do
    add_pid "$pid"
  done < <(
    ps -u "$USER_NAME" -o pid= -o pgid= | awk -v target="$pgid" '$2 == target { print $1 }'
  )
}

for pid in "${MATCHED_PIDS[@]}"; do
  add_pid "$pid"
  collect_descendants "$pid"
  add_pgid "$(pid_pgid "$pid")"
done

for pid in "${PIDS[@]}"; do
  add_pgid "$(pid_pgid "$pid")"
done

for pgid in "${PGIDS[@]}"; do
  collect_group_members "$pgid"
done

is_running_not_zombie() {
  local pid="$1"
  local stat
  stat="$(ps -p "$pid" -o stat= 2>/dev/null | awk '{ print $1 }')"
  [[ -n "$stat" && "$stat" != Z* ]]
}

signal_targets() {
  local signal="$1"
  local pgid
  local live_pids=()

  for pgid in "${PGIDS[@]}"; do
    kill "-$signal" -- "-$pgid" 2>/dev/null || true
  done

  for pid in "${PIDS[@]}"; do
    if is_running_not_zombie "$pid"; then
      live_pids+=("$pid")
    fi
  done
  if ((${#live_pids[@]} > 0)); then
    kill "-$signal" "${live_pids[@]}" 2>/dev/null || true
  fi
}

if ((${#PIDS[@]} == 0 && ${#PGIDS[@]} == 0)); then
  echo "[stop_own_pickplace] no previous $USER_NAME pick/place run found"
  exit 0
fi

echo "[stop_own_pickplace] stopping previous $USER_NAME pick/place run: pids=${PIDS[*]:-none} pgids=${PGIDS[*]:-none}"
signal_targets TERM

for _ in {1..20}; do
  STILL_RUNNING=()
  for pid in "${PIDS[@]}"; do
    if is_running_not_zombie "$pid"; then
      STILL_RUNNING+=("$pid")
    fi
  done
  if ((${#STILL_RUNNING[@]} == 0)); then
    echo "[stop_own_pickplace] stopped"
    exit 0
  fi
  sleep 0.5
done

echo "[stop_own_pickplace] force stopping: ${STILL_RUNNING[*]}"
signal_targets KILL

for _ in {1..10}; do
  REMAINING=()
  for pid in "${STILL_RUNNING[@]}"; do
    if is_running_not_zombie "$pid"; then
      REMAINING+=("$pid")
    fi
  done
  if ((${#REMAINING[@]} == 0)); then
    echo "[stop_own_pickplace] stopped"
    exit 0
  fi
  sleep 0.2
done

echo "[stop_own_pickplace] warning: still running after SIGKILL: ${REMAINING[*]}"
