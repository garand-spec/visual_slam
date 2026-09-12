#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
RUNTIME_ROOT="$PROJECT_ROOT/data/runtime/orbslam3"
PID_FILE="$RUNTIME_ROOT/slam.pid"
LOG_FILE="$RUNTIME_ROOT/slam.log"
mkdir -p "$RUNTIME_ROOT"

is_slam_pid() {
    local pid="$1"
    [[ "$pid" =~ ^[0-9]+$ ]] || return 1
    [[ -r "/proc/$pid/cmdline" ]] || return 1
    tr '\0' ' ' < "/proc/$pid/cmdline" | grep -Fq "$PROJECT_ROOT/bin/orbslam3_rervision"
}

if [[ -f "$PID_FILE" ]]; then
    old_pid="$(<"$PID_FILE")"
    if is_slam_pid "$old_pid"; then
        echo "SLAM is already running (PID $old_pid)."
        exit 0
    fi
    rm -f -- "$PID_FILE"
fi

if [[ -z "${DISPLAY:-}" ]]; then
    export DISPLAY=:1
fi

nohup "$PROJECT_ROOT/run_visual_slam_3d.sh" "$@" >>"$LOG_FILE" 2>&1 &
slam_pid=$!
printf '%s\n' "$slam_pid" >"$PID_FILE"
sleep 2

if is_slam_pid "$slam_pid"; then
    echo "SLAM started (PID $slam_pid, DISPLAY=$DISPLAY)."
    echo "Log: $LOG_FILE"
    echo "Use ./status_slam.sh to inspect tracking health."
else
    echo "SLAM failed to stay running. Recent log:"
    tail -n 30 "$LOG_FILE" || true
    rm -f -- "$PID_FILE"
    exit 1
fi
