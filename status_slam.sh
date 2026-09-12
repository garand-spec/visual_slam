#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
RUNTIME_ROOT="$PROJECT_ROOT/data/runtime/orbslam3"
PID_FILE="$RUNTIME_ROOT/slam.pid"
LATEST_FILE="$RUNTIME_ROOT/latest_run.txt"

if [[ -f "$PID_FILE" ]]; then
    slam_pid="$(<"$PID_FILE")"
    if [[ "$slam_pid" =~ ^[0-9]+$ ]] && kill -0 "$slam_pid" 2>/dev/null; then
        echo "process=RUNNING pid=$slam_pid"
    else
        echo "process=STOPPED (stale PID file)"
    fi
else
    echo "process=STOPPED"
fi

if [[ -f "$LATEST_FILE" ]]; then
    latest_run="$(<"$LATEST_FILE")"
    echo "latest_run=$latest_run"
    if [[ -f "$latest_run/status.txt" ]]; then
        cat "$latest_run/status.txt"
    else
        echo "status=waiting_for_first_frame"
    fi
else
    echo "status=no_run_recorded"
fi
