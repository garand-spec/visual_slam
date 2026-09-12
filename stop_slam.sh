#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
RUNTIME_ROOT="$PROJECT_ROOT/data/runtime/orbslam3"
PID_FILE="$RUNTIME_ROOT/slam.pid"

if [[ ! -f "$PID_FILE" ]]; then
    echo "SLAM is not running (no PID file)."
    exit 0
fi

slam_pid="$(<"$PID_FILE")"
if [[ ! "$slam_pid" =~ ^[0-9]+$ ]] || [[ ! -r "/proc/$slam_pid/cmdline" ]] ||
   ! tr '\0' ' ' < "/proc/$slam_pid/cmdline" | grep -Fq "$PROJECT_ROOT/bin/orbslam3_rervision"; then
    echo "Stale PID file removed; no matching SLAM process is running."
    rm -f -- "$PID_FILE"
    exit 0
fi

kill -INT "$slam_pid"
for _ in $(seq 1 20); do
    if ! kill -0 "$slam_pid" 2>/dev/null; then
        rm -f -- "$PID_FILE"
        echo "SLAM stopped cleanly; trajectory and map were exported."
        exit 0
    fi
    sleep 1
done

echo "SLAM did not finish within 20 seconds; sending TERM."
kill -TERM "$slam_pid"
for _ in $(seq 1 10); do
    if ! kill -0 "$slam_pid" 2>/dev/null; then
        rm -f -- "$PID_FILE"
        echo "SLAM stopped after TERM."
        exit 0
    fi
    sleep 1
done

echo "SLAM is still running after TERM; leaving it and the PID file in place."
echo "Re-run ./stop_slam.sh or inspect the process before killing it manually."
exit 1
