#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
localization_pid=""
rviz_pid=""

cleanup() {
    trap - EXIT INT TERM

    for pid in "$rviz_pid" "$localization_pid"; do
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid"
        fi
    done

    for pid in "$rviz_pid" "$localization_pid"; do
        if [[ -n "$pid" ]]; then
            wait "$pid" 2>/dev/null || true
        fi
    done
}

trap cleanup EXIT
trap 'exit 130' INT TERM

"$SCRIPT_DIR/start_m20_localization.sh" &
localization_pid=$!

"$SCRIPT_DIR/start_m20_rviz.sh" &
rviz_pid=$!

wait -n "$localization_pid" "$rviz_pid"
