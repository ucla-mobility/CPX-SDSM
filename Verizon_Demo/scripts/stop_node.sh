#!/usr/bin/env bash
set -euo pipefail
ROLE="${1:-}"
case "$ROLE" in vehicle|infrastructure) ;; *) echo "Usage: stop_node.sh role" >&2; exit 2;; esac
PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PID_FILE="$PROJECT_ROOT/run/$ROLE/node.pid"
if [ ! -f "$PID_FILE" ]; then
  echo "$ROLE node is not running."
  exit 0
fi
PID="$(cat "$PID_FILE")"
if kill -0 "$PID" 2>/dev/null; then
  kill -TERM "$PID"
  for _ in $(seq 1 15); do
    kill -0 "$PID" 2>/dev/null || break
    sleep 1
  done
fi
rm -f "$PID_FILE"
echo "$ROLE node stopped."
