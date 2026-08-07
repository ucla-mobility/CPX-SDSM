#!/usr/bin/env bash
set -euo pipefail
ROLE="${1:-}"
case "$ROLE" in vehicle|infrastructure) ;; *) echo "Usage: status_node.sh role" >&2; exit 2;; esac
PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
STATE_DIR="$PROJECT_ROOT/run/$ROLE"
PID_FILE="$STATE_DIR/node.pid"

if [ -f "$PID_FILE" ] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
  echo "STATE=RUNNING"
  echo "SUPERVISOR_PID=$(cat "$PID_FILE")"
else
  echo "STATE=STOPPED"
fi
if [ -f "$STATE_DIR/state.env" ]; then
  # shellcheck disable=SC1090
  source "$STATE_DIR/state.env"
  echo "RUN_ID=$RUN_ID"
  echo "LOG_DIR=$LOG_DIR"
  if [ -f "$LOG_DIR/status.json" ]; then
    python3 - "$LOG_DIR/status.json" <<'PY'
import json, sys
with open(sys.argv[1]) as f:
    s = json.load(f)
print("READY=true")
print("IDENTITY=%s" % s.get("identity", ""))
print("SENDER_ID=%s" % s.get("sender_id", ""))
print("CLIENT_MODE=%s" % s.get("client_mode", ""))
print("SESSION=%s" % s.get("session_id", ""))
for key, value in sorted(s.get("stats", {}).items()):
    print("%s=%s" % (key.upper(), value))
PY
  fi
  if [ -s "$LOG_DIR/source.jsonl" ]; then
    python3 - "$LOG_DIR/source.jsonl" <<'PY'
import json, sys
last = None
with open(sys.argv[1], encoding="utf-8") as stream:
    for line in stream:
        if line.strip():
            last = line
if last:
    record = json.loads(last)
    transport = record.get("_transport", {})
    location = transport.get("sender_location", {})
    print("SENDER_ID=%s" % transport.get("sender_id", "unknown"))
    print("SENDER_ROLE=%s" % transport.get("sender_role", "unknown"))
    print("ROUTING_LOCATION_SOURCE=%s" % location.get("source", "unknown"))
    print("ROUTING_LAT=%s" % location.get("lat", "unknown"))
    print("ROUTING_LON=%s" % location.get("lon", "unknown"))
    marker = record.get("transformed_det_box_score_label")
    print("MARKER_ATTACHED=%s" % (
        "true" if isinstance(marker, dict) else "false"))
    if isinstance(marker, dict):
        print("MARKER_SOURCE_TOPIC=%s" % marker.get(
            "source_topic", "unknown"))
        print("MARKER_COUNT=%s" % len(
            marker.get("message", {}).get("markers", [])))
PY
  else
    echo "ROUTING_LOCATION_SOURCE=waiting-for-source-message"
  fi
  echo "---- recent relay messages ----"
  grep -E '\[relay\] (stats|learned|publish exception)|READY' \
    "$LOG_DIR/relay.console.log" 2>/dev/null | tail -n 8 || true
  echo "---- recent GPS routing messages ----"
  grep -E 'Verizon routing location|Verizon routing switched|invalid GPSFix' \
    "$LOG_DIR/converter.console.log" 2>/dev/null | tail -n 5 || true
fi
