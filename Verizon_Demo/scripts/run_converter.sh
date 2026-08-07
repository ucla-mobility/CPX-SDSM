#!/usr/bin/env bash
set -euo pipefail
ROLE="${1:-}"
LOG_DIR="${2:-}"
PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck disable=SC1091
source "$PROJECT_ROOT/scripts/load_role.sh" "$ROLE"
[ -n "$LOG_DIR" ] || { echo "Usage: run_converter.sh role log_dir" >&2; exit 2; }
mkdir -p "$LOG_DIR"

LOCATION_ARGS=()
if [ -n "$LOCATION_TOPIC" ]; then
  if rosmsg md5 "$LOCATION_TYPE" >/dev/null 2>&1; then
    LOCATION_ARGS+=(--location-topic "$LOCATION_TOPIC" --location-msg-type "$LOCATION_TYPE")
  else
    echo "WARNING: $LOCATION_TYPE is unavailable; using configured fallback coordinates." >&2
  fi
fi

MARKER_ARGS=(--marker-msg-type "$MARKER_TYPE")
if [ -n "$MARKER_SOURCE_TOPIC" ]; then
  MARKER_ARGS+=(--marker-topic "$MARKER_SOURCE_TOPIC")
fi
if [ -n "$MARKER_RX_TOPIC" ]; then
  MARKER_ARGS+=(--marker-rx-topic "$MARKER_RX_TOPIC")
fi

exec python2 "$PROJECT_ROOT/app/ros1_converter_node.py" \
  --detection-mode \
  --detection-profile "$SOURCE_PROFILE" \
  --detection-topic "$SOURCE_TOPIC" \
  --detection-msg-type "$SOURCE_TYPE" \
  --detection-rate "$SOURCE_RATE_HZ" \
  --detection-bag-id "$SOURCE_ID" \
  --sender-role "$ROLE" \
  --sender-id "$SENDER_ID" \
  --default-lat "$DEFAULT_LAT" \
  --default-lon "$DEFAULT_LON" \
  --rx-topic "$JSON_RX_TOPIC" \
  --object-rx-topic "$OBJECT_RX_TOPIC" \
  --transport-log "$LOG_DIR/source.jsonl" \
  --received-transport-log "$LOG_DIR/received.jsonl" \
  --max-udp-bytes 60000 \
  "${MARKER_ARGS[@]}" \
  "${LOCATION_ARGS[@]}" \
  "${@:3}"
