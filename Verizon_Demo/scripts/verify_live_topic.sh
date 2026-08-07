#!/usr/bin/env bash
set -euo pipefail
ROLE="${1:-}"
WAIT_SECONDS="${2:-20}"
PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck disable=SC1091
source "$PROJECT_ROOT/scripts/load_role.sh" "$ROLE"

echo "Expected topic=$SOURCE_TOPIC"
echo "Expected type=$SOURCE_TYPE"
echo "Expected MD5=$SOURCE_MD5"
ACTUAL_TYPE="$(rostopic type "$SOURCE_TOPIC" 2>/dev/null || true)"
test "$ACTUAL_TYPE" = "$SOURCE_TYPE" || {
  echo "Topic absent or type mismatch: actual='$ACTUAL_TYPE'" >&2
  exit 3
}
ACTUAL_MD5="$(rosmsg md5 "$ACTUAL_TYPE")"
test "$ACTUAL_MD5" = "$SOURCE_MD5" || {
  echo "MD5 mismatch: actual=$ACTUAL_MD5" >&2
  exit 4
}
echo "TYPE_AND_MD5_OK"
if timeout "$WAIT_SECONDS" rostopic echo -n 1 "$SOURCE_TOPIC" >/dev/null; then
  echo "LIVE_MESSAGE_OK"
else
  echo "No message within ${WAIT_SECONDS}s. The type is correct, but verify LiDAR/tracking launch and input data." >&2
  exit 5
fi

if [ -n "$MARKER_SOURCE_TOPIC" ]; then
  echo "Expected marker topic=$MARKER_SOURCE_TOPIC"
  echo "Expected marker type=$MARKER_TYPE"
  echo "Expected marker MD5=$MARKER_MD5"
  MARKER_ACTUAL_TYPE="$(
    rostopic type "$MARKER_SOURCE_TOPIC" 2>/dev/null || true
  )"
  test "$MARKER_ACTUAL_TYPE" = "$MARKER_TYPE" || {
    echo "Marker topic absent or type mismatch: actual='$MARKER_ACTUAL_TYPE'" >&2
    exit 6
  }
  MARKER_ACTUAL_MD5="$(rosmsg md5 "$MARKER_ACTUAL_TYPE")"
  test "$MARKER_ACTUAL_MD5" = "$MARKER_MD5" || {
    echo "Marker MD5 mismatch: actual=$MARKER_ACTUAL_MD5" >&2
    exit 7
  }
  echo "MARKER_TYPE_AND_MD5_OK"
  if timeout "$WAIT_SECONDS" \
      rostopic echo -n 1 "$MARKER_SOURCE_TOPIC" >/dev/null; then
    echo "MARKER_LIVE_MESSAGE_OK"
  else
    echo "No MarkerArray within ${WAIT_SECONDS}s; verify format conversion." >&2
    exit 8
  fi
fi
