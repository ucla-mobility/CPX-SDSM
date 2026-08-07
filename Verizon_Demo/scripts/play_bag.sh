#!/usr/bin/env bash
# Replay only the structured tracking topic (and vehicle GPSFix), not the
# point-cloud/camera/visualization topics in the large bag.
set -euo pipefail
ROLE="${1:-}"
BAG_ARG="${2:-}"
MODE="${3:-once}"
PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck disable=SC1091
source "$PROJECT_ROOT/scripts/load_role.sh" "$ROLE"
BAG="${BAG_ARG:-$PROJECT_ROOT/$DEFAULT_BAG_REL}"
test -f "$BAG" || { echo "Bag not found: $BAG" >&2; exit 3; }

TOPICS=("$SOURCE_TOPIC")
if [ -n "$LOCATION_TOPIC" ]; then TOPICS+=("$LOCATION_TOPIC"); fi
LOOP_ARGS=()
if [ "$MODE" = "loop" ]; then LOOP_ARGS+=(--loop); elif [ "$MODE" != "once" ]; then
  echo "Third argument must be once or loop." >&2; exit 2
fi
echo "Playing $BAG"
echo "Topics: ${TOPICS[*]}"
exec rosbag play "$BAG" --clock "${LOOP_ARGS[@]}" --topics "${TOPICS[@]}"
