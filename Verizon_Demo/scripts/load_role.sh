#!/usr/bin/env bash
# Source this file: source scripts/load_role.sh vehicle|infrastructure
set -euo pipefail

ROLE_ARG="${1:-}"
case "$ROLE_ARG" in
  vehicle|infrastructure) ;;
  *) echo "Usage: source scripts/load_role.sh vehicle|infrastructure" >&2; return 2 ;;
esac

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PROJECT_ROOT
# shellcheck disable=SC1090
source "$PROJECT_ROOT/config/$ROLE_ARG.env"
SOURCE_VARIANT="${ETX_SOURCE_VARIANT:-tracking}"
case "$SOURCE_VARIANT" in
  tracking) ;;
  raw)
    SOURCE_PROFILE=autoware-raw
    SOURCE_TOPIC="$RAW_TOPIC"
    SOURCE_TYPE="$RAW_TYPE"
    SOURCE_MD5="$RAW_MD5"
    SOURCE_RATE_HZ=0
    SOURCE_ID="${ROLE_ARG}_raw_detection"
    DEFAULT_BAG_REL="$RAW_BAG_REL"
    ;;
  *) echo "ETX_SOURCE_VARIANT must be tracking or raw" >&2; return 2 ;;
esac
export ROLE SENDER_ID EXPECTED_IDENTITY REGISTRATION_REL SOURCE_PROFILE SOURCE_TOPIC
export CLIENT_MODE CONTROL_HELLO_SECONDS
export SOURCE_TYPE SOURCE_MD5 SOURCE_RATE_HZ SOURCE_ID OBJECT_RX_TOPIC
export MARKER_SOURCE_TOPIC MARKER_TYPE MARKER_MD5 MARKER_RX_TOPIC
export JSON_RX_TOPIC PUBLISH_TYPE RECEIVE_TYPE MAX_PUBLISH_RATE
export DEFAULT_LAT DEFAULT_LON LOCATION_TOPIC LOCATION_TYPE
export ORIGINAL_WS_CANDIDATES DEFAULT_BAG_REL
export SOURCE_VARIANT RAW_TOPIC RAW_TYPE RAW_MD5 RAW_BAG_REL

if [ ! -f /opt/ros/melodic/setup.bash ]; then
  echo "ROS Melodic not found: /opt/ros/melodic/setup.bash" >&2
  return 3
fi
# shellcheck disable=SC1091
source /opt/ros/melodic/setup.bash

OLD_IFS="$IFS"
IFS=:
for candidate in $ORIGINAL_WS_CANDIDATES; do
  if [ -f "$candidate" ]; then
    # shellcheck disable=SC1090
    source "$candidate"
    export ORIGINAL_WS_SETUP="$candidate"
    break
  fi
done
IFS="$OLD_IFS"

# The bundled message-only overlay guarantees the exact bag MD5 even on a
# machine where the original tracking workspace has not yet been sourced.
if [ -f "$PROJECT_ROOT/catkin_ws/devel/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$PROJECT_ROOT/catkin_ws/devel/setup.bash"
fi

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
export ETX_FORCE_CODEC_LITE="${ETX_FORCE_CODEC_LITE:-1}"
