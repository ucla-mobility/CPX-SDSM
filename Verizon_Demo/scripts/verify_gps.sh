#!/usr/bin/env bash
# Read-only routing-coordinate check. It does not change ROS parameters/files.
set -euo pipefail

ROLE="${1:-}"
WAIT_SECONDS="${2:-10}"
case "$ROLE" in
  vehicle|infrastructure) ;;
  *) echo "Usage: verify_gps.sh {vehicle|infrastructure} [wait_seconds]" >&2; exit 2 ;;
esac
case "$WAIT_SECONDS" in
  ''|*[!0-9]*) echo "wait_seconds must be a non-negative integer" >&2; exit 2 ;;
esac

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck disable=SC1091
source "$PROJECT_ROOT/scripts/load_role.sh" "$ROLE"

echo "ROLE=$ROLE"
printf 'CONFIGURED_LAT=%.9f\n' "$DEFAULT_LAT"
printf 'CONFIGURED_LON=%.9f\n' "$DEFAULT_LON"

if [ -z "$LOCATION_TOPIC" ]; then
  echo "LOCATION_MODE=STATIC_CONFIGURED"
  echo "GPS_CHECK_OK"
  exit 0
fi

echo "LOCATION_MODE=DYNAMIC_WITH_CONFIGURED_FALLBACK"
echo "LOCATION_TOPIC=$LOCATION_TOPIC"
echo "LOCATION_TYPE=$LOCATION_TYPE"

rosmsg md5 "$LOCATION_TYPE" >/dev/null 2>&1 || {
  echo "GPS_CHECK_FAILED: message type $LOCATION_TYPE is unavailable" >&2
  exit 3
}

ACTUAL_TYPE="$(rostopic type "$LOCATION_TOPIC" 2>/dev/null || true)"
if [ -z "$ACTUAL_TYPE" ]; then
  echo "GPS_CHECK_FAILED: topic $LOCATION_TOPIC is not currently advertised" >&2
  exit 4
fi
if [ "$ACTUAL_TYPE" != "$LOCATION_TYPE" ]; then
  echo "GPS_CHECK_FAILED: expected $LOCATION_TYPE, got $ACTUAL_TYPE" >&2
  exit 5
fi

SAMPLE_FILE="$(mktemp)"
trap 'rm -f "$SAMPLE_FILE"' EXIT
if ! timeout "${WAIT_SECONDS}s" rostopic echo -n 1 "$LOCATION_TOPIC" \
    >"$SAMPLE_FILE" 2>/dev/null; then
  echo "GPS_CHECK_FAILED: no message received within ${WAIT_SECONDS}s" >&2
  exit 6
fi

python2 - "$SAMPLE_FILE" "$DEFAULT_LAT" "$DEFAULT_LON" <<'PY'
from __future__ import print_function
import math
import sys
import yaml

with open(sys.argv[1]) as stream:
    sample = next(value for value in yaml.safe_load_all(stream)
                  if value is not None)
lat = float(sample["latitude"])
lon = float(sample["longitude"])
fallback_lat = float(sys.argv[2])
fallback_lon = float(sys.argv[3])

if math.isnan(lat) or math.isnan(lon) or math.isinf(lat) or math.isinf(lon):
    raise SystemExit("GPS_CHECK_FAILED: live coordinate is not finite")
if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
    raise SystemExit("GPS_CHECK_FAILED: live coordinate is outside WGS84 bounds")
if abs(lat) < 1.0e-9 and abs(lon) < 1.0e-9:
    raise SystemExit("GPS_CHECK_FAILED: receiver reported no-fix (0, 0)")

earth_radius_m = 6371000.0
phi1 = math.radians(fallback_lat)
phi2 = math.radians(lat)
dphi = math.radians(lat - fallback_lat)
dlambda = math.radians(lon - fallback_lon)
a = (math.sin(dphi / 2.0) ** 2 +
     math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2)
distance = 2.0 * earth_radius_m * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))

print("LIVE_LAT=%.9f" % lat)
print("LIVE_LON=%.9f" % lon)
print("DISTANCE_FROM_FALLBACK_M=%.1f" % distance)
status = sample.get("status", {})
if isinstance(status, dict) and "status" in status:
    print("GPS_STATUS=%s" % status["status"])
print("GPS_CHECK_OK")
PY
