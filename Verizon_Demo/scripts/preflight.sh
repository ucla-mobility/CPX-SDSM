#!/usr/bin/env bash
set -euo pipefail
ROLE="${1:-}"
OFFLINE="${2:-}"
PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck disable=SC1091
source "$PROJECT_ROOT/scripts/load_role.sh" "$ROLE"

echo "ROLE=$ROLE"
echo "SENDER_ID=$SENDER_ID"
echo "PROJECT_ROOT=$PROJECT_ROOT"
echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "ORIGINAL_WS_SETUP=${ORIGINAL_WS_SETUP:-not-found (bundled message overlay will be used)}"

case "$SENDER_ID" in
  ""|*[!A-Za-z0-9._-]*)
    echo "Invalid SENDER_ID=$SENDER_ID; use only letters, digits, dot, underscore, and hyphen." >&2
    exit 4
    ;;
esac

REGISTRATION="$PROJECT_ROOT/$REGISTRATION_REL"
test -r "$REGISTRATION" || { echo "Missing registration: $REGISTRATION" >&2; exit 4; }
python2 - <<PY
import json
p = r"$REGISTRATION"
with open(p) as f:
    a = json.load(f)
r = a.get("registration", {})
identity = a.get("frozen_config", {}).get("identity", {}).get("attributes", {})
certificate = r.get("certificate", {})
identity = "%s/%s/%s" % (
    identity.get("clientType", ""),
    identity.get("clientSubType", ""),
    identity.get("vendorId", ""))
print("REGISTRATION_FILE=present")
print("REGISTRATION_EXPIRY=%s" % certificate.get("expiration_time", "unknown"))
print("REGISTRATION_IDENTITY=%s" % identity)
if identity != "$EXPECTED_IDENTITY":
    raise SystemExit("registration identity mismatch")
PY

ACTUAL_MD5="$(rosmsg md5 "$SOURCE_TYPE")"
echo "SOURCE_TOPIC=$SOURCE_TOPIC"
echo "SOURCE_TYPE=$SOURCE_TYPE"
echo "SOURCE_MD5=$ACTUAL_MD5"
test "$ACTUAL_MD5" = "$SOURCE_MD5" || {
  echo "Message MD5 mismatch; do not run this bag/live topic." >&2
  exit 5
}
MARKER_ACTUAL_MD5="$(rosmsg md5 "$MARKER_TYPE")"
echo "MARKER_SOURCE_TOPIC=${MARKER_SOURCE_TOPIC:-disabled}"
echo "MARKER_RX_TOPIC=${MARKER_RX_TOPIC:-disabled}"
echo "MARKER_TYPE=$MARKER_TYPE"
echo "MARKER_MD5=$MARKER_ACTUAL_MD5"
test "$MARKER_ACTUAL_MD5" = "$MARKER_MD5" || {
  echo "MarkerArray MD5 mismatch; cannot safely reconstruct the topic." >&2
  exit 5
}
python2 - "$DEFAULT_LAT" "$DEFAULT_LON" <<'PY'
import math
import sys
lat, lon = map(float, sys.argv[1:3])
if math.isnan(lat) or math.isnan(lon) or math.isinf(lat) or math.isinf(lon):
    raise SystemExit("configured routing coordinate is not finite")
if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
    raise SystemExit("configured routing coordinate is outside WGS84 bounds")
if abs(lat) < 1.0e-9 and abs(lon) < 1.0e-9:
    raise SystemExit("configured routing coordinate cannot be (0, 0)")
print("ROUTING_FALLBACK_LAT=%.9f" % lat)
print("ROUTING_FALLBACK_LON=%.9f" % lon)
PY
if [ -n "$LOCATION_TOPIC" ]; then
  if rosmsg md5 "$LOCATION_TYPE" >/dev/null 2>&1; then
    echo "LOCATION_MODE=DYNAMIC_WITH_CONFIGURED_FALLBACK"
    echo "LOCATION_SUPPORT=$LOCATION_TOPIC ($LOCATION_TYPE)"
  else
    echo "LOCATION_MODE=CONFIGURED_FALLBACK"
    echo "LOCATION_SUPPORT=FALLBACK_ONLY ($LOCATION_TYPE is unavailable)"
  fi
else
  echo "LOCATION_MODE=STATIC_CONFIGURED"
  echo "LOCATION_SUPPORT=STATIC_COORDINATE"
fi

if [ "$OFFLINE" != "--offline" ]; then
  python - "$ROLE" <<'PY'
import socket, sys
targets = [
    ("thingspace.verizon.com", 443),
    ("imp-lax-1.prod-us-west-2.thingspace.verizon.com", 8883),
]
failed = []
for host, port in targets:
    try:
        addresses = socket.getaddrinfo(host, port, 0, socket.SOCK_STREAM)
        print("DNS_%s=OK (%d address(es))" % (host, len(addresses)))
        socket.create_connection((host, port), timeout=8).close()
        print("TCP_%s=OK" % port)
    except Exception as exc:
        if isinstance(exc, socket.gaierror):
            print("DNS_%s=FAILED (%s)" % (host, exc))
        else:
            print("TCP_%s=FAILED (%s)" % (port, exc))
        failed.append(port)
if failed:
    print("NETWORK_PREFLIGHT_FAILED: repair DNS/network, then rerun preflight")
    raise SystemExit(6)
PY
fi
echo "PREFLIGHT_OK"
