#!/usr/bin/env bash
set -euo pipefail
ROLE="${1:-}"
BAG="${2:-}"
PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck disable=SC1091
source "$PROJECT_ROOT/scripts/load_role.sh" "$ROLE"
test -f "$BAG" || { echo "Usage: inspect_bag.sh role /path/file.bag" >&2; exit 2; }

rosbag info "$BAG"
echo "---- required connection ----"
python2 - "$BAG" "$SOURCE_TOPIC" "$SOURCE_TYPE" "$SOURCE_MD5" <<'PY'
import rosbag, sys
bag_path, topic, expected_type, expected_md5 = sys.argv[1:]
with rosbag.Bag(bag_path, "r") as bag:
    found = []
    for connection in bag._get_connections(topics=[topic]):
        found.append((connection.datatype, connection.md5sum))
if not found:
    raise SystemExit("required topic is absent: %s" % topic)
for datatype, md5 in found:
    print("topic=%s type=%s md5=%s" % (topic, datatype, md5))
    if datatype != expected_type or md5 != expected_md5:
        raise SystemExit("type/MD5 mismatch")
print("BAG_SOURCE_OK")
PY
