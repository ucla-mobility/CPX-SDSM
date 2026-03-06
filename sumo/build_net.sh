#!/usr/bin/env bash
set -euo pipefail

if [[ -z "${SUMO_HOME:-}" ]]; then
  echo "[ERROR] SUMO_HOME is not set."
  exit 1
fi

NETCONVERT="$SUMO_HOME/bin/netconvert"
if [[ ! -x "$NETCONVERT" ]]; then
  echo "[ERROR] netconvert not found at: $NETCONVERT"
  exit 1
fi

echo "Building net/net.net.xml ..."
"$NETCONVERT" \
  --node-files net/nodes.nod.xml \
  --edge-files net/edges.edg.xml \
  -o net/net.net.xml

echo "Done."
